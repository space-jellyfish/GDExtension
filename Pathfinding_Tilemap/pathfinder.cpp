#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <cassert>
#include <memory>
#include <random>
#include <string>
#include <chrono>
#include "pathfinder.h"

using namespace std;
using namespace godot;


array<array<array<uint64_t, TILE_ID_COUNT - 1>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> tile_id_hash_keys;
array<array<array<uint64_t, TypeId::REGULAR>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> type_id_hash_keys;
array<array<uint64_t, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> agent_pos_hash_keys;
TileMap* cells;
unordered_map<uint8_t, int> tile_push_limits;
unordered_map<Vector2i, RRDIADLists, Vector2iHasher> inconsistent_abstract_dists; //goal_pos, rrd lists
unordered_map<Vector2i, RRDCADLists, Vector2iHasher> consistent_abstract_dists; //goal_pos, rrd lists
array<double, SASearchId::SEARCH_END> sa_cumulative_search_times{}; //search_id, cumulative time (ms); value-init to zero

void Pathfinder::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_player_pos", "pos"), &Pathfinder::set_player_pos);
    ClassDB::bind_method(D_METHOD("set_player_last_dir", "dir"), &Pathfinder::set_player_last_dir);
	ClassDB::bind_method(D_METHOD("set_tilemap", "t"), &Pathfinder::set_tilemap);
    ClassDB::bind_method(D_METHOD("set_tile_push_limits", "tpls"), &Pathfinder::set_tile_push_limits);
    ClassDB::bind_method(D_METHOD("generate_hash_keys"), &Pathfinder::generate_hash_keys);

    ClassDB::bind_method(D_METHOD("get_sa_cumulative_search_time", "search_id"), &Pathfinder::get_sa_cumulative_search_time);
    ClassDB::bind_method(D_METHOD("reset_sa_cumulative_search_times"), &Pathfinder::reset_sa_cumulative_search_times);
    ClassDB::bind_method(D_METHOD("pathfind_sa", "search_id", "max_depth", "allow_type_change", "min", "max", "start", "end"), &Pathfinder::pathfind_sa);

    ClassDB::bind_method(D_METHOD("rrd_clear_iad"), &Pathfinder::rrd_clear_iad);
    ClassDB::bind_method(D_METHOD("rrd_clear_cad"), &Pathfinder::rrd_clear_cad);
}

//updates hash
void SANode::init_lv_pos(Vector2i pos) {
    lv_pos = pos;
    hash ^= agent_pos_hash_keys[pos.y][pos.x];
}

//updates hash
//agent_pos is global
//consult agent_type_id if not TRACK_KILLABLE_TYPES
void SANode::init_lv(Vector2i min, Vector2i max, Vector2i agent_pos)  {
    int height = max.y - min.y;
    int width = max.x - min.x;
    lv.reserve(height);
    uint8_t agent_type_id = get_type_id(agent_pos);
    int agent_merge_priority = MERGE_PRIORITIES.at(agent_type_id);

    for (int y = min.y; y < max.y; ++y) {
        vector<uint16_t> row;
        row.reserve(width);

        for (int x = min.x; x < max.x; ++x) {
            Vector2i pos(x, y);
            uint8_t tile_id = get_tile_id(pos);
            uint8_t type_id = get_type_id(pos);

            //to reduce branching factor
            if (!TRACK_ZEROS && tile_id == TileId::ZERO) {
                tile_id = TileId::EMPTY;
            }
            if (type_id != TypeId::REGULAR && !TRACK_KILLABLE_TYPES && MERGE_PRIORITIES.at(type_id) <= agent_merge_priority) {
                type_id = TypeId::REGULAR;
            }
            row.push_back(get_stuff_id(get_back_id(pos), type_id, tile_id));

            if (tile_id > TileId::EMPTY) {
                hash ^= tile_id_hash_keys[y][x][tile_id-1];
            }
            if (type_id < TypeId::REGULAR) {
                hash ^= type_id_hash_keys[y][x][type_id];
            }
        }
        lv.push_back(move(row));
    }
}

void SANode::init_lv_back_ids(Vector2i min, Vector2i max) {
    int height = max.y - min.y;
    int width = max.x - min.x;
    //lv = vector<vector<uint16_t>>(height, vector<uint16_t>(width, REGULAR_TYPE_BITS)); //not necessarily faster
    lv.reserve(height);

    for (int y = min.y; y < max.y; ++y) {
        vector<uint16_t> row;
        row.reserve(width);

        for (int x = min.x; x < max.x; ++x) {
            row.push_back((get_back_id(Vector2i(x, y)) << TILE_AND_TYPE_ID_BITLEN) + REGULAR_TYPE_BITS);
        }
        lv.push_back(move(row));
    }
}

//updates hash
//assumes back_id unchanged
void SANode::set_lv_sid(Vector2i pos, uint16_t new_sid) {
    uint16_t old_sid = get_lv_sid(pos);
    uint8_t old_tile_id = get_tile_id(old_sid);
    uint8_t new_tile_id = get_tile_id(new_sid);
    uint8_t old_type_id = get_type_id(old_sid);
    uint8_t new_type_id = get_type_id(new_sid);

    if (old_type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[pos.y][pos.x][old_type_id];
    }
    if (new_type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[pos.y][pos.x][new_type_id];
    }
    if (old_tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[pos.y][pos.x][old_tile_id - 1];
    }
    if (new_tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[pos.y][pos.x][new_tile_id - 1];
    }
    lv[pos.y][pos.x] = new_sid;
}

//updates hash
void SANode::set_lv_pos(Vector2i pos) {
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
    lv_pos = pos;
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
}

//equivalent to set_lv_sid(pos, get_back_bits(get_lv_sid(pos))) but faster
void SANode::clear_lv_sid(Vector2i pos) {
    uint16_t stuff_id = get_lv_sid(pos);
    uint8_t type_id = get_type_id(stuff_id);
    uint8_t tile_id = get_tile_id(stuff_id);
    if (type_id != TypeId::REGULAR) {
        hash ^= type_id_hash_keys[pos.y][pos.x][type_id];
    }
    if (tile_id != TileId::EMPTY) {
        hash ^= tile_id_hash_keys[pos.y][pos.x][tile_id - 1];
    }
    lv[pos.y][pos.x] = get_back_bits(stuff_id);
}

//assume slide possible
//updates lv_pos, lv, hash
void SANode::perform_slide(Vector2i dir, int push_count) {
    Vector2i merge_lv_pos = lv_pos + (push_count + 1) * dir;

    for (int dist_to_merge=0; dist_to_merge <= push_count; ++dist_to_merge) {
        Vector2i curr_lv_pos = merge_lv_pos - dist_to_merge * dir;
        uint16_t prev_sid = get_lv_sid(curr_lv_pos - dir);
        uint16_t result_sid = prev_sid;

        if (dist_to_merge == 0) {
            uint16_t curr_sid = get_lv_sid(curr_lv_pos);
            result_sid = get_merged_stuff_id(prev_sid, curr_sid);
        }
        set_lv_sid(curr_lv_pos, result_sid);
    }

    //remove src tile
    clear_lv_sid(lv_pos);

    //update lv_pos
    set_lv_pos(lv_pos + dir);
}

void SANode::print_lv() const {
    for (int y=0; y < lv.size(); ++y) {
        for (int x=0; x < lv[0].size(); ++x) {
            if (x != lv[0].size() - 1) {
                printf("%d, ", lv[y][x]);
            }
            else {
                printf("%d", lv[y][x]);
            }
        }
        printf("\n");
    }
}

uint16_t SANode::get_lv_sid(Vector2i pos) const {
    return lv[pos.y][pos.x];
}

//lv_edge is furthest valid cell in dir
//return value is negative if lv_pos is out of bounds in dir
int SANode::get_dist_to_lv_edge(Vector2i dir) const {
    if (dir == Vector2i(1, 0)) {
        return lv[0].size() - 1 - lv_pos.x;
    }
    if (dir == Vector2i(0, 1)) {
        return lv.size() - 1 - lv_pos.y;
    }
    if (dir == Vector2i(-1, 0)) {
        return lv_pos.x;
    }
    return lv_pos.y;
}

//ignore tiles outside of lv
int SANode::get_slide_push_count(Vector2i dir, bool allow_type_change) const {
    Vector2i curr_lv_pos = lv_pos;
    uint16_t curr_stuff_id = get_lv_sid(curr_lv_pos);
    uint8_t curr_tile_id = get_tile_id(curr_stuff_id);
    uint8_t curr_type_id = get_type_id(curr_stuff_id);
    int tile_push_limit = tile_push_limits[curr_type_id];
    bool is_agent_enemy = T_ENEMY.find(curr_type_id) != T_ENEMY.end();
    int push_count = 0;
    int nearest_merge_push_count = -1;
    int dist_to_lv_edge = get_dist_to_lv_edge(dir);

    while (push_count <= min(tile_push_limit, dist_to_lv_edge - 1)) {
        uint8_t temp_type_id = curr_type_id;
        curr_lv_pos += dir;
        curr_stuff_id = get_lv_sid(curr_lv_pos);
        curr_type_id = get_type_id(curr_stuff_id);
        uint8_t curr_back_id = get_back_id(curr_stuff_id);
        if (!is_compatible(temp_type_id, curr_back_id) || (push_count > 0 && is_agent_enemy && curr_type_id == TypeId::PLAYER)) {
            return nearest_merge_push_count;
        }

        //push/merge logic
        //also bubble logic in case it becomes useful
        uint8_t temp_tile_id = curr_tile_id;
        curr_tile_id = get_tile_id(curr_stuff_id);

        if (is_ids_mergeable(temp_tile_id, curr_tile_id)) {
            //check for type change
            if (push_count == 0 && !allow_type_change && is_type_dominant(curr_type_id, temp_type_id)) {
                return -1;
            }

            if (nearest_merge_push_count == -1) {
                nearest_merge_push_count = push_count;
            }
            if (curr_tile_id != TileId::ZERO || curr_type_id != TypeId::REGULAR) {
                if (temp_tile_id == TileId::ZERO && curr_tile_id == TileId::EMPTY && curr_type_id == TypeId::REGULAR) {
                    return push_count; //bubble
                }
                return nearest_merge_push_count;
            }
        }

        if (push_count == tile_push_limit) {
            return nearest_merge_push_count;
        }
        push_count++;
    }
    return -1;
}


void SASearchNode::init_sanode(Vector2i min, Vector2i max, Vector2i start) {
    sanode = make_shared<SANode>();
    sanode->init_lv_pos(start - min);
    sanode->init_lv(min, max, start);
}

//updates prev, prev_push_count
shared_ptr<SASearchNode> SASearchNode::try_slide(Vector2i dir, bool allow_type_change) {
    int push_count = sanode->get_slide_push_count(dir, allow_type_change);
    if (push_count != -1) {
        shared_ptr<SASearchNode> m = make_shared<SASearchNode>();
        m->sanode = make_shared<SANode>(*sanode);
        m->prev = shared_from_this();
        m->prev_push_count = push_count;
        m->prune_backtrack(dir);
        m->sanode->perform_slide(dir, push_count);
        return m;
    }
    return nullptr;
}

//updates prev, prev_push_count
shared_ptr<SASearchNode> SASearchNode::try_split(Vector2i dir, bool allow_type_change) {
    uint16_t src_sid = sanode->get_lv_sid(sanode->lv_pos);
    int src_pow = get_tile_pow(get_tile_id(src_sid));
    if (!is_pow_splittable(src_pow)) {
        return nullptr;
    }

    //halve src tile, try_slide, then (re)set its tile_id
    //splitted is new tile, splitter is old tile
    uint16_t untyped_split_sid = get_back_bits(src_sid) + get_splitted_tid(get_tile_id(src_sid));
    uint16_t splitted_sid = untyped_split_sid + get_type_bits(src_sid);
    sanode->set_lv_sid(sanode->lv_pos, splitted_sid);
    shared_ptr<SASearchNode> ans = try_slide(dir, allow_type_change);
    if (ans != nullptr) {
        //insert splitter tile
        uint16_t splitter_sid = untyped_split_sid + REGULAR_TYPE_BITS;
        ans->sanode->set_lv_sid(sanode->lv_pos, splitter_sid);
    }
    //reset src tile in this
    sanode->set_lv_sid(sanode->lv_pos, src_sid);

    return ans;
}

//g/h/f are default-init to 0
//updates prev_action for slide/split, stores result in curr->neighbors
//return newly-created SASearchNode (this ensures duplicate detection works as intended)
shared_ptr<SASearchNode> SASearchNode::try_action(Vector3i normalized_action, Vector2i lv_end, bool allow_type_change) {
    //check for stored neighbor
    auto it = neighbors.find(normalized_action);
    if (it != neighbors.end()) {
        SANeighbor neighbor = (*it).second;

        if (neighbor.unprune_threshold) {
            //pruned or action invalid
            return nullptr;
        }
        if (neighbor.sanode) {
            shared_ptr<SASearchNode> ans = make_shared<SASearchNode>();
            ans->sanode = neighbor.sanode;
            if (normalized_action.z == ActionId::JUMP) {
                unsigned int jump_dist = manhattan_dist(sanode->lv_pos, ans->sanode->lv_pos);
                ans->prev_action = Vector3i(jump_dist * normalized_action.x, jump_dist * normalized_action.y, ActionId::JUMP);
            }
            else {
                ans->prev_action = normalized_action;
            }
            ans->prev = shared_from_this();
            ans->prev_push_count = neighbor.push_count;
            return ans;
        }
    }

    Vector2i dir(normalized_action.x, normalized_action.y);
    shared_ptr<SASearchNode> ans;
    switch(normalized_action.z) {
        case ActionId::SLIDE:
            ans = try_slide(dir, allow_type_change);
            break;
        case ActionId::SPLIT:
            ans = try_split(dir, allow_type_change);
            break;
        case ActionId::JUMP:
            ans = try_jump(dir, lv_end, allow_type_change);
            break;
        default:
            ans = try_slide(dir, allow_type_change);
    }
    if (ans) {
        if (normalized_action.z != ActionId::JUMP) {
            ans->prev_action = normalized_action;
        }
        //update neighbors
        neighbors[normalized_action] = {0, ans->sanode, ans->prev_push_count};
    }
    else {
        //action invalid; don't unprune
        neighbors[normalized_action] = {numeric_limits<unsigned int>::max(), nullptr, 0};
    }
    return ans;
}

//assume immediate neighbor is in_bounds
//updates prev, prev_action, prev_push_count
//doesn't change agent type/tile_id
//only jump through empty_and_regular tiles
//see Devlog/jump_conditions for details
//when generating from jp, generate split in all dirs, generate slide iff next tile isn't empty_and_regular
shared_ptr<SASearchNode> SASearchNode::try_jump(Vector2i dir, Vector2i lv_end, bool allow_type_change) {
    //check bounds NAH
    //check empty
    Vector2i curr_pos = sanode->lv_pos + dir;
    if (!is_tile_empty_and_regular(sanode->get_lv_sid(curr_pos))) {
        return nullptr;
    }

    //init next_dirs
    uint8_t src_type_id = get_type_id(sanode->get_lv_sid(sanode->lv_pos));
    bool horizontal = (H_DIRS.find(dir) != H_DIRS.end());
    vector<NextDir> next_dirs;
    Vector2i perp_dir1 = horizontal ? Vector2i(0, 1) : Vector2i(1, 0);
    Vector2i perp_dir2 = horizontal ? Vector2i(0, -1) : Vector2i(-1, 0);
    for (Vector2i next_dir : {perp_dir1, perp_dir2}) {
        if (!sanode->get_dist_to_lv_edge(next_dir)) {
            next_dirs.emplace_back(next_dir, false, false);
            continue;
        }
        uint16_t next_stuff_id = sanode->get_lv_sid(sanode->lv_pos + next_dir);
        bool blocked = !(is_tile_empty_and_regular(next_stuff_id) && is_compatible(src_type_id, get_back_id(next_stuff_id)));
        next_dirs.emplace_back(next_dir, true, blocked);
    }
    int dist_to_lv_edge = sanode->get_dist_to_lv_edge(dir);
    int curr_dist = 1;
    next_dirs.emplace_back(dir, curr_dist < dist_to_lv_edge, false);
    shared_ptr<SASearchNode> curr_jp;

    while (curr_dist <= dist_to_lv_edge) {
        uint16_t curr_stuff_id = sanode->get_lv_sid(curr_pos);

        //check obstruction
        if (!is_tile_empty_and_regular(curr_stuff_id) || !is_compatible(src_type_id, get_back_id(curr_stuff_id))) {
            return nullptr;
        }

        //get current jump point; reuse sanode if possible
        shared_ptr<SANode> prev_sanode = (curr_dist > 1) ? curr_jp->sanode : nullptr;
        curr_jp = get_jump_point(prev_sanode, dir, curr_pos, curr_dist);

        //check lv_end
        if (curr_pos == lv_end) {
            return curr_jp;
        }

        //check for jump conditions
        for (NextDir& next_dir : next_dirs) {
            //bound check
            if (!next_dir.in_bounds) { //once false, in_bounds stays false
                //curr_jp->prune_invalid_action_ids(next_dir.dir); //redundant, pathfind_sa_*() does bounds check
                continue;
            }

            //update in_bounds
            if (next_dir.dir == dir) {
                next_dir.in_bounds = curr_dist + 1 < dist_to_lv_edge;
            }

            //compatibility check
            uint16_t next_stuff_id = sanode->get_lv_sid(curr_pos + next_dir.dir);
            bool next_compatible = is_compatible(src_type_id, get_back_id(next_stuff_id));
            bool next_empty_and_regular = is_tile_empty_and_regular(next_stuff_id);
            if (!next_compatible) {
                curr_jp->prune_invalid_action_ids(next_dir.dir);
                //update blocked
                next_dir.blocked = !(next_empty_and_regular && next_compatible);
                continue;
            }

            //next empty check
            if (next_dir.dir == dir && next_empty_and_regular) {
                continue;
            }

            //prune jump if horizontal, perp, and not blocked
            if (horizontal && next_dir.dir != dir && !next_dir.blocked) {
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::JUMP)] = {1, nullptr, 0};
            }

            //prune slide if empty, prune jump if not
            if (next_empty_and_regular) {
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::SLIDE)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
            }
            else {
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::JUMP)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
            }

            //horizontal perp empty check
            if (horizontal && next_dir.dir != dir && next_empty_and_regular && next_dir.blocked) {
                return curr_jp;
            }

            for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
                //store next_action result in curr_jp->neighbors and if valid, return curr_jp
                //!(vertical && next_dir.dir == dir && empty) bc "next empty check"
                if (action_id == ActionId::JUMP && (horizontal || !next_empty_and_regular)) {
                    continue;
                }

                Vector3i normalized_next_action = Vector3i(next_dir.dir.x, next_dir.dir.y, action_id);
                //this does not create any permanent refs to curr_jp->sanode, so it can be reused for the next curr_jp
                shared_ptr<SASearchNode> neighbor = curr_jp->try_action(normalized_next_action, lv_end, allow_type_change);
                if (neighbor) {
                    if (!horizontal || next_dir.dir == dir || action_id != ActionId::SLIDE || next_dir.blocked || neighbor->prev_push_count) {
                        return curr_jp;
                    }
                }
            }

            //update blocked
            next_dir.blocked = !(next_empty_and_regular && next_compatible);
        }

        curr_pos += dir;
        ++curr_dist;
    }

    return nullptr;
}

//assume jp_pos != src_lv_pos
//assume jp_pos is within bounds
shared_ptr<SASearchNode> SASearchNode::get_jump_point(shared_ptr<SANode> prev_sanode, Vector2i dir, Vector2i jp_pos, unsigned int jump_dist) {
    shared_ptr<SASearchNode> ans = make_shared<SASearchNode>();
    ans->sanode = prev_sanode ? prev_sanode : make_shared<SANode>(*sanode);
    Vector2i src_lv_pos = ans->sanode->lv_pos;
    ans->sanode->set_lv_pos(jp_pos);
    ans->sanode->set_lv_sid(jp_pos, get_jumped_stuff_id(ans->sanode->get_lv_sid(src_lv_pos), ans->sanode->get_lv_sid(jp_pos)));
    ans->sanode->clear_lv_sid(src_lv_pos);

    //prev stuff
    ans->prev_action = Vector3i(jump_dist * dir.x, jump_dist * dir.y, ActionId::JUMP);
    ans->prev = shared_from_this();
    ans->prev_push_count = 0;

    //prune stuff
    ans->neighbors[Vector3i(-dir.x, -dir.y, ActionId::SLIDE)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
    ans->neighbors[Vector3i(-dir.x, -dir.y, ActionId::JUMP)] = {2 * jump_dist + 1, sanode, 0}; //see Pictures/reverse_jump_unprune_threshold

    return ans;
}

//prune all actions in dir
void SASearchNode::prune_invalid_action_ids(Vector2i dir) {
    for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
        neighbors[Vector3i(dir.x, dir.y, action_id)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
    }
}

//assume prev is set correctly
void SASearchNode::prune_backtrack(Vector2i dir) {
    //backtrack prevention: prune curr in neighbor->neighbors when storing neighbor in curr->neighbors
    //skip slide if -dir and prev slide merged with empty or prev split merged with empty
    //skip split if -dir and prev slide merged with same sign regular or prev split merged with same sign regular
    //unprune if there is 3+ dist improvement ({n, n+1} -> {n-1, n-2})
    uint16_t neighbor_sid = sanode->get_lv_sid(sanode->lv_pos + dir);
    uint8_t src_tile_id = get_tile_id(sanode->get_lv_sid(sanode->lv_pos));
    uint8_t neighbor_tile_id = get_tile_id(neighbor_sid);
    uint8_t neighbor_type_id = get_type_id(neighbor_sid);
    if (is_tile_empty_and_regular(neighbor_sid)) {
        neighbors[Vector3i(-dir.x, -dir.y, ActionId::SLIDE)] = {3, prev->sanode, 0};
    }
    if (is_same_sign_merge(src_tile_id, neighbor_tile_id) && neighbor_type_id == TypeId::REGULAR) {
        neighbors[Vector3i(-dir.x, -dir.y, ActionId::SPLIT)] = {3, prev->sanode, 0};
    }
}

//WARNING: use of this function invalidates the assumption that stored neighbors have the same g/h/f costs
//assume better_dist isn't the same, but compares equal to curr (same lv and lv_pos)
//if there are two prunes on the same action, keep the stronger one (higher unprune_threshold)
//since better_dist has better dist, curr will never generate again, so its neighbors can be cleared 
void SASearchNode::transfer_neighbors(shared_ptr<SASearchNode> better_dist, int dist_improvement) {
    unordered_map<Vector3i, SANeighbor, ActionHasher>& dest_neighbors = better_dist->neighbors;

    for (auto [normalized_action, neighbor] : neighbors) {
        //transfer if prune is invalid-based
        if (neighbor.unprune_threshold == numeric_limits<unsigned int>::max()) {
            if (dest_neighbors.find(normalized_action) == dest_neighbors.end()) {
                dest_neighbors[normalized_action] = {numeric_limits<unsigned int>::max(), nullptr, 0};
            }
            continue;
        }
        
        unsigned int eff_unprune_threshold = max(0, static_cast<int>(neighbor.unprune_threshold) - dist_improvement);
        auto it = dest_neighbors.find(normalized_action);
        if (it != dest_neighbors.end()) {
            SANeighbor& dest_neighbor = (*it).second;

            if (eff_unprune_threshold > dest_neighbor.unprune_threshold) {
                dest_neighbor.unprune_threshold = eff_unprune_threshold;
            }
            if (!dest_neighbor.sanode) {
                dest_neighbor.sanode = neighbor.sanode;
                dest_neighbor.push_count = neighbor.push_count;
            }
        }
        else {
            dest_neighbors[normalized_action] = {eff_unprune_threshold, neighbor.sanode, neighbor.push_count};
        }
    }
    neighbors.clear();
}

Array SASearchNode::trace_path_normalized_actions(int path_len) {
	Array ans;
	ans.resize(path_len);
	int index = path_len - 1;
	shared_ptr<SASearchNode> curr = shared_from_this();

	while (curr->prev != nullptr) {
        Vector3i normalized_prev_action = (curr->prev_action.z == ActionId::JUMP) ? Vector3i(sgn(curr->prev_action.x), sgn(curr->prev_action.y), ActionId::SLIDE) : curr->prev_action;
        int prev_action_dist = get_action_dist(curr->prev_action);

        for (int dist=0; dist < prev_action_dist; ++dist) {
            ans[index] = normalized_prev_action;
            --index;
        }
		curr = curr->prev;
	}
	UtilityFunctions::print("PF TRACED PATH SIZE: ", ans.size());
	return ans;
}

//get lv_pos and tile_id from SASearchNode
//assume array[TileId::EMPTY] == array[TileId::ZERO]
//combine lp_to_path_indices and pn_to_admissible_tile_ids into single structure? NAH too complicated
bool SASearchNode::is_on_prev_path(unique_ptr<PathInfo>& pi, int largest_affected_path_index) {
    auto indices_itr = pi->lp_to_path_indices.find(sanode->lv_pos);
    if (indices_itr == pi->lp_to_path_indices.end()) {
        return false;
    }

    //loop through admissible indices (inclusive of largest_affected_path_index)
    std::set<unsigned int>& prev_path_indices_at_lv_pos = (*indices_itr).second;
    for (auto index_itr = prev_path_indices_at_lv_pos.lower_bound(largest_affected_path_index); index_itr != prev_path_indices_at_lv_pos.end(); ++index_itr) {
        auto tile_ids_itr = pi->pn_to_admissible_tile_ids.find(PathNode(sanode->lv_pos, *index_itr));
        if (tile_ids_itr != pi->pn_to_admissible_tile_ids.end() && (*tile_ids_itr).second[get_tile_id(sanode->get_lv_sid(sanode->lv_pos))]) {
            return true;
        }
    }
    return false;
}

std::function<unsigned int(Vector2i)> SASearchNode::get_radius_getter(int iw_shape_id, Vector2i dest_lv_pos) {
    switch (iw_shape_id) {
        case IWShapeId::DIAMOND:
            return [dest_lv_pos](Vector2i curr_lv_pos) -> unsigned int { return manhattan_dist(curr_lv_pos, dest_lv_pos); };
        default:
            return [dest_lv_pos](Vector2i curr_lv_pos) -> unsigned int { return max(abs(curr_lv_pos.x - dest_lv_pos.x), abs(curr_lv_pos.y - dest_lv_pos.y)); };
    }
}

//for is_on_prev_path(), array[TileId::EMPTY] should be equal to array[TileId::ZERO]
//speed up with bit operations
void SASearchNode::relax_admissibility(bitset<TILE_ID_COUNT>& admissible_tile_ids) {
    /*
    for (int i = TileId::EMPTY + 1; i < TileId::ZERO - 1; ++i) {
        if (admissible_tile_ids[i + 1]) {
            admissible_tile_ids[i] = true;
        }
    }
    for (int i = TILE_ID_COUNT - 1; i > TileId::ZERO + 1; --i) {
        if (admissible_tile_ids[i - 1]) {
            admissible_tile_ids[i] = true;
        }
    }
    1-14 inclusive
    18-31 inclusive
    */
    uint32_t u = admissible_tile_ids.to_ulong();
    uint32_t first = u & 0x7FFF0000; //1-15 inclusive
    uint32_t second = u & 0x7FFF; //17-31 inclusive
    uint32_t ans = (u & 0x80008000) + ((first | first << 1) + (second | second >> 1) & 0x7FFF7FFF);
    admissible_tile_ids = bitset<TILE_ID_COUNT>(ans);
}

//don't restrict any tile_ids if ZERO was pushed/merged, there's no difference (bc of bubbling)
//according to StackOverflow/37009695, return foo() works
void SASearchNode::relax_admissibility(bitset<TILE_ID_COUNT>& admissible_tile_ids, bool is_next_merge, uint8_t adjacent_tile_id) {
    if (is_next_merge) {
        if (!is_tile_unsigned(adjacent_tile_id)) {
            bitset<TILE_ID_COUNT> new_admissible_tile_ids;

            //same sign merge
            if (get_signed_tile_pow(adjacent_tile_id) != TILE_POW_MAX && admissible_tile_ids[get_merged_tile_id(adjacent_tile_id, adjacent_tile_id)]) {
                new_admissible_tile_ids[adjacent_tile_id] = true;
            }
            //zero merge
            if (admissible_tile_ids[adjacent_tile_id]) {
                new_admissible_tile_ids[TileId::ZERO] = true;
                new_admissible_tile_ids[TileId::EMPTY] = true;
            }
            //opposite sign merge
            if (admissible_tile_ids[TileId::ZERO]) {
                new_admissible_tile_ids[get_opposite_tile_id(adjacent_tile_id)] = true;
            }
            admissible_tile_ids = new_admissible_tile_ids;
        }
    }
    else {
        if (!is_tile_unsigned(adjacent_tile_id)) {
            admissible_tile_ids[TileId::ZERO] = false;
            admissible_tile_ids[TileId::EMPTY] = false;
            admissible_tile_ids[adjacent_tile_id] = false;
            admissible_tile_ids[get_opposite_tile_id(adjacent_tile_id)] = false;
        }
    }
    relax_admissibility(admissible_tile_ids);
}

void SASearchNode::trace_node_info(unique_ptr<PathInfo>& pi, PathNode& pn, bitset<TILE_ID_COUNT>& admissible_tile_ids) {
    //store path_index
    pi->lp_to_path_indices[pn.lv_pos].insert(pn.index);

    //store admissible_tile_ids
    pi->pn_to_admissible_tile_ids[pn] = admissible_tile_ids;
}


//for testing; return search_id's cumulative search time in ms
double Pathfinder::get_sa_cumulative_search_time(int search_id) {
    //assert(search_id >= 0 && search_id < SASearchId::SEARCH_END);
    return sa_cumulative_search_times[search_id];
}

void Pathfinder::reset_sa_cumulative_search_times() {
    fill(sa_cumulative_search_times.begin(), sa_cumulative_search_times.end(), 0);
}

//use an rrd heuristic search for enclosure check
Array Pathfinder::pathfind_sa(int search_id, int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //type check
    if (!is_compatible(get_type_id(start), get_back_id(end))) {
        return Array();
    }

    //for timing
    auto start_time = chrono::high_resolution_clock::now();

    Array ans;
    switch (search_id) {
        case SASearchId::DIJKSTRA:
            ans = pathfind_sa_dijkstra(max_depth, allow_type_change, min, max, start, end);
            break;
        case SASearchId::HBJPD:
            ans = pathfind_sa_hbjpd(max_depth, allow_type_change, min, max, start, end);
            break;
        case SASearchId::MDA:
            ans = pathfind_sa_mda(max_depth, allow_type_change, min, max, start, end);
            break;
        case SASearchId::IADA:
            ans = pathfind_sa_iada(max_depth, allow_type_change, min, max, start, end);
            break;
        case SASearchId::HBJPMDA:
            ans = pathfind_sa_hbjpmda(max_depth, allow_type_change, min, max, start, end);
            break;
        case SASearchId::HBJPIADA:
            ans = pathfind_sa_hbjpiada(max_depth, allow_type_change, min, max, start, end);
            break;
        case SASearchId::IWDMDA:
            ans = pathfind_sa_iwdmda(max_depth, allow_type_change, min, max, start, end);
            break;
        default:
            ans = pathfind_sa_hbjpd(max_depth, allow_type_change, min, max, start, end);
    }

    //timing
    auto end_time = chrono::high_resolution_clock::now();
    sa_cumulative_search_times[search_id] += chrono::duration<double, milli>(end_time - start_time).count();

    return ans;
}

//WARNING: do not call directly; call via pathfind_sa(search_id) bc it does some important stuff
//don't generate from nodes at max_depth or if generate, don't do numeric_limits<int>::max() heuristic early exit until max_depth check
//open nodes are optimal since edges are unit cost
//best_dists is optimal bc open is optimal
Array Pathfinder::pathfind_sa_dijkstra(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //NEEDS PROFILING with deque
    //use ptrs bc prev requires a fixed addr, faster to copy, duplicates in open use less mem
    //use shared_ptr instead of unique_ptr bc prev/neighbors also require it
    open_sa_t open;
    //to prevent duplicate nodes with equal or worse cost from being pushed to open
    //see CBS section 4.3, duplicate detection and pruning
    //change to unordered_set<LightweightSANode> for less memory usage but no collision checking,
    //where LightweightSANode only stores hash (for KeyEqual) and prev/prev_action (for trace_path())
    //IMPORTANT: not the same as closed, bc (with the exception of DIJKSTRA) best_dists is non-optimal
    //closed is redundant; shared_ptrs are stored in best_dists, so trace_path() will still work
    closed_sa_t best_dists; //using f
    Vector2i lv_end = end - min;

    shared_ptr<SASearchNode> first = make_shared<SASearchNode>();
    first->init_sanode(min, max, start);
    //first can have prev_action and prev_push_count uninitialized
    open.push(first);
    best_dists.insert(first);

    while (!open.empty()) {
        //expand node
        shared_ptr<SASearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            return curr->trace_path_normalized_actions(curr->f);
        }
        //best_dists check is unnecessary bc open doesn’t receive duplicate nodes
        open.pop();
        //assert(curr->f == (*best_dists.find(curr))->f);

        //check max depth here bc all edges unit length
        if (curr->f == max_depth) {
            continue;
        }

        //generate neighbors
        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                //open doesn’t receive duplicate nodes -> every node generates at most once -> no need to push neighbor to sanode_ref_pool
                //if closed optimal, no node generates more than once:
                //assume closed optimal and a node generates more than once; then at some point node must be at top of open with best dist, and at some later point better path to node must be found
                //"node at top of open" implies node is optimal, thus better path doesn't exist, contradiction

                //calculate cost(s)
                //update f/g/h in pathfind_sa_*() bc they could be used differently in each search type
                //optimizations (use if no duplicate code is required or there are significant savings):
                    //if neighbor in best_dists, recycle h-cost instead of recalculating it
                    //move f-cost calculation to after best_dists check

                //check if neighbor is duplicate with equal or worse cost
                //this is necessary since from
                //0, 0, 0
                //P1, 1, 1
                //both {(1, 0, 0), (1, 0, 1), (0, -1, 1)} and {(0, -1, 0), (1, 0, 0), (1, 0, 0)} result in same node
                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    //neighbor is duplicate
                    //if (neighbor->f >= (*it)->f) {
                        //neighbor has equal or worse cost
                        continue;
                    //}
                    /* not possible bc open is optimal
                    else {
                        //use *it=neighbor (wrong syntax but i don't care) if neighbor path better, and curr!=best_dists.find(curr) in expanding best_dists check
                        //this way curr!=best_dists.find(curr) implies curr is a duplicate node with equal or worse dist
                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->f - neighbor->f); //so no neighbor info from better dist SASearchNodes is lost
                        best_dists.erase(it);
                    }*/
                }
                neighbor->f = curr->f + 1;

                //if neighbor hasn't been visited, it can't be ancestor of curr => no loop
                open.push(neighbor);
                best_dists.insert(neighbor);
            }
        }
    }
    return Array();
}

//open and best_dists not necessarily optimal
Array Pathfinder::pathfind_sa_hbjpd(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    open_sa_t open;
    closed_sa_t best_dists;
    Vector2i lv_end = end - min;

    shared_ptr<SASearchNode> first = make_shared<SASearchNode>();
    first->init_sanode(min, max, start);
    open.push(first);
    best_dists.insert(first);

    while (!open.empty()) {
        shared_ptr<SASearchNode> curr = open.top();

        if (curr->sanode->lv_pos + min == end) {
            return curr->trace_path_normalized_actions(curr->f);
        }
        //open may receive duplicate nodes (see Pictures/jpd_edge_case)
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        //branch prediction makes this relatively quick, so check max_depth when both expanding and generating (unless it is redundant)
        if (curr->f == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
                //generate split in all dirs, don't generate slide if next tile is empty_and_regular
                //generate jump iff next tile is empty_and_regular and dir is natural - handled in try_jump()
                //only search in dir of natural neighbors (except first node) - handled via pruning
                if (action_id == ActionId::SLIDE && is_tile_empty_and_regular(curr->sanode->get_lv_sid(curr->sanode->lv_pos + dir))) {
                    continue;
                }
                Vector3i normalized_action = Vector3i(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                neighbor->f = curr->f + get_action_dist(neighbor->prev_action);

                //place check here to catch nodes that exceed max_depth as early as possible
                //since open << closed for typical search, generating check is not much more expensive than expanding check
                if (neighbor->f > max_depth) {
                    continue;
                }

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    if (neighbor->f >= (*it)->f) {
                        continue;
                    }
                    else {
                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->f - neighbor->f);
                        best_dists.erase(it);
                    }
                }
                open.push(neighbor);
                best_dists.insert(neighbor);
            }  
        }
    }
    return Array();
}

//open and best_dists not necessarily optimal
//closed optimal bc heuristic consistent (see SA lec5)
Array Pathfinder::pathfind_sa_mda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    open_sa_t open;
    closed_sa_t best_dists; //hs must be same, so prune if g >= best_g; see also Pictures/best_dists_justification_astar
    Vector2i lv_end = end - min;

    shared_ptr<SASearchNode> first = make_shared<SASearchNode>();
    first->init_sanode(min, max, start);
    first->h = manhattan_dist(first->sanode->lv_pos, lv_end);
    first->f = first->h;
    //first can have prev_action and prev_push_count uninitialized
    open.push(first);
    best_dists.insert(first);

    while (!open.empty()) {
        shared_ptr<SASearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            return curr->trace_path_normalized_actions(curr->g);
        }
        //open may receive duplicate nodes (see Pictures/mda_closed_check_is_necessary)
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        if (curr->g == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                neighbor->g = curr->g + 1;
                neighbor->h = manhattan_dist(neighbor->sanode->lv_pos, lv_end);
                neighbor->f = neighbor->g + neighbor->h;

                //manhattan is consistent, so final path_len >= curr->f and f is monotonically increasing
                //f can increase more than one unit at a time, so == check when expanding doesn't work
                //if curr->g == max_depth, neighbor->g > max_depth, neighbor->f > max_depth, so no nodes are generated from max_depth
                if (neighbor->f > max_depth) {
                    continue;
                }

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    if (neighbor->g >= (*it)->g) {
                        continue;
                    }
                    else {
                        //check for 3+ dist improvement (bc I couldn't think of an example)
                        if (neighbor->g <= (*it)->g - 3) {
                            UtilityFunctions::print("MDA FOUND 3+ DIST IMPROVEMENT!!!");
                            first->sanode->print_lv();
                            UtilityFunctions::print("start: ", start);
                            UtilityFunctions::print("end: ", end);
                            assert(false);
                        }

                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->f - neighbor->f);
                        best_dists.erase(it);
                    }
                }
                open.push(neighbor);
                best_dists.insert(neighbor);
            }
        }
    }
    return Array();
}

//f can decrease
//open and returned path are not necessarily optimal
//if h(first) == numeric_limits<int>::max(), exit early bc no path exists
Array Pathfinder::pathfind_sa_iada(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    open_sa_t open;
    closed_sa_t best_dists; //hs must be same, so prune if g >= best_g; see also Pictures/best_dists_justification_astar
    Vector2i lv_end = end - min;
    uint8_t agent_type_id = get_type_id(start);
    rrd_init_iad(end);

    shared_ptr<SASearchNode> first = make_shared<SASearchNode>();
    first->init_sanode(min, max, start);
    first->h = rrd_resume_iad(end, start, agent_type_id);

    //enclosed goal check
    if (first->h == numeric_limits<int>::max()) {
        return Array();
    }

    first->f = first->h;
    //first can have prev_action and prev_push_count uninitialized
    open.push(first);
    best_dists.insert(first);

    while (!open.empty()) {
        shared_ptr<SASearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            return curr->trace_path_normalized_actions(curr->g);
        }
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        if (curr->g == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                //nodes can generate more than once -> ++(neighbor->g) can cause double increment
                neighbor->g = curr->g + 1;
                neighbor->h = rrd_resume_iad(end, min + neighbor->sanode->lv_pos, agent_type_id);
                neighbor->f = neighbor->g + neighbor->h;

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    if (neighbor->g >= (*it)->g) {
                        continue;
                    }
                    else {
                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->f - neighbor->f);
                        best_dists.erase(it);
                    }
                }
                open.push(neighbor);
                best_dists.insert(neighbor);
            }
        }
    }
    return Array();
}

//closed optimal, open and best_dists not
Array Pathfinder::pathfind_sa_hbjpmda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    open_sa_t open;
    closed_sa_t best_dists; //hs must be same, so prune if g >= best_g; see also Pictures/best_dists_justification_astar
    Vector2i lv_end = end - min;

    shared_ptr<SASearchNode> first = make_shared<SASearchNode>();
    first->init_sanode(min, max, start);
    first->h = manhattan_dist(first->sanode->lv_pos, lv_end);
    first->f = first->h;
    //first can have prev_action and prev_push_count uninitialized
    open.push(first);
    best_dists.insert(first);

    while (!open.empty()) {
        shared_ptr<SASearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            return curr->trace_path_normalized_actions(curr->g);
        }
        //open may receive duplicate nodes (see Pictures/mda_closed_check_is_necessary, replace agent w/ 2 and empty tiles w/ 1)
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        if (curr->g == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
                if (action_id == ActionId::SLIDE && is_tile_empty_and_regular(curr->sanode->get_lv_sid(curr->sanode->lv_pos + dir))) {
                    continue;
                }
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                neighbor->g = curr->g + get_action_dist(neighbor->prev_action);
                neighbor->h = manhattan_dist(neighbor->sanode->lv_pos, lv_end);
                neighbor->f = neighbor->g + neighbor->h;

                //manhattan is consistent, so final path_len >= curr->f and f is monotonically increasing
                //f can increase more than one unit at a time, so == check when expanding doesn't work
                //if curr->g == max_depth, neighbor->g > max_depth, neighbor->f > max_depth, so no nodes are generated from max_depth
                if (neighbor->f > max_depth) {
                    continue;
                }

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    if (neighbor->g >= (*it)->g) {
                        continue;
                    }
                    else {
                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->f - neighbor->f);
                        best_dists.erase(it);
                    }
                }
                open.push(neighbor);
                best_dists.insert(neighbor);
            }
        }
    }
    return Array();
}

//open and returned path are not necessarily optimal
//if h(first) == numeric_limits<int>::max(), exit early bc no path exists
Array Pathfinder::pathfind_sa_hbjpiada(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    open_sa_t open;
    closed_sa_t best_dists; //hs must be same, so prune if g >= best_g; see also Pictures/best_dists_justification_astar
    Vector2i lv_end = end - min;
    uint8_t agent_type_id = get_type_id(start);
    rrd_init_iad(end);

    shared_ptr<SASearchNode> first = make_shared<SASearchNode>();
    first->init_sanode(min, max, start);
    first->h = rrd_resume_iad(end, start, agent_type_id);

    //enclosed goal check
    if (first->h == numeric_limits<int>::max()) {
        return Array();
    }

    first->f = first->h;
    //first can have prev_action and prev_push_count uninitialized
    open.push(first);
    best_dists.insert(first);

    while (!open.empty()) {
        shared_ptr<SASearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            return curr->trace_path_normalized_actions(curr->g);
        }
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        if (curr->g == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
                if (action_id == ActionId::SLIDE && is_tile_empty_and_regular(curr->sanode->get_lv_sid(curr->sanode->lv_pos + dir))) {
                    continue;
                }
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                //nodes can generate more than once, += can cause double increment
                neighbor->g = curr->g + get_action_dist(neighbor->prev_action);
                neighbor->h = rrd_resume_iad(end, min + neighbor->sanode->lv_pos, agent_type_id);
                neighbor->f = neighbor->g + neighbor->h;

                if (neighbor->g > max_depth) {
                    curr->neighbors[normalized_action] = {numeric_limits<unsigned int>::max(), nullptr, 0}; //prune neighbor in case curr generates again
                    continue;
                }

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    if (neighbor->g >= (*it)->g) {
                        continue;
                    }
                    else {
                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->f - neighbor->f);
                        best_dists.erase(it);
                    }
                }
                open.push(neighbor);
                best_dists.insert(neighbor);
            }
        }
    }
    return Array();
}

//idea: iteratively widening diamond; ignore tiles outside of diamond when searching to obtain a path that informs search in the next iteration
//iwd SANodes should preserve all back_ids, even ones outside the diamond
//only use path from the previous iteration bc older iterations' paths are less relevant
    //using all paths would case uneven reduction biased towards nodes near the goal
//only check for h_reduction if within prev_radius + 1 of dest
//not optimal bc path-informed heuristic is inconsistent
//allow reduction to negative h? yes
//use manhattan_dist bc iw + iada would be too inaccurate
//use h_reduction proportional to pathlen/manhattan_radius_of_shape since larger radius is harder
    //NEEDS PROFILING (both speed and suboptimality); this makes path near start more suboptimal than path near goal (might be good?)
//apply proportional h_reduction to nodes within same path?
    //higher h_reduction for nodes closer to goal, since they have been validated already
//use a reduced h_reduction if upcoming location in path has been affected?
    //NAH, largest_affected_path_index update in path_informed_mda() accounts for pushing that occurs in prev_path
//for simulated annealing version, choose random h_reduction from an interval
    //use greater lower bound and smaller range for nodes closer to goal
//for multi-agent version, iwd SANodes are reusable
//if an iterative search ends with no valid path found, don't update any heuristics in the next iteration
//if prev iteration has multiple optimal paths, use all of them? NAH, prioritize speed
Array Pathfinder::pathfind_sa_iwdmda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //early exit to skip SANode construction
    if (start == end) {
        return Array();
    }
    int manhattan_dist_to_end = manhattan_dist(start, end);
    int manhattan_radius = 0;
    Vector2i lv_end = end - min;
    //skip the radius=0 search bc it is trivial? no, there could be walls
    shared_ptr<SANode> diamond = make_shared<SANode>();
    diamond->set_lv_pos(start - min);
    diamond->init_lv_back_ids(min, max);
    diamond->set_lv_sid(diamond->lv_pos, get_stuff_id(start));
    diamond->set_lv_sid(lv_end, get_stuff_id(end));

    while (manhattan_radius < manhattan_dist_to_end) {
        
    }
}

void Pathfinder::set_player_pos(Vector2i pos) {
    player_pos = pos;
}

void Pathfinder::set_player_last_dir(Vector2i dir) {
    player_last_dir = dir;
}

void Pathfinder::set_tilemap(TileMap* t) {
    cells = t;
}

void Pathfinder::set_tile_push_limits(Dictionary tpls) {
    Array keys = tpls.keys();
    for (int i=0; i < keys.size(); ++i) {
        int key = keys[i];
        tile_push_limits[key] = tpls[key];
    }
}

void Pathfinder::generate_hash_keys() {
    static bool generated = false;
    if (generated) {
        return;
    }

	std::mt19937_64 generator(HASH_KEY_GENERATOR_SEED); //fixed seed is okay
	std::uniform_int_distribution<uint64_t> distribution(std::numeric_limits<uint64_t>::min(), std::numeric_limits<uint64_t>::max());

    for (int y=0; y < MAX_SEARCH_HEIGHT; ++y) {
        for (int x=0; x < MAX_SEARCH_WIDTH; ++x) {
            //tile_id
            for (uint8_t tile_id=1; tile_id < TILE_ID_COUNT; ++tile_id) {
                tile_id_hash_keys[y][x][tile_id-1] = distribution(generator);
            }

            //type_id
            for (uint8_t type_id=0; type_id < TypeId::REGULAR; ++type_id) {
                type_id_hash_keys[y][x][type_id] = distribution(generator);
            }

            //agent_pos
            agent_pos_hash_keys[y][x] = distribution(generator);
        }
    }

    generated = true;
}

bool Pathfinder::is_immediately_trapped(Vector2i pos) {
    return false;
}

//closed not optimal bc path-informed heuristic not consistent
//return nullptr if no path exists
//assume open and best_dists contain first SASearchNode
shared_ptr<SAPISearchNode> Pathfinder::path_informed_mda(int max_depth, bool allow_type_change, Vector2i lv_end, open_sapi_t& open, closed_sapi_t& best_dists, unique_ptr<PathInfo>& pi, int h_reduction) {
    while (!open.empty()) {
        shared_ptr<SAPISearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            return curr;
        }
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        if (curr->g == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            if (!curr->sanode->get_dist_to_lv_edge(dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SASearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change);

                if (!neighbor) {
                    continue;
                }
                neighbor->g = curr->g + 1;
                neighbor->h = manhattan_dist(neighbor->sanode->lv_pos, lv_end);
                neighbor->f = neighbor->g + neighbor->h;

                //update largest_affected_path_index
                //does update order matter?
                    //ensure no upcoming nodes have been affected
                    //fix prev_path pushing a tile along, causing largest_affected_path_index to increase
                        //if prev action is part of prev_path, ignore prev_push_count when updating largest_affected_path_index
                Vector2i affected_lv_pos = curr->sanode->lv_pos;
                for (int push_count = 0; push_count <= neighbor->prev_push_count; ++push_count) {
                    affected_lv_pos += dir;
                    auto indices_itr = pi->lp_to_path_indices.find(affected_lv_pos);
                    if (indices_itr != pi->lp_to_path_indices.end()) {
                        std::set<unsigned int>& prev_path_indices_at_lv_pos = (*indices_itr).second;
                        //auto index_itr = (*indices_itr).lower_bound(affected_lv_pos);
                    }
                }

                //apply h_reduction
                if (neighbor->is_on_prev_path(pi, largest_affected_path_index)) {
                    neighbor->h -= H_REDUCTION_BASE;
                }
            }
        }
    }
}

shared_ptr<SAPISearchNode> Pathfinder::path_informed_hbjpmda(int max_depth, bool allow_type_change, Vector2i lv_end, open_sapi_t& open, closed_sapi_t& best_dists, unique_ptr<PathInfo>& pi, int h_reduction) {

}

//assume node qualifies for h_reduction
int Pathfinder::get_h_reduction(int radius) {

}

void Pathfinder::rrd_clear_iad() {
    inconsistent_abstract_dists.clear();
}

void Pathfinder::rrd_clear_cad() {
    consistent_abstract_dists.clear();
}


//consider using get_*_bits() and addition instead
uint16_t get_stuff_id(uint8_t back_id, uint8_t type_id, uint8_t tile_id) {
    return (back_id << TILE_AND_TYPE_ID_BITLEN) + (type_id << TILE_ID_BITLEN) + tile_id;
}

uint16_t get_type_bits(uint16_t stuff_id) {
    return stuff_id & TYPE_ID_MASK;
}

uint16_t get_back_bits(uint16_t stuff_id) {
    return stuff_id & BACK_ID_MASK;
}

uint8_t get_tile_id(uint16_t stuff_id) {
    return stuff_id & TILE_ID_MASK;
}

uint8_t get_type_id(uint16_t stuff_id) {
    return get_type_bits(stuff_id) >> TILE_ID_BITLEN;
}

uint8_t get_back_id(uint16_t stuff_id) {
    return get_back_bits(stuff_id) >> TILE_AND_TYPE_ID_BITLEN;
}

uint16_t get_stuff_id(Vector2i pos) {
    Vector2i tile_atlas_coords = cells->get_cell_atlas_coords(LayerId::TILE, pos);
    return (get_back_id(pos) << TILE_AND_TYPE_ID_BITLEN) + (max(tile_atlas_coords.y, 0) << TILE_ID_BITLEN) + (tile_atlas_coords.x + 1);
}

uint8_t get_tile_id(Vector2i pos) {
    //TileId::EMPTY is represented by atlas(-1, -1)
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).x + 1;
}

uint8_t get_type_id(Vector2i pos) {
    //TileId::EMPTY is represented by atlas(-1, -1)
    return max(cells->get_cell_atlas_coords(LayerId::TILE, pos).y, 0);
}

//assume atlas isn't (-1, -1), aka cell has been generated
uint8_t get_back_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::BACK, pos).x;
}

//resulting tile_id is EMPTY
uint16_t remove_tile_id(uint16_t stuff_id) {
    return stuff_id & BACK_AND_TYPE_ID_MASK;
}

//resulting type_id is REGULAR
uint16_t regular_type_id(uint16_t stuff_id) {
    return stuff_id & BACK_AND_TILE_ID_MASK + REGULAR_TYPE_BITS;
}

//resulting back_id is EMPTY
uint16_t remove_back_id(uint16_t stuff_id) {
    return stuff_id & TYPE_AND_TILE_ID_MASK;
}

bool is_compatible(uint8_t type_id, uint8_t back_id) {
    if (back_id == BackId::NONE) {
        return true;
    }
    if (B_WALL_OR_BORDER.find(back_id) != B_WALL_OR_BORDER.end()) {
        return false;
    }
    if (back_id == BackId::MEMBRANE) {
        return type_id == TypeId::PLAYER;
    }
    //back_id in B_SAVE_OR_GOAL
    return type_id == TypeId::PLAYER || type_id == TypeId::REGULAR;
}

bool is_ids_mergeable(uint8_t tile_id1, uint8_t tile_id2) {
    if (is_tile_unsigned(tile_id1) || is_tile_unsigned(tile_id2)) {
        return true;
    }
    int pow1 = get_signed_tile_pow(tile_id1);
    return pow1 == get_signed_tile_pow(tile_id2) && (pow1 < TILE_POW_MAX || get_tile_sign(tile_id1) != get_tile_sign(tile_id2));
}

bool is_ids_split_mergeable(uint8_t src_tile_id, uint8_t dest_tile_id) {
    return is_pow_splittable(get_tile_pow(src_tile_id)) && is_ids_mergeable(get_splitted_tid(src_tile_id), dest_tile_id);
}

//merges involving TileId::ZERO/EMPTY do not count
bool is_same_sign_merge(uint8_t tile_id1, uint8_t tile_id2) {
    return !is_tile_unsigned(tile_id1) && tile_id1 == tile_id2 && get_signed_tile_pow(tile_id1) < TILE_POW_MAX;
}

bool is_tile_unsigned(uint8_t tile_id) {
    return tile_id == TileId::EMPTY || tile_id == TileId::ZERO;
}

bool is_tile_unsigned_and_regular(uint16_t stuff_id) {
    return is_tile_unsigned(get_tile_id(stuff_id)) && get_type_id(stuff_id) == TypeId::REGULAR;
}

bool is_tile_empty_and_regular(uint16_t stuff_id) {
    return get_tile_id(stuff_id) == TileId::EMPTY && get_type_id(stuff_id) == TypeId::REGULAR;
}

bool is_pow_signs_mergeable(Vector2i ps1, Vector2i ps2) {
    if (ps1.x == -1 || ps2.x == -1) {
        return true;
    }
    return ps1.x == ps2.x && (ps1.x < TILE_POW_MAX || ps1.y != ps2.y);
}

bool is_pow_splittable(int pow) {
    return pow > 0;
}

//assume ids mergeable
//eff merge means merge is undoable with split in opposite dir => same sign, nonzero, dest is regular
bool is_eff_merge(uint16_t src_stuff_id, uint16_t dest_stuff_id) {
    uint8_t src_tile_id = get_tile_id(src_stuff_id);
    return get_type_id(dest_stuff_id) == TypeId::REGULAR && src_tile_id == get_tile_id(dest_stuff_id) && !is_tile_unsigned(src_tile_id);
}

//assume type_ids valid
bool is_type_preserved(uint8_t src_type_id, uint8_t dest_type_id) {
    return MERGE_PRIORITIES.at(src_type_id) >= MERGE_PRIORITIES.at(dest_type_id);
}

//assume type_ids valid
//equivalent to !is_type_preserved(dest_type_id, src_type_id)
bool is_type_dominant(uint8_t src_type_id, uint8_t dest_type_id) {
    return MERGE_PRIORITIES.at(src_type_id) > MERGE_PRIORITIES.at(dest_type_id);
}

//treats TileId::EMPTY as TileId::ZERO
Vector2i tid_to_ps(uint8_t tile_id) {
    if (is_tile_unsigned(tile_id)) {
        return Vector2i(-1, 1);
    }
    int signed_incremented_pow = tile_id - TileId::ZERO;
    return Vector2i(abs(signed_incremented_pow) - 1, sgn(signed_incremented_pow));
}

uint8_t ps_to_tid(Vector2i ps) {
    return (ps.x + 1) * ps.y + TileId::ZERO;
}

//treats TileId::EMPTY as TileId::ZERO
int get_tile_pow(uint8_t tile_id) {
    if (tile_id == TileId::EMPTY) {
        return -1;
    }
    return get_signed_tile_pow(tile_id);
}

//assume tile_id is signed
int get_signed_tile_pow(uint8_t tile_id) {
    return abs(tile_id - TileId::ZERO) - 1;
}

//returns one of {+1, -1}; for EMPTY and ZERO, either is fine
//slightly faster than get_true_tile_sign()
int get_tile_sign(uint8_t tile_id) {
    if (tile_id == TileId::ZERO) {
        return 1;
    }
    return sgn(tile_id - TileId::ZERO);
}

//return 0 for EMPTY and ZERO
int get_true_tile_sign(uint8_t tile_id) {
    if (is_tile_unsigned(tile_id)) {
        return 0;
    }
    return sgn(tile_id - TileId::ZERO);
}

//assume one of action.x, action.y is 0
int get_action_dist(Vector3i action) {
    return abs(action.x + action.y);
}

//assume tile_id isn't EMPTY
uint8_t get_opposite_tile_id(uint8_t tile_id) {
    //assert(tile_id != TileId::EMPTY);
    return TILE_ID_COUNT - tile_id;
}

//assumes split possible
Vector2i get_splitted_ps(Vector2i ps) {
    return Vector2i(ps.x - 1, ps.y);
}

//assumes split possible
uint8_t get_splitted_tid(uint8_t tile_id) {
    return tile_id - get_tile_sign(tile_id);
}

//assumes merge possible
//implements hostile death upon becoming 0
//return TileId::EMPTY in place of ZERO to reduce branching
uint16_t get_merged_stuff_id(uint16_t src_stuff_id, uint16_t dest_stuff_id) {
    uint16_t back_bits = get_back_bits(dest_stuff_id);
    uint8_t src_type_id = get_type_id(src_stuff_id);
    uint8_t dest_type_id = get_type_id(dest_stuff_id);
    uint8_t type_id = is_type_preserved(src_type_id, dest_type_id) ? src_type_id : dest_type_id;
    uint8_t tile_id = get_merged_tile_id(get_tile_id(src_stuff_id), get_tile_id(dest_stuff_id));

    //hostile death
    if (type_id == TypeId::HOSTILE && is_tile_unsigned(tile_id)) {
        type_id = TypeId::REGULAR;
    }

    return back_bits + (type_id << TILE_ID_BITLEN) + tile_id;
}

//get_merged_stuff_id but assuming dest is empty_and_regular
uint16_t get_jumped_stuff_id(uint16_t src_stuff_id, uint16_t dest_stuff_id) {
    return get_back_bits(dest_stuff_id) + remove_back_id(src_stuff_id);
}

//assumes merge possible
//return TileId::EMPTY in place of ZERO to reduce branching
uint8_t get_merged_tile_id(uint8_t tile_id1, uint8_t tile_id2) {
    if (is_tile_unsigned(tile_id1)) {
        return (!TRACK_ZEROS && tile_id2 == TileId::ZERO) ? TileId::EMPTY : tile_id2;
    }
    if (is_tile_unsigned(tile_id2)) {
        return (!TRACK_ZEROS && tile_id1 == TileId::ZERO) ? TileId::EMPTY : tile_id1;
    }

    int sgn1 = get_tile_sign(tile_id1);
    if (sgn1 != get_tile_sign(tile_id2)) {
        //opposite sign merge
        return TRACK_ZEROS ? TileId::ZERO : TileId::EMPTY;
    }
    //same sign
    return tile_id1 + sgn1;
}

//assumes merge possible
Vector2i get_merged_pow_sign(Vector2i ps1, Vector2i ps2) {
    if (ps1.x == -1) {
        return ps2;
    }
    if (ps2.x == -1) {
        return ps1;
    }
    if (ps1.y == ps2.y) {
        //same pow same sign
        return Vector2i(ps1.x + 1, ps1.y);
    }
    //same pow opposite sign
    return Vector2i(-1, 1);
}


//assume agent is compatible with goal_pos
void rrd_init_iad(Vector2i goal_pos) {
    if (inconsistent_abstract_dists.find(goal_pos) != inconsistent_abstract_dists.end()) {
        //goal is already initialized
        return;
    }
    inconsistent_abstract_dists.emplace(make_pair(goal_pos, RRDIADLists{}));
    inconsistent_abstract_dists[goal_pos].open.emplace(goal_pos, 0);
    inconsistent_abstract_dists[goal_pos].best_gs.emplace(goal_pos, 0);
}

//returns the inconsistent abstract distance from node to goal_pos
//returns numeric_limits<int>::max() if agent incompatible with node_pos or node_pos unreachable from goal
//assume agent is compatible with goal_pos and node_pos
//open not necessarily optimal bc edges not unit length
//closed is optimal bc closed
//inconsistent_abstract_dists becomes invalid if tilemap changes
//MAX_DEPTH IS DEPRECATED
//assume closed nodes don't exceed max_depth
//if pathfind func doesn't generate from nodes at max_depth, using max_depth = 2 * max_search_depth guarantees no wrong heuristics (false exits), regardless of heuristic informativeness
int rrd_resume_iad(Vector2i goal_pos, Vector2i node_pos, int agent_type_id) {
    //assert(inconsistent_abstract_dists.find(goal_pos) != inconsistent_abstract_dists.end());

    //check for stored ans; this requires closed bc best_dists not necessarily optimal
    auto& [open, closed, best_gs] = inconsistent_abstract_dists[goal_pos];
    auto it = closed.find(node_pos);
    if (it != closed.end()) {
        //this requires closed to store best_dist
        //or else an extra lookup in best_gs is required to get ans
        return (*it).second;
    }

    while (!open.empty()) {
        IADNode n = open.top();

        //no duplicate generation occurs, see sa_dijkstra comment for justification
        if (n.g != best_gs[n.pos]) {
            open.pop();
            continue;
        }
        closed[n.pos] = n.g;

        if (n.pos == node_pos) {
            //found optimal iad at node
            //add to closed before returning
            //no need to backtrack so parents aren't stored
            //don't pop n so search is resumable
            return n.g;
        }
        open.pop();

        /*
        //don't generate nodes exceeding max_depth
        //check here bc depth increases in unit steps
        if (n.depth == max_depth) {
            continue;
        }*/

        uint8_t curr_tile_id = get_tile_id(n.pos);
        for (Vector2i dir : DIRECTIONS) {
            Vector2i next_pos = n.pos + dir;

            if (!is_compatible(agent_type_id, get_back_id(next_pos))) {
                continue;
            }
            int next_g = n.g + get_action_iad(curr_tile_id, get_tile_id(next_pos));

            auto it = best_gs.find(next_pos);
            if (it != best_gs.end() && (*it).second <= next_g) {
                continue;
            }
            open.emplace(next_pos, next_g);
            best_gs[next_pos] = next_g;
        }
    }
    return numeric_limits<int>::max(); //unreachable
}

int get_action_iad(uint8_t src_tile_id, uint8_t dest_tile_id) {
    if (is_tile_unsigned(dest_tile_id)) {
        return 1;
    }
    if (is_tile_unsigned(src_tile_id)) {
        return 1 + get_tile_id_sep(TileId::ZERO, dest_tile_id);
    }
    int dist_to_zero = get_tile_id_sep(src_tile_id, TileId::ZERO);
    int dist_to_opposite = get_tile_id_sep(src_tile_id, get_opposite_tile_id(dest_tile_id));
    int min_dist_to_zero_or_opposite = min(dist_to_zero, dist_to_opposite);
    if (get_signed_tile_pow(dest_tile_id) == TILE_POW_MAX) {
        return 1 + min_dist_to_zero_or_opposite;
    }
    int dist_to_same = get_tile_id_sep(src_tile_id, dest_tile_id);
    return 1 + min(dist_to_same, min_dist_to_zero_or_opposite);
}

//assume neither is zero or empty
int get_tile_id_sep(uint8_t tile_id1, uint8_t tile_id2) {
    int ans = abs(tile_id1 - tile_id2) * IAD_SEP_FACTOR;
    int sgn_change_penalty = int(get_true_tile_sign(tile_id1) * get_true_tile_sign(tile_id2) == -1) * IAD_SIGN_CHANGE_PENALTY;
    return ans + sgn_change_penalty;
}


//DEPRECATED, see Pictures/greedy_is_not_optimal_when_parsing_sequence
void rrd_init_cad(Vector2i goal_pos) {
    if (consistent_abstract_dists.find(goal_pos) != consistent_abstract_dists.end()) {
        return;
    }
    consistent_abstract_dists.emplace(make_pair(goal_pos, RRDCADLists{}));
    consistent_abstract_dists[goal_pos].open.emplace(make_shared<CADNode>(goal_pos, 0, nullptr, Vector2i(0, 0)));
    consistent_abstract_dists[goal_pos].best_gs.emplace(goal_pos, 0);
}

//assume agent is compatible with goal_pos
//similar to iterative deepening iterative deepening (both are O(r^3))
//don't use rra bc agent_pos changes
//consistent_abstract_dists becomes invalid if tilemap or tpl[agent type] changes
//MAX_DEPTH IS DEPRECATED
//closed optimal bc dijkstra
int rrd_resume_cad(Vector2i goal_pos, Vector2i agent_pos) {
    //assert(consistent_abstract_dists.find(goal_pos) != consistent_abstract_dists.end());

    //check for stored ans
    auto& [open, closed, best_gs] = consistent_abstract_dists[goal_pos];
    shared_ptr<CADNode> dest = make_shared<CADNode>(agent_pos, 0, nullptr, Vector2i(0, 0));
    auto it = closed.find(dest);
    if (it != closed.end()) {
        return (*it)->g;
    }
    uint8_t agent_type_id = get_type_id(agent_pos);

    while (!open.empty()) {
        shared_ptr<CADNode> curr = open.top();

        if (curr->g != best_gs[curr->pos]) {
            open.pop();
            continue;
        }
        closed.insert(curr);

        if (curr->pos == agent_pos) {
            //add to closed before returning
            //don't pop curr so search is resumable
            return curr->g;
        }
        open.pop();

        /*
        //don't generate nodes exceeding max_depth
        if (curr->depth == max_depth) {
            continue;
        }*/

        for (Vector2i dir : DIRECTIONS) {
            Vector2i next_pos = curr->pos + dir;

            if (!is_compatible(agent_type_id, get_back_id(next_pos))) {
                continue;
            }
            shared_ptr<CADNode> neighbor = make_shared<CADNode>(next_pos, 0, curr, dir);
            neighbor->g = trace_cad(neighbor);

            auto it = best_gs.find(next_pos);
            if (it != best_gs.end() && (*it).second <= neighbor->g) {
                continue;
            }
            open.push(neighbor);
            best_gs[next_pos] = neighbor->g;
        }
    }
    return numeric_limits<int>::max();
}

//if both slide/split push_count == -1, h += 2
//assume no incompatibility along path
//assume n not null
int trace_cad(shared_ptr<CADNode> n) {
    if (n->prev == nullptr) {
        return 0;
    }
    unordered_set<Vector2i, Vector2iHasher> wildcards;
    vector<uint8_t> sequence;
    uint8_t agent_tile_id = get_tile_id(n->pos);
    Vector2i last_fwd_dir = -n->prev_dir;
    n = n->prev;
    
    while (n != nullptr) {
        sequence.push_back(get_tile_id(n->pos));
        if (is_perp(last_fwd_dir, n->prev_dir)) {

        }
        else {

        }
        n = n->prev;
    }
}

bool is_perp(Vector2i first, Vector2i second) {
    return first.x * second.x + first.y * second.y == 0;
}

int get_cad_push_count() {

}

int manhattan_dist(Vector2i pos1, Vector2i pos2) {
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}