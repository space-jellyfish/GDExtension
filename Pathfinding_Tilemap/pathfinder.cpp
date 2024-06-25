#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <cassert>
#include <memory>
#include <random>
#include <string>
#include "pathfinder.h"

using namespace std;
using namespace godot;


array<array<array<size_t, TILE_ID_COUNT - 1>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> tile_id_hash_keys;
array<array<array<size_t, TypeId::REGULAR>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> type_id_hash_keys;
array<array<size_t, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> agent_pos_hash_keys;
TileMap* cells;
int tile_push_limit;
unordered_map<Vector2i, pair<pq, um>, Vector2iHasher> abstract_dists;

void Pathfinder::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_player_pos", "pos"), &Pathfinder::set_player_pos);
    ClassDB::bind_method(D_METHOD("set_player_last_dir", "dir"), &Pathfinder::set_player_last_dir);
	ClassDB::bind_method(D_METHOD("set_tilemap", "t"), &Pathfinder::set_tilemap);
    ClassDB::bind_method(D_METHOD("set_tile_push_limit", "tpl"), &Pathfinder::set_tile_push_limit);
    ClassDB::bind_method(D_METHOD("generate_hash_keys"), &Pathfinder::generate_hash_keys);
    ClassDB::bind_method(D_METHOD("pathfind_sa", "search_id", "max_depth", "min", "max", "start", "end"), &Pathfinder::pathfind_sa);
}

Array SANode::trace_path(int path_len) {
	Array ans;
	ans.resize(path_len);
	int index = path_len - 1;
	shared_ptr<SANode> curr = shared_from_this();

	while (curr->prev != nullptr) {
        for (int action_index = curr->prev_actions.size() - 1; action_index >= 0; --action_index) {
            ans[index] = curr->prev_actions[action_index];
            --index;
        }
		curr = curr->prev;
	}
	UtilityFunctions::print("PF TRACED PATH SIZE: ", ans.size());
	return ans;
}

void SANode::print_lv() {
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
            uint16_t stuff_id = get_stuff_id(pos);
            uint8_t tile_id = get_tile_id(stuff_id);
            uint8_t type_id = get_type_id(stuff_id);

            //to reduce branching factor
            if (!TRACK_ZEROS && tile_id == TileId::ZERO) {
                stuff_id = remove_tile_id(stuff_id);
            }
            if (type_id != TypeId::REGULAR && !TRACK_KILLABLE_TYPES && MERGE_PRIORITIES.at(type_id) <= agent_merge_priority) {
                stuff_id = regular_type_id(stuff_id);
            }
            row.push_back(stuff_id);

            if (tile_id > TileId::EMPTY) {
                hash ^= tile_id_hash_keys[y][x][tile_id-1];
            }
            if (type_id < TypeId::REGULAR) {
                hash ^= type_id_hash_keys[y][x][type_id];
            }
        }
        lv.push_back(row);
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

uint16_t SANode::get_lv_sid(Vector2i pos) {
    return lv[pos.y][pos.x];
}

//lv_edge is furthest valid cell in dir
//return value is negative if lv_pos is out of bounds in dir
int SANode::get_dist_to_lv_edge(Vector2i dir) {
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
int SANode::get_slide_push_count(Vector2i dir, bool allow_type_change) {
    Vector2i curr_lv_pos = lv_pos;
    uint16_t curr_stuff_id = get_lv_sid(curr_lv_pos);
    uint8_t curr_tile_id = get_tile_id(curr_stuff_id);
    uint8_t curr_type_id = get_type_id(curr_stuff_id);
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

//assume slide possible
//updates lv_pos, lv, prev_eff_pushed, prev_eff_merged, hash
void SANode::perform_slide(Vector2i dir, int push_count) {
    Vector2i merge_lv_pos = lv_pos + (push_count + 1) * dir;

    //skip slide if -dir and prev slide merged with empty or prev split merged with empty
    //skip split if -dir and prev slide merged with same sign regular or prev split merged with same sign regular
    uint16_t neighbor_sid = get_lv_sid(lv_pos + dir);
    uint8_t src_tile_id = get_tile_id(get_lv_sid(lv_pos));
    uint8_t neighbor_tile_id = get_tile_id(neighbor_sid);
    uint8_t neighbor_type_id = get_type_id(neighbor_sid);
    if (is_tile_empty_and_regular(neighbor_sid)) {
        neighbors[Vector3i(-dir.x, -dir.y, ActionId::SLIDE)] = {true, weak_ptr<SANode>()};
    }
    if (is_same_sign_merge(src_tile_id, neighbor_tile_id) && neighbor_type_id == TypeId::REGULAR) {
        neighbors[Vector3i(-dir.x, -dir.y, ActionId::SPLIT)] = {true, weak_ptr<SANode>()};
    }

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

//updates prev_push_count
shared_ptr<SANode> SANode::try_slide(Vector2i dir, bool allow_type_change) {
    int push_count = get_slide_push_count(dir, allow_type_change);
    if (push_count != -1) {
        shared_ptr<SANode> m = make_shared<SANode>(*this);
        m->neighbors.clear();
        m->perform_slide(dir, push_count);
        m->prev_push_count = push_count;
        return m;
    }
    return nullptr;
}

//updates prev_push_count
shared_ptr<SANode> SANode::try_split(Vector2i dir, bool allow_type_change) {
    uint16_t src_sid = get_lv_sid(lv_pos);
    Vector2i ps = tid_to_ps(get_tile_id(src_sid));
    if (!is_pow_splittable(ps.x)) {
        return nullptr;
    }

    //halve src tile, try_slide, then (re)set its tile_id
    //splitted is new tile, splitter is old tile
    uint16_t untyped_split_sid = get_back_bits(src_sid) + get_splitted_tid(get_tile_id(src_sid));
    uint16_t splitted_sid = untyped_split_sid + get_type_bits(src_sid);
    set_lv_sid(lv_pos, splitted_sid);
    shared_ptr<SANode> ans = try_slide(dir, allow_type_change);
    if (ans != nullptr) {
        //insert splitter tile
        uint16_t splitter_sid = untyped_split_sid + (TypeId::REGULAR << TILE_ID_BITLEN);
        ans->set_lv_sid(lv_pos, splitter_sid);
    }
    //reset src tile in this
    set_lv_sid(lv_pos, src_sid);

    return ans;
}

//updates prev, prev_actions
//stores results
shared_ptr<SANode> SANode::try_action(Vector3i action, Vector2i lv_end, bool allow_type_change) {
    //check for stored neighbor
    Vector2i dir(action.x, action.y);
    auto it = neighbors.find(action);
    if (it != neighbors.end()) {
        if ((*it).second.first) {
            //pruned or action invalid
            return nullptr;
        }
        shared_ptr<SANode> ans = (*it).second.second.lock();
        if (ans) {
            //not expired
            return ans;
        }
    }

    shared_ptr<SANode> ans;
    switch(action.z) {
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
        if (action.z != ActionId::JUMP) {
            //update prev/prev_actions
            ans->prev = shared_from_this();
            ans->prev_actions = {action};
        }
        //update neighbors
        neighbors[action] = {false, ans};
    }
    else {
        //action invalid
        neighbors[action] = {true, weak_ptr<SANode>()};
    }
    return ans;
}

//get_jump_point() correctly inits prev_push_count to 0
//doesn't change agent type/tile_id
//doesn't split or push/merge any signed_or_regular tiles
//see Devlog/jump_conditions for details
//when expanding a jp, expand split in all dirs, expand slide iff next tile isn't unsigned_and_regular
shared_ptr<SANode> SANode::try_jump(Vector2i dir, Vector2i lv_end, bool allow_type_change) {
    //check bound
    int dist_to_lv_edge = get_dist_to_lv_edge(dir);
    if (!dist_to_lv_edge) {
        return nullptr;
    }
    int curr_dist = 1;
    Vector2i curr_pos = lv_pos + dir;

    //check empty
    if (!is_tile_empty_and_regular(get_lv_sid(curr_pos))) {
        return nullptr;
    }

    //init next_dirs
    uint16_t src_stuff_id = get_lv_sid(lv_pos);
    uint8_t src_type_id = get_type_id(src_stuff_id);
    uint8_t src_tile_id = get_tile_id(src_stuff_id);
    bool horizontal = (H_DIRS.find(dir) != H_DIRS.end());
    vector<tuple<Vector2i, bool, bool>> next_dirs; //next_dir, in_bounds, blocked (unused if next_dir == dir)
    Vector2i perp_dir1 = horizontal ? Vector2i(0, 1) : Vector2i(1, 0);
    Vector2i perp_dir2 = horizontal ? Vector2i(0, -1) : Vector2i(-1, 0);
    for (Vector2i next_dir : {perp_dir1, perp_dir2}) {
        if (!get_dist_to_lv_edge(next_dir)) {
            next_dirs.emplace_back(next_dir, false, false);
            continue;
        }
        uint16_t next_stuff_id = get_lv_sid(lv_pos + next_dir);
        bool blocked = !(is_tile_unsigned_and_regular(next_stuff_id) && is_compatible(src_type_id, get_back_id(next_stuff_id)));
        next_dirs.emplace_back(next_dir, true, blocked);
    }
    next_dirs.emplace_back(dir, curr_dist < dist_to_lv_edge, false);

    while (curr_dist <= dist_to_lv_edge) {
        uint16_t curr_stuff_id = get_lv_sid(curr_pos);

        //check obstruction
        if (!is_tile_empty_and_regular(curr_stuff_id) || !is_compatible(src_type_id, get_back_id(curr_stuff_id))) {
            return nullptr;
        }

        //get current jump point
        shared_ptr<SANode> curr_jp = get_jump_point(dir, curr_pos, curr_dist);

        //check lv_end
        if (curr_pos == lv_end) {
            return curr_jp;
        }

        //check for jump conditions
        for (auto& [next_dir, in_bounds, blocked] : next_dirs) {
            //bound check
            if (!in_bounds) { //once false, in_bounds stays false
                curr_jp->prune_action_ids(next_dir);
                continue;
            }

            //update in_bounds
            if (next_dir == dir) {
                in_bounds = curr_dist + 1 < dist_to_lv_edge;
            }

            //compatibility check
            uint16_t next_stuff_id = get_lv_sid(curr_pos + next_dir);
            bool next_compatible = is_compatible(src_type_id, get_back_id(next_stuff_id));
            bool next_empty_and_regular = is_tile_empty_and_regular(next_stuff_id);
            if (!next_compatible) {
                curr_jp->prune_action_ids(next_dir);
                //update blocked
                blocked = !(next_empty_and_regular && next_compatible);
                continue;
            }

            //next empty check
            if (next_dir == dir && next_empty_and_regular) {
                continue;
            }

            //prune jump if horizontal, perp, and not blocked
            if (horizontal && next_dir != dir && !blocked) {
                curr_jp->neighbors[Vector3i(next_dir.x, next_dir.y, ActionId::JUMP)] = {true, weak_ptr<SANode>()};
            }

            //prune slide if empty, prune jump if not
            if (next_empty_and_regular) {
                curr_jp->neighbors[Vector3i(next_dir.x, next_dir.y, ActionId::SLIDE)] = {true, weak_ptr<SANode>()};
            }
            else {
                curr_jp->neighbors[Vector3i(next_dir.x, next_dir.y, ActionId::JUMP)] = {true, weak_ptr<SANode>()};
            }

            //horizontal perp empty check
            if (horizontal && next_dir != dir && next_empty_and_regular && blocked) {
                return curr_jp;
            }

            for (int action_id=ActionId::SLIDE; action_id != ActionId::END; ++action_id) {
                //store next_action result in curr_jp->neighbors and if valid, return curr_jp
                //!(vertical && next_dir == dir && empty) bc "next empty check"
                if (action_id == ActionId::JUMP && (horizontal || !next_empty_and_regular)) {
                    continue;
                }

                Vector3i next_action = Vector3i(next_dir.x, next_dir.y, action_id);
                shared_ptr<SANode> neighbor = curr_jp->try_action(next_action, lv_end, allow_type_change);
                if (neighbor) {
                    if (!horizontal || next_dir == dir || action_id != ActionId::SLIDE || blocked || neighbor->prev_push_count) {
                        return curr_jp;
                    }
                }
            }

            //update blocked
            blocked = !(next_empty_and_regular && next_compatible);
        }

        curr_pos += dir;
        ++curr_dist;
    }

    return nullptr;
}

//assume jp_pos is within bounds
shared_ptr<SANode> SANode::get_jump_point(Vector2i dir, Vector2i jp_pos, int jump_dist) {
    shared_ptr<SANode> ans = make_shared<SANode>(*this);
    ans->neighbors.clear();
    ans->set_lv_pos(jp_pos);
    ans->set_lv_sid(jp_pos, get_jumped_stuff_id(get_lv_sid(lv_pos), get_lv_sid(jp_pos)));
    ans->clear_lv_sid(lv_pos);

    //prev stuff
    Vector3i action = Vector3i(dir.x, dir.y, ActionId::SLIDE);
    ans->prev_actions = vector<Vector3i>(jump_dist, action);
    ans->prev = shared_from_this();
    ans->prev_push_count = 0;

    //prune stuff
    ans->neighbors[Vector3i(-dir.x, -dir.y, ActionId::SLIDE)] = {true, weak_ptr<SANode>()};
    ans->neighbors[Vector3i(-dir.x, -dir.y, ActionId::JUMP)] = {true, weak_ptr<SANode>()};

    return ans;
}

//prune all actions in dir
void SANode::prune_action_ids(Vector2i dir) {
    for (int action_id=ActionId::SLIDE; action_id != ActionId::END; ++action_id) {
        neighbors[Vector3i(dir.x, dir.y, action_id)] = {true, weak_ptr<SANode>()};
    }
}

Array Pathfinder::pathfind_sa(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    switch (search_id) {
        case SearchId::DIJKSTRA:
            return pathfind_sa_dijkstra(max_depth, min, max, start, end);
        case SearchId::HBJPD:
            return pathfind_sa_hbjpd(max_depth, min, max, start, end);
        case SearchId::MDA:
            return pathfind_sa_mda(max_depth, min, max, start, end);
        default:
            return pathfind_sa_hbjpd(max_depth, min, max, start, end);
    }
}

//assume no type_id change allowed
//open nodes are optimal since edges are unit cost
//closed nodes are optimal bc dijkstra
Array Pathfinder::pathfind_sa_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    priority_queue<shared_ptr<SANode>, vector<shared_ptr<SANode>>, SANodeComparer> open;
    //change to unordered_set<LightweightSANode> for less memory usage but no collision checking,
    //where LightweightSANode only stores hash (for KeyEqual) and prev/prev_actions (for trace_path())
    unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
    //to prevent duplicate nodes with equal or worse cost from being pushed to open
    unordered_map<shared_ptr<SANode>, int, SANodeHashGetter, SANodeEquator> best_dists;
    Vector2i lv_end = end - min;

    shared_ptr<SANode> first = make_shared<SANode>();
    first->init_lv_pos(start - min);
    first->init_lv(min, max, start);
    //first can have prev_actions and prev_push_count uninitialized
    open.push(first);
    best_dists[first] = first->f;

    while (!open.empty()) {
        shared_ptr<SANode> curr = open.top();

        if (curr->f > max_depth) {
            return Array();
        }
        if (curr->lv_pos == lv_end) {
            return curr->trace_path(curr->f);
        }
        //closed check is unnecessary bc open doesn’t receive duplicate nodes
        open.pop();
        assert(closed.find(curr) == closed.end());
        closed.insert(curr);

        for (Vector2i dir : DIRECTIONS) {
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i action(dir.x, dir.y, action_id);
                shared_ptr<SANode> neighbor = curr->try_action(action, lv_end, false); //don't allow type change

                if (!neighbor) {
                    continue;
                }

                //check if neighbor closed
                //this is necessary since from
                //0, 0, 0
                //P1, 1, 1
                //both {(1, 0, 0), (1, 0, 1)} and {(0, 1, 0), (1, 0, 0), (1, 0, 0), (0, -1, 0)} result in same node
                if (closed.find(neighbor) != closed.end()) {
                    continue;
                }
                assert(neighbor->f == curr->f); //every node expanded at most once, so there is no double-increment
                ++(neighbor->f); //update f/g/h in pathfind funcs bc they could be used differently in each func

                //check if neighbor has been expanded with shorter dist
                auto it = best_dists.find(neighbor);
                if (it != best_dists.end() && (*it).second <= neighbor->f) {
                    continue;
                }
                open.push(neighbor);
                best_dists[neighbor] = neighbor->f;
            }
        }
    }
    return Array();
}

//assume no type_id change
//open nodes not necessarily optimal
Array Pathfinder::pathfind_sa_hbjpd(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    priority_queue<shared_ptr<SANode>, vector<shared_ptr<SANode>>, SANodeComparer> open;
    unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
    unordered_map<shared_ptr<SANode>, int, SANodeHashGetter, SANodeEquator> best_dists;
    Vector2i lv_end = end - min;

    shared_ptr<SANode> first = make_shared<SANode>();
    first->init_lv_pos(start - min);
    first->init_lv(min, max, start);
    open.push(first);
    best_dists[first] = first->f;

    while (!open.empty()) {
        shared_ptr<SANode> curr = open.top();

        if (curr->f > max_depth) {
            return Array();
        }
        if (curr->lv_pos + min == end) {
            return curr->trace_path(curr->f);
        }
        //open may receive duplicate nodes (see Pictures/jpd_edge_case)
        open.pop();
        if (closed.find(curr) != closed.end()) {
            continue;
        }
        closed.insert(curr);

        for (Vector2i dir : DIRECTIONS) {
            for (int action_id=ActionId::SLIDE; action_id != ActionId::END; ++action_id) {
                //expand split in all dirs, expand slide iff next tile isn't unsigned_and_regular
                //expand jump iff next tile is unsigned_and_regular and dir is natural - handled in jump()
                //only search in dir of natural neighbors (except first node) - handled via pruning
                Vector3i action = Vector3i(dir.x, dir.y, action_id);
                shared_ptr<SANode> neighbor = curr->try_action(action, lv_end, false); //don't allow type change

                if (!neighbor) {
                    continue;
                }
                if (closed.find(neighbor) != closed.end()) {
                    continue;
                }
                neighbor->f += neighbor->prev_actions.size();

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end() && (*it).second <= neighbor->f) {
                    continue;
                }
                open.push(neighbor);
                best_dists[neighbor] = neighbor->f;
            }  
        }
    }
    return Array();
}

//open not necessarily optimal
//closed optimal bc heuristic consistent (see SA lec5)
Array Pathfinder::pathfind_sa_mda(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    priority_queue<shared_ptr<SANode>, vector<shared_ptr<SANode>>, SANodeComparer> open;
    unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
    unordered_map<shared_ptr<SANode>, int, SANodeHashGetter, SANodeEquator> best_dists; //hs must be same, so prune if g >= best_g; see also Pictures/best_dists_justification_astar
    Vector2i lv_end = end - min;

    shared_ptr<SANode> first = make_shared<SANode>();
    first->init_lv_pos(start - min);
    first->init_lv(min, max, start);
    first->h = manhattan_dist(first->lv_pos, lv_end);
    first->f = first->h;
    //first can have prev_actions and prev_push_count uninitialized
    open.push(first);
    best_dists[first] = first->g;

    while (!open.empty()) {
        shared_ptr<SANode> curr = open.top();

        if (curr->f > max_depth) { //final path_len is at least curr->f
            return Array();
        }
        if (curr->lv_pos == lv_end) {
            return curr->trace_path(curr->g);
        }
        //open may receive duplicate nodes (see Pictures/mda_closed_check_is_necessary)
        open.pop();
        if (closed.find(curr) != closed.end()) {
            continue;
        }
        closed.insert(curr);

        for (Vector2i dir : DIRECTIONS) {
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i action(dir.x, dir.y, action_id);
                shared_ptr<SANode> neighbor = curr->try_action(action, lv_end, false); //don't allow type change

                if (!neighbor) {
                    continue;
                }

                //check if neighbor closed (see Pictures/mda_closed_check_is_necessary)
                if (closed.find(neighbor) != closed.end()) {
                    continue;
                }
                ++(neighbor->g);

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end() && (*it).second <= neighbor->g) {
                    continue;
                }
                neighbor->h = manhattan_dist(neighbor->lv_pos, lv_end);
                neighbor->f = neighbor->g + neighbor->h;
                open.push(neighbor);
                best_dists[neighbor] = neighbor->g;
            }
        }
    }
    return Array();
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

void Pathfinder::set_tile_push_limit(int tpl) {
    tile_push_limit = tpl;
}

void Pathfinder::generate_hash_keys() {
    static bool generated = false;
    if (generated) {
        return;
    }

	std::mt19937_64 generator(0); //fixed seed is okay
	std::uniform_int_distribution<size_t> distribution(std::numeric_limits<size_t>::min(), std::numeric_limits<size_t>::max());

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

bool Pathfinder::is_tile(Vector2i pos) {
    return get_tile_id(pos) != 0;
}

bool Pathfinder::is_immediately_trapped(Vector2i pos) {
    return false;
}


//consider using get_*_bits() and addition instead
uint16_t get_stuff_id(uint8_t back_id, uint8_t type_id, uint8_t tile_id) {
    return (back_id << (TILE_ID_BITLEN + TYPE_ID_BITLEN)) + (type_id << TILE_ID_BITLEN) + tile_id;
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
    return get_back_bits(stuff_id) >> (TILE_ID_BITLEN + TYPE_ID_BITLEN);
}

uint16_t get_stuff_id(Vector2i pos) {
    return (get_back_id(pos) << 8) + (get_type_id(pos) << 5) + get_tile_id(pos);
}

uint8_t get_tile_id(Vector2i pos) {
    //TileId::EMPTY is represented by atlas(-1, -1)
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).x + 1;
}

uint8_t get_type_id(Vector2i pos) {
    //TileId::EMPTY is represented by atlas(-1, -1)
    return max(cells->get_cell_atlas_coords(LayerId::TILE, pos).y, 0);
}

//assume atlas isn't (-1, -1), cell has been generated
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

//for EMPTY and ZERO, either +-1 is fine
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

//assume tile_id isn't EMPTY
uint8_t get_opposite_tile_id(uint8_t tile_id) {
    assert(tile_id != TileId::EMPTY);
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
    uint16_t tile_bits = get_merged_tile_id(get_tile_id(src_stuff_id), get_tile_id(dest_stuff_id));

    //hostile death
    if (tile_bits == TileId::ZERO && type_id == TypeId::HOSTILE) {
        type_id = TypeId::REGULAR;
    }
    uint16_t type_bits = type_id << TILE_ID_BITLEN;

    return back_bits + type_bits + tile_bits;
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
        return TileId::EMPTY;
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


void rrd_init(uint8_t agent_type_id, Vector2i goal_pos) {
    if (abstract_dists.find(goal_pos) != abstract_dists.end()) {
        return;
    }

    pq open;
    um closed;
    pair<pq, um> lists {open, closed};

    if (is_compatible(agent_type_id, get_back_id(goal_pos))) {
        AbstractNode n(goal_pos, 0);
        open.push(n);
    }

    abstract_dists[goal_pos] = lists;
}

bool rrd_resume(uint8_t agent_type_id, Vector2i goal_pos, Vector2i node_pos) {
    if (!is_compatible(agent_type_id, get_back_id(node_pos)) || !is_compatible(agent_type_id, get_back_id(goal_pos))) {
        return false;
    }

    pair<pq, um>& lists = abstract_dists[goal_pos];
    pq& open = lists.first;
    um& closed = lists.second;

    if (closed.find(node_pos) != closed.end()) {
        return true;
    }

    while (!open.empty()) {
        AbstractNode n = open.top();

        if (closed.find(n.pos) != closed.end()) { //don't re-expand
            assert(n.pos != node_pos);
            continue;
        }
        closed[n.pos] = n.g;

        if (n.pos == node_pos) {
            //found optimal abs_dist at node
            //no need to backtrack so parents aren't stored
            //don't pop n so search is resumable
            //add to closed so get_abs_dist() can access n.g
            return true;
        }
        open.pop();

        uint8_t curr_tile_id = get_tile_id(n.pos);
        for (Vector2i dir : DIRECTIONS) {
            Vector2i next_pos = n.pos + dir;

            if (closed.find(next_pos) != closed.end()) { //closed is all optimal
                continue;
            }
            uint8_t next_tile_id = get_tile_id(next_pos);
            int next_g = n.g + get_move_abs_dist(curr_tile_id, next_tile_id);
            AbstractNode m(next_pos, next_g);
            open.push(m);
        }
    }
    return false; //unreachable
}

int get_move_abs_dist(uint8_t src_tile_id, uint8_t dest_tile_id) {
    if (is_tile_unsigned(dest_tile_id)) {
        return 1;
    }
    if (is_tile_unsigned(src_tile_id)) {
        return get_sep_abs_dist(TileId::ZERO, dest_tile_id);
    }
    if (get_signed_tile_pow(dest_tile_id) == TILE_POW_MAX) {
        return min(get_sep_abs_dist(src_tile_id, TileId::ZERO), get_sep_abs_dist(src_tile_id, get_opposite_tile_id(dest_tile_id)));
    }
    return get_sep_abs_dist(src_tile_id, dest_tile_id);
}

//assume neither is zero or empty
int get_sep_abs_dist(uint8_t tile_id1, uint8_t tile_id2) {
    int ans = abs(tile_id1 - tile_id2) * 2;
    int sgn_change_penalty = int(get_true_tile_sign(tile_id1) * get_true_tile_sign(tile_id2) == -1) * ABSTRACT_DIST_SIGN_CHANGE_PENALTY;
    return ans + sgn_change_penalty;
}

int abstract_dist(uint8_t agent_type_id, Vector2i goal_pos, Vector2i node_pos) {
    assert(abstract_dists.find(goal_pos) != abstract_dists.end());

    pair<pq, um>& lists = abstract_dists[goal_pos];
    um& closed = lists.second;

    auto it = closed.find(node_pos);
    if (it != closed.end()) {
        return (*it).second;
    }

    if (rrd_resume(agent_type_id, goal_pos, node_pos)) {
        return closed[node_pos];
    }
    return numeric_limits<int>::max();
}

int manhattan_dist(Vector2i pos1, Vector2i pos2) {
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}