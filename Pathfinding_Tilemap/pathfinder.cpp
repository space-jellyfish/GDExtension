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

void Pathfinder::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_player_pos", "pos"), &Pathfinder::set_player_pos);
    ClassDB::bind_method(D_METHOD("set_player_last_dir", "dir"), &Pathfinder::set_player_last_dir);
	ClassDB::bind_method(D_METHOD("set_tilemap", "t"), &Pathfinder::set_tilemap);
    ClassDB::bind_method(D_METHOD("set_tile_push_limit", "tpl"), &Pathfinder::set_tile_push_limit);
    ClassDB::bind_method(D_METHOD("generate_hash_keys"), &Pathfinder::generate_hash_keys);
    ClassDB::bind_method(D_METHOD("pathfind", "search_id", "max_depth", "min", "max", "start", "end"), &Pathfinder::pathfind);

    //testing
    ClassDB::bind_method(D_METHOD("rrd_init", "agent_type_id", "goal_pos"), &Pathfinder::rrd_init);
    ClassDB::bind_method(D_METHOD("rrd_resume", "agent_type_id", "goal_pos", "node_pos"), &Pathfinder::rrd_resume);
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
	//UtilityFunctions::print("PF TRACED PATH SIZE: ", ans.size());
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
void SANode::init_lv(Vector2i min, Vector2i max)  {
    int height = max.y - min.y;
    int width = max.x - min.x;
    lv.reserve(height);

    for (int y = min.y; y < max.y; ++y) {
        vector<int> row;
        row.reserve(width);

        for (int x = min.x; x < max.x; ++x) {
            Vector2i pos(x, y);
            int stuff_id = get_stuff_id(pos);
            int tile_id = get_tile_id(stuff_id);

            //to reduce branching factor
            //note cell retains its type
            if (!TRACK_ZEROS && tile_id == TileId::ZERO) {
                stuff_id -= TileId::ZERO;
            }
            row.push_back(stuff_id);

            
            if (tile_id > TileId::EMPTY) {
                hash ^= tile_id_hash_keys[y][x][tile_id-1];
            }
            
            int type_id = get_type_id(stuff_id);
            if (type_id < TypeId::REGULAR) {
                hash ^= type_id_hash_keys[y][x][type_id];
            }
        }
        lv.push_back(row);
    }
}

//updates hash
//assumes back_id unchanged
void SANode::set_lv_sid(Vector2i pos, int new_sid) {
    int old_sid = get_lv_sid(pos);
    int old_tile_id = get_tile_id(old_sid);
    int new_tile_id = get_tile_id(new_sid);
    int old_type_id = get_type_id(old_sid);
    int new_type_id = get_type_id(new_sid);

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
    int stuff_id = get_lv_sid(pos);
    int type_id = get_type_id(stuff_id);
    int tile_id = get_tile_id(stuff_id);
    if (type_id != TypeId::REGULAR) {
        hash ^= type_id_hash_keys[pos.y][pos.x][type_id];
    }
    if (tile_id != TileId::EMPTY) {
        hash ^= tile_id_hash_keys[pos.y][pos.x][tile_id - 1];
    }
    lv[pos.y][pos.x] = get_back_bits(stuff_id);
}

int SANode::get_lv_sid(Vector2i pos) {
    return lv[pos.y][pos.x];
}

//longest distance from lv_pos in dir without leaving lv
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
    int curr_stuff_id = get_lv_sid(curr_lv_pos);
    int curr_tile_id = get_tile_id(curr_stuff_id);
    int curr_type_id = get_type_id(curr_stuff_id);
    bool is_agent_enemy = T_ENEMY.find(curr_type_id) != T_ENEMY.end();
    int push_count = 0;
    int nearest_merge_push_count = -1;
    int dist_to_lv_edge = get_dist_to_lv_edge(dir);

    while (push_count <= min(tile_push_limit, dist_to_lv_edge - 1)) {
        int temp_type_id = curr_type_id;
        curr_lv_pos += dir;
        curr_stuff_id = get_lv_sid(curr_lv_pos);
        curr_type_id = get_type_id(curr_stuff_id);
        int curr_back_id = get_back_id(curr_stuff_id);
        if (!is_compatible(temp_type_id, curr_back_id) || (push_count > 0 && is_agent_enemy && curr_type_id == TypeId::PLAYER)) {
            return nearest_merge_push_count;
        }

        //push/merge logic
        //also bubble logic in case it becomes useful
        int temp_tile_id = curr_tile_id;
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
    int neighbor_sid = get_lv_sid(lv_pos + dir);
    int src_tile_id = get_tile_id(get_lv_sid(lv_pos));
    int neighbor_tile_id = get_tile_id(neighbor_sid);
    int neighbor_type_id = get_type_id(neighbor_sid);
    prev_merged_empty = is_tile_empty_and_regular(neighbor_sid);
    prev_merged_ssr = is_same_sign_merge(src_tile_id, neighbor_tile_id) && neighbor_type_id == TypeId::REGULAR;

    for (int dist_to_merge=0; dist_to_merge <= push_count; ++dist_to_merge) {
        Vector2i curr_lv_pos = merge_lv_pos - dist_to_merge * dir;
        int prev_sid = get_lv_sid(curr_lv_pos - dir);
        int result_sid = prev_sid;

        if (dist_to_merge == 0) {
            int curr_sid = get_lv_sid(curr_lv_pos);
            result_sid = get_merged_stuff_id(prev_sid, curr_sid);
        }
        set_lv_sid(curr_lv_pos, result_sid);
    }

    //remove src tile
    clear_lv_sid(lv_pos);

    //update lv_pos
    set_lv_pos(lv_pos + dir);
}

shared_ptr<SANode> SANode::try_slide(Vector2i dir, bool allow_type_change) {
    int push_count = get_slide_push_count(dir, allow_type_change);
    if (push_count != -1) {
        shared_ptr<SANode> m = make_shared<SANode>(*this);
        m->perform_slide(dir, push_count);
        return m;
    }
    return nullptr;
}

shared_ptr<SANode> SANode::try_split(Vector2i dir, bool allow_type_change) {
    int src_sid = get_lv_sid(lv_pos);
    Vector2i ps = tid_to_ps(get_tile_id(src_sid));
    if (!is_pow_splittable(ps.x)) {
        return nullptr;
    }

    //halve src tile, try_slide, then (re)set its tile_id
    //splitted is new tile, splitter is old tile
    int untyped_split_sid = get_back_bits(src_sid) + get_splitted_tid(get_tile_id(src_sid));
    int splitted_sid = untyped_split_sid + get_type_bits(src_sid);
    set_lv_sid(lv_pos, splitted_sid);
    shared_ptr<SANode> ans = try_slide(dir, allow_type_change);
    if (ans != nullptr) {
        //insert splitter tile
        int splitter_sid = untyped_split_sid + (TypeId::REGULAR << TILE_ID_BITLEN);
        ans->set_lv_sid(lv_pos, splitter_sid);
    }
    //reset src tile in this
    set_lv_sid(lv_pos, src_sid);

    return ans;
}

//updates prev, prev_actions
//doesn't update f/g/h
shared_ptr<SANode> SANode::try_action(Vector3i action, bool allow_type_change) {
    Vector2i dir(action.x, action.y);
    shared_ptr<SANode> ans;

    switch(action.z) {
        case ActionId::SLIDE:
            ans = try_slide(dir, allow_type_change);
            break;
        case ActionId::SPLIT:
            ans = try_split(dir, allow_type_change);
            break;
        default:
            ans = try_slide(dir, allow_type_change);
    }
    if (ans != nullptr) {
        ans->prev = shared_from_this();
        ans->prev_actions = {action};
    }
    return ans;
}

//avoids splitting and eff merging
//doesn't update f/g/h
//doesn't change agent type/tile_id
//add as JP if adj cell empty or if agent can push or merge with adj tile
shared_ptr<SANode> SANode::jump(Vector2i dir, Vector2i lv_end) {
    int src_stuff_id = get_lv_sid(lv_pos);
    int agent_type_id = get_type_id(src_stuff_id);
    int src_tile_id = get_tile_id(src_stuff_id);
    Vector2i curr_pos = lv_pos + dir;
    int dist_to_lv_edge = get_dist_to_lv_edge(dir);
    int dist = 1;
    int zero_count = 0; //number of regular zeros from lv_pos+dir to jp_pos inclusive
    bool horizontal = (H_DIRS.find(dir) != H_DIRS.end());
    Vector2i dir1 = horizontal ? Vector2i(0, 1) : Vector2i(1, 0);
    Vector2i dir2 = horizontal ? Vector2i(0, -1) : Vector2i(-1, 0);
    if (!get_dist_to_lv_edge(dir1)) {
        dir1 = Vector2i(0, 0);
    }
    if (!get_dist_to_lv_edge(dir2)) {
        dir2 = Vector2i(0, 0);
    }

    while (dist <= dist_to_lv_edge) {
        int curr_stuff_id = get_lv_sid(curr_pos);

        if (!is_tile_unsigned_and_regular(curr_stuff_id)) {
            return nullptr;
        }
        int curr_back_id = get_back_id(curr_stuff_id);
        if (!is_compatible(agent_type_id, curr_back_id)) {
            return nullptr;
        }

        //update zero_count
        int curr_tile_id = get_tile_id(curr_stuff_id);
        if (curr_tile_id == TileId::ZERO) {
            ++zero_count;
        }

        //get current jump point for secondary jump checking
        shared_ptr<SANode> curr_jp = get_jump_point(dir, curr_pos, dist, zero_count);
        if (curr_pos == lv_end) {
            return curr_jp;
        }

        //remember to do bound checking
        if (dir1 != Vector2i(0, 0)) {
            Vector2i pos1 = curr_pos + dir1;
            int stuff_id1 = get_lv_sid(pos1);
            if (!horizontal && curr_jp->jump(dir1, lv_end)) {
                return curr_jp;
            }
            else if (is_tile_unsigned_and_regular(stuff_id1)) {
                return curr_jp;
            }
            int tile_id1 = get_tile_id(stuff_id1);
            int type_id1 = get_type_id(stuff_id1);
            if (is_ids_mergeable(src_tile_id, tile_id1) && is_type_preserved(agent_type_id, type_id1)) {
                return curr_jp;
            }
        }
        if (dir2 != Vector2i(0, 0)) {
            Vector2i pos2 = curr_pos + dir2;
            int stuff_id2 = get_lv_sid(pos2);
            if (!horizontal && curr_jp->jump(dir2, lv_end)) {
                return curr_jp;
            }
            else if (is_tile_unsigned_and_regular(stuff_id2)) {
                return curr_jp;
            }
            int tile_id2 = get_tile_id(stuff_id2);
            int type_id2 = get_type_id(stuff_id2);
            if (is_ids_mergeable(src_tile_id, tile_id2) && is_type_preserved(agent_type_id, type_id2)) {
                return curr_jp;
            }
        }
        if (dist < dist_to_lv_edge) {
            Vector2i next_pos = curr_pos + dir;
            int next_stuff_id = get_lv_sid(next_pos);
            int next_tile_id = get_tile_id(next_stuff_id);
            int next_type_id = get_type_id(next_stuff_id);
            if (!is_tile_unsigned_and_regular(next_stuff_id) && is_ids_mergeable(src_tile_id, next_tile_id) && is_type_preserved(agent_type_id, next_type_id)) {
                return curr_jp;
            }
        }

        curr_pos += dir;
        ++dist;
    }
    return nullptr;
}

//assume jp_pos is within bounds
//doesn't update f/g/h
//zero_count is number of zeros (all regular) from lv_pos+dir to jp_pos inclusive
shared_ptr<SANode> SANode::get_jump_point(Vector2i dir, Vector2i jp_pos, int jump_dist, int zero_count) {
    shared_ptr<SANode> ans = make_shared<SANode>(*this);
    ans->set_lv_pos(jp_pos);
    ans->set_lv_sid(jp_pos, get_moved_stuff_id(get_lv_sid(lv_pos), get_lv_sid(jp_pos)));
    //clear src sid and zeros in path of jump
    for (Vector2i pos=lv_pos; pos != jp_pos; pos += dir) {
        ans->clear_lv_sid(pos);
    }
    //add pushed zeros ahead of jp
    int dist_to_lv_edge = ans->get_dist_to_lv_edge(dir);
    int dist = 1;
    Vector2i curr_pos = jp_pos + dir;
    int max_dist = min(dist_to_lv_edge, zero_count, tile_push_limit);
    while (dist <= max_dist) {
        int curr_stuff_id = ans->get_lv_sid(curr_pos);
        int curr_back_id = get_back_id(curr_stuff_id);
        if (!is_compatible(TypeId::REGULAR, curr_back_id)) {
            break;
        }
        int curr_type_id = get_type_id(curr_stuff_id);

        if (curr_type_id == TypeId::REGULAR) {
            int curr_tile_id = get_tile_id(curr_stuff_id);

            if (curr_tile_id == TileId::EMPTY) {
                ans->set_lv_sid(curr_pos, ans->get_lv_sid(curr_pos) + TileId::ZERO);
            }
            else if (curr_tile_id == TileId::ZERO) {
                ++zero_count;
                max_dist = min(dist_to_lv_edge, zero_count, tile_push_limit);
            }
            else {
                break;
            }
        }
        else {
            break;
        }
        ++dist;
        curr_pos += dir;
    }
    Vector3i action = Vector3i(dir.x, dir.y, ActionId::SLIDE);
    ans->prev_actions = vector<Vector3i>(jump_dist, action);
    ans->prev = shared_from_this();
    //skip slide if -dir and prev slide merged with empty or prev split merged with empty
    //skip split if -dir and prev slide merged with same sign regular or prev split merged with same sign regular
    ans->prev_merged_empty = zero_count == 0;
    ans->prev_merged_ssr = false;
    return ans;
}

Array Pathfinder::pathfind(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    switch (search_id) {
        case SearchId::DIJKSTRA:
            return pathfind_sa_dijkstra(max_depth, min, max, start, end);
        case SearchId::HBJPD:
            return pathfind_sa_hbjpd(max_depth, min, max, start, end);
        case SearchId::ASTAR:
            return pathfind_sa_astar(max_depth, min, max, start, end);
        case SearchId::IDASTAR:
            return pathfind_sa_idastar(max_depth, min, max, start, end);
        default:
            return pathfind_sa_idastar(max_depth, min, max, start, end);
    }
}

//assume no type_id change allowed
//open nodes are optimal since edges are unit cost
Array Pathfinder::pathfind_sa_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    priority_queue<shared_ptr<SANode>, vector<shared_ptr<SANode>>, SANodeComparer> open;
    unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
    unordered_map<size_t, int> best_dists;
    Vector2i lv_end = end - min;

    shared_ptr<SANode> first = make_shared<SANode>();
    first->init_lv_pos(start - min);
    first->init_lv(min, max);
    open.push(first);
    best_dists[first->hash] = first->f;

    while (!open.empty()) {
        shared_ptr<SANode> curr = open.top();

        if (curr->f > max_depth) {
            return Array();
        }
        //closed check is unnecessary bc open doesn’t receive duplicate nodes
        assert(closed.find(curr) == closed.end());

        if (curr->lv_pos == lv_end) {
            return curr->trace_path(curr->f);
        }
        open.pop();
        closed.insert(curr);

        for (Vector2i dir : DIRECTIONS) {
            Vector3i last_action = curr->prev_actions.back();
            bool dir_is_backtrack = (dir == -Vector2i(last_action.x, last_action.y));

            for (int action_id=ActionId::SLIDE; action_id != ActionId::END; ++action_id) {
                //THESE NEED PROFILING
                //skip slide if -dir and prev slide merged with empty or prev split merged with empty
	            //skip split if -dir and prev slide merged with same sign regular or prev split merged with same sign regular
                if (dir_is_backtrack) {
                    if (action_id == ActionId::SLIDE && curr->prev_merged_empty) {
                        continue;
                    }
                    if (action_id == ActionId::SPLIT && curr->prev_merged_ssr) {
                        continue;
                    }
                }
                Vector3i action(dir.x, dir.y, action_id);
                shared_ptr<SANode> neighbor = curr->try_action(action, false); //don't allow type change

                //check if action possible
                if (neighbor == nullptr) {
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
                ++(neighbor->f);

                //check if neighbor has been expanded with shorter dist
                auto it = best_dists.find(neighbor->hash);
                if (it != best_dists.end() && (*it).second <= neighbor->f) {
                    continue;
                }
                open.push(neighbor);
                best_dists[neighbor->hash] = neighbor->f;
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
    unordered_map<size_t, int> best_dists;
    Vector2i lv_end = end - min;

    shared_ptr<SANode> first = make_shared<SANode>();
    first->init_lv_pos(start - min);
    first->init_lv(min, max);
    open.push(first);
    best_dists[first->hash] = first->f;

    while (!open.empty()) {
        shared_ptr<SANode> curr = open.top();

        if (curr->f > max_depth) {
            return Array();
        }
        if (curr->lv_pos + min == end) {
            return curr->trace_path(curr->f);
        }
        //open may receive duplicate nodes (see Pictures/jpd_edge_case)
        if (closed.find(curr) != closed.end()) {
            continue;
        }
        open.pop();
        closed.insert(curr);

        for (Vector2i dir : DIRECTIONS) {
            Vector3i last_action = curr->prev_actions.back();
            bool dir_is_backtrack = (dir == -Vector2i(last_action.x, last_action.y));

            for (int action_id=ActionId::SLIDE; action_id != ActionId::END; ++action_id) {
                //skip slide if -dir and prev slide merged with empty or prev split merged with empty
	            //skip split if -dir and prev slide merged with same sign regular or prev split merged with same sign regular
                if (dir_is_backtrack) {
                    if (action_id == ActionId::SLIDE && curr->prev_merged_empty) {
                        continue;
                    }
                    if (action_id == ActionId::SPLIT && curr->prev_merged_ssr) {
                        continue;
                    }
                }
            }  
        }
    }
}

Array Pathfinder::pathfind_sa_astar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //if (dist to player > threshold) {
    //    astar using abstract dist
    //}
    //else {
    //    obstruct one of four player dirs or attempt merge with player
    //}
    Array ans;
    return ans;
}

Array Pathfinder::pathfind_sa_idastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    Array ans;
    return ans;
}

Array Pathfinder::pathfind_sa_rrdastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {

}

Array Pathfinder::pathfind_sa_ididjpastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    int tile_depth = 0;
    int search_depth = 0;

    Array ans;
    return ans;
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
            for (int tile_id=1; tile_id < TILE_ID_COUNT; ++tile_id) {
                tile_id_hash_keys[y][x][tile_id-1] = distribution(generator);
            }

            //type_id
            for (int type_id=0; type_id < TypeId::REGULAR; ++type_id) {
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

void Pathfinder::rrd_init(int agent_type_id, Vector2i goal_pos) {
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

bool Pathfinder::rrd_resume(int agent_type_id, Vector2i goal_pos, Vector2i node_pos) {
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

        int curr_tile_id = get_tile_id(n.pos);
        for (Vector2i dir : DIRECTIONS) {
            Vector2i next_pos = n.pos + dir;

            if (closed.find(next_pos) != closed.end()) { //closed is all optimal
                continue;
            }
            int next_tile_id = get_tile_id(next_pos);
            int next_g = n.g + get_move_abs_dist(curr_tile_id, next_tile_id);
            AbstractNode m(next_pos, next_g);
            open.push(m);
        }
    }
    return false; //unreachable
}

int Pathfinder::get_move_abs_dist(int src_tile_id, int dest_tile_id) {
    if (dest_tile_id % TileId::ZERO == 0) {
        return 1;
    }
    if (src_tile_id % TileId::ZERO == 0) {
        return get_sep_abs_dist(TileId::ZERO, dest_tile_id);
    }
    if (get_tile_pow(dest_tile_id) == TILE_POW_MAX) {
        return min(get_sep_abs_dist(src_tile_id, TileId::ZERO), get_sep_abs_dist(src_tile_id, get_opposite_tile_id(dest_tile_id)));
    }
    return get_sep_abs_dist(src_tile_id, dest_tile_id);
}

//assume neither is zero or empty
int Pathfinder::get_sep_abs_dist(int tile_id1, int tile_id2) {
    int ans = abs(tile_id1 - tile_id2) * 2;
    int sgn_change_penalty = int(get_true_tile_sign(tile_id1) * get_true_tile_sign(tile_id2) == -1) * ABSTRACT_DIST_SIGN_CHANGE_PENALTY;
    return ans + sgn_change_penalty;
}

int Pathfinder::get_abs_dist(int agent_type_id, Vector2i goal_pos, Vector2i node_pos) {
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


//consider using get_*_bits() and addition instead
int get_stuff_id(int back_id, int type_id, int tile_id) {
    return (back_id << (TILE_ID_BITLEN + TYPE_ID_BITLEN)) + (type_id << TILE_ID_BITLEN) + tile_id;
}

int get_type_bits(int stuff_id) {
    return stuff_id & TYPE_ID_MASK;
}

int get_back_bits(int stuff_id) {
    return stuff_id & BACK_ID_MASK;
}

int get_tile_id(int stuff_id) {
    return stuff_id & TILE_ID_MASK;
}

int get_type_id(int stuff_id) {
    return get_type_bits(stuff_id) >> TILE_ID_BITLEN;
}

int get_back_id(int stuff_id) {
    return get_back_bits(stuff_id) >> (TILE_ID_BITLEN + TYPE_ID_BITLEN);
}

int get_stuff_id(Vector2i pos) {
    return (get_back_id(pos) << 8) + (get_type_id(pos) << 5) + get_tile_id(pos);
}

int get_tile_id(Vector2i pos) {
    //TileId::EMPTY is represented by atlas(-1, -1)
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).x + 1;
}

int get_type_id(Vector2i pos) {
    //TileId::EMPTY is represented by atlas(-1, -1)
    return max(cells->get_cell_atlas_coords(LayerId::TILE, pos).y, 0);
}

//assume atlas isn't (-1, -1), cell has been generated
int get_back_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::BACK, pos).x;
}

bool is_compatible(int type_id, int back_id) {
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

bool is_ids_mergeable(int tile_id1, int tile_id2) {
    if (is_tile_unsigned(tile_id1) || is_tile_unsigned(tile_id2)) {
        return true;
    }
    Vector2i ps1 = tid_to_ps(tile_id1);
    Vector2i ps2 = tid_to_ps(tile_id2);
    return is_pow_signs_mergeable(ps1, ps2);
}

//merges involving TileId::ZERO/EMPTY do not count
bool is_same_sign_merge(int tile_id1, int tile_id2) {
    return !is_tile_unsigned(tile_id1) && tile_id1 == tile_id2 && get_tile_pow(tile_id1) < TILE_POW_MAX;
}

bool is_tile_unsigned(int tile_id) {
    return tile_id == TileId::EMPTY || tile_id == TileId::ZERO;
}

bool is_tile_unsigned_and_regular(int stuff_id) {
    return is_tile_unsigned(get_tile_id(stuff_id)) && get_type_id(stuff_id) == TypeId::REGULAR;
}

bool is_tile_empty_and_regular(int stuff_id) {
    return get_tile_id(stuff_id) == TileId::EMPTY && get_type_id(stuff_id) == TypeId::REGULAR;
}

bool is_pow_signs_mergeable(Vector2i ps1, Vector2i ps2) {
    if (ps1.x == -1 || ps2.x == -1) {
        return true;
    }
    if (ps1.x == ps2.x && (ps1.x < TILE_POW_MAX || ps1.y != ps2.y)) {
        return true;
    }
    return false;
}

bool is_pow_splittable(int pow) {
    return pow > 0;
}

//assume ids mergeable
//eff merge means merge is undoable with split in opposite dir => same sign, nonzero, dest is regular
bool is_eff_merge(int src_stuff_id, int dest_stuff_id) {
    int src_tile_id = get_tile_id(src_stuff_id);
    return get_type_id(dest_stuff_id) == TypeId::REGULAR && src_tile_id == get_tile_id(dest_stuff_id) && !is_tile_unsigned(src_tile_id);
}

//assume type_ids valid
bool is_type_preserved(int src_type_id, int dest_type_id) {
    return MERGE_PRIORITIES.at(src_type_id) >= MERGE_PRIORITIES.at(dest_type_id);
}

//assume type_ids valid
//equivalent to !is_type_preserved(dest_type_id, src_type_id), but faster
bool is_type_dominant(int src_type_id, int dest_type_id) {
    return MERGE_PRIORITIES.at(src_type_id) > MERGE_PRIORITIES.at(dest_type_id);
}

//treats TileId::EMPTY as TileId::ZERO
Vector2i tid_to_ps(int tile_id) {
    if (is_tile_unsigned(tile_id)) {
        return Vector2i(-1, 1);
    }
    int signed_incremented_pow = tile_id - TileId::ZERO;
    return Vector2i(abs(signed_incremented_pow) - 1, sgn(signed_incremented_pow));
}

int ps_to_tid(Vector2i ps) {
    return (ps.x + 1) * ps.y + TileId::ZERO;
}

//treats TileId::EMPTY as TileId::ZERO
int get_tile_pow(int tile_id) {
    if (tile_id == TileId::EMPTY) {
        return -1;
    }
    return abs(tile_id - TileId::ZERO) - 1;
}

//for EMPTY and ZERO, either +-1 is fine
int get_tile_sign(int tile_id) {
    if (tile_id == TileId::ZERO) {
        return 1;
    }
    return sgn(tile_id - TileId::ZERO);
}

//return 0 for EMPTY and ZERO
int get_true_tile_sign(int tile_id) {
    if (is_tile_unsigned(tile_id)) {
        return 0;
    }
    return sgn(tile_id - TileId::ZERO);
}

//assume tile_id isn't EMPTY
int get_opposite_tile_id(int tile_id) {
    assert(tile_id != TileId::EMPTY);
    return TILE_ID_COUNT - tile_id;
}

//assumes split possible
Vector2i get_splitted_ps(Vector2i ps) {
    return Vector2i(ps.x - 1, ps.y);
}

//assumes split possible
int get_splitted_tid(int tile_id) {
    return tile_id - get_tile_sign(tile_id);
}

//assumes merge possible
//return TileId::EMPTY in place of ZERO to reduce branching
int get_merged_stuff_id(int src_stuff_id, int dest_stuff_id) {
    int back_bits = get_back_bits(dest_stuff_id);
    int src_type_id = get_type_id(src_stuff_id);
    int dest_type_id = get_type_id(dest_stuff_id);
    int type_bits = (is_type_preserved(src_type_id, dest_type_id) ? src_type_id : dest_type_id) << TILE_ID_BITLEN;
    int tile_bits = get_merged_tile_id(get_tile_id(src_stuff_id), get_tile_id(dest_stuff_id));
    return back_bits + type_bits + tile_bits;
}

//get_merged_stuff_id but assuming dest is regular zero or empty
int get_moved_stuff_id(int src_stuff_id, int dest_stuff_id) {
    int back_bits = get_back_bits(dest_stuff_id);
    int type_bits = get_type_bits(src_stuff_id);
    int tile_bits = get_tile_id(src_stuff_id);
    return back_bits + type_bits + tile_bits;
}

//assumes merge possible
//return TileId::EMPTY in place of ZERO to reduce branching
int get_merged_tile_id(int tile_id1, int tile_id2) {
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