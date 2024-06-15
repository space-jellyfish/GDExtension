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
		ans[index] = curr->prev_action;
		curr = curr->prev;
		--index;
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

            //reduce branching factor
            if (tile_id == TileId::ZERO) {
                stuff_id -= TileId::ZERO;
            }
            row.push_back(stuff_id);

            
            if (tile_id > 0) {
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
    lv[pos.y][pos.x] = new_sid;
    hash ^= type_id_hash_keys[pos.y][pos.x][get_type_id(old_sid)];
    hash ^= tile_id_hash_keys[pos.y][pos.x][get_tile_id(old_sid)];
    hash ^= type_id_hash_keys[pos.y][pos.x][get_type_id(new_sid)];
    hash ^= tile_id_hash_keys[pos.y][pos.x][get_tile_id(new_sid)];
}

//updates hash
void SANode::set_lv_pos(Vector2i pos) {
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
    lv_pos = pos;
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
}

int SANode::get_lv_sid(Vector2i pos) {
    return lv[pos.y][pos.x];
}

//longest distance from lv_pos in dir without leaving lv
int SANode::get_dist_to_lv_edge(Vector2i dir) {
    if (dir == DIRECTIONS[0]) {
        return lv[0].size() - 1 - lv_pos.x;
    }
    if (dir == DIRECTIONS[1]) {
        return lv.size() - 1 - lv_pos.y;
    }
    if (dir == DIRECTIONS[2]) {
        return lv_pos.x;
    }
    return lv_pos.y;
}

//ignore tiles outside of lv
int SANode::get_slide_push_count(Vector2i dir) {
    Vector2i curr_lv_pos = lv_pos;
    int curr_stuff_id = get_lv_sid(curr_lv_pos);
    int curr_tile_id = get_tile_id(curr_stuff_id);
    int src_type_id = get_type_id(curr_stuff_id);
    int curr_type_id = src_type_id;
    int push_count = 0;
    int nearest_merge_push_count = -1;
    int dist_to_lv_edge = get_dist_to_lv_edge(dir);

    while (push_count <= tile_push_limit) {
        //obstruction
        if (push_count >= dist_to_lv_edge) {
            break;
        }

        int temp_type_id = curr_type_id;
        curr_lv_pos += dir;
        curr_stuff_id = get_lv_sid(curr_lv_pos);
        curr_type_id = get_type_id(curr_stuff_id);
        int curr_back_id = get_back_id(curr_stuff_id);
        if (!is_compatible(temp_type_id, curr_back_id) || (push_count > 0 && T_ENEMY.find(src_type_id) != T_ENEMY.end() && curr_type_id == TypeId::PLAYER)) {
            return nearest_merge_push_count;
        }

        //push/merge logic
        //also bubble logic in case it becomes useful
        int temp_tile_id = curr_tile_id;
        curr_tile_id = get_tile_id(curr_stuff_id);

        if (is_ids_mergeable(temp_tile_id, curr_tile_id)) {
            if (nearest_merge_push_count == -1) {
                nearest_merge_push_count = push_count;
            }
            if (curr_tile_id != TileId::ZERO) {
                if (temp_tile_id == TileId::ZERO && curr_tile_id == TileId::EMPTY) {
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

    //update prev_eff_pushed
    int neighbor_tid = get_tile_id(get_lv_sid(lv_pos + dir));
    prev_eff_pushed = (neighbor_tid == TileId::EMPTY || neighbor_tid == TileId::ZERO) ? false : true;

    for (int dist_to_merge=0; dist_to_merge <= push_count; ++dist_to_merge) {
        Vector2i curr_lv_pos = merge_lv_pos - dist_to_merge * dir;
        int prev_sid = get_lv_sid(curr_lv_pos - dir);
        int result_sid = prev_sid;

        //NEEDS PROFILING
        //update prev_eff_pushed
        //if (dist_to_merge == push_count - 1) {
        //    int neighbor_tid = get_tile_id(prev_sid);
        //    prev_eff_pushed = (neighbor_tid == TileId::EMPTY || neighbor_tid == TileId::ZERO) ? false : true;
        //}

        if (dist_to_merge == 0) {
            int curr_sid = get_lv_sid(curr_lv_pos);
            result_sid = get_merged_stuff_id(prev_sid, curr_sid);

            //update prev_eff_merged
            prev_eff_merged = false;
            if (push_count == 0) {
                int prev_pow = tid_to_ps(get_tile_id(prev_sid)).x;
                int curr_pow = tid_to_ps(get_tile_id(curr_sid)).x;
                if (prev_pow == curr_pow && prev_pow != -1) {
                    prev_eff_merged = true;
                }
            }
        }
        set_lv_sid(curr_lv_pos, result_sid);
    }

    //remove src tile
    set_lv_sid(lv_pos, get_lv_sid(lv_pos) & BACK_ID_MASK);

    //update lv_pos
    set_lv_pos(lv_pos + dir);
}

shared_ptr<SANode> SANode::try_slide(Vector2i dir) {
    int push_count = get_slide_push_count(dir);
    if (push_count != -1) {
        shared_ptr<SANode> m = make_shared<SANode>(*this);
        m->perform_slide(dir, push_count);
        return m;
    }
    return nullptr;
}

shared_ptr<SANode> SANode::try_split(Vector2i dir) {
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
    shared_ptr<SANode> ans = try_slide(dir);
    if (ans != nullptr) {
        //insert splitter tile
        int splitter_sid = untyped_split_sid + (TypeId::REGULAR << TILE_ID_BITLEN);
        ans->set_lv_sid(lv_pos, splitter_sid);
    }
    //reset src tile in this
    set_lv_sid(lv_pos, src_sid);

    return ans;
}

//updates prev, prev_action
shared_ptr<SANode> SANode::try_action(Vector3i action) {
    Vector2i dir(action.x, action.y);
    shared_ptr<SANode> ans;

    switch(action.z) {
        case ActionId::SLIDE:
            ans = try_slide(dir);
            break;
        case ActionId::SPLIT:
            ans = try_split(dir);
            break;
        default:
            ans = try_slide(dir);
    }
    if (ans != nullptr) {
        ans->prev = shared_from_this();
        ans->prev_action = action;
    }
    return ans;
}

Array Pathfinder::pathfind(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    switch (search_id) {
        case SearchId::DIJKSTRA:
            return pathfind_sa_dijkstra(max_depth, min, max, start, end);
        case SearchId::JPD:
            return pathfind_sa_jpd(max_depth, min, max, start, end);
        case SearchId::ASTAR:
            return pathfind_sa_astar(max_depth, min, max, start, end);
        case SearchId::IDASTAR:
            return pathfind_sa_idastar(max_depth, min, max, start, end);
        default:
            return pathfind_sa_idastar(max_depth, min, max, start, end);
    }
}

//assume no type_id change
//open nodes are optimal since edges are unit cost
Array Pathfinder::pathfind_sa_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    priority_queue<shared_ptr<SANode>, vector<shared_ptr<SANode>>, SANodeComparer> open;
    unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
    unordered_map<size_t, int> best_dists;
    int agent_type_id = get_type_id(start);

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

        if (curr->lv_pos + min == end) {
            return curr->trace_path(curr->f);
        }
        open.pop();
        closed.insert(curr);

        for (Vector2i dir : DIRECTIONS) {
            bool dir_is_backtrack = (dir == -Vector2i(curr->prev_action.x, curr->prev_action.y));

            for (int action_id=ActionId::SLIDE; action_id != ActionId::END; ++action_id) {
                //THESE NEED PROFILING
                //if -dir and slide and prev effective push count is 0, skip
                //if -dir and split and prev was effective merge, skip
                if (dir_is_backtrack) {
                    if (action_id == ActionId::SLIDE && !curr->prev_eff_pushed) {
                        continue;
                    }
                    if (action_id == ActionId::SPLIT && curr->prev_eff_merged) {
                        continue;
                    }
                }
                Vector3i action(dir.x, dir.y, action_id);
                shared_ptr<SANode> neighbor = curr->try_action(action);

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

Array Pathfinder::pathfind_sa_jpd(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {

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
        return get_sep_abs_dist(0, dest_tile_id) - 1;
    }
    if (abs(dest_tile_id) == TILE_POW_MAX) {
        return min(get_sep_abs_dist(src_tile_id, 0), get_sep_abs_dist(src_tile_id, -dest_tile_id));
    }
    return get_sep_abs_dist(src_tile_id, dest_tile_id);
}

int Pathfinder::get_sep_abs_dist(int tile_id1, int tile_id2) {
    int ans = abs(tile_id1 - tile_id2) * 2;
    int sgn_change_penalty = abs(sgn(tile_id1) - sgn(tile_id2)) * ABSTRACT_DIST_SIGN_CHANGE_PENALTY_FACTOR;
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
    if (tile_id1 == TileId::EMPTY || tile_id2 == TileId::EMPTY || tile_id1 == TileId::ZERO || tile_id2 == TileId::ZERO) {
        return true;
    }
    Vector2i ps1 = tid_to_ps(tile_id1);
    Vector2i ps2 = tid_to_ps(tile_id2);
    return is_pow_signs_mergeable(ps1, ps2);
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

//treats TileId::EMPTY as TileId::ZERO
Vector2i tid_to_ps(int tile_id) {
    if (tile_id == TileId::ZERO || tile_id == TileId::EMPTY) {
        return Vector2i(-1, 1);
    }
    int signed_incremented_pow = tile_id - TileId::ZERO;
    return Vector2i(abs(signed_incremented_pow) - 1, sgn(signed_incremented_pow));
}

int ps_to_tid(Vector2i ps) {
    return (ps.x + 1) * ps.y + TileId::ZERO;
}

//assumes split possible
Vector2i get_splitted_ps(Vector2i ps) {
    return Vector2i(ps.x - 1, ps.y);
}

//assumes split possible
int get_splitted_tid(int tile_id) {
    return tile_id + sgn(TileId::ZERO - tile_id);
}

//assumes merge possible
//return TileId::EMPTY in place of ZERO to reduce branching
int get_merged_stuff_id(int src_stuff_id, int dest_stuff_id) {
    int back_bits = get_back_bits(dest_stuff_id);
    int src_type_id = get_type_id(src_stuff_id);
    int dest_type_id = get_type_id(dest_stuff_id);
    int type_bits = ((MERGE_PRIORITIES.at(src_type_id) > MERGE_PRIORITIES.at(dest_type_id)) ? src_type_id : dest_type_id) << TILE_ID_BITLEN;
    int tile_bits = get_merged_tile_id(get_tile_id(src_stuff_id), get_tile_id(dest_stuff_id));
    return back_bits + type_bits + tile_bits;
}

//assumes merge possible
//return TileId::EMPTY in place of ZERO to reduce branching
int get_merged_tile_id(int tile_id1, int tile_id2) {
    if (tile_id1 == TileId::EMPTY || tile_id1 == TileId::ZERO) {
        return (tile_id2 == TileId::ZERO) ? TileId::EMPTY : tile_id2;
    }
    if (tile_id2 == TileId::EMPTY || tile_id2 == TileId::ZERO) {
        return (tile_id1 == TileId::ZERO) ? TileId::EMPTY : tile_id1;
    }

    int sgn1 = sgn(tile_id1 - TileId::ZERO);
    if (sgn1 != sgn(tile_id2 - TileId::ZERO)) {
        //opposite sign
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