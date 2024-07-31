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

    ClassDB::bind_method(D_METHOD("get_sa_cumulative_search_time", "sa_search_id"), &Pathfinder::get_sa_cumulative_search_time);
    ClassDB::bind_method(D_METHOD("reset_sa_cumulative_search_times"), &Pathfinder::reset_sa_cumulative_search_times);
    ClassDB::bind_method(D_METHOD("pathfind_sa", "search_id", "max_depth", "allow_type_change", "min", "max", "start", "end"), &Pathfinder::pathfind_sa);

    ClassDB::bind_method(D_METHOD("rrd_clear_iad"), &Pathfinder::rrd_clear_iad);
    ClassDB::bind_method(D_METHOD("rrd_clear_cad"), &Pathfinder::rrd_clear_cad);
}

//if jump fails, return EN with same lv_pos
EnclosureNode EnclosureNode::jump(Vector2i dir, Vector2i lv_end, shared_ptr<SANode> env, uint8_t agent_type_id) {
    //check bounds, empty
    int dist_to_lv_edge = env->get_dist_to_lv_edge(lv_pos, dir);
    if (!dist_to_lv_edge) {
        return *this;
    }
    Vector2i curr_pos = lv_pos + dir;
    if (!is_compatible(agent_type_id, get_back_id(env->get_lv_sid(curr_pos)))) {
        return *this;
    }
    int curr_dist = 1;

    //init next_dirs
    array<NextDir, 2> next_dirs;
    bool horizontal = (H_DIRS.find(dir) != H_DIRS.end());
    Vector2i perp_dir1 = horizontal ? Vector2i(0, 1) : Vector2i(1, 0);
    Vector2i perp_dir2 = horizontal ? Vector2i(0, -1) : Vector2i(-1, 0);
    auto next_dirs_itr = next_dirs.begin();
    for (Vector2i next_dir : {perp_dir1, perp_dir2}) {
        if (!env->get_dist_to_lv_edge(curr_pos, next_dir)) {
            *next_dirs_itr = NextDir(next_dir, false, false);
        }
        else {
            uint8_t next_back_id = get_back_id(env->get_lv_sid(lv_pos + next_dir));
            bool blocked = !is_compatible(agent_type_id, next_back_id);
            *next_dirs_itr = NextDir(next_dir, true, blocked);
        }
        ++next_dirs_itr;
    }

    while (curr_dist <= dist_to_lv_edge) {
        
    }
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
    int tile_atlas_y = cells->get_cell_atlas_coords(LayerId::TILE, pos).y;
    if (tile_atlas_y == -1) {
        return TypeId::REGULAR;
    }
    return tile_atlas_y;
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

Vector2i get_normalized_dir(Vector3i action) {
    return Vector2i(sgn(action.x), sgn(action.y));
}

Vector3i get_normalized_action(Vector3i action) {
    if (action.z != ActionId::JUMP) {
        return action;
    }
    return Vector3i(sgn(action.x), sgn(action.y), ActionId::SLIDE);
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


//updates hash
void SANode::init_lv_pos(Vector2i _lv_pos) {
    lv_pos = _lv_pos;
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
}

//updates hash
//assume lv_pos is initialized
//consult agent_type_id if not TRACK_KILLABLE_TYPES
void SANode::init_lv(Vector2i min, Vector2i max)  {
    int height = max.y - min.y;
    int width = max.x - min.x;
    lv.reserve(height);
    Vector2i agent_pos = min + lv_pos;
    uint8_t agent_type_id = get_type_id(agent_pos);
    int agent_merge_priority = MERGE_PRIORITIES.at(agent_type_id);

    for (int y = min.y; y < max.y; ++y) {
        int lv_y = y - min.y;
        vector<uint16_t> row;
        row.reserve(width);

        for (int x = min.x; x < max.x; ++x) {
            int lv_x = x - min.x;
            Vector2i pos(x, y);
            uint8_t tile_id = get_tile_id(pos);
            uint8_t type_id = get_type_id(pos);

            //to reduce branching factor
            if (!TRACK_ZEROS && tile_id == TileId::ZERO) {
                tile_id = TileId::EMPTY;
            }
            if (type_id != TypeId::REGULAR && !TRACK_KILLABLE_TYPES && pos != agent_pos && MERGE_PRIORITIES.at(type_id) <= agent_merge_priority) {
                type_id = TypeId::REGULAR;
            }
            row.push_back(get_stuff_id(get_back_id(pos), type_id, tile_id));

            if (tile_id > TileId::EMPTY) {
                hash ^= tile_id_hash_keys[lv_y][lv_x][tile_id-1];
            }
            if (type_id < TypeId::REGULAR) {
                hash ^= type_id_hash_keys[lv_y][lv_x][type_id];
            }
        }
        lv.push_back(move(row));
    }
}

//init type_id to REGULAR, tile_id to EMPTY (no hash update necessary)
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

//assume type_id REGULAR, tile_id EMPTY (no hash keys have been applied)
void SANode::init_lv_ttid(Vector2i _lv_pos, Vector2i pos) {
    //assert(remove_back_id(get_lv_sid(_lv_pos)) == REGULAR_TYPE_BITS);
    uint8_t type_id = get_type_id(pos);
    uint8_t tile_id = get_tile_id(pos);
    if (tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[_lv_pos.y][_lv_pos.x][tile_id-1];
    }
    if (type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[_lv_pos.y][_lv_pos.x][type_id];
    }
    lv[_lv_pos.y][_lv_pos.x] += (type_id << TILE_ID_BITLEN) - REGULAR_TYPE_BITS + tile_id;
}

//init_lv_ttid() but with check to prevent duplicate hash update
void SANode::init_lv_ttid_idempotent(Vector2i _lv_pos, Vector2i pos) {
    if (remove_back_id(get_lv_sid(_lv_pos)) != REGULAR_TYPE_BITS) {
        return;
    }
    init_lv_ttid(_lv_pos, pos);
}

//updates hash
//assumes back_id unchanged
void SANode::set_lv_sid(Vector2i _lv_pos, uint16_t new_sid) {
    uint16_t old_sid = get_lv_sid(_lv_pos);
    uint8_t old_tile_id = get_tile_id(old_sid);
    uint8_t new_tile_id = get_tile_id(new_sid);
    uint8_t old_type_id = get_type_id(old_sid);
    uint8_t new_type_id = get_type_id(new_sid);

    if (old_type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[_lv_pos.y][_lv_pos.x][old_type_id];
    }
    if (new_type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[_lv_pos.y][_lv_pos.x][new_type_id];
    }
    if (old_tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[_lv_pos.y][_lv_pos.x][old_tile_id - 1];
    }
    if (new_tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[_lv_pos.y][_lv_pos.x][new_tile_id - 1];
    }
    lv[_lv_pos.y][_lv_pos.x] = new_sid;
}

//updates hash
void SANode::set_lv_pos(Vector2i _lv_pos) {
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
    lv_pos = _lv_pos;
    hash ^= agent_pos_hash_keys[lv_pos.y][lv_pos.x];
}

//equivalent to set_lv_sid(_lv_pos, get_back_bits(get_lv_sid(_lv_pos))) but faster
void SANode::clear_lv_sid(Vector2i _lv_pos) {
    uint16_t stuff_id = get_lv_sid(_lv_pos);
    uint8_t type_id = get_type_id(stuff_id);
    uint8_t tile_id = get_tile_id(stuff_id);
    if (type_id != TypeId::REGULAR) {
        hash ^= type_id_hash_keys[_lv_pos.y][_lv_pos.x][type_id];
    }
    if (tile_id != TileId::EMPTY) {
        hash ^= tile_id_hash_keys[_lv_pos.y][_lv_pos.x][tile_id - 1];
    }
    lv[_lv_pos.y][_lv_pos.x] = get_back_bits(stuff_id);
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
            uint16_t stuff_id = lv[y][x];
            uint8_t back_id = get_back_id(stuff_id);
            uint8_t type_id = get_type_id(stuff_id);
            uint8_t tile_id = get_tile_id(stuff_id);
            if (x != lv[0].size() - 1) {
                printf("(%d, %d, %d), ", back_id, type_id, tile_id);
            }
            else {
                printf("(%d, %d, %d)", back_id, type_id, tile_id);
            }
        }
        printf("\n");
    }
}

uint16_t SANode::get_lv_sid(Vector2i _lv_pos) const {
    return lv[_lv_pos.y][_lv_pos.x];
}

//lv_edge is furthest valid cell in dir
//return value is negative if lv_pos is out of bounds in dir
int SANode::get_dist_to_lv_edge(Vector2i _lv_pos, Vector2i dir) const {
    if (dir == Vector2i(1, 0)) {
        return lv[0].size() - 1 - _lv_pos.x;
    }
    if (dir == Vector2i(0, 1)) {
        return lv.size() - 1 - _lv_pos.y;
    }
    if (dir == Vector2i(-1, 0)) {
        return _lv_pos.x;
    }
    return _lv_pos.y;
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
    int dist_to_lv_edge = get_dist_to_lv_edge(lv_pos, dir);

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

//don't make duplicate init_lv_ttid() calls
//assume ttids at new_radius are uninitialized
//assume check_bounds accepts global_pos
//assume new_radius > 0
void SANode::widen_diamond(Vector2i min, Vector2i end, int new_radius, const BoundsChecker& check_bounds) {
    for (int dx = 0; dx <= new_radius; ++dx) {
        int dy = new_radius - dx;

        for (int x_sign : {-1, 1}) {
            for (int y_sign : {-1, 1}) {
                Vector2i offset(dx * x_sign, dy * y_sign);
                Vector2i pos = end + offset;
                if (check_bounds(pos)) {
                    init_lv_ttid(pos - min, pos);
                }

                if (!dy) {
                    break;
                }
            }
            if (!dx) {
                break;
            }
        }
    }
}

//don't make duplicate init_lv_ttid() calls
//assume ttids at new_radius are uninitialized
//assume check_bounds accepts global_pos
//assume new_radius > 0
void SANode::widen_square(Vector2i min, Vector2i end, int new_radius, const BoundsChecker& check_bounds) {
    for (int y_offset : {-new_radius, new_radius}) {
        for (int x_offset = -new_radius; x_offset <= new_radius; ++x_offset) {
            Vector2i offset(x_offset, y_offset);
            Vector2i pos = end + offset;
            if (check_bounds(pos)) {
                init_lv_ttid(pos - min, pos);
            }
        }
    }
    for (int x_offset : {-new_radius, new_radius}) {
        for (int y_offset = -new_radius + 1; y_offset < new_radius; ++y_offset) {
            Vector2i offset(x_offset, y_offset);
            Vector2i pos = end + offset;
            if (check_bounds(pos)) {
                init_lv_ttid(pos - min, pos);
            }
        }
    }
}


//assume prev != nullptr
//don't combine into try_action() since try_action() must work with SASearchNode
//don't modify pi->lp_to_path_indices (so largest_prev_path_indices remains valid)
//pass dir bc it's faster than calling get_normalized_dir()
//does update order matter?
    //ensure no upcoming nodes have been affected (no higher_path_index nodes at different lv_pos)
    //fix prev_path pushing a tile along, causing largest_affected_path_index to increase
        //if prev action is part of prev_path and prev is validated and admissible, ignore prev_push_count when updating largest_affected_path_index
        //prev must be h_reduced (validated_and_admissible), bc if prev_path_action merge, prev_action push, largest_affected_path_index becomes incorrect
void SAPISearchNode::init_lapi(unique_ptr<PathInfo>& pi, Vector2i dir) {
    largest_affected_path_index = prev->largest_affected_path_index;
    Vector2i affected_lv_pos = prev->sanode->lv_pos + dir;
    //update_lapi() helper variables
    std::set<int>* largest_prev_path_indices = nullptr; //prev_path_indices_at_lp with highest *rbegin()
    int largest_affected_lp_path_index = 0;
    int penultimate_affected_lp_path_index = 0;
    update_lapi_helpers(pi, affected_lv_pos, largest_prev_path_indices, largest_affected_lp_path_index, penultimate_affected_lp_path_index);

    //ignore prev pushed if prev h_reduced and prev_action on prev_path
    if (prev->virtual_path_index != -1 && prev_action == pi->normalized_actions[prev->virtual_path_index]) {
        update_lapi(largest_prev_path_indices, largest_affected_path_index);
        return;
    }
    for (int push_count = 1; push_count <= prev_push_count; ++push_count) {
        affected_lv_pos += dir;
        update_lapi_helpers(pi, affected_lv_pos, largest_prev_path_indices, largest_affected_lp_path_index, penultimate_affected_lp_path_index);
    }
    update_lapi(largest_prev_path_indices, max(largest_affected_path_index, penultimate_affected_lp_path_index));
}

void SAPISearchNode::update_lapi(std::set<int>* largest_prev_path_indices, int effective_largest_affected_path_index) {
    if (largest_prev_path_indices == nullptr) {
        return;
    }
    auto index_itr = (*largest_prev_path_indices).lower_bound(effective_largest_affected_path_index);
    if (index_itr != (*largest_prev_path_indices).end()) {
        largest_affected_path_index = *index_itr;
    }
}

void SAPISearchNode::update_lapi_helpers(unique_ptr<PathInfo>& pi, Vector2i affected_lv_pos, std::set<int>*& largest_prev_path_indices, int& largest_affected_lp_path_index, int& penultimate_affected_lp_path_index) {
    auto indices_itr = pi->lp_to_path_indices.find(affected_lv_pos);
    if (indices_itr != pi->lp_to_path_indices.end()) {
        //prev_path visits affected_lv_pos
        std::set<int>& prev_path_indices_at_lp = (*indices_itr).second;
        //assert(!prev_path_indices_at_lp.empty());
        int max_path_index_at_lp = *prev_path_indices_at_lp.rbegin();
        if (max_path_index_at_lp > largest_affected_lp_path_index) {
            penultimate_affected_lp_path_index = largest_affected_lp_path_index;
            largest_affected_lp_path_index = max_path_index_at_lp;
            largest_prev_path_indices = &prev_path_indices_at_lp;
        }
        else if (max_path_index_at_lp > penultimate_affected_lp_path_index) {
            penultimate_affected_lp_path_index = max_path_index_at_lp;
        }
    }
}


//for testing; return search_id's cumulative search time in ms
double Pathfinder::get_sa_cumulative_search_time(int sa_search_id) {
    //assert(search_id >= 0 && sa_search_id < SASearchId::SEARCH_END);
    return sa_cumulative_search_times[sa_search_id];
}

void Pathfinder::reset_sa_cumulative_search_times() {
    fill(sa_cumulative_search_times.begin(), sa_cumulative_search_times.end(), 0);
}

//use an rrd heuristic search for enclosure check
Array Pathfinder::pathfind_sa(int search_id, int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //trivial check
    if (start == end) {
        return Array();
    }

    //type check
    if (!is_compatible(get_type_id(start), get_back_id(end))) {
        return Array();
    }

    //search_size check; DO NOT COMMENT
    assert(max.x - min.x <= MAX_SEARCH_WIDTH && max.y - min.y <= MAX_SEARCH_HEIGHT);

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
    //use ptrs bc prev requires a fixed addr, faster to copy
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
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
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
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
                //generate split in all dirs, don't generate slide if next tile is empty_and_regular
                //generate jump iff next tile is empty_and_regular and dir is natural - handled in try_jump()
                //only search in dir of natural neighbors (except first node) - handled via pruning
                if (action_id == ActionId::SLIDE && is_tile_empty_and_regular(curr->sanode->get_lv_sid(curr->sanode->lv_pos + dir))) {
                    continue;
                }
                Vector3i normalized_action(dir.x, dir.y, action_id);
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
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
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
                        //this is for fun, no code changes are needed
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
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
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
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
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
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
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

//apply proportional h_reduction to nodes within same path? ONE
    //higher h_reduction for nodes with higher virtual_path_index, since they have more validation
//use base h_reduction proportional to pathlen/manhattan_radius_of_shape since larger radius is harder? TWO
    //NEEDS PROFILING (both speed and suboptimality); this makes path near start more suboptimal than path near goal (might be good?)
//for simulated annealing version, choose random h_reduction from an interval? THREE
    //use greater lower bound and smaller range for nodes with higher virtual_path_index

//use a reduced h_reduction if upcoming location in path has been affected?
    //NAH, largest_affected_path_index update in path_informed_mda() accounts for pushing that occurs in prev_path
//for multi-agent version, iwd SANodes are reusable
//if an iterative search ends with no valid path found, don't update any heuristics in the next iteration
//if prev iteration has multiple optimal paths, use all of them? NAH, prioritize speed
Array Pathfinder::pathfind_sa_iwdmda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //don't skip the radius = 0 search (there could be walls)
    int manhattan_dist_to_end = manhattan_dist(start, end);
    int radius = 0;
    Vector2i lv_end = end - min;
    RadiusGetterDiamond get_radius(lv_end);
    BoundsChecker check_bounds(min, max);

    shared_ptr<SANode> shape_sanode = make_shared<SANode>();
    shape_sanode->set_lv_pos(start - min);
    shape_sanode->init_lv_back_ids(min, max);
    shape_sanode->set_lv_sid(shape_sanode->lv_pos, get_stuff_id(start));
    unique_ptr<PathInfo> pi = make_unique<PathInfo>();

    //enclosed goal check (radius == -1, no tile_id at dest)
    path_informed_mda(max_depth, false, shape_sanode, lv_end, pi, false, false, 0, get_radius);
    if (!pi->normalized_actions.size()) {
        return Array();
    }
    shape_sanode->set_lv_sid(lv_end, get_stuff_id(end));

    while (radius < manhattan_dist_to_end - 1) {
        path_informed_mda(max_depth, allow_type_change, shape_sanode, lv_end, pi, true, false, radius, get_radius);
        ++radius;
        shape_sanode->widen_diamond(min, end, radius, check_bounds);
    }
    path_informed_mda(max_depth, allow_type_change, shape_sanode, lv_end, pi, true, false, radius, get_radius);
    shape_sanode->fill_complement(min, max, radius, get_radius);
    path_informed_mda(max_depth, allow_type_change, shape_sanode, lv_end, pi, false, false, 0, get_radius); //radius is DONT_CARE
    return pi->normalized_actions;
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

//use sanode to avoid calling get_back_id(pos) unnecessarily
//use hbjpmda bc without tiles, hbjp is faster
bool Pathfinder::is_goal_enclosed(shared_ptr<SANode> env, Vector2i lv_end) {
    priority_queue<EnclosureNode, vector<EnclosureNode>, EnclosureNodeComparer> open;
    vector<vector<int>> best_dists = vector<vector<int>>(env->lv.size(), std::vector<int>(env->lv[0].size(), 0));

    EnclosureNode first(lv_end);
    first.h = manhattan_dist(lv_end, env->lv_pos);
    first.f = first.h;
    open.push(first);
    //best_dists[first.lv_pos.y][first.lv_pos.x] = 0;

    while (!open.empty()) {
        EnclosureNode curr = open.top();

        if (curr.lv_pos == env->lv_pos) {
            return false;
        }
        open.pop();
        if (curr.g != best_dists[curr.lv_pos.y][curr.lv_pos.x]) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS) {
            
        }
    }
}

//assume node qualifies for h_reduction
int Pathfinder::get_h_reduction(int virtual_path_index, bool sim_anneal) {
    return H_REDUCTION_BASE + max(virtual_path_index - H_REDUCTION_VIRTUAL_PATH_INDEX_OFFSET, 0) * H_REDUCTION_VIRTUAL_PATH_INDEX_FACTOR;
}

void Pathfinder::rrd_clear_iad() {
    inconsistent_abstract_dists.clear();
}

void Pathfinder::rrd_clear_cad() {
    consistent_abstract_dists.clear();
}
