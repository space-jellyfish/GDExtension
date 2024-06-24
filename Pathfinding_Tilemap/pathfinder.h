#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/tile_map.hpp>
//#include <godot_cpp/classes/gd_script.hpp>
#include <unordered_map>
#include <unordered_set>
#include <queue>

using namespace std;
using namespace godot;

//each cell is represented as | BackId (8) | TypeId (3) | TileId (5) |
//Vector2i max is exclusive
//sign in pow_sign must be +-1
//only REGULAR zeros are bubblable

//treat EMPTY and ZERO as distinct nodes; there are no zeros unless TRACK_ZEROS is true
//use TRACK_ZEROS if zeros are relevant to gameplay logic

//killable type is type with merge priority <= that of agent type
//treat killing other type as distinct node; there are no killable types unless TRACK_KILLABLE_TYPES is true
//for SA search, don't use TRACK_KILLABLE_TYPES

//if hostile merges with killable type into 0, killable type is killed, then hostile dies, resulting in regular 0
//don't allow merge onto incompatible back_id, even if result type is compatible

enum TileId {
    EMPTY = 0,
    ZERO = 16,
};

enum TypeId {
    PLAYER = 0,
    INVINCIBLE,
    HOSTILE,
    VOID,
    REGULAR,
};

enum BackId {
	NONE = 0,
	BORDER_ROUND,
	BORDER_SQUARE,
	MEMBRANE,
	BLACK_WALL,
	BLUE_WALL,
	RED_WALL,
	SAVEPOINT,
	GOAL,
};

enum LayerId {
    BACK = 0,
    TILE
};

enum ColorId {
	ALL = 4,
	RED = 29,
	BLUE = 30,
	BLACK = 31,
	GRAY = 32,
};

enum SearchId {
	DIJKSTRA = 0,
    HBJPD, //horizontally biased jump point dijkstra
	ASTAR,
	IDASTAR,
};

enum ActionId {
	SLIDE = 0,
	SPLIT,
    JUMP,
	END,
};

struct AbstractNode {
    Vector2i pos = Vector2i(0, 0);
    int g = numeric_limits<int>::max();

    AbstractNode(Vector2i position, int abs_dist) {
        pos = position;
        g = abs_dist;
    }
};

struct AbstractNodeComparer {
	bool operator() (AbstractNode first, AbstractNode second) {
		return first.g > second.g;
	}
};

struct Vector2iHasher {
    size_t operator() (const Vector2i& v) const {
        size_t hash_x = hash<int32_t>{}(v.x);
        size_t hash_y = hash<int32_t>{}(v.y);
        return hash_x ^ (hash_y + 0x9e3779b9 + (hash_x << 6) + (hash_x >> 2));
    }
};

struct DirHasher {
    size_t operator() (const Vector2i& dir) const {
        return (3 * dir.x + dir.y + 3) >> 1;
    }
};

struct ActionHasher {
    size_t operator() (const Vector3i& action) const {
        return ((3 * action.x + action.y + 3) >> 1) + (action.z << 2);
    }
};

struct SANode : public enable_shared_from_this<SANode> {
    Vector2i lv_pos;
    vector<vector<uint16_t>> lv;
    int g=0, h=0, f=0; //use f for dijkstra
    //for fast equality check
    size_t hash;
    //for backtracing
    vector<Vector3i> prev_actions;
    shared_ptr<SANode> prev = nullptr;
    int prev_push_count = 0;
    //to store intermediate results/prevent backtracking
    //nullptr indicates neighbor is either pruned or invalid
    unordered_map<Vector3i, weak_ptr<SANode>, ActionHasher> neighbors; //action, SANode


    Array trace_path(int path_len);
    void print_lv();

    //these should update hash
    void init_lv_pos(Vector2i pos);
    void init_lv(Vector2i min, Vector2i max, Vector2i agent_pos);
    void set_lv_sid(Vector2i pos, uint16_t new_sid);
    void set_lv_pos(Vector2i pos);
    void clear_lv_sid(Vector2i pos);

    uint16_t get_lv_sid(Vector2i pos);
    int get_dist_to_lv_edge(Vector2i dir);
    int get_slide_push_count(Vector2i dir, bool allow_type_change);
    void perform_slide(Vector2i dir, int push_count);
    
    shared_ptr<SANode> try_slide(Vector2i dir, bool allow_type_change);
    shared_ptr<SANode> try_split(Vector2i dir, bool allow_type_change);
    shared_ptr<SANode> try_action(Vector3i action, Vector2i lv_end, bool allow_type_change);

    shared_ptr<SANode> try_jump(Vector2i dir, Vector2i lv_end, bool allow_type_change);
    shared_ptr<SANode> get_jump_point(Vector2i dir, Vector2i jp_pos, int jump_dist);
    void prune_action_ids(Vector2i dir);
};

struct SANodeHashGetter  {
    size_t operator() (const shared_ptr<SANode> n) const {
        return n->hash;
    }
};

struct SANodeEquator {
	bool operator() (const shared_ptr<SANode> first, const shared_ptr<SANode> second) const {
		//return (first->lv_pos == second->lv_pos && first->lv == second->lv);
        return first->hash == second->hash;
	}
};

//if f tied, use g; higher g indicates higher confidence
struct SANodeComparer {
	bool operator() (shared_ptr<SANode> first, shared_ptr<SANode> second) {
		if (first->f > second->f) {
			return true;
		}
		if (first->f < second->f) {
			return false;
		}
		return first->g < second->g;
	}
};

typedef priority_queue<AbstractNode, vector<AbstractNode>, AbstractNodeComparer> pq;
typedef unordered_map<Vector2i, int, Vector2iHasher> um;

class Pathfinder : public Node {
    friend struct SANode;
    GDCLASS(Pathfinder, Node);

private:
    Vector2i player_pos;
    Vector2i player_last_dir;
    //GDScript* GV;
    unordered_map<Vector2i, pair<pq, um>, Vector2iHasher> abstract_dists; //goal_pos, [open, closed]; assume all entries are actively used

protected:
    static void _bind_methods();

public:
    Array pathfind(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_hbjpd(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_astar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_idastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_rrdastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_ididjpastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);

    void set_player_pos(Vector2i pos);
    void set_player_last_dir(Vector2i dir);
    void set_tilemap(TileMap* t);
    void set_tile_push_limit(int tpl);
    void generate_hash_keys();

    bool is_tile(Vector2i pos);
    bool is_immediately_trapped(Vector2i pos);

    void rrd_init(uint8_t agent_type_id, Vector2i goal_pos);
    bool rrd_resume(uint8_t agent_type_id, Vector2i goal_pos, Vector2i node_pos);
    int get_move_abs_dist(uint8_t src_tile_id, uint8_t dest_tile_id);
    int get_sep_abs_dist(uint8_t tile_id1, uint8_t tile_id2);
    int get_abs_dist(uint8_t agent_type_id, Vector2i goal_pos, Vector2i node_pos);
};

const unordered_set<uint8_t> B_WALL_OR_BORDER = {BackId::BORDER_ROUND, BackId::BORDER_SQUARE, BackId::BLACK_WALL, BackId::BLUE_WALL, BackId::RED_WALL};
const unordered_set<uint8_t> B_SAVE_OR_GOAL = {BackId::SAVEPOINT, BackId::GOAL};
const unordered_set<uint8_t> T_ENEMY = {TypeId::INVINCIBLE, TypeId::HOSTILE, TypeId::VOID};
const unordered_set<Vector2i, DirHasher> DIRECTIONS = {Vector2i(1, 0), Vector2i(0, 1), Vector2i(-1, 0), Vector2i(0, -1)};
const unordered_set<Vector2i, DirHasher> H_DIRS = {Vector2i(1, 0), Vector2i(-1, 0)};
const unordered_set<Vector2i, DirHasher> V_DIRS = {Vector2i(0, 1), Vector2i(0, -1)};
const unordered_map<uint8_t, int> MERGE_PRIORITIES = {{TypeId::PLAYER, 1}, {TypeId::INVINCIBLE, 3}, {TypeId::HOSTILE, 2}, {TypeId::VOID, 4}, {TypeId::REGULAR, 0}};
const int TILE_POW_MAX = 14;
const int ABSTRACT_DIST_SIGN_CHANGE_PENALTY = 2;
const int MAX_SEARCH_WIDTH = 17;
const int MAX_SEARCH_HEIGHT = 17;
const int TILE_ID_BITLEN = 5;
const int TYPE_ID_BITLEN = 3;
const int BACK_ID_BITLEN = 8;
const int TILE_ID_COUNT = 1 << TILE_ID_BITLEN;
const uint16_t TILE_ID_MASK = TILE_ID_COUNT - 1;
const uint16_t TYPE_ID_MASK = ((1 << TYPE_ID_BITLEN) - 1) << TILE_ID_BITLEN;
const uint16_t BACK_ID_MASK = ((1 << BACK_ID_BITLEN) - 1) << (TILE_ID_BITLEN + TYPE_ID_BITLEN);
const uint16_t BACK_AND_TYPE_ID_MASK = BACK_ID_MASK + TYPE_ID_MASK;
const uint16_t BACK_AND_TILE_ID_MASK = BACK_ID_MASK + TILE_ID_MASK;
const uint16_t TYPE_AND_TILE_ID_MASK = TYPE_ID_MASK + TILE_ID_MASK;
const uint16_t REGULAR_TYPE_BITS = TypeId::REGULAR << TILE_ID_BITLEN;
const bool TRACK_ZEROS = false;
const bool TRACK_KILLABLE_TYPES = false;

extern array<array<array<size_t, TILE_ID_COUNT - 1>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> tile_id_hash_keys;
extern array<array<array<size_t, TypeId::REGULAR>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> type_id_hash_keys;
extern array<array<size_t, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> agent_pos_hash_keys;
extern TileMap* cells;
extern int tile_push_limit;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

uint16_t get_stuff_id(uint8_t back_id, uint8_t type_id, uint8_t tile_id);
uint16_t get_type_bits(uint16_t stuff_id);
uint16_t get_back_bits(uint16_t stuff_id);
uint8_t get_tile_id(uint16_t stuff_id);
uint8_t get_type_id(uint16_t stuff_id);
uint8_t get_back_id(uint16_t stuff_id);
uint16_t get_stuff_id(Vector2i pos);
uint8_t get_tile_id(Vector2i pos);
uint8_t get_type_id(Vector2i pos);
uint8_t get_back_id(Vector2i pos);
uint16_t remove_tile_id(uint16_t stuff_id);
uint16_t regular_type_id(uint16_t stuff_id);
uint16_t remove_back_id(uint16_t stuff_id);
bool is_compatible(uint8_t type_id, uint8_t back_id);
bool is_ids_mergeable(uint8_t tile_id1, uint8_t tile_id2);
bool is_ids_split_mergeable(uint8_t src_tile_id, uint8_t dest_tile_id);
bool is_same_sign_merge(uint8_t tile_id1, uint8_t tile_id2);
bool is_tile_unsigned(uint8_t tile_id);
bool is_tile_unsigned_and_regular(uint16_t stuff_id);
bool is_tile_empty_and_regular(uint16_t stuff_id);
bool is_pow_signs_mergeable(Vector2i ps1, Vector2i ps2);
bool is_pow_splittable(int pow);
bool is_eff_merge(uint8_t tile_id1, uint8_t tile_id2);
bool is_type_preserved(uint8_t src_type_id, uint8_t dest_type_id);
bool is_type_dominant(uint8_t src_type_id, uint8_t dest_type_id);
Vector2i tid_to_ps(uint8_t tile_id);
uint8_t ps_to_tid(Vector2i ps);
int get_tile_pow(uint8_t tile_id);
int get_signed_tile_pow(uint8_t tile_id);
int get_tile_sign(uint8_t tile_id);
int get_true_tile_sign(uint8_t tile_id);
uint8_t get_opposite_tile_id(uint8_t tile_id);
Vector2i get_splitted_ps(Vector2i ps);
uint8_t get_splitted_tid(uint8_t tile_id);

//these assume merge is possible
uint16_t get_merged_stuff_id(uint16_t src_stuff_id, uint16_t dest_stuff_id);
uint16_t get_jumped_stuff_id(uint16_t src_stuff_id, uint16_t dest_stuff_id);
uint8_t get_merged_tile_id(uint8_t tile_id1, uint8_t tile_id2);
Vector2i get_merged_pow_sign(Vector2i ps1, Vector2i ps2);


#endif