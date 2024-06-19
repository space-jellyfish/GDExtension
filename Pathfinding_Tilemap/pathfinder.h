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
//treat EMPTY and ZERO as distinct nodes unless TRACK_ZEROS is false
//SA search allows killing other types as long as agent_type is preserved

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
    size_t operator() (const Vector2i v) const {
        size_t hash_x = hash<int>{}(v.x);
        size_t hash_y = hash<int>{}(v.y);
        return hash_x ^ (hash_y + 0x9e3779b9 + (hash_x << 6) + (hash_x >> 2));
    }
};


struct SANode : public enable_shared_from_this<SANode> {
    Vector2i lv_pos;
    vector<vector<int>> lv;
    //Vector3i prev_action;
    vector<Vector3i> prev_actions = {Vector3i(0, 0, 0)};
    shared_ptr<SANode> prev = nullptr;
    bool prev_merged_empty = false; //to prevent backtracking
    bool prev_merged_ssr = false; //to prevent backtracking; ssr means same sign regular
    int g=0, h=0, f=0; //use f for dijkstra
    size_t hash;

    Array trace_path(int path_len);
    void print_lv();

    //these should update hash
    void init_lv_pos(Vector2i pos);
    void init_lv(Vector2i min, Vector2i max);
    void set_lv_sid(Vector2i pos, int new_sid);
    void set_lv_pos(Vector2i pos);
    void clear_lv_sid(Vector2i pos);

    int get_lv_sid(Vector2i pos);
    int get_dist_to_lv_edge(Vector2i dir);
    int get_slide_push_count(Vector2i dir, bool allow_type_change);
    void perform_slide(Vector2i dir, int push_count);
    
    shared_ptr<SANode> try_slide(Vector2i dir, bool allow_type_change);
    shared_ptr<SANode> try_split(Vector2i dir, bool allow_type_change);
    shared_ptr<SANode> try_action(Vector3i action, bool allow_type_change);

    shared_ptr<SANode> jump(Vector2i dir, Vector2i lv_end);
    shared_ptr<SANode> get_jump_point(Vector2i dir, Vector2i jp_pos, int jump_dist, int zero_count);
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

    void rrd_init(int agent_type_id, Vector2i goal_pos);
    bool rrd_resume(int agent_type_id, Vector2i goal_pos, Vector2i node_pos);
    int get_move_abs_dist(int src_tile_id, int dest_tile_id);
    int get_sep_abs_dist(int tile_id1, int tile_id2);
    int get_abs_dist(int agent_type_id, Vector2i goal_pos, Vector2i node_pos);
};

const unordered_set<int> B_WALL_OR_BORDER = {BackId::BORDER_ROUND, BackId::BORDER_SQUARE, BackId::BLACK_WALL, BackId::BLUE_WALL, BackId::RED_WALL};
const unordered_set<int> B_SAVE_OR_GOAL = {BackId::SAVEPOINT, BackId::GOAL};
const unordered_set<int> T_ENEMY = {TypeId::INVINCIBLE, TypeId::HOSTILE, TypeId::VOID};
const unordered_set<Vector2i> DIRECTIONS = {Vector2i(1, 0), Vector2i(0, 1), Vector2i(-1, 0), Vector2i(0, -1)};
const unordered_set<Vector2i> H_DIRS = {Vector2i(1, 0), Vector2i(-1, 0)};
const unordered_set<Vector2i> V_DIRS = {Vector2i(0, 1), Vector2i(0, -1)};
const unordered_map<int, int> MERGE_PRIORITIES = {{TypeId::PLAYER, 1}, {TypeId::INVINCIBLE, 3}, {TypeId::HOSTILE, 2}, {TypeId::VOID, 4}, {TypeId::REGULAR, 0}};
const int TILE_POW_MAX = 14;
const int ABSTRACT_DIST_SIGN_CHANGE_PENALTY = 2;
const int MAX_SEARCH_WIDTH = 17;
const int MAX_SEARCH_HEIGHT = 17;
const int TILE_ID_BITLEN = 5;
const int TYPE_ID_BITLEN = 3;
const int BACK_ID_BITLEN = 8;
const int TILE_ID_COUNT = 1 << TILE_ID_BITLEN;
const int TILE_ID_MASK = TILE_ID_COUNT - 1;
const int TYPE_ID_MASK = ((1 << TYPE_ID_BITLEN) - 1) << TILE_ID_BITLEN;
const int BACK_ID_MASK = ((1 << BACK_ID_BITLEN) - 1) << (TILE_ID_BITLEN + TYPE_ID_BITLEN);
const bool TRACK_ZEROS = false;

extern array<array<array<size_t, TILE_ID_COUNT - 1>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> tile_id_hash_keys;
extern array<array<array<size_t, TypeId::REGULAR>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> type_id_hash_keys;
extern array<array<size_t, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> agent_pos_hash_keys;
extern TileMap* cells;
extern int tile_push_limit;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int get_stuff_id(int back_id, int type_id, int tile_id);
int get_type_bits(int stuff_id);
int get_back_bits(int stuff_id);
int get_tile_id(int stuff_id);
int get_type_id(int stuff_id);
int get_back_id(int stuff_id);
int get_stuff_id(Vector2i pos);
int get_tile_id(Vector2i pos);
int get_type_id(Vector2i pos);
int get_back_id(Vector2i pos);
bool is_compatible(int type_id, int back_id);
bool is_ids_mergeable(int tile_id1, int tile_id2);
bool is_same_sign_merge(int tile_id1, int tile_id2);
bool is_tile_unsigned(int tile_id);
bool is_tile_unsigned_and_regular(int stuff_id);
bool is_tile_empty_and_regular(int stuff_id);
bool is_pow_signs_mergeable(Vector2i ps1, Vector2i ps2);
bool is_pow_splittable(int pow);
bool is_eff_merge(int tile_id1, int tile_id2);
bool is_type_preserved(int src_type_id, int dest_type_id);
bool is_type_dominant(int src_type_id, int dest_type_id);
Vector2i tid_to_ps(int tile_id);
int ps_to_tid(Vector2i ps);
int get_tile_pow(int tile_id);
int get_tile_sign(int tile_id);
int get_true_tile_sign(int tile_id);
int get_opposite_tile_id(int tile_id);
Vector2i get_splitted_ps(Vector2i ps);
int get_splitted_tid(int tile_id);

//these assume merge is possible
int get_merged_stuff_id(int src_stuff_id, int dest_stuff_id);
int get_moved_stuff_id(int src_stuff_id, int dest_stuff_id);
int get_merged_tile_id(int tile_id1, int tile_id2);
Vector2i get_merged_pow_sign(Vector2i ps1, Vector2i ps2);


#endif