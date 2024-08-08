#ifndef PATHFINDER_HPP
#define PATHFINDER_HPP

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/tile_map.hpp>
//#include <godot_cpp/classes/gd_script.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>

using namespace std;
using namespace godot;

//IMPORTANT:
//there's a namespace conflict, specify "std::" when using std::set
//don't leave any unjustified asserts commented out
//after all asserts are commented out for release, they will be indistinguishable from justified asserts
//don't use pair/tuple, use structs instead for strong typing (with member initializer lists)
//ensure no overflow happens when left shifting (<<); see https://en.cppreference.com/w/cpp/language/implicit_conversion#Integral_promotion
//for map/unordered_map, emplace()/insert() don't do anything if key exists
//don't emplace()/insert() after find() bc it requires duplicate lookup

//each cell is represented as | BackId (8) | TypeId (3) | TileId (5) |
//Vector2i max is exclusive
//sign in pow_sign must be +-1
//only REGULAR zeros are bubblable
//allow tile situated on an incompatible back_id to move into a compatible back_id

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
    PLAYER,
    INVINCIBLE,
    HOSTILE,
    VOID,
    REGULAR,
};

enum BackId {
	NONE,
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
    BACK,
    TILE
};

enum ColorId {
	ALL = 4,
	RED = 29,
	BLUE = 30,
	BLACK = 31,
	GRAY = 32,
};

enum SASearchId {
	DIJKSTRA,
    HBJPD, //horizontally biased jump point dijkstra
	MDA, //manhattan distance astar
    IADA, //inconsistent abstract distance astar
    IADANR, //* no re-expansion
    HBJPMDA,
    HBJPIADA,
    HBJPIADANR,
    IWDMDA, //iterative widening diamond *
    IWDHBJPMDA,
    IWSMDA, //iterative widening square *
    IWSHBJPMDA,
    SAIWDMDA, //simulated annealing *
    SAIWDHBJPMDA,
    SAIWSMDA,
    SAIWSHBJPMDA,
	//IDA/EPEA, QUANT, FMT/RRT
    SEARCH_END,
};

enum MASearchId {
    LRA, //local repair astar
    NF, //network flow; doesn't work, see Pictures/flow_based_does_not_work
    WHCA, //windowed hierarchical cooperative astar
    EPEA, //enhanced partial expansion astar
    ICTS, //increasing cost tree search
    CBS, //conflict-based search
        //add constraints on wake in addition to tile itself
    QCBS, //quant cbs; include conflicts with tiles previously affected by other agents
    QRCBS, //quant request-and-conflict-based search; send (request r, int max_cost) pairs for help
        //request types:
            //move out of the way? handle via constraint tree
            //change tile_id at pos to one in a set of values
};

enum ActionId {
	SLIDE,
	SPLIT,
    JUMP,
	ACTION_END,
};

enum IWShapeId {
    DIAMOND,
    SQUARE,
};

//collision-free
//mind the undefined behavior
struct Vector2iHasher {
    uint64_t operator() (const Vector2i& v) const {
        //uint64_t hash_x = hash<int32_t>{}(v.x);
        //uint64_t hash_y = hash<int32_t>{}(v.y);
        //return hash_x ^ (hash_y + 0x9e3779b9 + (hash_x << 6) + (hash_x >> 2));
        //return (v.x << 32) + v.y; //undefined
        uint64_t ans = v.y;
        //assert(sizeof(v.x) == 4);
        memcpy(&ans, &v.x, sizeof(v.x));
        return ans;
    }
};

//collision-free
struct DirHasher {
    uint64_t operator() (const Vector2i& dir) const {
        return (3 * dir.x + dir.y + 3) >> 1;
    }
};

//collision-free
struct ActionHasher {
    uint64_t operator() (const Vector3i& action) const {
        return ((3 * action.x + action.y + 3) >> 1) + (action.z << 2);
    }
};

const unordered_set<uint8_t> B_WALL_OR_BORDER = {BackId::BORDER_ROUND, BackId::BORDER_SQUARE, BackId::BLACK_WALL, BackId::BLUE_WALL, BackId::RED_WALL};
const unordered_set<uint8_t> B_SAVE_OR_GOAL = {BackId::SAVEPOINT, BackId::GOAL};
const unordered_set<uint8_t> T_ENEMY = {TypeId::INVINCIBLE, TypeId::HOSTILE, TypeId::VOID};
const unordered_set<Vector2i, DirHasher> DIRECTIONS = {Vector2i(1, 0), Vector2i(0, 1), Vector2i(-1, 0), Vector2i(0, -1)};
const unordered_set<Vector2i, DirHasher> H_DIRS = {Vector2i(1, 0), Vector2i(-1, 0)};
const unordered_set<Vector2i, DirHasher> V_DIRS = {Vector2i(0, 1), Vector2i(0, -1)};
const unordered_map<uint8_t, int> MERGE_PRIORITIES = {{TypeId::PLAYER, 1}, {TypeId::INVINCIBLE, 3}, {TypeId::HOSTILE, 2}, {TypeId::VOID, 4}, {TypeId::REGULAR, 0}};
const int HASH_KEY_GENERATOR_SEED = 2050;
const int TILE_POW_MAX = 14;
const int IAD_SEP_FACTOR = 1;
const int IAD_SIGN_CHANGE_PENALTY = 2;
const float H_REDUCTION_ITERATION_FACTOR = 1.0;
const float H_REDUCTION_VIRTUAL_PATH_INDEX_FACTOR = 0.7;
const int H_REDUCTION_VIRTUAL_PATH_INDEX_OFFSET = 3; //to give path_informed search more "room" to find better path
const int H_REDUCTION_BASE = 2;
const int MAX_SEARCH_WIDTH = 17;
const int MAX_SEARCH_HEIGHT = 17;
const int TILE_ID_BITLEN = 5;
const int TYPE_ID_BITLEN = 3;
const int BACK_ID_BITLEN = 8;
const int TILE_AND_TYPE_ID_BITLEN = TILE_ID_BITLEN + TYPE_ID_BITLEN;
const int TILE_ID_COUNT = 1 << TILE_ID_BITLEN;
const uint16_t TILE_ID_MASK = TILE_ID_COUNT - 1;
const uint16_t TYPE_ID_MASK = ((1 << TYPE_ID_BITLEN) - 1) << TILE_ID_BITLEN;
const uint16_t BACK_ID_MASK = ((1 << BACK_ID_BITLEN) - 1) << TILE_AND_TYPE_ID_BITLEN;
const uint16_t BACK_AND_TYPE_ID_MASK = BACK_ID_MASK + TYPE_ID_MASK;
const uint16_t BACK_AND_TILE_ID_MASK = BACK_ID_MASK + TILE_ID_MASK;
const uint16_t TYPE_AND_TILE_ID_MASK = TYPE_ID_MASK + TILE_ID_MASK;
const uint16_t REGULAR_TYPE_BITS = TypeId::REGULAR << TILE_ID_BITLEN;
const bool TRACK_ZEROS = false;
const bool TRACK_KILLABLE_TYPES = false;

struct NextDir {
    Vector2i dir;
    bool in_bounds;
    bool blocked; //unused if next_dir == jump_dir

    NextDir() {} //so std::array of NextDir in try_jump() can be initialized
    NextDir(Vector2i _dir, bool _in_bounds, bool _blocked) : dir(_dir), in_bounds(_in_bounds), blocked(_blocked) {}
};

struct PathNode {
    Vector2i lv_pos;
    int index;

    PathNode(Vector2i _lv_pos, int _index) : lv_pos(_lv_pos), index(_index) {}
};

struct PathNodeHasher {
    uint64_t operator() (const PathNode& n) const {
        uint64_t lv_pos_hash = Vector2iHasher{}(n.lv_pos);
        uint64_t index_hash = hash<int>{}(n.index);
        return lv_pos_hash ^ (index_hash + 0x9e3779b9 + (lv_pos_hash << 6) + (lv_pos_hash >> 2));
    }
};

struct PathNodeEquator {
    bool operator() (const PathNode& first, const PathNode& second) const {
        return first.lv_pos == second.lv_pos && first.index == second.index;
    }
};

//for informing h_reductions in iterative widening searches
struct PathInfo {
    unordered_map<Vector2i, std::set<int>, Vector2iHasher> lp_to_path_indices; //lv_pos, indices in path
    //use bitset bc array<bool> doesn't support bit operation, and uint32_t doesn't support []operator
    //for get_virtual_path_index(), bitset[TileId::EMPTY] should be equal to bitset[TileId::ZERO]
    unordered_map<PathNode, bitset<TILE_ID_COUNT>, PathNodeHasher, PathNodeEquator> pn_to_admissible_tile_ids;
    Array normalized_actions;
};

struct IADNode {
    Vector2i pos;
    int g;

    IADNode(Vector2i _pos, int _g) : pos(_pos), g(_g) {}
};

struct IADNodeComparer {
	bool operator() (const IADNode& first, const IADNode& second) {
		return first.g > second.g;
	}
};

struct CADNode {
    Vector2i pos;
    int g;
    shared_ptr<CADNode> prev;
    Vector2i prev_dir;
    //vector<uint8_t> behinds; //get from lv

    CADNode(Vector2i _pos, int _g, shared_ptr<CADNode> _prev, Vector2i _prev_dir) : pos(_pos), g(_g), prev(_prev), prev_dir(_prev_dir) {}
};

struct CADNodeComparer {
    bool operator() (const shared_ptr<CADNode>& first, const shared_ptr<CADNode>& second) {
		return first->g > second->g;
    }
};

struct CADNodeHasher {
    uint64_t operator() (const shared_ptr<CADNode>& n) const {
        return Vector2iHasher{}(n->pos);
    }
};

struct CADNodeEquator {
    bool operator() (const shared_ptr<CADNode>& first, const shared_ptr<CADNode>& second) const {
        return first->pos == second->pos;
    }
};

//inline is unnecessary, see StackOverflow/1759300
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

uint16_t make_stuff_id(uint8_t back_id, uint8_t type_id, uint8_t tile_id);
uint16_t make_back_bits(uint8_t back_id);
uint16_t make_type_bits(uint8_t type_id);
uint16_t get_type_bits(uint16_t stuff_id);
uint16_t get_back_bits(uint16_t stuff_id);
uint8_t get_tile_id(uint16_t stuff_id);
uint8_t get_type_id(uint16_t stuff_id);
uint8_t get_back_id(uint16_t stuff_id);
uint16_t get_stuff_id(Vector2i pos);
uint8_t get_tile_id(Vector2i pos);
uint8_t get_type_id(Vector2i pos);
uint8_t get_back_id(Vector2i pos);
uint16_t reset_tile_id(uint16_t stuff_id);
uint16_t reset_type_id(uint16_t stuff_id);
uint16_t reset_back_id(uint16_t stuff_id);
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
int get_action_dist(Vector3i action);
uint8_t get_opposite_tile_id(uint8_t tile_id);
Vector2i get_splitted_ps(Vector2i ps);
uint8_t get_splitted_tid(uint8_t tile_id);

//these assume merge is possible
uint16_t get_merged_stuff_id(uint16_t src_stuff_id, uint16_t dest_stuff_id);
uint16_t get_jumped_stuff_id(uint16_t src_stuff_id, uint16_t dest_stuff_id);
uint8_t get_merged_tile_id(uint8_t tile_id1, uint8_t tile_id2);
Vector2i get_merged_pow_sign(Vector2i ps1, Vector2i ps2);

Vector2i get_normalized_dir(Vector3i action);
Vector3i get_normalized_action(Vector3i action);


//heuristics (only cells in lv allowed for rrd)
//sharable across multiple pathfind()s with different min, max (all positions are global)
//if rrd finds that goal_pos is enclosed, return numeric_limits<int>::max() so pathfinder can exit early
//inconsistent abstract distance
//propagate h-cost using separation between tile_ids
void rrd_init_iad(Vector2i goal_pos);
int rrd_resume_iad(Vector2i goal_pos, Vector2i node_pos, int agent_type_id);
int get_action_iad(uint8_t src_tile_id, uint8_t dest_tile_id);
int get_tile_id_sep(uint8_t tile_id1, uint8_t tile_id2);
//consistent abstract distance
//DEPRECATED, see Pictures/greedy_is_not_optimal_when_parsing_sequence
//see extra.h for the idea
void rrd_init_cad(Vector2i goal_pos);
int rrd_resume_cad(Vector2i goal_pos, Vector2i agent_pos);
int trace_cad(shared_ptr<CADNode> n);
bool is_perp(Vector2i first, Vector2i second);
int get_cad_push_count();
int manhattan_dist(Vector2i pos1, Vector2i pos2);


//functors
struct RadiusGetterDiamond {
    Vector2i dest_lv_pos;

    RadiusGetterDiamond(Vector2i _dest_lv_pos) : dest_lv_pos(_dest_lv_pos) {}
    unsigned int operator()(Vector2i curr_lv_pos) const { return manhattan_dist(curr_lv_pos, dest_lv_pos); }
};

struct RadiusGetterSquare {
    Vector2i dest_lv_pos;

    RadiusGetterSquare(Vector2i _dest_lv_pos) : dest_lv_pos(_dest_lv_pos) {}
    unsigned int operator()(Vector2i curr_lv_pos) const { return max(abs(curr_lv_pos.x - dest_lv_pos.x), abs(curr_lv_pos.y - dest_lv_pos.y)); }
};

struct BoundsChecker {
    Vector2i min, max;

    BoundsChecker(Vector2i _min, Vector2i _max) : min(_min), max(_max) {}
    bool operator()(Vector2i pos) const { return pos.x >= min.x && pos.x < max.x && pos.y >= min.y && pos.y < max.y; }
};

//shared bc multiple SASearchNodes could use same SANode
struct SANode : public enable_shared_from_this<SANode> {
    Vector2i lv_pos;
    vector<vector<uint16_t>> lv;
    //zobrist; for fast equality check
    uint64_t hash;

    //these should update hash
    void init_lv_pos(Vector2i _lv_pos);
    void init_lv(Vector2i min, Vector2i max);
    void init_lv_back_ids(Vector2i min, Vector2i max);
    void init_lv_ttid(Vector2i _lv_pos, Vector2i pos);
    void init_lv_ttid_idempotent(Vector2i _lv_pos, Vector2i pos);
    void set_lv_sid(Vector2i _lv_pos, uint16_t new_sid);
    void set_lv_pos(Vector2i _lv_pos);
    void reset_lv_sid(Vector2i _lv_pos);
    void perform_slide(Vector2i dir, int push_count);

    void print_lv() const;
    uint16_t get_lv_sid(Vector2i _lv_pos) const;
    int get_dist_to_lv_edge(Vector2i _lv_pos, Vector2i dir) const;
    int get_slide_push_count(Vector2i dir, bool allow_type_change) const;

    //for iterative widening search
    void widen_diamond(Vector2i min, Vector2i end, int new_radius, const BoundsChecker& check_bounds);
    void widen_square(Vector2i min, Vector2i end, int new_radius, const BoundsChecker& check_bounds);
    template <typename RadiusGetter>
    void fill_complement(Vector2i min, Vector2i max, int radius, const RadiusGetter& get_radius);
};

struct SANeighbor {
    unsigned int unprune_threshold;
    shared_ptr<SANode> sanode;
    unsigned int push_count;

    SANeighbor() {} //so operator[] works
    SANeighbor(unsigned int _unprune_threshold, shared_ptr<SANode> _sanode, unsigned int _push_count)
        : unprune_threshold(_unprune_threshold)
        , sanode(_sanode)
        , push_count(_push_count)
        {}
};

//for is_goal_enclosed()
struct EnclosureNode {
    Vector2i lv_pos;
    int g=0, h=0, f=0;

    EnclosureNode(Vector2i _lv_pos) : lv_pos(_lv_pos) {}
    EnclosureNode jump(Vector2i dir, shared_ptr<SANode> env, uint8_t agent_type_id);
};

struct EnclosureNodeComparer {
    bool operator() (const EnclosureNode& first, const EnclosureNode& second) const {
        if (first.f > second.f) {
            return true;
        }
        if (first.f < second.f) {
            return false;
        }
        return first.g < second.g;
    }
};

//shared bc multiple children have prev pointer
template <typename SASearchNode_t>
struct SASearchNodeBase : public enable_shared_from_this<SASearchNodeBase<SASearchNode_t>> {
    shared_ptr<SANode> sanode;
    int g=0, h=0, f=0; //use f for dijkstra
    //for backtracing
    shared_ptr<SASearchNode_t> prev = nullptr;
    Vector3i prev_action; //[x_displacement, y_displacement, action_id]
    unsigned int prev_push_count = 0;
    //to store intermediate results/prevent backtracking (perform_slide(), try_jump())
    //normalized_action, {unprune_threshold, neighbor_sanode, push_count}
    //if neighbor is invalid, unprune_threshold == numeric_limits<unsigned int>::max()
    //if neighbor not pruned, unprune_threshold == 0
    //nullptr is placeholder for prune, it doesn't indicate anything in particular (creating the SANode would be wasteful)
    //missing entry indicates unknown
    unordered_map<Vector3i, SANeighbor, ActionHasher> neighbors;

    void init_sanode(Vector2i min, Vector2i max, Vector2i start);
    shared_ptr<SASearchNode_t> try_slide(Vector2i dir, bool allow_type_change);
    shared_ptr<SASearchNode_t> try_split(Vector2i dir, bool allow_type_change);
    template <typename BestDists_t>
    shared_ptr<SASearchNode_t> try_action(Vector3i normalized_action, Vector2i lv_end, bool allow_type_change, const BestDists_t& best_dists);

    template <typename BestDists_t>
    shared_ptr<SASearchNode_t> try_jump(Vector2i dir, Vector2i lv_end, bool allow_type_change, const BestDists_t& best_dists);
    shared_ptr<SASearchNode_t> get_jump_point(shared_ptr<SANode> prev_sanode, Vector2i dir, Vector2i jp_pos, unsigned int jump_dist);
    void prune_invalid_action_ids(Vector2i dir);
    void prune_backtrack(Vector2i dir);
    void transfer_neighbors(shared_ptr<SASearchNode_t> better_dist, int dist_improvement);

    Array trace_path_normalized_actions(int path_len);
    template <typename RadiusGetter>
    void trace_path_informers(unique_ptr<PathInfo>& pi, int path_len, int radius, const RadiusGetter& get_radius);
    int get_virtual_path_index(unique_ptr<PathInfo>& pi, int largest_affected_path_index);
    void relax_admissibility(bitset<TILE_ID_COUNT>& admissible_tile_ids);
    void relax_admissibility(bitset<TILE_ID_COUNT>& admissible_tile_ids, bool is_next_merge, uint8_t adjacent_tile_id);
    void trace_node_info(unique_ptr<PathInfo>& pi, const PathNode& pn, const bitset<TILE_ID_COUNT>& admissible_tile_ids);
};

struct SASearchNode : public SASearchNodeBase<SASearchNode> {};

//single agent path informed search node
struct SAPISearchNode : public SASearchNodeBase<SAPISearchNode> {
    int largest_affected_path_index = 0;
    int virtual_path_index = -1;

    void init_lapi(unique_ptr<PathInfo>& pi, Vector2i dir);
    void update_lapi(std::set<int>* largest_prev_path_indices, int effective_largest_affected_path_index);
    void update_lapi_helpers(unique_ptr<PathInfo>& pi, Vector2i affected_lv_pos, std::set<int>*& largest_prev_path_indices, int& largest_affected_lp_path_index, int& penultimate_affected_lp_path_index);
};

template <typename SASearchNode_t>
struct SASearchNodeBaseHashGetter  {
    uint64_t operator() (const shared_ptr<SASearchNodeBase<SASearchNode_t>>& n) const {
        return n->sanode->hash;
    }
};

template <typename SASearchNode_t>
struct SASearchNodeBaseEquator {
	bool operator() (const shared_ptr<SASearchNodeBase<SASearchNode_t>>& first, const shared_ptr<SASearchNodeBase<SASearchNode_t>>& second) const {
		//return (first->lv_pos == second->lv_pos && first->lv == second->lv);
        //return first->hash == second->hash && first->lv_pos == second->lv_pos && first->lv == second->lv;
        if  (first->sanode->hash == second->sanode->hash) {
            if (first->sanode->lv_pos == second->sanode->lv_pos && first->sanode->lv == second->sanode->lv) {
                return true;
            }
            //UtilityFunctions::print("HASH COLLISION");
        }
        return false;
	}
};

//if f tied, use g; higher g indicates higher confidence
template <typename SASearchNode_t>
struct SASearchNodeBaseFComparer {
	bool operator() (const shared_ptr<SASearchNodeBase<SASearchNode_t>>& first, const shared_ptr<SASearchNodeBase<SASearchNode_t>>& second) {
		if (first->f > second->f) {
			return true;
		}
		if (first->f < second->f) {
			return false;
		}
		return first->g < second->g;
	}
};

template <typename SASearchNode_t>
struct SASearchNodeBaseGComparer {
    bool operator() (const shared_ptr<SASearchNodeBase<SASearchNode_t>>& first, const shared_ptr<SASearchNodeBase<SASearchNode_t>>& second) {
        return first->g > second->g;
    }
};


class Pathfinder : public Node {
    //friend struct SANode;
    GDCLASS(Pathfinder, Node);

private:
    Vector2i player_pos;
    Vector2i player_last_dir;
    //GDScript* GV;

protected:
    static void _bind_methods();

public:
    //check for numeric_limits<int>::max() to exit early if using an rrd heuristic
    double get_sa_cumulative_search_time(int sa_search_id);
    void reset_sa_cumulative_search_times();
    Array pathfind_sa(int search_id, int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_dijkstra(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_hbjpd(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_mda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_iada(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_iadanr(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_hbjpmda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_hbjpiada(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_hbjpiadanr(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_sa_iwdmda(int max_depth, bool allow_type_change, Vector2i min, Vector2i max, Vector2i start, Vector2i end);

    //should starts/ends be same size?
    Array pathfind_ma(int search_id, int max_sa_depth, bool allow_type_change, Vector2i min, Vector2i max, vector<Vector2i> starts, vector<Vector2i> ends);

    void set_player_pos(Vector2i pos);
    void set_player_last_dir(Vector2i dir);
    void set_tilemap(TileMap* t);
    void set_tile_push_limits(Dictionary tpls);
    void generate_hash_keys();

    bool is_immediately_trapped(Vector2i pos);
    bool is_goal_enclosed(shared_ptr<SANode> env, Vector2i lv_end);
    //iterative widening helper functions
    template <typename RadiusGetter>
    void path_informed_mda(int max_depth, bool allow_type_change, shared_ptr<SANode> start, Vector2i lv_end, unique_ptr<PathInfo>& pi, bool trace_informers, bool sim_anneal, int radius, const RadiusGetter& get_radius);
    template <typename RadiusGetter>
    void path_informed_hbjpmda(int max_depth, bool allow_type_change, shared_ptr<SANode> start, Vector2i lv_end, unique_ptr<PathInfo>& pi, bool trace_informers, bool sim_anneal, int radius, const RadiusGetter& get_radius);
    int get_h_reduction(int virtual_path_index, bool sim_anneal);

    //move back to global scope once testing is done
    void rrd_clear_iad();
    void rrd_clear_cad();
};

typedef priority_queue<IADNode, vector<IADNode>, IADNodeComparer> open_iad_t;
typedef priority_queue<shared_ptr<CADNode>, vector<shared_ptr<CADNode>>, CADNodeComparer> open_cad_t;
typedef unordered_set<shared_ptr<CADNode>, CADNodeHasher, CADNodeEquator> closed_cad_t;
typedef unordered_map<Vector2i, int, Vector2iHasher> best_dist_t; //node_pos, best_g
typedef priority_queue<shared_ptr<SASearchNode>, vector<shared_ptr<SASearchNode>>, SASearchNodeBaseFComparer<SASearchNode>> open_sa_fsort_t;
typedef priority_queue<shared_ptr<SASearchNode>, vector<shared_ptr<SASearchNode>>, SASearchNodeBaseGComparer<SASearchNode>> open_sa_gsort_t;
typedef unordered_set<shared_ptr<SASearchNode>, SASearchNodeBaseHashGetter<SASearchNode>, SASearchNodeBaseEquator<SASearchNode>> closed_sa_t;
typedef priority_queue<shared_ptr<SAPISearchNode>, vector<shared_ptr<SAPISearchNode>>, SASearchNodeBaseFComparer<SAPISearchNode>> open_sapi_fsort_t;
typedef unordered_set<shared_ptr<SAPISearchNode>, SASearchNodeBaseHashGetter<SAPISearchNode>, SASearchNodeBaseEquator<SAPISearchNode>> closed_sapi_t;

struct RRDIADLists {
    open_iad_t open;
    best_dist_t closed;
    best_dist_t best_gs;
};

struct RRDCADLists {
    open_cad_t open;
    closed_cad_t closed;
    best_dist_t best_gs;
};

extern array<array<array<uint64_t, TILE_ID_COUNT - 1>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> tile_id_hash_keys;
extern array<array<array<uint64_t, TypeId::REGULAR>, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> type_id_hash_keys;
extern array<array<uint64_t, MAX_SEARCH_WIDTH>, MAX_SEARCH_HEIGHT> agent_pos_hash_keys;
extern TileMap* cells;
extern unordered_map<uint8_t, int> tile_push_limits; //type_id, tpl
//assume all entries are actively used
//closed is necessary to store guaranteed optimal results
extern unordered_map<Vector2i, RRDIADLists, Vector2iHasher> inconsistent_abstract_dists; //goal_pos, rrd lists
extern unordered_map<Vector2i, RRDCADLists, Vector2iHasher> consistent_abstract_dists; //goal_pos, rrd lists
extern array<double, SASearchId::SEARCH_END> sa_cumulative_search_times; //search_id, cumulative time (ms)

//templated implementations
#include "sanode.tpp"
#include "sa_search_node.tpp"
#include "pathfinder.tpp"

#endif