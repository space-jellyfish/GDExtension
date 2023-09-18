/*
This code is distributed under the Influence Of My Intrusive Thoughts License. Please use it however you wish.
*/

#ifndef PATHFINDER
#define PATHFINDER

#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3i.hpp>
//#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <stack>
#include <random>
#include <functional>

using namespace godot;

enum StuffId {
	BORDER = -1,
	ZERO = 16, //pow offset
	NEG_ONE = 1,
	POS_ONE = 31,
	EMPTY = 0,
	MEMBRANE = 32,
	BLACK_WALL = 64,
	BLUE_WALL = 96,
	RED_WALL = 128,
	SAVEPOINT = 160,
	GOAL = 192,
};

enum ActionType {
	SLIDE = 0,
	SPLIT,
	//SHIFT,
	END
};

enum SearchType {
	JPASTAR = 0,
	IDASTAR,
	ASTAR,
	STRAIGHT,
	MERGE_GREEDY,
	MERGE_LARGE_TO_SMALL,
	MERGE_SMALL_TO_LARGE,
};

class Pathfinder : public Node {
	GDCLASS(Pathfinder, Node);

private:
	const int CELL_VALUE_COUNT = 131;
	const int TILE_VALUE_COUNT = 31;

	const char* level_hash_numbers_cstr = "level_hash_numbers";
	const char* x_hash_numbers_cstr = "x_hash_numbers";
	const char* y_hash_numbers_cstr = "y_hash_numbers";
	String level_hash_numbers_str = String(level_hash_numbers_cstr);
	String x_hash_numbers_str = String(x_hash_numbers_cstr);
	String y_hash_numbers_str = String(y_hash_numbers_cstr);
	StringName level_hash_numbers_strn = StringName(level_hash_numbers_cstr);
	StringName x_hash_numbers_strn = StringName(x_hash_numbers_cstr);
	StringName y_hash_numbers_strn = StringName(y_hash_numbers_cstr);
	NodePath level_hash_numbers_path = NodePath(level_hash_numbers_str);
	NodePath x_hash_numbers_path = NodePath(x_hash_numbers_str);
	NodePath y_hash_numbers_path = NodePath(y_hash_numbers_str);

	Variant gv;
	static std::vector<std::vector<std::vector<size_t>>> level_hash_numbers;
	static std::vector<size_t> x_hash_numbers;
	static std::vector<size_t> y_hash_numbers;

	int tile_pow_max;
	int max_depth;
	int tile_push_limit;
	bool is_player;

protected:
	static void _bind_methods();
	
public:
	//Pathfinder(Vector2i resolution_t);
    Array pathfind(int search_type, const Array& level, Vector2i start, Vector2i end);
	Array pathfind_jpastar(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	Array pathfind_idastar(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	Array pathfind_astar(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	Array pathfind_straight(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	Array pathfind_merge_greedy(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	Array pathfind_merge_lts(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	Array pathfind_merge_stl(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end);
	void try_action(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector3i action);
	void try_slide(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, std::function<bool(Vector2i)>& within_bounds);
	void try_split(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, std::function<bool(Vector2i)>& within_bounds);
	bool is_splittable(int val);
	bool is_enclosed(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, bool is_player);
	std::function<bool(Vector2i)> get_bound_checker(std::vector<std::vector<int>>& level, Vector2i dir);
	LevelStateBFS* jump(LevelStateBFS* state, Vector2i dir, Vector2i end, std::function<bool(Vector2i)>& within_bounds, bool can_split);
	int heuristic(Vector2i pos, Vector2i goal);
	void testing();

	int back_index(int cell_val);
	bool is_wall(int back_index);

	static void generate_hash_numbers(Vector2i resolution_t); //inits hash number arrays
	//void get_hash_arrays();
	size_t z_hash(const std::vector<std::vector<int>>& level, const Vector2i pos);
	void update_hash_pos(size_t& hash, Vector2i prev, Vector2i next);
	void update_hash_tile(size_t& hash, Vector2i pos, int tile_val);

	void set_gv(Variant _gv);
	Variant get_gv();
	void set_tile_pow_max(int _tile_pow_max);
	int get_tile_pow_max();
	void set_max_depth(int _max_depth);
	int get_max_depth();
	void set_tile_push_limit(int _tile_push_limit);
	int get_tile_push_limit();
	void set_is_player(bool _is_player);
	bool get_is_player();
};

struct LevelState {
	Vector2i pos = Vector2i(0, 0);
	Vector3i prev_action = Vector3i(0, 0, 0);
	//heuristics
	int g = std::numeric_limits<int>::max();
	int h = std::numeric_limits<int>::max();
	int f = std::numeric_limits<int>::max();
	//level
	std::vector<std::vector<int>> level;
	//hash (only encodes tile values and locations)
	size_t hash = 0;

	LevelState(Vector2i level_size);
	//void reserve_level(Vector2i level_size);
};

struct LevelStateBFS : LevelState {
	LevelStateBFS* prev = NULL;

	using LevelState::LevelState;
	//LevelStateBFS(Vector2i level_size);
	Array trace_path();
};

struct LevelStateDFS : LevelState {
	LevelStateDFS* prev = NULL;
	int child_count = 0;

	using LevelState::LevelState;
	//LevelStateDFS(Vector2i level_size);
};


//if f tied, use g; higher g indicates higher confidence
struct LevelStateComparer {
	bool operator() (LevelState* first, LevelState* second) {
		if (first->f > second->f) {
			return true;
		}
		if (first->f < second->f) {
			return false;
		}
		return first->g < second->g;
	}
};

struct LevelStateKeyHasher {
    std::size_t operator() (const std::pair<Vector2i, std::vector<std::vector<int>>>& state) const {
        std::size_t hash = 0;

		hash ^= std::hash<int>{}(state.first.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		hash ^= std::hash<int>{}(state.first.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);

        for (const std::vector<int>& row : state.second) {
            for (const int tile : row) {
                hash ^= std::hash<int>{}(tile) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            }
        }
        return hash;
    }
};

struct LevelStateKeyEquator {
	bool operator() (const std::pair<Vector2i, std::vector<std::vector<int>>>& state1, const std::pair<Vector2i, std::vector<std::vector<int>>>& state2) const {
		return state1 == state2; //this works
	}
};

struct LevelStateHashGetter {
	std::size_t operator() (const LevelState* state) const {
		return state->hash;
	}
};

struct LevelStateEquator {
	bool operator() (const LevelState* first, const LevelState* second) const {
		return (first->pos == second->pos && first->level == second->level);
	}
};

struct TileState {
	Vector2i pos = Vector2i(0, 0);
	int h = std::numeric_limits<int>::max(); //distance to end
};

struct TileStateComparer {
	bool operator() (TileState* first, TileState* second) {
		return first->h > second->h;
	}
};

#endif