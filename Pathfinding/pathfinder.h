#ifndef PATHFINDER
#define PATHFINDER

#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3i.hpp>
//#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

enum StuffId {
	RED_WALL = -3,
	BLUE_WALL = -2,
	BLACK_WALL = -1,
	ZERO = 1,
	NEG_ONE = 31,
	POW_OFFSET = 16,
	EMPTY = 0,
	MEMBRANE = 32,
	SAVEPOINT = 64,
	GOAL = 96,
};

enum ActionType {
	SLIDE = 0,
	SPLIT,
	SHIFT,
	END
};

enum SearchType {
	PERFECT = 0,
	STRAIGHT,
	MERGE_GREEDY,
	MERGE_LARGE_TO_SMALL,
	MERGE_SMALL_TO_LARGE,
};

class Pathfinder : public Node {
	GDCLASS(Pathfinder, Node);
	
public:
    Array pathfind(int search_type, const Array& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max);
	Array pathfind_astar(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max);
	Array pathfind_straight(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max);
	Array pathfind_merge_greedy(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max);
	Array pathfind_merge_lts(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max);
	Array pathfind_merge_stl(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::vector<int>> try_action(std::vector<std::vector<int>> level, Vector2i pos, Vector3i action, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::vector<int>> try_slide(std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::vector<int>> try_split(std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, int tile_push_limit, bool is_player, int tile_pow_max);
	bool is_enclosed(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, bool is_player);
	int heuristic(Vector2i pos, Vector2i goal);
	void testing();

protected:
	static void _bind_methods();
};

struct LevelState {
	Vector2i pos = Vector2i(0, 0); //of tile
	LevelState* prev = NULL;
	Vector3i prev_action = Vector3i(0, 0, 0);

	//heuristics
	int g = std::numeric_limits<int>::max();
	int h = std::numeric_limits<int>::max();
	int f = std::numeric_limits<int>::max();

	std::vector<std::vector<int>> level;


	LevelState(Vector2i level_size);
	Array trace_path();
};

//use f heuristic for sorting priority queue
struct LevelStateComparer {
	bool operator() (LevelState* first, LevelState* second) {
		return first->f > second->f;
	}
};

struct LevelStateHasher {
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

struct LevelStateEquator {
	bool operator() (const std::pair<Vector2i, std::vector<std::vector<int>>>& state1, const std::pair<Vector2i, std::vector<std::vector<int>>>& state2) const {
		return state1 == state2; //this works
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