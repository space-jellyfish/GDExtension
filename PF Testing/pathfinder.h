#ifndef PATHFINDER
#define PATHFINDER

#include <vector>
#include <deque>
#include <string>
#include <memory>

void print_vector_2d(std::vector<std::vector<int>>& v);
void print_vector_2d(std::vector<std::vector<int>>& v, int x0, int y0, int x1, int y1);
void print_dir(std::tuple<int, int> dir, std::string extra);
void print_pos(std::tuple<int, int> dir, std::string extra);
void print_action(std::tuple<int, int, int> action, std::string extra);
void print_path(std::vector<std::tuple<int, int, int>> path);
std::tuple<int, int> add(std::tuple<int, int> a, std::tuple<int, int> b);
std::tuple<int, int> sub(std::tuple<int, int> a, std::tuple<int, int> b);


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
	END
};

enum SearchType {
	IDASTAR = 0,
	ASTAR,
	STRAIGHT,
	MERGE_GREEDY,
	MERGE_LARGE_TO_SMALL,
	MERGE_SMALL_TO_LARGE,
};

class Pathfinder {
	
public:
    std::vector<std::tuple<int, int, int>> pathfind(int search_type, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::tuple<int, int, int>> pathfind_idastar(const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::tuple<int, int, int>> pathfind_astar(const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::tuple<int, int, int>> pathfind_straight(const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::tuple<int, int, int>> pathfind_merge_greedy(const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::tuple<int, int, int>> pathfind_merge_lts(const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::tuple<int, int, int>> pathfind_merge_stl(const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::vector<int>> try_action(std::vector<std::vector<int>> level, std::tuple<int, int> pos, std::tuple<int, int, int> action, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::vector<int>> try_slide(std::vector<std::vector<int>>& level, std::tuple<int, int> pos, std::tuple<int, int> dir, int tile_push_limit, bool is_player, int tile_pow_max);
	std::vector<std::vector<int>> try_split(std::vector<std::vector<int>>& level, std::tuple<int, int> pos, std::tuple<int, int> dir, int tile_push_limit, bool is_player, int tile_pow_max);
	int heuristic(std::tuple<int, int> pos, std::tuple<int, int> goal);
};

struct LevelState {
	std::tuple<int, int> pos {0, 0};
	std::tuple<int, int, int> prev_action {0, 0, 0};
	//heuristics
	int g = std::numeric_limits<int>::max();
	int h = std::numeric_limits<int>::max();
	int f = std::numeric_limits<int>::max();
	//level
	std::vector<std::vector<int>> level;

	LevelState(std::tuple<int, int> level_size);
};

struct LevelStateBFS : LevelState {
	LevelStateBFS* prev = NULL;

	using LevelState::LevelState;
	std::vector<std::tuple<int, int, int>> trace_path();
};

struct LevelStateDFS : LevelState {
	//std::shared_ptr<LevelStateDFS*> prev = NULL;
	LevelStateDFS* prev = NULL;
	int child_count = 0;

	using LevelState::LevelState;
};

//use f heuristic for sorting priority queue
struct LevelStateComparer {
	bool operator() (LevelState* first, LevelState* second) {
		if (first->f > second->f) {
			return true;
		}
		if (first->f < second->f) {
			return false;
		}
		return first->g < second->g; //prioritize larger g
	}
};

struct LevelStateHasher {
    std::size_t operator() (const std::pair<std::tuple<int, int>, std::vector<std::vector<int>>>& state) const {
        std::size_t hash = 0;

		hash ^= std::hash<int>{}(std::get<0>(state.first)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		hash ^= std::hash<int>{}(std::get<1>(state.first)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);

        for (const std::vector<int>& row : state.second) {
            for (const int tile : row) {
                hash ^= std::hash<int>{}(tile) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            }
        }
        return hash;
    }
};

struct LevelStateEquator {
	bool operator() (const std::pair<std::tuple<int, int>, std::vector<std::vector<int>>>& state1, const std::pair<std::tuple<int, int>, std::vector<std::vector<int>>>& state2) const {
		return state1 == state2;
	}
};

#endif