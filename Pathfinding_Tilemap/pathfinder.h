#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/tile_map.hpp>
#include <godot_cpp/classes/gd_script.hpp>
#include <unordered_map>
#include <unordered_set>
#include <queue>

namespace godot {

//each cell is represented as | BackId (8) | TypeId (3) | TileId (5) |
enum TileId {
    EMPTY = 0,
    ZERO = 16,
};

enum TypeId {
    PLAYER = 0,
    INVINCIBLE,
    HOSTILE,
    REGULAR,
    VOID,
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
	ASTAR,
	IDASTAR,
};

struct AbstractNode {
    Vector2i pos = Vector2i(0, 0);
    int g = std::numeric_limits<int>::max();

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
        size_t hash_x = std::hash<int>{}(v.x);
        size_t hash_y = std::hash<int>{}(v.y);
        return hash_x ^ (hash_y + 0x9e3779b9 + (hash_x << 6) + (hash_x >> 2));
    }
};

typedef std::priority_queue<AbstractNode, std::vector<AbstractNode>, AbstractNodeComparer> pq;
typedef std::unordered_map<Vector2i, int, Vector2iHasher> um;

const std::unordered_set<int> B_WALL_OR_BORDER = {BackId::BORDER_ROUND, BackId::BORDER_SQUARE, BackId::BLACK_WALL, BackId::BLUE_WALL, BackId::RED_WALL};
const std::unordered_set<int> B_SAVE_OR_GOAL = {BackId::SAVEPOINT, BackId::GOAL};
const std::unordered_set<int> T_ENEMY = {TypeId::INVINCIBLE, TypeId::HOSTILE, TypeId::VOID};
const std::vector<Vector2i> DIRECTIONS = {Vector2i(1, 0), Vector2i(0, 1), Vector2i(-1, 0), Vector2i(0, -1)};
const int TILE_POW_MAX = 14;
const int ABSTRACT_DIST_SIGN_CHANGE_PENALTY_FACTOR = 2;

class Pathfinder : public Node {
    GDCLASS(Pathfinder, Node);

private:
    Vector2i player_pos;
    Vector2i player_last_dir;
    TileMap* cells;
    GDScript* GV;
    std::unordered_map<Vector2i, std::pair<pq, um>, Vector2iHasher> abstract_dists; //goal_pos, [open, closed]; assume all entries are actively used

protected:
    static void _bind_methods();

public:
    Pathfinder();
    ~Pathfinder();
    Array pathfind(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_astar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_idastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);
    Array pathfind_ididjpastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end);

    void _ready() override;
    void set_player_pos(Vector2i pos);
    void set_player_last_dir(Vector2i dir);
    void set_tilemap(TileMap* t);

    bool is_immediately_trapped(Vector2i pos);
    bool can_move_in_dir(Vector2i pos, Vector2i dir);
    int get_type_id(Vector2i pos);
    int get_tile_id(Vector2i pos);
    int get_back_id(Vector2i pos);
    bool is_tile(Vector2i pos);
    bool is_compatible(int type_id, int back_id);

    void rrd_init(int agent_type_id, Vector2i goal_pos);
    int rrd_resume(int agent_type_id, Vector2i goal_pos, Vector2i node_pos);
    int abstract_distance(int src_tile_id, int dest_tile_id);
    int abstract_distance_util(int tile_id1, int tile_id2);
};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

}

#endif