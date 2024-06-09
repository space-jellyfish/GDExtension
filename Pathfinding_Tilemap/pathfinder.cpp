#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <cassert>
#include "pathfinder.h"

using namespace godot;

void Pathfinder::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_player_pos", "pos"), &Pathfinder::set_player_pos);
    ClassDB::bind_method(D_METHOD("set_player_last_dir", "dir"), &Pathfinder::set_player_last_dir);
	ClassDB::bind_method(D_METHOD("set_tilemap", "t"), &Pathfinder::set_tilemap);
    ClassDB::bind_method(D_METHOD("pathfind", "search_id", "max_depth", "min", "max", "start", "end"), &Pathfinder::pathfind);

    //testing
    ClassDB::bind_method(D_METHOD("rrd_init", "agent_type_id", "goal_pos"), &Pathfinder::rrd_init);
    ClassDB::bind_method(D_METHOD("rrd_resume", "agent_type_id", "goal_pos", "node_pos"), &Pathfinder::rrd_resume);
    ClassDB::bind_method(D_METHOD("abstract_distance", "src_tile_id", "dest_tile_id"), &Pathfinder::abstract_distance);
}

Pathfinder::Pathfinder() {
    printf("HELLO WORLD\n");
}

Pathfinder::~Pathfinder() {

}

Array Pathfinder::pathfind(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    switch (search_id) {
        case SearchId::DIJKSTRA:
            return pathfind_dijkstra(max_depth, min, max, start, end);
        case SearchId::ASTAR:
            return pathfind_astar(max_depth, min, max, start, end);
        case SearchId::IDASTAR:
            return pathfind_idastar(max_depth, min, max, start, end);
        default:
            return pathfind_idastar(max_depth, min, max, start, end);
    }
}

Array Pathfinder::pathfind_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    Array ans;
    return ans;
}

Array Pathfinder::pathfind_astar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    //if (dist to player > threshold) {
    //    astar using abstract dist
    //}
    //else {
    //    obstruct one of four player dirs or attempt merge with player
    //}
    Array ans;
    return ans;
}

Array Pathfinder::pathfind_idastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    Array ans;
    return ans;
}

Array Pathfinder::pathfind_ididjpastar(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    int tile_depth = 0;
    int search_depth = 0;

    Array ans;
    return ans;
}

void Pathfinder::_ready() {
    UtilityFunctions::print("Hello World from GDExtension\n");
    GV = get_node<GDScript>("/root/GV");
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

bool Pathfinder::is_immediately_trapped(Vector2i pos) {
    return false;
}

bool Pathfinder::can_move_in_dir(Vector2i pos, Vector2i dir) {
    return false;
}

int Pathfinder::get_type_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).y;
}

int Pathfinder::get_tile_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).x + 1;
}

int Pathfinder::get_back_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::BACK, pos).x;
}

bool Pathfinder::is_tile(Vector2i pos) {
    return get_tile_id(pos) != 0;
}

bool Pathfinder::is_compatible(int type_id, int back_id) {
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

void Pathfinder::rrd_init(int agent_type_id, Vector2i goal_pos) {
    if (abstract_dists.contains(goal_pos)) {
        return;
    }

    pq open;
    um closed;
    std::pair<pq, um> lists {open, closed};

    if (is_compatible(agent_type_id, get_back_id(goal_pos))) {
        AbstractNode n(goal_pos, 0);
        open.push(n);
    }

    abstract_dists[goal_pos] = lists;
}

int Pathfinder::rrd_resume(int agent_type_id, Vector2i goal_pos, Vector2i node_pos) {
    assert(abstract_dists.contains(goal_pos));

    if (!is_compatible(agent_type_id, get_back_id(node_pos)) || !is_compatible(agent_type_id, get_back_id(goal_pos))) {
        return std::numeric_limits<int>::max(); //unreachable
    }

    std::pair<pq, um>& lists = abstract_dists[goal_pos];
    pq& open = lists.first;
    um& closed = lists.second;

    auto itr = closed.find(node_pos);
    if (itr != closed.end()) {
        return (*itr).second;
    }

    while (!open.empty()) {
        AbstractNode n = open.top();
        if (n.pos == node_pos) {
            //found optimal abs_dist at node
            //no need to backtrack so parents aren't stored
            //don't pop n so search is resumable
            return n.g;
        }

        open.pop();
        if (closed.contains(n.pos)) { //remove duplicates in pq
            continue;
        }
        closed[n.pos] = n.g;

        int curr_tile_id = get_tile_id(n.pos);
        for (Vector2i dir : DIRECTIONS) {
            Vector2i next_pos = n.pos + dir;

            if (closed.contains(next_pos)) { //closed is all optimal
                continue;
            }
            int next_tile_id = get_tile_id(next_pos);
            int next_g = n.g + abstract_distance(curr_tile_id, next_tile_id);
            AbstractNode m(next_pos, next_g);
            open.push(m);
        }
    }
    return std::numeric_limits<int>::max(); //unreachable
}

int Pathfinder::abstract_distance(int src_tile_id, int dest_tile_id) {
    if (dest_tile_id % TileId::ZERO == 0) {
        return 1;
    }
    if (src_tile_id % TileId::ZERO == 0) {
        return abstract_distance_util(0, dest_tile_id) - 1;
    }
    if (abs(dest_tile_id) == TILE_POW_MAX) {
        return std::min(abstract_distance_util(src_tile_id, 0), abstract_distance_util(src_tile_id, -dest_tile_id));
    }
    return abstract_distance_util(src_tile_id, dest_tile_id);
}

int Pathfinder::abstract_distance_util(int tile_id1, int tile_id2) {
    int ans = abs(tile_id1 - tile_id2) * 2;
    int sgn_change_penalty = abs(sgn(tile_id1) - sgn(tile_id2)) * ABSTRACT_DIST_SIGN_CHANGE_PENALTY_FACTOR;
    return ans + sgn_change_penalty;
}