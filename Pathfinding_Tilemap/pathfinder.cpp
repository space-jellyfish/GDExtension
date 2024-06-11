#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <cassert>
#include <memory>
#include <random>
#include "pathfinder.h"

using namespace std;
using namespace godot;

void Pathfinder::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_player_pos", "pos"), &Pathfinder::set_player_pos);
    ClassDB::bind_method(D_METHOD("set_player_last_dir", "dir"), &Pathfinder::set_player_last_dir);
	ClassDB::bind_method(D_METHOD("set_tilemap", "t"), &Pathfinder::set_tilemap);
    ClassDB::bind_method(D_METHOD("pathfind", "search_id", "max_depth", "min", "max", "start", "end"), &Pathfinder::pathfind);

    //testing
    ClassDB::bind_method(D_METHOD("rrd_init", "agent_type_id", "goal_pos"), &Pathfinder::rrd_init);
    ClassDB::bind_method(D_METHOD("rrd_resume", "agent_type_id", "goal_pos", "node_pos"), &Pathfinder::rrd_resume);
}

Array SANode::trace_path() {
	Array ans;
	ans.resize(g);
	int index = g - 1;
	shared_ptr<SANode> curr = shared_from_this();

	while (curr->prev != nullptr) {
		ans[index] = curr->prev_action;
		curr = curr->prev;
		--index;
	}
	//UtilityFunctions::print("PF TRACED PATH SIZE: ", ans.size());
	return ans;
}

Pathfinder::Pathfinder() {

}

Pathfinder::~Pathfinder() {

}

Array Pathfinder::pathfind(int search_id, int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    switch (search_id) {
        case SearchId::DIJKSTRA:
            return pathfind_sa_dijkstra(max_depth, min, max, start, end);
        case SearchId::ASTAR:
            return pathfind_sa_astar(max_depth, min, max, start, end);
        case SearchId::IDASTAR:
            return pathfind_sa_idastar(max_depth, min, max, start, end);
        default:
            return pathfind_sa_idastar(max_depth, min, max, start, end);
    }
}

//assume no type_id change
Array Pathfinder::pathfind_sa_dijkstra(int max_depth, Vector2i min, Vector2i max, Vector2i start, Vector2i end) {
    priority_queue<shared_ptr<SANode>, vector<shared_ptr<SANode>>, SANodeComparer> open;
    unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
    int agent_type_id = get_type_id(start);

    shared_ptr<SANode> first = make_shared<SANode>();
    first->lv_pos = start - min;
    init_sanode_lv(first, min, max);
    first->hash = get_z_hash(min, max, start);
    open.push(first);

    while (!open.empty()) {
        shared_ptr<SANode> curr = open.top();
        open.pop();

        if (curr->lv_pos + min == end) {
            return curr->trace_path();
        }

        if (curr->f > max_depth) {
            return Array();
        }

        for (Vector2i dir : DIRECTIONS) {

        }
    }
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

shared_ptr<SANode> Pathfinder::try_action(shared_ptr<SANode> n, Vector3i action) {
    Vector2i dir(action.x, action.y);

    switch(action.z) {
        case ActionId::SLIDE:
            return try_slide(n, dir);
        case ActionId::SPLIT:
            return try_split(n, dir);
        default:
            return try_slide(n, dir);
    }
}
/*
func try_slide(pos_t:Vector2i, dir:Vector2i) -> bool:
	var push_count:int = get_slide_push_count(pos_t, dir);
	if push_count != -1:
		perform_slide(pos_t, dir, push_count);
		set_player_pos_t(player_pos_t + dir);
		if get_type_id(player_pos_t) != GV.TypeId.PLAYER:
			is_player_alive = false;
		return true;
	return false;
*/
shared_ptr<SANode> Pathfinder::try_slide(shared_ptr<SANode> n, Vector2i dir) {
    int push_count = get_slide_push_count(n, dir);
    if (push_count != -1) {

    }
}

shared_ptr<SANode> Pathfinder::try_split(shared_ptr<SANode> n, Vector2i dir) {

}

//longest distance from n->lv_pos in dir without leaving lv
int Pathfinder::get_dist_to_lv_edge(shared_ptr<SANode> n, Vector2i dir) {
    if (dir == DIRECTIONS[0]) {
        return n->lv[0].size() - 1 - n->lv_pos.x;
    }
    if (dir == DIRECTIONS[1]) {
        return n->lv.size() - 1 - n->lv_pos.y;
    }
    if (dir == DIRECTIONS[2]) {
        return n->lv_pos.x;
    }
    return n->lv_pos.y;
}

//ignore tiles outside of n->lv
int Pathfinder::get_slide_push_count(shared_ptr<SANode> n, Vector2i dir) {
    Vector2i curr_lv_pos = n->lv_pos;
    int curr_tile_id = get_tile_id(n, curr_lv_pos);
    int src_type_id = get_type_id(n, curr_lv_pos);
    int curr_type_id = src_type_id;
    int push_count = 0;
    int nearest_merge_push_count = -1;
    int dist_to_lv_edge = get_dist_to_lv_edge(n, dir);

    while (push_count <= tile_push_limit) {
        //obstruction
        if (push_count >= dist_to_lv_edge) {
            break;
        }

        int temp_type_id = curr_type_id;
        curr_lv_pos += dir;
        curr_type_id = get_type_id(n, curr_lv_pos);
        int curr_back_id = get_back_id(n, curr_lv_pos);
        if (!is_compatible(temp_type_id, curr_back_id) || (push_count > 0 && T_ENEMY.find(src_type_id) != T_ENEMY.end() && curr_type_id == TypeId::PLAYER)) {
            return nearest_merge_push_count;
        }

        //push/merge logic
        int temp_tile_id = curr_tile_id;
        curr_tile_id = get_tile_id(n, curr_lv_pos);

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

shared_ptr<SANode> Pathfinder::perform_slide(shared_ptr<SANode> n, Vector2i dir, int push_count) {
    Vector2i merge_lv_pos = n->lv_pos + (push_count + 1) * dir;

    for (int dist_to_merge=0; dist_to_merge <= push_count; ++dist_to_merge) {
        Vector2i curr_lv_pos = merge_lv_pos - dist_to_merge * dir;
        int prev_sid = get_stuff_id(n, curr_lv_pos - dir);
        int result_sid = prev_sid;
        if (dist_to_merge == 0) {
            int curr_sid = get_stuff_id(n, curr_lv_pos);
            result_sid = get_merged_stuff_id(prev_sid, curr_sid);
        }
        n->lv[curr_lv_pos.y][curr_lv_pos.x] = result_sid;
    }

    //remove src tile
    cells->set_cell(LayerId::TILE,)
}

/*
func perform_slide(pos_t:Vector2i, dir:Vector2i, push_count:int):
	var merge_pos_t:Vector2i = pos_t + (push_count + 1) * dir;
	for dist_to_merge_pos_t in range(0, push_count + 1):
		var curr_pos_t:Vector2i = merge_pos_t - dist_to_merge_pos_t * dir;
		var prev_coord:Vector2i = $Cells.get_cell_atlas_coords(GV.LayerId.TILE, curr_pos_t - dir);
		var result_coord = prev_coord;
		if dist_to_merge_pos_t == 0: #merge at curr_pos_t
			var curr_coord:Vector2i = $Cells.get_cell_atlas_coords(GV.LayerId.TILE, curr_pos_t);
			result_coord = get_merged_atlas_coords(prev_coord, curr_coord);
		$Cells.set_cell(GV.LayerId.TILE, curr_pos_t, GV.LayerId.TILE, result_coord);
	
	#remove source tile
	$Cells.set_cell(GV.LayerId.TILE, pos_t, GV.LayerId.TILE, -Vector2i.ONE);
*/

void Pathfinder::_ready() {
    UtilityFunctions::print("Hello World from GDExtension\n");
    GV = get_node<GDScript>("/root/GV");
    generate_hash_keys();
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

void Pathfinder::init_sanode_lv(shared_ptr<SANode> n, Vector2i min, Vector2i max)  {
    int height = max.y - min.y;
    int width = max.x - min.x;
    n->lv.reserve(height);

    for (int y = min.y; y < max.y; ++y) {
        vector<int> row;
        row.reserve(width);

        for (int x = min.x; x < max.x; ++x) {
            Vector2i pos(x, y);
            row.push_back(get_stuff_id(pos));
        }
        n->lv.push_back(row);
    }
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

size_t Pathfinder::get_z_hash(Vector2i min, Vector2i max, Vector2i start) {
    size_t hash = 0;
    for (int y=min.y; y < max.y; ++y) {
        for (int x=min.x; x < max.x; ++x) {
            int tile_id = get_tile_id(Vector2i(x, y));
            if (tile_id > 0) {
                hash ^= tile_id_hash_keys[y][x][tile_id-1];
            }

            int type_id = get_type_id(Vector2i(x, y));
            if (type_id < TypeId::REGULAR) {
                hash ^= type_id_hash_keys[y][x][type_id];
            }
        }
    }
    hash ^= agent_pos_hash_keys[start.y][start.x];
    return hash;
}

bool Pathfinder::is_immediately_trapped(Vector2i pos) {
    return false;
}

int Pathfinder::get_stuff_id(Vector2i pos) {
    return (get_back_id(pos) << 8) + (get_type_id(pos) << 5) + get_tile_id(pos);
}

int Pathfinder::get_tile_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).x + 1;
}

int Pathfinder::get_type_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::TILE, pos).y;
}

int Pathfinder::get_back_id(Vector2i pos) {
    return cells->get_cell_atlas_coords(LayerId::BACK, pos).x;
}

int Pathfinder::get_stuff_id(shared_ptr<SANode> n, Vector2i lv_pos) {
    return n->lv[lv_pos.y][lv_pos.x];
}

int Pathfinder::get_tile_id(shared_ptr<SANode> n, Vector2i lv_pos) {
    return get_stuff_id(n, lv_pos) & TILE_ID_MASK;
}

int Pathfinder::get_type_id(shared_ptr<SANode> n, Vector2i lv_pos) {
    return (get_stuff_id(n, lv_pos) & TYPE_ID_MASK) >> TILE_ID_BITLEN;
}

int Pathfinder::get_back_id(shared_ptr<SANode> n, Vector2i lv_pos) {
    return (get_stuff_id(n, lv_pos) & BACK_ID_MASK) >> (TILE_ID_BITLEN + TYPE_ID_BITLEN);
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

bool Pathfinder::is_ids_mergeable(int tile_id1, int tile_id2) {
    if (tile_id1 == 0 || tile_id2 == 0) {
        return true;
    }
    Vector2i ps1 = tid_to_ps(tile_id1);
    Vector2i ps2 = tid_to_ps(tile_id2);
    return is_pow_signs_mergeable(ps1, ps2);
}

bool Pathfinder::is_pow_signs_mergeable(Vector2i ps1, Vector2i ps2) {
    if (ps1.x == -1 || ps2.x == -1) {
        return true;
    }
    if (ps1.x == ps2.x && (ps1.x < TILE_POW_MAX || ps1.y != ps2.y)) {
        return true;
    }
    return false;
}

Vector2i Pathfinder::tid_to_ps(int tile_id) {
    assert(tile_id != TileId::EMPTY);
    if (tile_id == TileId::ZERO) {
        return Vector2i(-1, 1);
    }
    int signed_incremented_pow = tile_id - TileId::ZERO;
    return Vector2i(abs(signed_incremented_pow) - 1, sgn(signed_incremented_pow));
}

int Pathfinder::ps_to_tid(Vector2i ps) {
    return (ps.x + 1) * ps.y + TileId::ZERO;
}

//assumes ids are mergeable
int Pathfinder::get_merged_stuff_id(int src_stuff_id, int dest_stuff_id) {
    
}

//assumes ids are mergeable
//doesn't use pow_sign since it cannot represent TileId::EMPTY
int Pathfinder::get_merged_tile_id(int tile_id1, int tile_id2) {
    if (tile_id1 == TileId::EMPTY || tile_id1 == TileId::ZERO) {
        return tile_id2;
    }
    if (tile_id2 == TileId::EMPTY || tile_id2 == TileId::ZERO) {
        return tile_id1;
    }

    int sgn1 = sgn(tile_id1 - TileId::ZERO);
    if (sgn1 != sgn(tile_id2 - TileId::ZERO)) {
        return TileId::ZERO;
    }
    return tile_id1 + sgn1;
}

//assumes vals are mergeable
Vector2i Pathfinder::get_merged_pow_sign(Vector2i ps1, Vector2i ps2) {
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

void Pathfinder::rrd_init(int agent_type_id, Vector2i goal_pos) {
    if (abstract_dists.contains(goal_pos)) {
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

        if (closed.contains(n.pos)) { //don't push duplicates to pq
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

            if (closed.contains(next_pos)) { //closed is all optimal
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
    assert(abstract_dists.contains(goal_pos));

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