/* deprecated, see IDIDA*

//constexpr std::vector<float> TILE_ID_DISTRIBUTION;
const int MIN_ABSTRACT_DIST_TO_ZERO = 8;

int abstract_dist_to_zero(int tile_id) {
    return MIN_ABSTRACT_DIST_TO_ZERO + abstract_dist_from_zero(tile_id);
}

int abstract_dist_from_zero(int tile_id) {
    int absolute_dist_from_zero = abs(tile_id - TileId::ZERO);
    return absolute_dist_from_zero < 2 ? 2 : ans;
}
*/

/*
size_t get_z_hash(Vector2i min, Vector2i max, Vector2i start);

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
*/

/*
void _ready() override;

void Pathfinder::_ready() {
    UtilityFunctions::print("Hello World from GDExtension\n");
    //GV = get_node<GDScript>("/root/GV");
    generate_hash_keys();
}

Pathfinder::Pathfinder() {
    PF = this;
}

Pathfinder::~Pathfinder() {

}
*/

/* wrong and hasty conclusions not backed by casework
                //if -dir and slide and prev effective push count is 0, skip
                //if -dir and split and prev was effective merge, skip
*/

/* in perform_slide(), slightly faster but less comprehensible way of getting prev_merged_ssr
    prev_merged_ssr = !is_tile_unsigned(src_tile_id) && push_count == 0 && src_tile_id == neighbor_tile_id && neighbor_type_id == TypeId::REGULAR;
    //TILE_POW_MAX check is unnecessary bc push_count == 0
    //!is_tile_unsigned(src_tile_id) is technically required but logically unnecessary bc zero cannot split anyways
*/

/* old incomplete jump stuff
        if (dir1 != Vector2i(0, 0)) {
            Vector2i pos1 = curr_pos + dir1;
            uint16_t stuff_id1 = get_lv_sid(pos1);
            if (horizontal) {
                bool dir1_blocked = !is_tile_empty_and_regular(stuff_id1);
                if (dir1_blocked && is_tile_unsigned_and_regular(stuff_id1)) {
                    return curr_jp;
                }
            }
            else {
                shared_ptr<SANode> secondary_jp = curr_jp->jump(dir1, lv_end, allow_type_change);
                if (secondary_jp) {
                    curr_jp->jump_points[dir1] = secondary_jp;
                    return curr_jp;
                }
            }


            if (!horizontal && curr_jp->jump(dir1, lv_end, allow_type_change)) {
                return curr_jp;
            }
            else if (is_tile_unsigned_and_regular(stuff_id1)) {
                return curr_jp;
            }
            uint8_t tile_id1 = get_tile_id(stuff_id1);
            uint8_t type_id1 = get_type_id(stuff_id1);
            //or src_tile_id splittable and get_splitted_tid(src_tile_id) is mergeable
            if (is_ids_mergeable(src_tile_id, tile_id1) && is_type_preserved(agent_type_id, type_id1)) {
                return curr_jp;
            }
        }
        if (dir2 != Vector2i(0, 0)) {
            Vector2i pos2 = curr_pos + dir2;
            uint16_t stuff_id2 = get_lv_sid(pos2);
            if (!horizontal && curr_jp->jump(dir2, lv_end, allow_type_change)) {
                return curr_jp;
            }
            else if (is_tile_unsigned_and_regular(stuff_id2)) {
                return curr_jp;
            }
            uint8_t tile_id2 = get_tile_id(stuff_id2);
            uint8_t type_id2 = get_type_id(stuff_id2);
            if (is_ids_mergeable(src_tile_id, tile_id2) && is_type_preserved(agent_type_id, type_id2)) {
                return curr_jp;
            }
        }
        if (dist < dist_to_lv_edge) {
            Vector2i next_pos = curr_pos + dir;
            uint16_t next_stuff_id = get_lv_sid(next_pos);
            uint8_t next_tile_id = get_tile_id(next_stuff_id);
            uint8_t next_type_id = get_type_id(next_stuff_id);
            if (!is_tile_unsigned_and_regular(next_stuff_id) && is_ids_mergeable(src_tile_id, next_tile_id) && is_type_preserved(agent_type_id, next_type_id)) {
                return curr_jp;
            }
        }
*/

/* deprecated; jump() should not push/pop zeros

//zero_count is number of zeros (all regular) from lv_pos+dir to jp_pos inclusive
int zero_count = 0; //number of regular zeros from lv_pos+dir to jp_pos inclusive

    //add pushed zeros ahead of jp
    int dist_to_lv_edge = ans->get_dist_to_lv_edge(dir);
    int dist = 1;
    Vector2i curr_pos = jp_pos + dir;
    int max_dist = min(dist_to_lv_edge, zero_count, tile_push_limit);
    while (dist <= max_dist) {
        uint16_t curr_stuff_id = ans->get_lv_sid(curr_pos);
        uint8_t curr_back_id = get_back_id(curr_stuff_id);
        if (!is_compatible(TypeId::REGULAR, curr_back_id)) {
            break;
        }
        uint8_t curr_type_id = get_type_id(curr_stuff_id);

        if (curr_type_id == TypeId::REGULAR) {
            uint8_t curr_tile_id = get_tile_id(curr_stuff_id);

            if (curr_tile_id == TileId::EMPTY) {
                ans->set_lv_sid(curr_pos, ans->get_lv_sid(curr_pos) + TileId::ZERO);
            }
            else if (curr_tile_id == TileId::ZERO) {
                ++zero_count;
                max_dist = min(dist_to_lv_edge, zero_count, tile_push_limit);
            }
            else {
                break;
            }
        }
        else {
            break;
        }
        ++dist;
        curr_pos += dir;
    }
*/

/* more deprecated jump() stuff
        for (auto& [perp_dir, in_bounds, blocked] : perp_dirs) {
            if (horizontal) {
                if (!blocked) {
                    curr_jp->neighbors[Vector3i(perp_dir.x, perp_dir.y, ActionId::JUMP)] = nullptr;
                }
                if (perp_empty && blocked) {
                    //reverse SLIDE is pruned during node expansion
                    neighbors[action] = curr_jp;
                    return curr_jp;
                }
                shared_ptr<SANode> slide_neighbor = curr_jp->try_action(action_slide, allow_type_change);
                if (slide_neighbor && !slide_neighbor->prev_push_count && blocked && (allow_type_change || is_type_preserved(src_type_id, perp_type_id))) { //slide mergeable
                    neighbors[action] = curr_jp;
                    return curr_jp;
                }
                shared_ptr<SANode> 

                if (is_tile_empty_and_regular(perp_stuff_id) && blocked) {
                    return curr_jp;
                }
                bool slide_mergeable = is_ids_mergeable(src_tile_id, perp_tile_id);
                bool split_mergeable = is_ids_split_mergeable(src_tile_id, perp_tile_id);
                if ((perp_tile_id == TileId::ZERO || slide_mergeable || split_mergeable) && (blocked || split_mergeable) && (allow_type_change || is_type_preserved(src_type_id, perp_type_id))) {
                    return curr_jp;
                }
            }
            else { //vertical
                if (is_tile_unsigned_and_regular(perp_stuff_id)) {
                    shared_ptr<SANode> secondary_jp = curr_jp->jump(perp_dir, lv_end, allow_type_change);
                    if (secondary_jp) {
                        curr_jp->neighbors[{perp_dir, ActionId::JUMP}] = secondary_jp;
                        return curr_jp;
                    }
                    continue;
                }
                bool slide_mergeable = is_ids_mergeable(src_tile_id, perp_tile_id);
                bool split_mergeable = is_ids_split_mergeable(src_tile_id, perp_tile_id);
                if ((perp_tile_id == TileId::ZERO || slide_mergeable || split_mergeable) && (allow_type_change || is_type_preserved(src_type_id, perp_type_id))) {
                    return curr_jp;
                }
            }
        }

        //next_dir
        //bound check
        if (dist == dist_to_lv_edge) {
            break;
        }
        //compatibility check
        Vector2i next_pos = curr_pos + dir;
        uint16_t next_stuff_id = get_lv_sid(next_pos);
        uint8_t next_back_id = get_back_id(next_stuff_id);
        if (!is_compatible(src_type_id, next_back_id)) {
            break;
        }
        //jump logic (same for both vertical/horizontal)
        if (!is_tile_empty_and_regular(next_stuff_id)) { //either jp at curr_pos or nullptr
            uint8_t next_type_id = get_type_id(next_stuff_id);
            uint8_t next_tile_id = get_tile_id(next_stuff_id);
            bool slide_mergeable = is_ids_mergeable(src_tile_id, next_tile_id);
            bool split_mergeable = is_ids_split_mergeable(src_tile_id, next_tile_id);
            if ((next_tile_id == TileId::ZERO || slide_mergeable || split_mergeable) && (allow_type_change || is_type_preserved(src_type_id, next_type_id))) {
                return curr_jp;
            }
            return nullptr;
        }
*/

/*
//DEPRECATED: requires weak_ptrs for pruned SANodes to be set correctly, otherwise some nodes might not get unpruned; but are they?
clear_neighbor_prunes()

//store weak_ptr even if pruned to determine whether or not to unprune? NAH
SANode::neighbors
*/

/* closed stuff
unordered_set<shared_ptr<SANode>, SANodeHashGetter, SANodeEquator> closed;
//closed contains neighbor -> neighbor has been generated with optimal dist -> neighbor is pruned by best_dist check

from mda():
                //if closed check returns positive, neighbor heuristic calculation is skipped; worth it? idk
                if (closed.find(neighbor) != closed.end()) {
                    continue;
                }
from hbjpmda():
                //closed check to skip heuristic calculation
                if (closed.find(neighbor) != closed.end()) {
                    continue;
                }
*/

/* old best_dists stuff
                        //path is strictly better, update heuristics and prev stuff and neighbors and repush to open
                        (*it).first->f = neighbor->f;
                        //since curr->f <= (*it).first->f, (*it).first cannot be ancestor of curr, and updating prev won't create loop
                        (*it).first->prev = curr;
                        (*it).first->prev_actions = neighbor->prev_actions;
                        (*it).first->prev_push_count = neighbor->prev_push_count;
                        //note this breaks weak_ptrs to neighbor
                        //there are no shared_ptrs to neighbor yet, so this is fine
                        neighbor = (*it).first;

                        (*it).first->g = neighbor->g;
                        (*it).first->h = neighbor->h;
                        (*it).first->f = neighbor->f;
                        (*it).first->prev = curr;
                        (*it).first->prev_actions = neighbor->prev_actions;
                        (*it).first->prev_push_count = neighbor->prev_push_count;
                        neighbor = (*it).first;
*/

/*
void clear_neighbor_prunes(unsigned int dist_improvement);

//idea: unprune neighbor if neighbor is valid and path to neighbor via curr is strictly better
//unnecessary for dijkstra bc its open is optimal
//prunes will be restored upon curr->try_action()
//if open contains multiple pointers to curr, only the one with best dist can make use of the unprune
void SANode::clear_neighbor_prunes(unsigned int dist_improvement) {
    for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        if (dist_improvement >= (*it).second.first) {
            (*it).second.first = 0;
        }
    }
}
*/

/*from try_action():
        if (action.z != ActionId::JUMP) {
            //update prev/prev_actions
            ans->prev = shared_from_this();
            ans->prev_actions = {action};
        }
*/

/*cost calculation before transfer_neighbors()

dijkstra:
hbjpd:
neighbor->f += neighbor->prev_actions.size();
mda:
++(neighbor->g);
hbjpmda:
neighbor->g += neighbor->prev_actions.size();
*/

/*sanode_ref_pool stuff replaced by separating SANode and SASearchNode
from try_action():
//IMPORTANT: bc neighbors uses weak_ptr, stored result disappears when the returned shared_ptr<SANode> goes out of scope

from header:
//every search type can create neighbor loops (see Pictures/dijkstra_neighbor_loop), so shared_ptr doesn't work
//to ensure weak_ptrs remain valid for the duration of the sa_search, sanode_ref_pool is used

extern vector<shared_ptr<SANode>> sanode_ref_pool; //to store strong refs so SANode weak_ptrs remain valid; remember to clear when done pathfind_sa
vector<shared_ptr<SANode>> sanode_ref_pool; //to store strong refs so SANode weak_ptrs remain valid; remember to clear when done pathfind_sa

from try_jump():
                        //preserve stored neighbor from curr_jp->try_action
                        sanode_ref_pool.push_back(neighbor);
from pathfind_sa():
    //clear sanode_ref_pool
    sanode_ref_pool.clear();
from pathfind_sa_iada() generating best_dists check:
    sanode_ref_pool.push_back(neighbor); //preserve neighbor ref in case curr generates again
from pathfind_sa_hbjpiada() generating best_dists check:
    sanode_ref_pool.push_back(neighbor); //preserve neighbor ref in case curr generates again
*/

/*
    ClassDB::bind_method(D_METHOD("rrd_init_iad", "goal_pos"), &Pathfinder::rrd_init_iad);
    ClassDB::bind_method(D_METHOD("rrd_init_cad", "goal_pos"), &Pathfinder::rrd_init_cad);
    ClassDB::bind_method(D_METHOD("rrd_resume_iad", "goal_pos", "node_pos", "agent_type_id"), &Pathfinder::rrd_resume_iad);
    ClassDB::bind_method(D_METHOD("rrd_resume_cad", "goal_pos", "agent_pos"), &Pathfinder::rrd_resume_cad);
*/

/* consistent abstract distance ideas (DEPRECATED, see Pictures/greedy_is_not_optimal_when_parsing_sequence)

//given sequence of tile_ids from agent to goal (inclusive) (if pushable, sequence <- empty)
//if push, make all affected (within tpl+merge range) tiles wildcard (except if pushed is ZERO, only make pushed location transparent)
//but if tiles are made transparent, future pushes won't be detected
//bc tile_ids in sequence might not be collinear, sequence should not handle pushing
//for every reverse action, store the tpl nodes behind curr_node for push checking
//every nonzero tile is at least two steps away from turning into zero (assume agent merges with opposite sign and steps back) thus h+=2 for every break in rrd path
//sequence break occurs at tile if from agent, no combination of slide/split makes tile reachable; start next sequence where sequence break occurred, using best possible agent_tile_id
//non-step-back case is covered by rrd; if self-intersecting, loop section is at least as long as step-back
//subdijkstra to search for specific tile ids that enable merge doesn’t work since player could’ve pushed one along
//store results, don't re-process entire sequence each time

//make push-affected cells wildcard - agent can assume any tile_id after entering
//if push then turn, wildcard all push-affected tiles, append empty to seq
//use unordered_set or if good, Tilemap’s underlying container to store wildcards (every CADNode needs one)
//if push-affected tiles intersect with already-visited rra path from agent end, only propagate wildcard if intersection cell is nonempty
//todo: figure out how to analyze seq for breaks, and make use of prev seq result

*/

/* rrd closed checks (replaced by best_gs checks)

        //closed check is necessary bc open may receive duplicate nodes (see Pictures/rrd_iad_expanding_closed_check_is_necessary)
        if (closed.find(n.pos) != closed.end()) {
            //assert(n.pos != node_pos);
            open.pop();
            continue;
        }

            //see Pictures/rrd_iad_generating_closed_check_is_necessary
            if (closed.find(next_pos) != closed.end()) {
                continue;
            }

        //see Pictures/rrd_cad_expanding_closed_check_is_necessary
        it = closed.find(curr);
        if (it != closed.end()) {
            open.pop();
            continue;
        }
*/

/* from update min_dist_to_outside_of_shape in trace_path_informers()
min_validated_path_index is redundant bc trace_path_informers() only traces validated nodes

                //set min_validated_path_index
                if (path_index > pi->min_validated_path_index && !min_dist_to_outside_of_shape) {
                    pi->min_validated_path_index = path_index;
                }
*/

/* SAPISearchNode::init_largest_affected_path_index()
    for (int push_count = 1; push_count <= prev_push_count; ++push_count) {
        affected_lv_pos += dir;
        auto indices_itr = pi->lp_to_path_indices.find(affected_lv_pos);
        if (indices_itr != pi->lp_to_path_indices.end()) {
            //prev_path visits affected_lv_pos
            std::set<int>& prev_path_indices_at_lp = (*indices_itr).second;
            auto index_itr = prev_path_indices_at_lp.lower_bound(largest_affected_path_index);
            if (index_itr != prev_path_indices_at_lp.end()) {
                largest_affected_path_index = *index_itr;
            }
        }
    }
*/

/*std::function
    std::function<unsigned int(Vector2i)> get_radius_getter(int iw_shape_id, Vector2i dest_lv_pos);
    std::function<bool(Vector2i)> get_bounds_checker(Vector2i min, Vector2i max);
*/

/* lambdas deprecated, see functors
template <typename RadiusGetter>
RadiusGetter Pathfinder::get_radius_getter(int iw_shape_id, Vector2i dest_lv_pos) {
    switch (iw_shape_id) {
        case IWShapeId::DIAMOND:
            return [dest_lv_pos](Vector2i curr_lv_pos) -> unsigned int { return manhattan_dist(curr_lv_pos, dest_lv_pos); };
        default:
            return [dest_lv_pos](Vector2i curr_lv_pos) -> unsigned int { return max(abs(curr_lv_pos.x - dest_lv_pos.x), abs(curr_lv_pos.y - dest_lv_pos.y)); };
    }
}

    template <typename RadiusGetter>
    RadiusGetter get_radius_getter(int iw_shape_id, Vector2i dest_lv_pos);

template <typename RadiusGetter>
RadiusGetter Pathfinder::get_radius_getter(int iw_shape_id, Vector2i dest_lv_pos) {
    switch (iw_shape_id) {
        case IWShapeId::DIAMOND:
            return RadiusGetterDiamond(dest_lv_pos);
        case IWShapeId::SQUARE:
            return RadiusGetterSquare(dest_lv_pos);
    }
}
*/

/*
struct Radius_getter {
  Radius_getter(Vector2i dest_lv_pos, unsigned int(*func)(Vector2i dest_lv_pos, Vector2i curr_lv_pos))
    : dest_lv_pos{dest_lv_pos}
    , func{func}
    {}
  unsigned int operator()(Vector2i curr_lv_pos) {
    return func(curr_lv_pos, curr_lv_pos);
  }
  private:
  Vector2i dest_lv_pos;
  unsigned int(*func)(Vector2i dest_lv_pos, Vector2i curr_lv_pos);
};

Radius_getter get_radius_getter(int iw_shape_id, Vector2i dest_lv_pos) {
    switch (iw_shape_id) {
        case IWShapeId::DIAMOND:
            return {dest_lv_pos, [](Vector2i dest_lv_pos, Vector2i curr_lv_pos) -> unsigned int { ... }};
    //...
}
*/

/*
shared_ptr<SAPISearchNode> path_informed_hbjpmda(int max_depth, bool allow_type_change, Vector2i lv_end, open_sapi_t& open, closed_sapi_t& best_dists, unique_ptr<PathInfo>& pi, int h_reduction);
*/

/* from pathfind_sa_iwdmda(); moved to pathfind_sa()
    //early exit to skip SANode construction
    if (start == end) {
        return Array();
    }
*/

/* try_jump() prune non-natural neighbors

            if (ans) {
                //prune non-natural non-forced neighbor
                if (horizontal && next_dir.dir != dir && !next_dir.blocked) {
                    if (next_empty_and_regular) {
                        ans->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::SLIDE)] = {1, nullptr, 0};
                        ans->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::JUMP)] = {1, nullptr, 0};
                    }
                    else {
                        ans->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::JUMP)] = {1, nullptr, 0};

                        Vector3i normalized_next_action = Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::SLIDE);
                        shared_ptr<SASearchNode_t> neighbor = ans->try_action(normalized_next_action, lv_end, allow_type_change);
                        if (neighbor && !neighbor->prev_push_count) {
                            ans->neighbors[normalized_next_action] = {1, nullptr, 0};
                        }
                    }
                }
                assert(ans == curr_jp);
                continue;
            }
*/

/* try_jump() check obstruction
uint16_t curr_stuff_id = sanode->get_lv_sid(curr_pos);

//check obstruction
if (!is_tile_empty_and_regular(curr_stuff_id) || !is_compatible(src_type_id, get_back_id(curr_stuff_id))) {
    return nullptr;
}*/

/*
void set_lv_ttid(Vector2i _lv_pos, uint8_t type_id, uint8_t tile_id);

void SANode::set_lv_ttid(Vector2i _lv_pos, uint8_t type_id, uint8_t tile_id) {
    uint16_t old_sid = get_lv_sid(_lv_pos);
    uint8_t old_tile_id = get_tile_id(old_sid);
    uint8_t old_type_id = get_type_id(old_sid);

    if (old_type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[_lv_pos.y][_lv_pos.x][old_type_id];
    }
    if (old_tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[_lv_pos.y][_lv_pos.x][old_tile_id - 1];
    }
    if (type_id < TypeId::REGULAR) {
        hash ^= type_id_hash_keys[_lv_pos.y][_lv_pos.x][type_id];
    }
    if (tile_id > TileId::EMPTY) {
        hash ^= tile_id_hash_keys[_lv_pos.y][_lv_pos.x][tile_id - 1];
    }
    lv[_lv_pos.y][_lv_pos.x] = make_stuff_id(get_back_id(old_sid), type_id, tile_id);
}
*/

//if neighbor sanode is nullptr, normalized_perp_jump is either vertical or invalid => don't add constraint
//assert(!(ignore_prune && normalized_action.z == ActionId::CONSTRAINED_JUMP));

/*
const unordered_set<Vector2i, DirHasher> H_DIRS = {Vector2i(1, 0), Vector2i(-1, 0)};
const unordered_set<Vector2i, DirHasher> V_DIRS = {Vector2i(0, 1), Vector2i(0, -1)};
*/

/*
    int debug = max_scan_dist ? *max_scan_dist : -1;
    if (ignore_prune) {
        UtilityFunctions::print("\tcjump from ", sanode->lv_pos, " in ", dir, " started (msd = ", debug, ")");
    }
    else {
        UtilityFunctions::print("cjump from ", sanode->lv_pos, " in ", dir, " started (msd = ", debug, ")");
    }

                        if (sanode->lv_pos == Vector2i(4, 4) && dir == Vector2i(0, 1) && next_dir == Vector2i(-1, 0))
                            UtilityFunctions::print("g: ", g, " dist_av: ", dist_av, " vg: ", (*bd_it)->g, " pmsd: ", perp_max_scan_dist);

    if (sanode->lv_pos == Vector2i(4, 4) && dir == Vector2i(-1, 0)) {
        int msd = max_scan_dist ? *max_scan_dist : -1;
        UtilityFunctions::print("max_jump_dist from ", sanode->lv_pos, " in ", dir, ": ", max_jump_dist, " (msd = ", msd, ")");
    }

                if (action_id == ActionId::CONSTRAINED_JUMP) {
                    UtilityFunctions::print("secondary cjump from ", curr_pos, " in ", next_dir.dir, " (primary from ", sanode->lv_pos, " in ", dir, ")");
                }
*/