#ifndef SA_SEARCH_NODE_TPP
#define SA_SEARCH_NODE_TPP

#ifndef PATHFINDER_HPP
#error __FILE__ should only be included from pathfinder.h.
#endif

#include "pathfinder.h"


template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::reset() {
    sanode = nullptr;
    g = 0;
    h = 0;
    f = 0;
    prev = nullptr;
    neighbors.clear();
}

template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::init_sanode(Vector2i min, Vector2i max, Vector2i start) {
    sanode = node_pool.acquire<SANode>();
    sanode->init_lv_pos(start - min);
    sanode->init_lv(min, max);
}

//updates prev, prev_push_count
template <typename SASearchNode_t>
shared_ptr<SASearchNode_t> SASearchNodeBase<SASearchNode_t>::try_slide(Vector2i dir, bool allow_type_change) {
    int push_count = sanode->get_slide_push_count(dir, allow_type_change);
    if (push_count != -1) {
        shared_ptr<SASearchNode_t> m = node_pool.acquire<SASearchNode_t>();
        m->sanode = node_pool.acquire<SANode>();
        *(m->sanode) = SANode(*sanode);
        m->prev = static_pointer_cast<SASearchNode_t>(this->shared_from_this());
        m->prev_push_count = push_count;
        m->prune_backtrack(dir);
        m->sanode->perform_slide(dir, push_count);
        return m;
    }
    return nullptr;
}

//updates prev, prev_push_count
template <typename SASearchNode_t>
shared_ptr<SASearchNode_t> SASearchNodeBase<SASearchNode_t>::try_split(Vector2i dir, bool allow_type_change) {
    uint16_t src_sid = sanode->get_lv_sid(sanode->lv_pos);
    int src_pow = get_tile_pow(get_tile_id(src_sid));
    if (!is_pow_splittable(src_pow)) {
        return nullptr;
    }

    //halve src tile, try_slide, then (re)set its tile_id
    //splitted is new tile, splitter is old tile
    uint16_t untyped_split_sid = get_back_bits(src_sid) + get_splitted_tid(get_tile_id(src_sid));
    uint16_t splitted_sid = untyped_split_sid + get_type_bits(src_sid);
    sanode->set_lv_sid(sanode->lv_pos, splitted_sid);
    shared_ptr<SASearchNode_t> ans = try_slide(dir, allow_type_change);
    if (ans != nullptr) {
        //insert splitter tile
        uint16_t splitter_sid = untyped_split_sid + REGULAR_TYPE_BITS;
        ans->sanode->set_lv_sid(sanode->lv_pos, splitter_sid);
    }
    //reset src tile in this
    sanode->set_lv_sid(sanode->lv_pos, src_sid);

    return ans;
}

//g/h/f are default-init to 0
//updates prev_action for slide/split, stores result in curr->neighbors
//return newly-created SASearchNode (this ensures duplicate detection works as intended)
template <typename SASearchNode_t>
template <typename OpenList_t>
shared_ptr<SASearchNode_t> SASearchNodeBase<SASearchNode_t>::try_action(Vector3i normalized_action, Vector2i lv_end, bool allow_type_change, OpenList_t& open, bool ignore_prune, int* max_constrained_jump_scan_dist) {
    //check for stored neighbor
    auto it = neighbors.find(normalized_action);
    if (it != neighbors.end()) {
        SANeighbor neighbor = (*it).second;

        if (!ignore_prune && neighbor.unprune_threshold) {
            //pruned or action invalid
            return nullptr;
        }
        if (neighbor.sanode) {
            shared_ptr<SASearchNode_t> ans = node_pool.acquire<SASearchNode_t>();
            ans->sanode = neighbor.sanode;
            if (normalized_action.z == ActionId::JUMP || normalized_action.z == ActionId::CONSTRAINED_JUMP) {
                unsigned int jump_dist = manhattan_dist(sanode->lv_pos, ans->sanode->lv_pos);
                ans->prev_action = Vector3i(jump_dist * normalized_action.x, jump_dist * normalized_action.y, normalized_action.z);
            }
            else {
                ans->prev_action = normalized_action;
            }
            ans->prev = static_pointer_cast<SASearchNode_t>(this->shared_from_this());
            ans->prev_push_count = neighbor.push_count;
            return ans;
        }
    }

    Vector2i dir(normalized_action.x, normalized_action.y);
    shared_ptr<SASearchNode_t> ans;
    switch(normalized_action.z) {
        case ActionId::SLIDE:
            ans = try_slide(dir, allow_type_change);
            break;
        case ActionId::SPLIT:
            ans = try_split(dir, allow_type_change);
            break;
        case ActionId::JUMP:
            ans = try_jump(dir, lv_end, allow_type_change, open);
            break;
        case ActionId::CONSTRAINED_JUMP:
            ans = try_constrained_jump(dir, lv_end, allow_type_change, open, ignore_prune, max_constrained_jump_scan_dist);
            break;
        default:
            break;
    }
    if (ans) {
        if (normalized_action.z == ActionId::SLIDE || normalized_action.z == ActionId::SPLIT) {
            ans->prev_action = normalized_action;
        }
        //update neighbors
        neighbors[normalized_action] = {0, ans->sanode, ans->prev_push_count};
    }
    else {
        //action invalid; don't unprune
        neighbors[normalized_action] = {numeric_limits<unsigned int>::max(), nullptr, 0};
    }
    return ans;
}

//assume immediate neighbor is in_bounds
//jump_dist is bounded by sanode size (MAX_SEARCH_WIDTH/MAX_SEARCH_HEIGHT)
//updates prev, prev_action, prev_push_count
//doesn't change agent type/tile_id
//only jump through empty_and_regular tiles
//see Devlog/jump_conditions for details
//when generating from jp, generate split in all dirs, generate slide iff next tile isn't empty_and_regular
template <typename SASearchNode_t>
template <typename OpenList_t>
shared_ptr<SASearchNode_t> SASearchNodeBase<SASearchNode_t>::try_jump(Vector2i dir, Vector2i lv_end, bool allow_type_change, OpenList_t& open) {
    //check bounds NAH, pathfind_sa*() checks while generating
    //check obstruction
    uint8_t src_type_id = get_type_id(sanode->get_lv_sid(sanode->lv_pos));
    Vector2i curr_pos = sanode->lv_pos + dir;
    uint16_t curr_stuff_id = sanode->get_lv_sid(curr_pos);
    if (!is_tile_empty_and_regular(curr_stuff_id) || !is_compatible(src_type_id, get_back_id(curr_stuff_id))) {
        return nullptr;
    }
    bool next_obstructed = false;

    //init next_dirs
    bool horizontal = (dir.x != 0);
    array<NextDir, 3> next_dirs;
    auto next_dirs_itr = next_dirs.begin();
    Vector2i perp_dir1 = horizontal ? Vector2i(0, 1) : Vector2i(1, 0);
    Vector2i perp_dir2 = horizontal ? Vector2i(0, -1) : Vector2i(-1, 0);
    for (Vector2i next_dir : {perp_dir1, perp_dir2}) {
        if (!sanode->get_dist_to_lv_edge(curr_pos, next_dir)) {
            *next_dirs_itr = NextDir(next_dir, false, false);
        }
        else {
            uint16_t next_stuff_id = sanode->get_lv_sid(sanode->lv_pos + next_dir);
            bool blocked = !(is_tile_empty_and_regular(next_stuff_id) && is_compatible(src_type_id, get_back_id(next_stuff_id)));
            *next_dirs_itr = NextDir(next_dir, true, blocked);
        }
        ++next_dirs_itr;
    }
    int dist_to_lv_edge = sanode->get_dist_to_lv_edge(sanode->lv_pos, dir);
    int curr_dist = 1;
    *next_dirs_itr = NextDir(dir, curr_dist < dist_to_lv_edge, false);

    shared_ptr<SASearchNode_t> curr_jp;
    shared_ptr<SASearchNode_t> ans;

    while (curr_dist <= dist_to_lv_edge) {
        //UtilityFunctions::print("scan node");
        //get current jump point; reuse sanode if possible
        shared_ptr<SANode> prev_sanode = (curr_dist > 1) ? curr_jp->sanode : nullptr;
        curr_jp = get_jump_point(prev_sanode, dir, curr_pos, curr_dist, ActionId::JUMP);

        //check if visited with equal or better dist
        auto it = open.find(curr_jp);
        if (it != nullptr) {
            int curr_g = g + curr_dist;
            if (it->g <= curr_g) {
                //see Pictures/node_along_jump_that_is_visited_with_better_g_handles_remainder_of_jump
                transfer_neighbors(it, curr_g - it->g);
                it->neighbors[Vector3i(-dir.x, -dir.y, ActionId::CONSTRAINED_JUMP)] = {static_cast<unsigned int>(2 * curr_dist + 1), nullptr, 0};
                return nullptr;
            }
            else {
                //found better path, declare curr_jp as jump point
                //search function handles neighbor transfer
                return curr_jp;
            }
        }

        //check lv_end
        if (curr_pos == lv_end) {
            return curr_jp;
        }

        //check for jump conditions
        for (NextDir& next_dir : next_dirs) {
            //bound check
            if (!next_dir.in_bounds) { //once false, in_bounds stays false
                //curr_jp->prune_invalid_action_ids(next_dir.dir); //redundant, pathfind_sa_*() does bounds check
                continue;
            }
            //update in_bounds?
            //if next_dir.dir != dir, no bc in_bounds cannot change
            //if next_dir.dir == dir, yes bc while loop only ensures curr_pos is in_bounds

            uint16_t next_stuff_id = sanode->get_lv_sid(curr_pos + next_dir.dir);
            bool next_compatible = is_compatible(src_type_id, get_back_id(next_stuff_id));
            bool next_empty_and_regular = is_tile_empty_and_regular(next_stuff_id);
            //update next_obstructed; this can be safely skipped if ans (while loop will exit)
            //update in_bounds since it uses the same if expression
            if (!ans && next_dir.dir == dir) {
                next_obstructed = !next_empty_and_regular || !next_compatible;
                next_dir.in_bounds = curr_dist + 1 < dist_to_lv_edge;
            }

            //compatibility check
            if (!next_compatible) {
                curr_jp->prune_invalid_action_ids(next_dir.dir);
                //update blocked
                next_dir.blocked = true;
                continue;
            }

            if (next_empty_and_regular) {
                //next empty check
                if (next_dir.dir == dir) {
                    //blocked in jump_dir is DON'T CARE, no need to update
                    continue;
                }

                //prune slide if empty (to skip upcoming try_action(), not redundant)
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::SLIDE)] = {numeric_limits<unsigned int>::max(), nullptr, 0};

                if (horizontal && next_dir.dir != dir) {
                    //horizontal forced neighbor check
                    if (next_dir.blocked) {
                        ans = curr_jp;
                        //ans found, no need to update blocked
                        continue;
                    }
                    else {
                        //prune jump if horizontal, perp, and not blocked
                        //unprune_threshold should be 1, see Pictures/jps_nonnatural_nonforced_unprune_threshold_should_be_one
                        curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::JUMP)] = {1, nullptr, 0};
                    }
                }
            }
            else {
                //prune jump if not
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::JUMP)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
            }

            for (int action_id=ActionId::SLIDE; action_id != ActionId::CONSTRAINED_JUMP; ++action_id) {
                //store next_action result in curr_jp->neighbors and if valid, return curr_jp
                //if next_dir.dir == dir, either next empty check activates or !next_empty_and_regular (jump is pruned)
                if (action_id == ActionId::JUMP && (horizontal || !next_empty_and_regular)) {
                    continue;
                }

                Vector3i normalized_next_action(next_dir.dir.x, next_dir.dir.y, action_id);
                //this does not create any permanent refs to curr_jp->sanode, so it can be reused for the next curr_jp
                shared_ptr<SASearchNode_t> neighbor = curr_jp->try_action(normalized_next_action, lv_end, allow_type_change, open, false, nullptr);
                if (neighbor) {
                    if (!horizontal || next_dir.dir == dir || next_dir.blocked || action_id != ActionId::SLIDE || neighbor->prev_push_count) {
                        ans = curr_jp;
                    }
                    else {
                        //horizontal, perp, not blocked, slide, self merges
                        curr_jp->neighbors[normalized_next_action] = {1, neighbor->sanode, 0};
                    }
                }
                if (ans) {
                    break;
                }
            }

            //update blocked
            //assume next is compatible
            next_dir.blocked = !next_empty_and_regular;
        }
        //check ans
        if (ans) {
            return ans;
        }

        //check obstruction
        if (next_obstructed) {
            return nullptr;
        }
        curr_pos += dir;
        ++curr_dist;
    }

    return nullptr;
}

//see Pictures/4c_cjps
//if max_scan_dist is nullptr, assume unconstrained
//if vertical, assume node v (left/right cjps neighbor) is initialized
//if MJD is zero, don't try jump (in dir) at all
//since d > 0, MJD is strictly less than |av| => don't delete constraint if next node in dir not in_bounds
//return a_i when jp found; constraint is effectively updated when a_i continues the vertical jump
template <typename SASearchNode_t>
template <typename OpenList_t>
shared_ptr<SASearchNode_t> SASearchNodeBase<SASearchNode_t>::try_constrained_jump(Vector2i dir, Vector2i lv_end, bool allow_type_change, OpenList_t& open, bool ignore_prune, int* max_scan_dist) {
    //check bounds NAH, pathfind_sa*() checks while generating
    //check obstruction
    uint8_t src_type_id = get_type_id(sanode->get_lv_sid(sanode->lv_pos));
    Vector2i curr_pos = sanode->lv_pos + dir;
    uint16_t curr_stuff_id = sanode->get_lv_sid(curr_pos);
    if (!is_tile_empty_and_regular(curr_stuff_id) || !is_compatible(src_type_id, get_back_id(curr_stuff_id))) {
        if (max_scan_dist) {
            *max_scan_dist = numeric_limits<int>::max();
        }
        return nullptr;
    }
    bool next_obstructed = false;

    //init next_dirs
    bool horizontal = (dir.x != 0);
    array<NextDirConstrained, 3> next_dirs;
    auto next_dirs_itr = next_dirs.begin();
    Vector2i perp_dir1 = horizontal ? Vector2i(0, 1) : Vector2i(1, 0);
    Vector2i perp_dir2 = horizontal ? Vector2i(0, -1) : Vector2i(-1, 0);
    for (Vector2i next_dir : {perp_dir1, perp_dir2}) {
        if (!sanode->get_dist_to_lv_edge(curr_pos, next_dir)) {
            *next_dirs_itr = NextDirConstrained(next_dir, false, false, 0);
        }
        else {
            uint16_t next_stuff_id = sanode->get_lv_sid(sanode->lv_pos + next_dir);
            bool blocked = !(is_tile_empty_and_regular(next_stuff_id) && is_compatible(src_type_id, get_back_id(next_stuff_id)));

            //init perp_max_scan_dist (DON'T CARE if horizontal)
            int perp_max_scan_dist = numeric_limits<int>::max();
            if (!horizontal) {
                Vector3i normalized_perp_jump(next_dir.x, next_dir.y, ActionId::CONSTRAINED_JUMP);

                //get hjump point to use as key to open to get g_v to calculate d (ignoring prune)
                //this should not cause infinite recursion since horizontal cjump doesn't call cjump
                shared_ptr<SASearchNode_t> v = try_action(normalized_perp_jump, lv_end, allow_type_change, open, true, nullptr);
                if (v) {
                    auto bd_it = open.find(v);
                    if (bd_it != nullptr) {
                        //v->g is default-init to 0, calculate |av| manually
                        int dist_av = manhattan_dist(sanode->lv_pos, v->sanode->lv_pos);
                        int d = g + dist_av - bd_it->g;
                        perp_max_scan_dist = dist_av - (d + 1) / 2;
                    }
                    //else gv = ga + |av|, no constraint
                }
            }
            *next_dirs_itr = NextDirConstrained(next_dir, true, blocked, perp_max_scan_dist);
        }
        ++next_dirs_itr;
    }
    int dist_to_lv_edge = sanode->get_dist_to_lv_edge(sanode->lv_pos, dir);
    int curr_dist = 1;
    *next_dirs_itr = NextDirConstrained(dir, curr_dist < dist_to_lv_edge, false, numeric_limits<int>::max());

    shared_ptr<SASearchNode_t> curr_jp;
    shared_ptr<SASearchNode_t> ans;
    int max_jump_dist = max_scan_dist ? min(*max_scan_dist, dist_to_lv_edge) : dist_to_lv_edge;

    while (curr_dist <= max_jump_dist) {
        //UtilityFunctions::print("scan node");
        //get current jump point; reuse sanode if possible
        shared_ptr<SANode> prev_sanode = (curr_dist > 1) ? curr_jp->sanode : nullptr;
        curr_jp = get_jump_point(prev_sanode, dir, curr_pos, curr_dist, ActionId::CONSTRAINED_JUMP);

        //check if visited with equal or better dist
        auto it = open.find(curr_jp);
        if (it != nullptr) {
            int curr_g = g + curr_dist;
            if (it->g <= curr_g) {
                if (!ignore_prune) {
                    //see Pictures/node_along_jump_that_is_visited_with_better_g_handles_remainder_of_jump
                    transfer_neighbors(it, curr_g - it->g);
                    it->neighbors[Vector3i(-dir.x, -dir.y, ActionId::CONSTRAINED_JUMP)] = {static_cast<unsigned int>(2 * curr_dist + 1), nullptr, 0};
                    return nullptr;
                }
            }
            else {
                //found better path, declare curr_jp as jump point
                //search function handles neighbor transfer
                return curr_jp;
            }
        }

        //check lv_end
        if (curr_pos == lv_end) {
            return curr_jp;
        }

        //check for jump conditions
        for (NextDirConstrained& next_dir : next_dirs) {
            //bound check
            if (!next_dir.in_bounds) { //once false, in_bounds stays false
                //curr_jp->prune_invalid_action_ids(next_dir.dir); //redundant, pathfind_sa_*() does bounds check
                continue;
            }
            //update in_bounds?
            //if next_dir.dir != dir, no bc in_bounds cannot change
            //if next_dir.dir == dir, yes bc while loop only ensures curr_pos is in_bounds

            uint16_t next_stuff_id = sanode->get_lv_sid(curr_pos + next_dir.dir);
            bool next_compatible = is_compatible(src_type_id, get_back_id(next_stuff_id));
            bool next_empty_and_regular = is_tile_empty_and_regular(next_stuff_id);
            //update next_obstructed; this can be safely skipped if ans (while loop will exit)
            //update in_bounds since it uses the same if expression
            if (!ans && next_dir.dir == dir) {
                next_obstructed = !next_empty_and_regular || !next_compatible;
                next_dir.in_bounds = curr_dist + 1 < dist_to_lv_edge;
            }

            //compatibility check
            if (!next_compatible) {
                curr_jp->prune_invalid_action_ids(next_dir.dir);
                //update blocked, max_scan_dist
                next_dir.blocked = true;
                next_dir.max_scan_dist = numeric_limits<int>::max();
                continue;
            }

            if (next_empty_and_regular) {
                //next empty check
                if (next_dir.dir == dir) {
                    //blocked in jump_dir is DON'T CARE, no need to update
                    continue;
                }

                //prune slide if empty (to skip upcoming try_action(), not redundant)
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::SLIDE)] = {numeric_limits<unsigned int>::max(), nullptr, 0};

                if (horizontal && next_dir.dir != dir) {
                    //horizontal forced neighbor check
                    if (next_dir.blocked) {
                        ans = curr_jp;
                        //ans found, no need to update blocked
                        continue;
                    }
                    else {
                        //prune jump if horizontal, perp, and not blocked
                        //unprune_threshold should be 1, see Pictures/jps_nonnatural_nonforced_unprune_threshold_should_be_one
                        curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::CONSTRAINED_JUMP)] = {1, nullptr, 0};
                    }
                }
            }
            else {
                //prune jump if not
                curr_jp->neighbors[Vector3i(next_dir.dir.x, next_dir.dir.y, ActionId::CONSTRAINED_JUMP)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
            }

            for (int action_id : {ActionId::SLIDE, ActionId::SPLIT, ActionId::CONSTRAINED_JUMP}) {
                //store next_action result in curr_jp->neighbors and if valid, return curr_jp
                //if next_dir.dir == dir, either next empty check activates or !next_empty_and_regular (jump is pruned)
                if (action_id == ActionId::CONSTRAINED_JUMP && (horizontal || !next_empty_and_regular)) {
                    continue;
                }

                Vector3i normalized_next_action(next_dir.dir.x, next_dir.dir.y, action_id);
                //this does not create any permanent refs to curr_jp->sanode, so it can be reused for the next curr_jp
                shared_ptr<SASearchNode_t> neighbor = curr_jp->try_action(normalized_next_action, lv_end, allow_type_change, open, false, &next_dir.max_scan_dist);
                if (neighbor) {
                    if (!horizontal || next_dir.dir == dir || next_dir.blocked || action_id != ActionId::SLIDE || neighbor->prev_push_count) {
                        ans = curr_jp;
                    }
                    else {
                        //horizontal, perp, not blocked, slide, self merges
                        curr_jp->neighbors[normalized_next_action] = {1, neighbor->sanode, 0};
                    }
                }
                if (ans) {
                    break;
                }
            }

            //update blocked, max_scan_dist
            //assume next is compatible
            next_dir.blocked = !next_empty_and_regular;
            if (next_dir.blocked) {
                next_dir.max_scan_dist = numeric_limits<int>::max();
            }
        }
        //check ans
        if (ans) {
            return ans;
        }

        //check obstruction
        if (next_obstructed) {
            if (max_scan_dist) {
                *max_scan_dist = numeric_limits<int>::max();
            }
            return nullptr;
        }
        curr_pos += dir;
        ++curr_dist;
    }

    return nullptr;
}

//assume jp_pos != src_lv_pos
//assume jp_pos is within bounds
template <typename SASearchNode_t>
shared_ptr<SASearchNode_t> SASearchNodeBase<SASearchNode_t>::get_jump_point(shared_ptr<SANode> prev_sanode, Vector2i dir, Vector2i jp_pos, unsigned int jump_dist, int action_id) {
    shared_ptr<SASearchNode_t> ans = node_pool.acquire<SASearchNode_t>();
    if (prev_sanode) {
        ans->sanode = prev_sanode;
    }
    else {
        ans->sanode = node_pool.acquire<SANode>();
        *(ans->sanode) = SANode(*sanode);
    }
    Vector2i src_lv_pos = ans->sanode->lv_pos;
    ans->sanode->set_lv_pos(jp_pos);
    ans->sanode->set_lv_sid(jp_pos, get_jumped_stuff_id(ans->sanode->get_lv_sid(src_lv_pos), ans->sanode->get_lv_sid(jp_pos)));
    ans->sanode->reset_lv_sid(src_lv_pos);

    //prev stuff
    ans->prev_action = Vector3i(jump_dist * dir.x, jump_dist * dir.y, action_id);
    ans->prev = static_pointer_cast<SASearchNode_t>(this->shared_from_this());
    ans->prev_push_count = 0;

    //prune stuff
    ans->neighbors[Vector3i(-dir.x, -dir.y, ActionId::SLIDE)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
    //unprune threshold should be 2 * true_reverse_jump_dist + 1; since true_reverse_jump_dist unknown, use 3 to be safe
    //(see Pictures/reverse_jump_unprune_threshold, Pictures/prev_is_not_reverse_jump_in_general, JPS pruning rules)
    //don't use unprune_threshold = numeric_limits<unsigned int>::max() (see Pictures/jps_needs_unpruning)
    ans->neighbors[Vector3i(-dir.x, -dir.y, action_id)] = {3, nullptr, 0};
    //ans->neighbors[Vector3i(-dir.x, -dir.y, action_id)] = {2 * jump_dist + 1, sanode, 0};

    return ans;
}

//prune all actions in dir
template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::prune_invalid_action_ids(Vector2i dir) {
    for (int action_id=ActionId::SLIDE; action_id != ActionId::ACTION_END; ++action_id) {
        neighbors[Vector3i(dir.x, dir.y, action_id)] = {numeric_limits<unsigned int>::max(), nullptr, 0};
    }
}

//assume prev is set correctly
template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::prune_backtrack(Vector2i dir) {
    //backtrack prevention: prune curr in neighbor->neighbors when storing neighbor in curr->neighbors
    //skip slide if -dir and prev slide merged with empty or prev split merged with empty
    //skip split if -dir and prev slide merged with same sign regular or prev split merged with same sign regular
    //unprune if there is 3+ dist improvement ({n, n+1} -> {n-1, n-2})
    uint16_t neighbor_sid = sanode->get_lv_sid(sanode->lv_pos + dir);
    uint8_t src_tile_id = get_tile_id(sanode->get_lv_sid(sanode->lv_pos));
    uint8_t neighbor_tile_id = get_tile_id(neighbor_sid);
    uint8_t neighbor_type_id = get_type_id(neighbor_sid);
    if (is_tile_empty_and_regular(neighbor_sid)) {
        neighbors[Vector3i(-dir.x, -dir.y, ActionId::SLIDE)] = {3, prev->sanode, 0};
    }
    if (is_same_sign_merge(src_tile_id, neighbor_tile_id) && neighbor_type_id == TypeId::REGULAR) {
        neighbors[Vector3i(-dir.x, -dir.y, ActionId::SPLIT)] = {3, prev->sanode, 0};
    }
}

//assume better_dist isn't the same, but compares equal to curr (same lv and lv_pos)
//if there are two prunes on the same action, keep the stronger one (higher unprune_threshold)
//since curr will never generate again, its neighbors can be safely cleared
template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::transfer_neighbors(shared_ptr<SASearchNode_t> better_dist, int dist_improvement) {
    unordered_map<Vector3i, SANeighbor, ActionHasher>& dest_neighbors = better_dist->neighbors;

    for (auto [normalized_action, neighbor] : neighbors) {
        //transfer if prune is invalid-based
        if (neighbor.unprune_threshold == numeric_limits<unsigned int>::max()) {
            dest_neighbors[normalized_action] = {numeric_limits<unsigned int>::max(), nullptr, 0};
            continue;
        }
        
        unsigned int eff_unprune_threshold = max(0, static_cast<int>(neighbor.unprune_threshold) - dist_improvement);
        auto it = dest_neighbors.find(normalized_action);
        if (it != dest_neighbors.end()) {
            SANeighbor& dest_neighbor = (*it).second;

            if (eff_unprune_threshold > dest_neighbor.unprune_threshold) {
                dest_neighbor.unprune_threshold = eff_unprune_threshold;
            }
            if (!dest_neighbor.sanode) {
                dest_neighbor.sanode = neighbor.sanode;
                dest_neighbor.push_count = neighbor.push_count;
            }
        }
        else {
            dest_neighbors[normalized_action] = {eff_unprune_threshold, neighbor.sanode, neighbor.push_count};
        }
    }
    //neighbors.clear();
}

template <typename SASearchNode_t>
Array SASearchNodeBase<SASearchNode_t>::trace_path_normalized_actions(int path_len) {
	Array ans;
	ans.resize(path_len);
	int index = path_len - 1;
	shared_ptr<SASearchNode_t> curr = static_pointer_cast<SASearchNode_t>(this->shared_from_this());

	while (curr->prev != nullptr) {
        Vector3i normalized_prev_action = get_normalized_action(curr->prev_action);
        int prev_action_dist = get_action_dist(curr->prev_action);

        for (int dist=0; dist < prev_action_dist; ++dist) {
            ans[index] = normalized_prev_action;
            --index;
        }
		curr = curr->prev;
	}
	UtilityFunctions::print("PF TRACED PATH SIZE: ", ans.size());
	return ans;
}

//validated node - a node inside shape, or the immediate parent of final re-entry into shape
//critical node - node where agent tile_id must match in order to follow prev path
//admissible tile_id - (while following prev path) agent tile_id that will coincide with that of prev_path at the next
    //critical node (if there are any upcoming) if optimal slide/split decisions are made

//idea: do h_reduction if tile_id is admissible and upcoming locations in path have not been disturbed (visited or push-affected)
//i.e. apply to all SANodes that are guaranteed to be following prev path, if decision between slide/split is allowed at non-critical nodes
    //note: prev_path (and PathInfo) could become invalid if tpl decreases
        //increasing tpl can cause extra ZEROs to remain
    //only trace validated nodes
        //use lazy/just-in-time radius checking
        //include dest node bc h_reduction will save node expansions
        //exclude nodes before parent of final re-entry (see Pictures/trace_path_info_should_exclude_node_if_upcoming_location_exits_shape)
    //trace nodes along jump (if there is jump) bc next iteration path might use them
    //h_reduction based on pos alone is too inaccurate
    //calculate all admissible tile_ids at each pos
        //account for restrictions due to pushing
//to accomodate requests in QRCBS, distinguish push/merge ZERO from push/merge EMPTY
//do admissible tile_id tracing with DP in get_virtual_path_index() (from first critical/traced node with higher path_index)?

//distinguish push from push-with-partner?
    //NAH, both could be essential for prev path to work
//only apply h_reduction if agent tile_id matches or node is non-critical?
    //NAH, calculating admissible tile ids is more accurate
//trace nodes outside shape for useful wall info?
    //NAH, see Pictures/trace_path_info_should_ignore_nodes_outside_iwshape
//comparing entire shape is unnecessary
    //but what if a supportive tile for prev path is visited or push-affected?
        //track admissible supportive tile_ids too?
            //but what if there is a way to affect 2+ supportive tiles without invalidating prev path?
        //NAH, ignore supportive tiles
//create unordered_map<lv_pos, set of indices at which lv_pos occurs in prev path> prev_path_indices
    //path_informed_mdanr() stores largest_affected_path_index
    //use std::set::lower_bound() to update it
//account for Pictures/both_push_and_merge_are_admissible and generalization to higher tpl
    //NAH, too hard; see Pictures/similar_to_both_push_and_merge_are_admissible_but_push_not_admissible
//account for n is 2nd-to-last node, path contains merge into goal, but push is possible
    //NAH, heuristic improvement is negligible
//return unnamed PathInfo object so RVO happens? NAH, that still requires copying the members

//returns pi with lp_to_path_indices and pn_to_admissible_tile_ids initialized
template <typename SASearchNode_t>
template <typename RadiusGetter>
void SASearchNodeBase<SASearchNode_t>::trace_path_informers(unique_ptr<PathInfo>& pi, int path_len, int radius, const RadiusGetter& get_radius) {
    int min_dist_to_outside_of_shape = radius + 1;
    int path_index = path_len - 1;
    shared_ptr<SASearchNode_t> curr = static_pointer_cast<SASearchNode_t>(this->shared_from_this());
    Vector2i curr_lv_pos = curr->sanode->lv_pos;
    bitset<TILE_ID_COUNT> next_admissible_tile_ids;
    next_admissible_tile_ids.set();
    bool is_next_merge = true;
    uint8_t adjacent_tile_id = TileId::EMPTY;

    //trace nodes inside shape
    while (curr->prev != nullptr && min_dist_to_outside_of_shape) {
        Vector2i prev_dir = get_normalized_dir(curr->prev_action);
        int prev_action_dist = get_action_dist(curr->prev_action);

        while (prev_action_dist > 0 && min_dist_to_outside_of_shape) {
            relax_admissibility(next_admissible_tile_ids, is_next_merge, adjacent_tile_id);
            trace_node_info(pi, PathNode(curr_lv_pos, path_index), next_admissible_tile_ids);
            
            //update path_index
            --path_index;

            //update curr_lv_pos
            curr_lv_pos -= prev_dir;

            //update min_dist_to_outside_of_shape
            --min_dist_to_outside_of_shape;
            if (!min_dist_to_outside_of_shape) {
                min_dist_to_outside_of_shape = radius + 1 - get_radius(curr->sanode->lv_pos);
            }

            //update prev_action_dist
            --prev_action_dist;
        }

        //update is_next_merge
        is_next_merge = curr->prev_push_count == 0;

        //update curr
        curr = curr->prev;

        //update adjacent_tile_id
        adjacent_tile_id = get_tile_id(curr->sanode->get_lv_sid(curr->sanode->lv_pos + prev_dir));
    }

    //trace start node/parent of final re-entry
    relax_admissibility(next_admissible_tile_ids, is_next_merge, adjacent_tile_id);
    trace_node_info(pi, PathNode(curr_lv_pos, path_index), next_admissible_tile_ids);
}

//get lv_pos and tile_id from SASearchNode
//assume array[TileId::EMPTY] == array[TileId::ZERO]
//break ties in favor of higher path_index
//combine lp_to_path_indices and pn_to_admissible_tile_ids into single structure? NAH too complicated
template <typename SASearchNode_t>
int SASearchNodeBase<SASearchNode_t>::get_virtual_path_index(unique_ptr<PathInfo>& pi, int largest_affected_path_index) {
    auto indices_itr = pi->lp_to_path_indices.find(sanode->lv_pos);
    if (indices_itr == pi->lp_to_path_indices.end()) {
        return -1;
    }

    //loop through admissible indices (inclusive of largest_affected_path_index)
    uint8_t curr_tile_id = get_tile_id(sanode->get_lv_sid(sanode->lv_pos));
    std::set<int>& prev_path_indices_at_lp = (*indices_itr).second;
    //init_lapi() calls lower_bound() already, see Pictures/lower_bound_is_not_necessary_in_get_virtual_path_index for casework
    //assert(prev_path_indices_at_lp.find(largest_affected_path_index) == prev_path_indices_at_lp.lower_bound(largest_affected_path_index));

    for (auto index_itr = prev_path_indices_at_lp.rbegin(); index_itr.base() != prev_path_indices_at_lp.find(largest_affected_path_index); ++index_itr) {
        auto tile_ids_itr = pi->pn_to_admissible_tile_ids.find(PathNode(sanode->lv_pos, *index_itr));
        if (tile_ids_itr != pi->pn_to_admissible_tile_ids.end() && (*tile_ids_itr).second[curr_tile_id]) {
            return *index_itr;
        }
    }
    return -1;
}

//for get_virtual_path_index(), array[TileId::EMPTY] should be equal to array[TileId::ZERO]
//speed up with bit operations
template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::relax_admissibility(bitset<TILE_ID_COUNT>& admissible_tile_ids) {
    /*
    for (int i = TileId::EMPTY + 1; i < TileId::ZERO - 1; ++i) {
        if (admissible_tile_ids[i + 1]) {
            admissible_tile_ids[i] = true;
        }
    }
    for (int i = TILE_ID_COUNT - 1; i > TileId::ZERO + 1; --i) {
        if (admissible_tile_ids[i - 1]) {
            admissible_tile_ids[i] = true;
        }
    }
    1-14 inclusive
    18-31 inclusive
    */
    uint32_t u = admissible_tile_ids.to_ulong();
    uint32_t first = u & 0x7FFF0000; //1-15 inclusive
    uint32_t second = u & 0x7FFF; //17-31 inclusive
    uint32_t ans = (u & 0x80008000) + ((first | first << 1) + (second | second >> 1) & 0x7FFF7FFF);
    admissible_tile_ids = bitset<TILE_ID_COUNT>(ans);
}

//don't restrict any tile_ids if ZERO was pushed/merged, there's no difference (bc of bubbling)
//according to StackOverflow/37009695, return foo() works
template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::relax_admissibility(bitset<TILE_ID_COUNT>& admissible_tile_ids, bool is_next_merge, uint8_t adjacent_tile_id) {
    if (is_next_merge) {
        if (!is_tile_unsigned(adjacent_tile_id)) {
            bitset<TILE_ID_COUNT> new_admissible_tile_ids;

            //same sign merge
            if (get_signed_tile_pow(adjacent_tile_id) != TILE_POW_MAX && admissible_tile_ids[get_merged_tile_id(adjacent_tile_id, adjacent_tile_id)]) {
                new_admissible_tile_ids[adjacent_tile_id] = true;
            }
            //zero merge
            if (admissible_tile_ids[adjacent_tile_id]) {
                new_admissible_tile_ids[TileId::ZERO] = true;
                new_admissible_tile_ids[TileId::EMPTY] = true;
            }
            //opposite sign merge
            if (admissible_tile_ids[TileId::ZERO]) {
                new_admissible_tile_ids[get_opposite_tile_id(adjacent_tile_id)] = true;
            }
            admissible_tile_ids = new_admissible_tile_ids;
        }
    }
    else {
        if (!is_tile_unsigned(adjacent_tile_id)) {
            admissible_tile_ids[TileId::ZERO] = false;
            admissible_tile_ids[TileId::EMPTY] = false;
            admissible_tile_ids[adjacent_tile_id] = false;
            admissible_tile_ids[get_opposite_tile_id(adjacent_tile_id)] = false;
        }
    }
    relax_admissibility(admissible_tile_ids);
}

template <typename SASearchNode_t>
void SASearchNodeBase<SASearchNode_t>::trace_node_info(unique_ptr<PathInfo>& pi, const PathNode& pn, const bitset<TILE_ID_COUNT>& admissible_tile_ids) {
    //store path_index
    pi->lp_to_path_indices[pn.lv_pos].insert(pn.index);

    //store admissible_tile_ids
    pi->pn_to_admissible_tile_ids[pn] = admissible_tile_ids;
}

#endif
