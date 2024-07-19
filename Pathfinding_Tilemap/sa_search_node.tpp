//#include "pathfinder.h"

#ifndef SA_SEARCH_NODE_TPP
#define SA_SEARCH_NODE_TPP

#ifndef PATHFINDER_HPP
#error __FILE__ should only be included from pathfinder.h.
#endif

#include "pathfinder.h"


//validated node - a node inside shape, or the immediate parent of final re-entry into shape
//critical node - node where agent tile_id must match in order to follow prev path
//admissible tile_id - (while following prev path) agent tile_id that will coincide with that of prev_path at the next
    //critical node (if there are any upcoming) if optimal slide/split decisions are made at non-critical nodes

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
//do admissible tile_id tracing with DP in is_on_prev_path() (from first critical/traced node with higher path_index)?

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
    //path_informed_mda() stores largest_affected_path_index
    //use std::set::upper_bound() to update it
//account for Pictures/both_push_and_merge_are_admissible and generalization to higher tpl
    //NAH, too hard; see Pictures/similar_to_both_push_and_merge_are_admissible_but_push_not_admissible
//account for n is 2nd-to-last node, path contains merge into goal, but push is possible
    //NAH, heuristic improvement is negligible
//return unnamed PathInfo object so RVO happens? NAH, that still requires copying the members
template<typename RadiusGetter>
unique_ptr<PathInfo> SASearchNode::trace_path_info(int path_len, int radius, const RadiusGetter& get_radius) {
    unique_ptr<PathInfo> pi = make_unique<PathInfo>();
    int min_dist_to_outside_of_shape = radius + 1;
    int path_index = path_len - 1;
    shared_ptr<SASearchNode> curr = shared_from_this();
    Vector2i curr_lv_pos = curr->sanode->lv_pos;
    bitset<TILE_ID_COUNT> next_admissible_tile_ids;
    next_admissible_tile_ids.fill(true);
    uint8_t adjacent_tile_id = TileId::EMPTY;
    bool is_next_merge = true;

    //trace nodes inside shape
    while (curr->prev != nullptr && min_dist_to_outside_of_shape) {
        Vector2i prev_dir = Vector2i(sgn(curr->prev_action.x), sgn(curr->prev_action.y));
        int prev_action_dist = get_action_dist(curr->prev_action);

        while (prev_action_dist > 0 && min_dist_to_outside_of_shape) {
            //store path_index
            auto indices_itr = pi->path_indices.find(curr_lv_pos);
            if (indices_itr != pi->path_indices.end()) {
                (*indices_itr).second.insert(path_index);
            }
            else {
                pi->path_indices[curr_lv_pos] = {path_index};
            }

            //store admissible_tile_ids
            //use emplace if possible
            PathNode pn(curr_lv_pos, path_index);
            relax_admissibility(next_admissible_tile_ids, is_next_merge, adjacent_tile_id);
            
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

        //update curr
        curr = curr->prev;

        //update adjacent_tile_id
        adjacent_tile_id = get_tile_id(curr->sanode->get_lv_sid(curr->sanode->lv_pos + prev_dir));
    }

    //trace start node/parent of final re-entry
    if (curr != nullptr) {

    }

    return pi;
}

#endif
