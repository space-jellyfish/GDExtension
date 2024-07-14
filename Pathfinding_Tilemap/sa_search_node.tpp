//#include "pathfinder.h"

#ifndef SA_SEARCH_NODE_TPP
#define SA_SEARCH_NODE_TPP

#ifndef PATHFINDER_HPP
#error __FILE__ should only be included from pathfinder.h.
#endif

#include "pathfinder.h"


//idea: do h_reduction if admissible tileid at same pos and upcoming locations in path have not been disturbed (visited or push-affected)
    //only trace nodes inside diamond/square
        //use lazy/just-in-time checking
        //if path exits diamond/square and re-enters, 
    //trace nodes along jump (if there is jump)
    //h_reduction based on pos alone is too inaccurate
    //calculate all admissible tile_ids at each pos, no need for ordered path or next pointers
    //comparing entire diamond/square is unnecessary
        //but what if a supportive tile for prev path is visited or push-affected?
            //track admissible supportive tile_ids too?
                //but what if there is a way to affect 2+ supportive tiles without invalidating prev path?
            //NAH, ignore supportive tiles
//account for restrictions due to pushing
//create unordered_map<lv_pos, set of indices at which lv_pos occurs in prev path> prev_path_indices
    //path_informed_mda() stores largest_affected_path_index
    //use std::set::upper_bound() to update it
//account for Pictures/both_push_and_merge_are_admissible and generalization to higher tpl
    //NAH, too hard; see Pictures/similar_to_both_push_and_merge_are_admissible_but_push_not_admissible
//account for n is 2nd-to-last node, path contains merge into goal, but push is possible
    //NAH, heuristic improvement is negligible
//return unnamed PathInfo object so RVO happens? NAH, that still requires copying the members
template<typename RadiusGetter>
void SASearchNode::trace_path_info(int path_len, PathInfo& pi, int radius, const RadiusGetter& get_radius) {
    int min_dist_to_edge = radius;
    shared_ptr<SASearchNode> curr = shared_from_this();
    while (curr->prev != nullptr) {

    }
}

#endif
