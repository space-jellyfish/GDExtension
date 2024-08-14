#ifndef PATHFINDER_TPP
#define PATHFINDER_TPP

#ifndef PATHFINDER_HPP
#error __FILE__ should only be included from pathfinder.h.
#endif

#include "pathfinder.h"


//closed not optimal bc path-informed heuristic not consistent
//return empty PathInfo containers if no path exists
template <typename RadiusGetter>
void Pathfinder::path_informed_mda(int max_depth, bool allow_type_change, shared_ptr<SANode> start, Vector2i lv_end, unique_ptr<PathInfo>& pi, bool trace_informers, bool sim_anneal, int radius, const RadiusGetter& get_radius) {
    open_sapi_fsort_t open;
    closed_sapi_t best_dists;

    shared_ptr<SAPISearchNode> first = make_shared<SAPISearchNode>();
    first->sanode = start;
    first->h = manhattan_dist(start->lv_pos, lv_end);
    first->f = first->h;
    open.push(first);
    best_dists.insert(first);
    
    while (!open.empty()) {
        shared_ptr<SAPISearchNode> curr = open.top();

        if (curr->sanode->lv_pos == lv_end) {
            pi->normalized_actions = curr->trace_path_normalized_actions(curr->g);
            //don't clear previous path informers unless tracing for a new path
            if (trace_informers) {
                pi->lp_to_path_indices.clear();
                pi->pn_to_admissible_tile_ids.clear();
                curr->trace_path_informers(pi, curr->g, radius, get_radius);
            }
            return;
        }
        open.pop();
        if (curr != *best_dists.find(curr)) {
            continue;
        }

        if (curr->g == max_depth) {
            continue;
        }

        for (Vector2i dir : DIRECTIONS_HFIRST) {
            if (!curr->sanode->get_dist_to_lv_edge(curr->sanode->lv_pos, dir)) {
                continue;
            }
            for (int action_id=ActionId::SLIDE; action_id != ActionId::JUMP; ++action_id) {
                Vector3i normalized_action(dir.x, dir.y, action_id);
                shared_ptr<SAPISearchNode> neighbor = curr->try_action(normalized_action, lv_end, allow_type_change, best_dists, false, nullptr);

                if (!neighbor) {
                    continue;
                }
                neighbor->g = curr->g + 1;
                neighbor->h = manhattan_dist(neighbor->sanode->lv_pos, lv_end);
                neighbor->init_lapi(pi, dir);

                //apply h_reduction
                int virtual_path_index = neighbor->get_virtual_path_index(pi, neighbor->largest_affected_path_index);
                if (virtual_path_index != -1) {
                    UtilityFunctions::print("h_reduction at ", neighbor->sanode->lv_pos, ", vpi = ", virtual_path_index);
                    neighbor->h -= get_h_reduction(virtual_path_index, sim_anneal);
                    neighbor->virtual_path_index = virtual_path_index;
                }
                neighbor->f = neighbor->g + neighbor->h;

                //reduced_f <= f <= true path_len
                if (neighbor->f > max_depth) {
                    continue;
                }

                auto it = best_dists.find(neighbor);
                if (it != best_dists.end()) {
                    if (neighbor->g >= (*it)->g) {
                        continue;
                    }
                    else {
                        neighbor->sanode = (*it)->sanode; //this ensures no duplicate SANodes in the SASearchNodes in open
                        (*it)->transfer_neighbors(neighbor, (*it)->g - neighbor->g); //hs not necessarily equal
                        best_dists.erase(it);
                    }
                }
                open.push(neighbor);
                best_dists.insert(neighbor);
            }
        }
    }
    pi->normalized_actions.clear();
    UtilityFunctions::print("no path found");
}

template <typename RadiusGetter>
void Pathfinder::path_informed_jpmda(int max_depth, bool allow_type_change, shared_ptr<SANode> start, Vector2i lv_end, unique_ptr<PathInfo>& pi, bool trace_informers, bool sim_anneal, int radius, const RadiusGetter& get_radius) {

}

#endif
