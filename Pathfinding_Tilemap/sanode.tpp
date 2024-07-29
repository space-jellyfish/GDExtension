#ifndef SANODE_TPP
#define SANODE_TPP

#ifndef PATHFINDER_HPP
#error __FILE__ should only be included from pathfinder.h.
#endif

#include "pathfinder.h"


//assume ttids in shape and lv_pos are initialized, and other locations uninitialized
//don't make duplicate init_lv_ttid() calls
template <typename RadiusGetter>
void SANode::fill_complement(Vector2i min, Vector2i max, int radius, const RadiusGetter& get_radius) {
    for (int x = min.x; x < max.x; ++x) {
        for (int y = min.y; y < max.y; ++y) {
            Vector2i pos(x, y);
            Vector2i lp = pos - min;
            if (lp != lv_pos && get_radius(lp) > radius) {
                init_lv_ttid(lp, pos);
            }
        }
    }
}

#endif