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