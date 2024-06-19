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