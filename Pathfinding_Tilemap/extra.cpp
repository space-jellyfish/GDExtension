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