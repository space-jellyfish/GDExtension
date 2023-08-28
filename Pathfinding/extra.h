/* for 2d typed array
struct LevelHasher {
    std::size_t operator() (const TypedArray<TypedArray<int>>& level) const {
        std::size_t hash = 0;

        for (int y=0; y < level.size(); ++y) {
			TypedArray<int> row = level[y];
            for (int x=0; x < row.size(); ++x) {
                hash ^= std::hash<int>{}(row[x]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            }
        }
        return hash;
    }
};
*/

/* testing
    std::vector<std::vector<int>> test = {{1, 2}, {3, 4}};
    std::vector<std::vector<int>> toast = {{1, 2}, {3, 4}};

    std::unordered_map<std::vector<std::vector<int>>, bool, LevelHasher> visited;

    visited[test] = true;
    test[0][0] = 5;
    //std::cout << visited[toast] << std::endl;

    std::priority_queue<LevelState*, std::vector<LevelState*>, LevelStateCompare> wavefront;
    if (!wavefront.empty()) {
        std::cout << "nonempty" << std::endl;
    }
    else {
        std::cout << "empty" << std::endl;
    }
    LevelState* first = new LevelState;
    LevelState* second = new LevelState;
    first->f = 1;
    second->f = 2;
    wavefront.push(first);
    wavefront.push(second);
    //std::cout << wavefront.top()->f << std::endl;
    delete first;
    delete second;

    //assignVector(toast);
*/

/* can't encode tile on top of membrane/savepoint/goal
Stuff IDs
	2			1
	1			0
	0			-20
	-1			-21
	-2			-1
	empty		-22
	savepoint	-23
	goal			-24
	black wall	-40
	membrane	-41
	blue wall		-42
	red wall		-43
*/

/*
template <typename T> std::vector<T> typed_array_to_vector_1d(TypedArray<T>& arr);
template <typename T> std::vector<std::vector<T>> typed_array_to_vector_2d(TypedArray<TypedArray<T>>& arr);
*/

/*
[[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
[-1, 0, 0, 0, 0, 0, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1],
[-1, 0, -2, -2, -2, -2, 0, -2, 0, -2, -2, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -1],
[-1, 0, -2, 0, 0, -2, 0, -2, 0, 0, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, 0, -2, -2, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -1],
[-1, 0, 0, 0, -2, -2, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -1],
[-1, -2, 0, -2, -2, 0, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -1],
[-1, 0, 0, -2, 0, 0, 0, 0, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, 0, -2, -2, 0, 0, 0, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, 0, -2, -2, -2, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, 0, 0, -2, -2, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, -2, -2, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 18, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, 0, -2, 0, -2, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 18, 1, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, -2, 0, -2, 0, 0, 0, 0, -2, -2, 0, 0, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, 18, 17, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, 0, -2, -2, -2, -2, 0, 0, 0, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, 17, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, 0, 0, 0, -2, -2, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, -2, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, 0, 0, 0, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -2, 0, -1],
[-1, 0, -2, 0, -2, 0, 0, 0, 0, 0, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, -2, 0, -2, 0, -1],
[96, 48, -2, 0, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 0, -2, 0, -2, 0, -1],
[96, 32, 0, 0, -2, 32, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, 0, 0, -1],
[-1, -1, -1, -1, -1, 96, 96, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]]
*/

//bool is_enclosed_by_wall(std::vector<std::vector<int>> level, std::vector<std::vector<bool>>& wall_visited, Vector2i wall_pos, Vector2i end_pos);

/* from try_split()
	//check if possible without pushing
	if (!is_player && target_val >> 5 == 1) { //blocked by membrane
		return std::vector<std::vector<int>>();
	}
	else if (target_val == StuffId::EMPTY || target_val == StuffId::SAVEPOINT || target_val == StuffId::GOAL || (is_player && target_val == StuffId::MEMBRANE)) {
		//split into tile-free location
		level[pos.y][pos.x] -= pow_sign;
		level[target.y][target.x] += split_val;
	}
	else {
		int split_pow = abs(split_val - StuffId::POW_OFFSET);
		target_val %= StuffId::MEMBRANE;
		int target_pow = abs(target_val - StuffId::POW_OFFSET);
		if (split_pow == target_pow && split_pow < tile_pow_max) { //split-merge
			level[pos.y][pos.x] -= pow_sign;
			if (split_pow + target_pow == StuffId::MEMBRANE) { //opposite sign
				level[target.y][target.x] += StuffId::ZERO - target_val;
			}
			else { //same sign
				level[target.y][target.x] += pow_sign;
			}
		}
	}
*/

/*
		if (stack.empty()) {
			//move pruned elements into stack
			std::vector<LevelState*> temp;
			temp.reserve(pruned.size());
			while (!pruned.empty()) {
				temp.push_back(pruned.top());
				pruned.pop();
			}
			for (int i=temp.size()-1; i >= 0; --i) {
				stack.push(temp[i]);
			}

			//update threshold
			threshold = temp.front()->f;
			UtilityFunctions::print("PF NEW ITERATION");
		}
*/

/*
				if (visited.count(key)) { //level state exists
					temp = visited.at(key);

					if (curr->g + 1 < temp->g) { //level state exists
						temp->g = curr->g + 1;
						temp->f = temp->g + temp->h;
						temp->prev = curr;
						temp->prev_action = action;
					}
				}
*/

/*
Array vector_to_array_1d_reversed(std::vector<Vector3i>& vec) {
	Array ans;
	ans.resize(vec.size());
	for (int i=0; i < vec.size(); ++i) {
		ans[i] = vec[vec.size() - 1 - i];
	}
	return ans;
}
*/