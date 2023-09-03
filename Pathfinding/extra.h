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

/*
int z_hash(const Vector2i pos, const std::vector<std::vector<int>>& level) {
	int hash = 0;

	return hash;
}

int cell_hash(const Vector2i pos, int id) {
	int hash = id;
	hash ^= std::hash<int>{}(pos.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
	hash ^= std::hash<int>{}(pos.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
	return hash;
}
*/

/*
std::vector<int> array_to_vector_1d_int(const Array& arr) {
	std::vector<int> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(int32_t(arr[i]));
	}
	return ans;
}

std::vector<std::vector<int>> array_to_vector_2d_int(const Array& arr) {
	std::vector<std::vector<int>> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(array_to_vector_1d_int((const Array&) arr[i]));
	}
	return ans;
}

std::vector<size_t> array_to_vector_1d_size_t(const Array& arr) {
	std::vector<size_t> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(uint64_t(arr[i]));
	}
	return ans;
}

std::vector<std::vector<size_t>> array_to_vector_2d_size_t(const Array& arr) {
	std::vector<std::vector<size_t>> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(array_to_vector_1d_size_t(arr[i]));
	}
	return ans;
}

std::vector<std::vector<std::vector<size_t>>> array_to_vector_3d_size_t(const Array& arr) {
	std::vector<std::vector<std::vector<size_t>>> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(array_to_vector_2d_size_t(arr[i]));
	}
	return ans;
}

std::vector<int> array_to_vector_1d_int(const Array& arr);
std::vector<std::vector<int>> array_to_vector_2d_int(const Array& arr);
std::vector<size_t> array_to_vector_1d_size_t(const Array& arr);
std::vector<std::vector<size_t>> array_to_vector_2d_size_t(const Array& arr);
std::vector<std::vector<std::vector<size_t>>> array_to_vector_3d_size_t(const Array& arr);
*/

/*
void Pathfinder::set_hash_numbers(const Array& level_nums, const Array& x_nums, const Array& y_nums) {
	level_hash_numbers = array_to_vector_3d<size_t>(level_nums);
	x_hash_numbers = array_to_vector_1d<size_t>(x_nums);
	y_hash_numbers = array_to_vector_1d<size_t>(y_nums);
}
*/

/*
	std::vector<std::vector<std::vector<size_t>>> level_hash_numbers; //don't regenerate for every tile
	std::vector<size_t> x_hash_numbers;
	std::vector<size_t> y_hash_numbers;

//generate array of random numbers used for zobrist hashing
//to access random number, use [pos.y][pos.x][s_id - StuffId::RED_WALL]
void Pathfinder::generate_hash_numbers(Vector2i resolution_t) {
	if (!level_hash_numbers.is_empty()) {
		return;
	}
	//random size_t generator
	//std::random_device rd;
	std::mt19937_64 generator(0); //fixed seed is okay
	std::uniform_int_distribution<size_t> distribution(std::numeric_limits<size_t>::min(), std::numeric_limits<size_t>::max()); //inclusive?

	//level hash numbers
	level_hash_numbers.resize(resolution_t.y);

	for (int y=0; y < resolution_t.y; ++y) {
		//std::vector<std::vector<size_t>> row;
		Array row = Array();
		level_hash_numbers[y] = row;
		row.resize(resolution_t.x);

		for (int x=0; x < resolution_t.x; ++x) {
			//std::vector<size_t> stuff;
			Array stuff = Array();
			row[x] = stuff;
			stuff.resize(StuffId::MEMBRANE);

			stuff[0] = 0; //no tile at cell
			for (int tile_val=1; tile_val < StuffId::MEMBRANE; ++tile_val) {
				stuff[tile_val] = uint64_t(distribution(generator));
			}
			//row.push_back(stuff);
		}
		//level_hash_numbers.push_back(row);
	}

	//pos hash numbers
	x_hash_numbers.resize(resolution_t.x);
	y_hash_numbers.resize(resolution_t.y);
	for (int x=0; x < resolution_t.x; ++x) {
		x_hash_numbers[x] = uint64_t(distribution(generator));
	}
	for (int y=0; y < resolution_t.y; ++y) {
		y_hash_numbers[y] = uint64_t(distribution(generator));
	}
}

*/

/* void _ready() override;
void Pathfinder::_ready() {
	//get gv and hash number array refs
	gv = get_node<Node>(gv_path);
	level_hash_numbers = gv->get(level_hash_numbers_str);
	x_hash_numbers = gv->get(x_hash_numbers_str);
	y_hash_numbers = gv->get(y_hash_numbers_str);
}
*/

/*
	const char* gv_path_cstr = "/root/GV";
	String gv_path_str = String(gv_path_cstr);
	NodePath gv_path = NodePath(gv_path_str);
*/

/*
	int id = 0;
	Variant give;

	void set_id(int _id);
	int get_id();

	void set_give(Variant _give);
	Variant get_give();

void Pathfinder::set_id(int _id) {
	id = _id;
}

int Pathfinder::get_id() {
	return id;
}

void Pathfinder::set_give(Variant _give) {
	give = _give;
}

Variant Pathfinder::get_give() {
	return give;
}

	ClassDB::bind_method(D_METHOD("set_id", "_id"), &Pathfinder::set_id);
	ClassDB::bind_method(D_METHOD("get_id"), &Pathfinder::get_id);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "id"), "set_id", "get_id");

	ClassDB::bind_method(D_METHOD("set_give", "_give"), &Pathfinder::set_give);
	ClassDB::bind_method(D_METHOD("get_give"), &Pathfinder::get_give);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "give"), "set_give", "get_give");
*/

/*


private:
	static std::vector<std::vector<std::vector<size_t>>> level_hash_numbers;
	static std::vector<size_t> x_hash_numbers;
	static std::vector<size_t> y_hash_numbers;
*/

/*
	Array level_hash_numbers;
	Array x_hash_numbers;
	Array y_hash_numbers;

void Pathfinder::get_hash_arrays() {
	level_hash_numbers = gv.get(level_hash_numbers_strn);
	x_hash_numbers = gv.get(x_hash_numbers_strn);
	y_hash_numbers = gv.get(y_hash_numbers_strn);
}
*/