/*
redo path find when a tile along path changes
get lv array on ready, use set_cell to keep it updated
treat cells outside lv array as wall
non-player tiles can leave, but not enter membrane
two +-2^12 tiles cannot merge

Sources:
https://en.wikipedia.org/wiki/Iterative_deepening_A*
https://research.cs.wisc.edu/techreports/1970/TR88.pdf
http://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-aaai11.pdf
*/

#include "pathfinder.h"

using namespace godot;

template <typename T> std::vector<T> array_to_vector_1d(const Array& arr);
template <typename T> std::vector<std::vector<T>> array_to_vector_2d(const Array& arr);
template <typename T> std::vector<std::vector<std::vector<T>>> array_to_vector_3d(const Array& arr);
template <typename T> Array vector_to_array_1d(const std::vector<T>& vec);
template <typename T> Array vector_to_array_2d(const std::vector<std::vector<T>>& vec);
void print_path(const Array& path);
void print_pos(const Vector2& pos);


//treats ZERO and EMPTY as different level states
//for max performance, use EMPTY in place of ZERO in level (if 0s can be safely ignored)
Array Pathfinder::pathfind(int search_type, const Array& level, Vector2i start, Vector2i end) {

	//wall/membrane check
	std::vector<std::vector<int>> level_vec = array_to_vector_2d<int>(level);
	int end_back_index = back_index(level_vec[end.y][end.x]);
	if (is_wall(end_back_index) || (end_back_index == 1 && !is_player)) {
		return Array();
	}

	//enclosure check
	if (is_enclosed(level_vec, end, start, is_player)) {
		return Array();
	}

	//hash initial state
	size_t hash = z_hash(level_vec, start);

    switch(search_type) {
		case SearchType::JPASTAR:
			return pathfind_jpastar(hash, level_vec, start, end);
		case SearchType::IDASTAR:
			return pathfind_idastar(hash, level_vec, start, end);
		case SearchType::ASTAR:
			return pathfind_astar(hash, level_vec, start, end);
		case SearchType::STRAIGHT:
			return pathfind_straight(hash, level_vec, start, end);
		case SearchType::MERGE_GREEDY:
			return pathfind_merge_greedy(hash, level_vec, start, end);
		case SearchType::MERGE_LARGE_TO_SMALL:
			return pathfind_merge_lts(hash, level_vec, start, end);
		case SearchType::MERGE_SMALL_TO_LARGE:
			return pathfind_merge_stl(hash, level_vec, start, end);
		default:
			return pathfind_jpastar(hash, level_vec, start, end);
	}
}

Array Pathfinder::pathfind_jpastar(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {

	Vector2i level_size(level[0].size(), level.size());

}

Array Pathfinder::pathfind_idastar(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {

	Vector2i level_size(level[0].size(), level.size());
	std::stack<LevelStateDFS*, std::vector<LevelStateDFS*>> stack;
	Array path;
	//std::unordered_map<std::pair<Vector2i, std::vector<std::vector<int>>>, LevelStateDFS*, LevelStateHasher, LevelStateEquator> in_path;
	std::unordered_map<LevelStateDFS*, LevelStateDFS*, LevelStateHashGetter, LevelStateEquator> in_path;

	int root_h = heuristic(start, end);
	int threshold = root_h;
	int next_threshold = std::numeric_limits<int>::max();

	//add root level state
	LevelStateDFS* root = memnew(LevelStateDFS(level_size));
	//LevelStateDFS* root = new LevelStateDFS(level_size);
	root->pos = start;
	root->g = 0;
	root->h = root_h;
	root->f = root_h;
	root->level = level;
	root->hash = hash;
	in_path[root] = root;
	stack.push(root);

	while (!stack.empty() || next_threshold != std::numeric_limits<int>::max()) {
		if (stack.empty()) {
			//add first level state
			LevelStateDFS* first = memnew(LevelStateDFS(level_size));
			//LevelStateDFS* first = new LevelStateDFS(level_size);
			first->pos = start;
			first->g = 0;
			first->h = root_h;
			first->f = root_h;
			first->level = level;
			first->hash = hash;
			in_path[first] = first;
			stack.push(first);

			//update threshold
			threshold = next_threshold;
			next_threshold = std::numeric_limits<int>::max();
			UtilityFunctions::print("PF NEW ITERATION, threshold = ", threshold);
		}
		//remove from stack
		LevelStateDFS* curr = stack.top();
		stack.pop();

		//add to path
		if (curr->g) {
			path.push_back(curr->prev_action);
		}
		in_path[curr] = curr;

		//check if end
		if (curr->pos == end) {
			UtilityFunctions::print("PF FOUND PATH");
			while (!stack.empty()) {
				memdelete(stack.top());
				//delete stack.top();
				stack.pop();
			}
			for (auto& entry : in_path) {
				memdelete(entry.second);
				//delete entry.second;
			}
			return path;
		}

		//check depth
		if (curr->g < max_depth) {
			//find neighbors
			int curr_val = curr->level[curr->pos.y][curr->pos.x] & 0x1f; //tile value
			bool can_split = is_splittable(curr_val);
			std::vector<LevelStateDFS*> neighbors;

			for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
				if (action_type == ActionType::SPLIT && !can_split) {
					continue;
				}

				//dir is (x, y), action is (x, y, *)
				for (Vector2i dir : {Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0), Vector2i(0, 1)}) {
					Vector3i action = Vector3i(dir.x, dir.y, action_type);
					size_t temp_hash = curr->hash;
					std::vector<std::vector<int>> temp_level = curr->level;
					try_action(temp_hash, temp_level, curr->pos, action);

					if (temp_level.empty()) { //action not possible
						continue;
					}
					//std::pair<Vector2i, std::vector<std::vector<int>>> key(curr->pos + dir, temp_level);

					LevelStateDFS* temp = memnew(LevelStateDFS(level_size));
					//LevelStateDFS* temp = new LevelStateDFS(level_size);
					temp->pos = curr->pos + dir;
					temp->level = temp_level;
					//std::cout << "temp level size: " << temp->level.size() << std::endl;
					temp->hash = temp_hash;
					if (in_path.find(temp) == in_path.end()) { //find() uses both hasher and equator
						temp->prev = curr;
						temp->prev_action = action;
						temp->g = curr->g + 1;
						temp->h = heuristic(temp->pos, end);
						temp->f = temp->g + temp->h;

						//add level state
						if (temp->f <= threshold) {
							neighbors.push_back(temp);
						}
						else if (temp->f < next_threshold) { //update next threshold
							next_threshold = temp->f;
						}
					}
					else {
						memdelete(temp);
						//delete temp;
					}
				}
			} //end find neighbors

			//add neighbors (in order)
			std::sort(neighbors.begin(), neighbors.end(), [](LevelStateDFS* first, LevelStateDFS* second) {
				if (first->f > second->f) {
					return true;
				}
				if (first->f < second->f) {
					return false;
				}
				return first->g < second->g;
			});
			for (LevelStateDFS* temp : neighbors) {
				stack.push(temp);
				++curr->child_count;
			}
		}

		//backtrack (Delete childless level states)
		while (curr != NULL && curr->child_count == 0) {
			//get parent pointer
			LevelStateDFS* parent = curr->prev;

			//Delete curr
			if (!path.is_empty()) {
				path.pop_back();
			}
			in_path.erase(curr);
			memdelete(curr);
			//delete curr;

			//decrement parent child_count
			if (parent != NULL) {
				--parent->child_count;
			}

			//update curr
			curr = parent;
		}
	}

	UtilityFunctions::print("PF NO PATH FOUND");
	return Array();
}

Array Pathfinder::pathfind_astar(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {
	
	Vector2i level_size(level[0].size(), level.size());
	std::priority_queue<LevelStateBFS*, std::vector<LevelStateBFS*>, LevelStateComparer> wavefront;
	//std::unordered_map<std::pair<Vector2i, std::vector<std::vector<int>>>, LevelStateBFS*, LevelStateHasher, LevelStateEquator> visited;
	std::unordered_map<LevelStateBFS*, LevelStateBFS*, LevelStateHashGetter, LevelStateEquator> visited;
	
	//add first level state
	LevelStateBFS* first = memnew(LevelStateBFS(level_size));
	//LevelStateBFS* first = new LevelStateBFS(level_size);
	first->pos = start;
	first->g = 0;
	first->h = heuristic(start, end);
	first->f = first->g + first->h;
	first->level = level;
	first->hash = hash;
	visited[first] = first;
	wavefront.push(first);

	//expand until end reaches top of queue
	while (!wavefront.empty()) {
		//get top
		LevelStateBFS* curr = wavefront.top();
		wavefront.pop();

		//check if end
		if (curr->pos == end) {
			UtilityFunctions::print("PF FOUND PATH");
			Array ans = curr->trace_path();
			for (auto& entry : visited) {
				memdelete(entry.second);
				//delete entry.second;
			}
			return ans;
		}

		//check depth
		if (curr->g >= max_depth) {
			continue;
		}

		//add/update neighbors
		int curr_val = curr->level[curr->pos.y][curr->pos.x] & 0x1f; //tile value
		bool can_split = is_splittable(curr_val);

		for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
			if (action_type == ActionType::SPLIT && !can_split) {
				continue;
			}

			//dir is (x, y), action is (x, y, *)
			for (Vector2i dir : {Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0), Vector2i(0, 1)}) {
				Vector3i action = Vector3i(dir.x, dir.y, action_type);
				size_t temp_hash = curr->hash;
				std::vector<std::vector<int>> temp_level = curr->level;
				try_action(temp_hash, temp_level, curr->pos, action);
				//UtilityFunctions::print("PF TEMP LEVEL SIZE: ", (int)temp_level.size());

				if (temp_level.empty()) { //action not possible
					continue;
				}
				//std::pair<Vector2i, std::vector<std::vector<int>>> key(curr->pos + dir, temp_level);

				LevelStateBFS* temp = memnew(LevelStateBFS(level_size));
				//LevelStateBFS* temp = new LevelStateBFS(level_size);
				temp->pos = curr->pos + dir;
				temp->level = temp_level;
				temp->hash = temp_hash;
				auto old_it = visited.find(temp);
				if (old_it != visited.end()) { //key exists
					memdelete(temp);
					//delete temp;
					temp = old_it->second;

					//if path better, update heuristic and prev
					if (curr->g + 1 < temp->g) {
						temp->g = curr->g + 1;
						temp->f = temp->g + temp->h;
						temp->prev = curr;
						temp->prev_action = action;

						//force queue update with make_heap()?
						wavefront.push(temp);
					}
				}
				else { //create New level state
					temp->prev = curr;
					temp->prev_action = action;
					temp->g = curr->g + 1;
					temp->h = heuristic(temp->pos, end);
					temp->f = temp->g + temp->h;
					visited[temp] = temp;
					wavefront.push(temp);
				}
			}
		} //end add/update neighbors
	} //end search

	UtilityFunctions::print("PF NO PATH FOUND");
	for (auto& entry : visited) {
		memdelete(entry.second);
		//delete entry.second;
	}
	return Array(); //no path found
}

Array Pathfinder::pathfind_straight(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {
	return Array();
}

Array Pathfinder::pathfind_merge_greedy(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {
	return Array();
}

Array Pathfinder::pathfind_merge_lts(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {
	return Array();
}

Array Pathfinder::pathfind_merge_stl(size_t hash, std::vector<std::vector<int>>& level, Vector2i start, Vector2i end) {
	return Array();
}

//assume dir is (y, x)
//assume level nonempty
//assume cell at pos is a tile
//assume tile_pow_max <= 14
//won't modify hash if action failed
//returns updated level if action possible, else empty vector
void Pathfinder::try_action(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector3i action) {
	//std::cout << "start try action" << std::endl;
	//std::cout << "ACTION LEVEL SIZE: " << level.size() << std::endl;
	Vector2i dir = Vector2i(action.x, action.y);
	int dist_to_out = get_dist_to_out(level, pos, dir);

	switch(action.z) {
		case ActionType::SLIDE:
			try_slide(hash, level, pos, dir, dist_to_out);
			return;
		case ActionType::SPLIT:
			try_split(hash, level, pos, dir, dist_to_out);
			return;
		default:
			try_slide(hash, level, pos, dir, dist_to_out);
	}
	//std::cout << "end try action" << std::endl;
	//std::cout << "level size: " << level.size() << std::endl;
}

void Pathfinder::try_slide(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, int dist_to_out) {
	//std::cout << "start try slide"; print_pos(pos);
	//std::cout << "level size: " << level.size() << std::endl;

	Vector2i curr_pos = pos;
	int curr_cell_id = level[pos.y][pos.x];
	int curr_tile_id = get_tile_id(curr_cell_id);
	std::vector<int> tile_ids = {curr_tile_id}; //store these bc mod is expensive

	Vector2i next_pos;
	int next_cell_id;
	int next_back_id;
	int next_tile_id;

	int tile_push_count = 0;
	do {
		//std::cout << "start bound check" << std::endl;
		//next_pos, bound check
		next_pos = curr_pos + dir;
		--dist_to_out;
		if (dist_to_out <= 0) {
			level.clear();
			return; //out of bounds
		}

		next_cell_id = level[next_pos.y][next_pos.x];
		next_back_id = back_index(next_cell_id);
		if (is_wall(next_back_id)) {
			level.clear();
			return; //obstructed by wall
		}
		if ((!is_player || tile_push_count) && next_back_id == CellId::MEMBRANE) {
			level.clear();
			return; //obstructed by membrane
		}
		//std::cout << "start merge check" << std::endl;

		next_tile_val = next_val & 0x1f;
		int curr_tile_pow = abs(curr_tile_val - CellId::ZERO) - 1;
		int next_tile_pow = abs(next_tile_val - CellId::ZERO) - 1;
		bool mergeable_pows = (curr_tile_pow == next_tile_pow && curr_tile_pow != tile_pow_max);
		if (next_tile_val == CellId::ZERO || (next_tile_val != CellId::EMPTY && (mergeable_pows || curr_tile_val == CellId::ZERO))) { //merge
			//std::cout << "start merge" << std::endl;
			//push 0?
			if (tile_push_count != tile_push_limit && next_tile_val == CellId::ZERO) { //can push if unobstructed
				Vector2i temp_pos = next_pos + dir;
				if (within_bounds(temp_pos)) {
					int temp_val = level[temp_pos.y][temp_pos.x];
					if (temp_val == CellId::EMPTY || temp_val == CellId::SAVEPOINT || temp_val == CellId::GOAL) {
						level[temp_pos.y][temp_pos.x] += CellId::ZERO;
						update_hash_tile_id(hash, temp_pos, CellId::ZERO);
					}
				} //else 0 gets popped
			}

			//set id at merge location
			if (curr_tile_val == CellId::ZERO) {}
			else if (next_tile_val == CellId::ZERO) {
				level[next_pos.y][next_pos.x] += curr_tile_val - next_tile_val;
				update_hash_tile_id(hash, next_pos, next_tile_val);
				update_hash_tile_id(hash, next_pos, curr_tile_val);
			}
			else if (curr_tile_val == CellId::POS_ONE || curr_tile_val == CellId::NEG_ONE) { //ones
				if (curr_tile_val + next_tile_val == CellId::MEMBRANE) { //opposite sign
					level[next_pos.y][next_pos.x] += CellId::ZERO - next_tile_val;
					update_hash_tile_id(hash, next_pos, next_tile_val);
					update_hash_tile_id(hash, next_pos, CellId::ZERO);
				}
				else { //same sign
					int dval = (curr_tile_val > CellId::ZERO) ? -14 : 14;
					level[next_pos.y][next_pos.x] += dval;
					update_hash_tile_id(hash, next_pos, next_tile_val);
					update_hash_tile_id(hash, next_pos, next_tile_val + dval);
				}
			}
			else {
				if (curr_tile_pow + next_tile_pow == CellId::MEMBRANE) { //opposite sign
					level[next_pos.y][next_pos.x] += CellId::ZERO - next_tile_val;
					update_hash_tile_id(hash, next_pos, next_tile_val);
					update_hash_tile_id(hash, next_pos, CellId::ZERO);
				}
				else { //same sign
					int tile_sign = (curr_tile_val >= CellId::ZERO) ? 1 : -1;
					level[next_pos.y][next_pos.x] += tile_sign;
					update_hash_tile_id(hash, next_pos, next_tile_val);
					update_hash_tile_id(hash, next_pos, next_tile_val + tile_sign);
				}
			}

			//shift rest of line
			int temp_val = tile_vals.back();
			tile_vals.pop_back();
			while (!tile_vals.empty()) {
				level[curr_pos.y][curr_pos.x] += tile_vals.back() - temp_val;
				update_hash_tile_id(hash, curr_pos, temp_val);
				update_hash_tile_id(hash, curr_pos, tile_vals.back());
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				curr_pos -= dir;
			}

			//curr_pos is now pos
			level[curr_pos.y][curr_pos.x] -= temp_val;
			update_hash_tile_id(hash, curr_pos, temp_val);
			update_hash_tile_type(hash, pos, pos + dir);
			return;
		}
		else if (next_tile_val == 0) { //slide
			//std::cout << "start slide" << std::endl;
			//shift line
			int temp_val = next_tile_val;
			while (!tile_vals.empty()) {
				level[next_pos.y][next_pos.x] += tile_vals.back() - temp_val;
				update_hash_tile_id(hash, next_pos, temp_val);
				update_hash_tile_id(hash, next_pos, tile_vals.back());
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				next_pos -= dir;
			}

			//next_pos is now pos
			level[next_pos.y][next_pos.x] -= temp_val;
			update_hash_tile_id(hash, next_pos, temp_val);
			update_hash_tile_type(hash, pos, pos + dir);
			return;
		}

		curr_pos = next_pos;
		curr_val = next_val;
		curr_tile_val = next_tile_val;
		tile_vals.push_back(curr_tile_val);
		++tile_push_count;
	} while (tile_push_count <= tile_push_limit);

	//push limit exceeded
	level.clear();
	//std::cout << "end try slide" << std::endl;
}

//assume tile val can split
void Pathfinder::try_split(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, int dist_to_out) {
	//std::cout << "start try split" << std::endl;
	Vector2i target = pos + dir;
	if (!within_bounds(target)) {
		level.clear();
		return;
	}

	int tile_id = get_tile_id(level[pos.y][pos.x]);
	auto [tile_pow, tile_sign] = get_pow_and_sign(tile_id);

	//use try_slide() to handle push logic, then set splitted tile
	level[pos.y][pos.x] -= tile_sign;
	try_slide(hash, level, pos, dir, dist_to_out);
	if (level.empty()) { //slide (and thus split) not possible
		return;
	}

	int split_id = tile_id - tile_sign;
	level[pos.y][pos.x] += split_id;
	update_hash_tile_id(hash, pos, tile_id);
	//std::cout << "end try split" << std::endl;
}

//simple floodfill using A*
//assume start/end aren't walls
bool Pathfinder::is_enclosed(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, bool is_player) {

	std::priority_queue<TileState*, std::vector<TileState*>, TileStateComparer> wavefront;
	std::vector<std::vector<bool>> visited(level.size(), std::vector<bool>(level[0].size(), false));

	TileState* first = memnew(TileState);
	//TileState* first = new TileState;
	first->pos = start;
	first->h = heuristic(start, end);
	visited[start.y][start.x] = true;
	wavefront.push(first);

	while (!wavefront.empty()) {
		TileState* curr = wavefront.top();
		wavefront.pop();

		if (curr->pos == end) {
			return false;
		}

		for (Vector2i dir : {Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0), Vector2i(0, 1)}) {
			Vector2i next_pos = curr->pos + dir;
			if (next_pos.x < 0 || next_pos.y < 0 || next_pos.x >= level[0].size() || next_pos.y >= level.size()) { //can't
				continue;
			}
			int next_back_index = back_index(level[next_pos.y][next_pos.x]);
			if (is_wall(next_back_index) || (next_back_index == 1 && !is_player)) { //can't
				continue;
			}

			if (!visited[next_pos.y][next_pos.x]) {
				TileState* next = memnew(TileState);
				//TileState* next = new TileState;
				next->pos = next_pos;
				next->h = heuristic(next_pos, end);
				visited[next_pos.y][next_pos.x] = true;
				wavefront.push(next);
			}
		}
	}
	return true;
}

LevelStateBFS* Pathfinder::jump(LevelStateBFS* state, Vector2i dir, Vector2i end, bool can_split) { //use dp?
	Vector2i curr_pos = state->pos;
	size_t curr_hash = state->hash;
	std::vector<std::vector<int>> curr_level = state->level;

	while (within_bounds(curr_pos + dir)) {
		int next_back_index = back_index(state->level[curr_pos.y][curr_pos.x]);
		if (is_wall(next_back_index) || (!is_player && next_back_index == 1)) { //obstructed by wall/membrane
			return NULL;
		}

		//step
		for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
			if (action_type == ActionType::SPLIT && !can_split) {
				continue;
			}
			Vector3i action(dir.x, dir.y, action_type);
			try_action(curr_hash, curr_level, curr_pos, action);

		}



	}
}

//use manhattan distance for 4-directional movement
int Pathfinder::heuristic(Vector2i pos, Vector2i goal) {
	return abs(pos.x - goal.x) + abs(pos.y - goal.y);
}

void Pathfinder::testing() {
	UtilityFunctions::print("PF TESTING");
	//while (1);
	Array test;
	test.push_back(Array());
	Array first_row = test[0];
	int i = first_row[0];
}

bool Pathfinder::is_splittable(int tile_id) {
	return (val != CellId::NEG_ONE && val != CellId::POS_ONE && val != CellId::ZERO);
}

bool Pathfinder::is_wall(int back_id) {
	return back_id == CellId::BLACK_WALL || back_id == CellId::BLUE_WALL || back_id == CellId::RED_WALL;
}

int Pathfinder::get_tile_id(int cell_id) {
	return cell_id & 0x1f;
}

int Pathfinder::get_back_id(int cell_id) {
	return (cell_id >> 5) << 5;
}

std::pair<int, int> Pathfinder::get_pow_and_sign(int tile_id) {
	int pow = abs(tile_val - CellId::ZERO) - 1;
	int sign = (tile_val >= CellId::ZERO) ? 1 : -1;
	return std::make_pair(pow, sign);
}

int Pathfinder::get_dist_to_out(std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir) {
	if (dir.x) {
		if (dir.x > 0) {
			return level[0].size() - pos.x;
		}
		else {
			return pos.x + 1;
		}
	}
	else {
		if (dir.y > 0) {
			return level.size() - pos.y;
		}
		else {
			return pos.y + 1;
		}
	}
}

//generate array of random numbers used for zobrist hashing
//to access random number, use [pos.y][pos.x][s_id - CellId::RED_WALL]
void Pathfinder::generate_hash_keys(Vector2i resolution_t) {
	static bool generated = false;
	if (generated) {
		return;
	}

	//random size_t generator
	//std::random_device rd;
	std::mt19937_64 generator(0); //fixed seed is okay
	std::uniform_int_distribution<size_t> distribution(std::numeric_limits<size_t>::min(), std::numeric_limits<size_t>::max()); //inclusive?

	//tile_id hash keys
	for (int y=0; y < resolution_t.y; ++y) {
		for (int x=0; x < resolution_t.x; ++x) {
			tile_id_hash_keys[y][x][0] = 0;
			for (int tile_id=1; tile_id < CellId::MEMBRANE; ++tile_id) {
				tile_id_hash_keys[y][x][tile_id] = distribution(generator);
			}
		}
	}

	//tile_type hash keys
	for (int tile_type = 0; tile_type < TileType::REGULAR; ++tile_type) {
		for (int y=0; y < resolution_t.y; ++y) {
			for (int x=0; x < resolution_t.x; ++x) {
				tile_type_hash_keys[tile_type][y][x] = distribution(generator);
			}
		}
	}

	generated = true;
	//std::cout << "hash numbers generated" << std::endl;
}

size_t Pathfinder::z_hash(const std::vector<std::vector<int>>& level, const Vector2i player_pos, const std::vector<Vector2i>& hostiles_pos) {
	assert(level.size() <= MAX_SEARCH_HEIGHT && level[0].size() <= MAX_SEARCH_WIDTH);
	
	size_t hash = 0;
	for (int y=0; y < level.size(); ++y) {
		for (int x=0; x < level[0].size(); ++x) {
			int tile_id = get_tile_id(level[y][x]);
			hash ^= tile_id_hash_keys[y][x][tile_id];
		}
	}

	//player pos
	hash ^= tile_type_hash_keys[TileType::DARK][player_pos.y][player_pos.x];

	//hostiles' pos
	for (Vector2i pos : hostiles_pos) {
		hash ^= tile_type_hash_keys[TileType::HOSTILE][pos.y][pos.x];
	}

	return hash;
}

void Pathfinder::update_hash_tile_type(size_t& hash, int tile_type, Vector2i prev, Vector2i next) {
	hash ^= tile_type_hash_keys[tile_type][prev.y][prev.x];
	hash ^= tile_type_hash_keys[tile_type][next.y][next.x];
}

void Pathfinder::update_hash_tile_id(size_t& hash, Vector2i pos, int tile_id) {
	hash ^= tile_id_hash_keys[pos.y][pos.x][tile_id];
}

void Pathfinder::set_gv(Variant _gv) {
	gv = _gv;
}

Variant Pathfinder::get_gv() {
	//Variant ans = gv;
	//return ans;
	return gv;
}

void Pathfinder::set_tile_pow_max(int _tile_pow_max) {
	tile_pow_max = _tile_pow_max;
}

int Pathfinder::get_tile_pow_max() {
	return tile_pow_max;
}

void Pathfinder::set_max_depth(int _max_depth) {
	max_depth = _max_depth;
}

int Pathfinder::get_max_depth() {
	return max_depth;
}

void Pathfinder::set_tile_push_limit(int _tile_push_limit) {
	tile_push_limit = _tile_push_limit;
}

int Pathfinder::get_tile_push_limit() {
	return tile_push_limit;
}

void Pathfinder::set_is_player(bool _is_player) {
	is_player = _is_player;
}

bool Pathfinder::get_is_player() {
	return is_player;
}

LevelState::LevelState(Vector2i level_size) {
	level.resize(level_size.y);
	for (auto& row : level) {
		row.reserve(level_size.x);
	}
}

Array LevelStateBFS::trace_path() {
	Array ans;
	ans.resize(this->g);
	int index = this->g - 1;
	LevelStateBFS* curr = this;

	while (curr->prev != NULL) {
		ans[index] = curr->prev_action;
		curr = curr->prev;
		--index;
	}
	UtilityFunctions::print("PF TRACED PATH SIZE: ", ans.size());
	return ans;
}

void Pathfinder::_bind_methods() {
	//pathfind(int search_type, Array& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max)
	ClassDB::bind_method(D_METHOD("pathfind", "search_type", "level", "start", "end"), &Pathfinder::pathfind);
	ClassDB::bind_method(D_METHOD("testing"), &Pathfinder::testing);

	ClassDB::bind_static_method("Pathfinder", D_METHOD("generate_hash_keys", "resolution_t"), &Pathfinder::generate_hash_keys);
	//ClassDB::bind_method(D_METHOD("get_hash_arrays"), &Pathfinder::get_hash_arrays);

	ClassDB::bind_method(D_METHOD("set_gv", "_gv"), &Pathfinder::set_gv);
	ClassDB::bind_method(D_METHOD("get_gv"), &Pathfinder::get_gv);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gv"), "set_gv", "get_gv");

	ClassDB::bind_method(D_METHOD("set_tile_pow_max", "_tile_pow_max"), &Pathfinder::set_tile_pow_max);
	ClassDB::bind_method(D_METHOD("get_tile_pow_max"), &Pathfinder::get_tile_pow_max);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_pow_max"), "set_tile_pow_max", "get_tile_pow_max");

	ClassDB::bind_method(D_METHOD("set_max_depth", "_max_depth"), &Pathfinder::set_max_depth);
	ClassDB::bind_method(D_METHOD("get_max_depth"), &Pathfinder::get_max_depth);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_depth"), "set_max_depth", "get_max_depth");

	ClassDB::bind_method(D_METHOD("set_tile_push_limit", "_tile_push_limit"), &Pathfinder::set_tile_push_limit);
	ClassDB::bind_method(D_METHOD("get_tile_push_limit"), &Pathfinder::get_tile_push_limit);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_push_limit"), "set_tile_push_limit", "get_tile_push_limit");

	ClassDB::bind_method(D_METHOD("set_is_player", "_is_player"), &Pathfinder::set_is_player);
	ClassDB::bind_method(D_METHOD("get_is_player"), &Pathfinder::get_is_player);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_player"), "set_is_player", "get_is_player");
}







template <typename T> std::vector<T> array_to_vector_1d(const Array& arr) {
	std::vector<T> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(arr[i]);
	}
	return ans;
}

template <> std::vector<size_t> array_to_vector_1d<size_t>(const Array& arr) {
	std::vector<size_t> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(uint64_t(arr[i]));
	}
	return ans;
}

template <typename T> std::vector<std::vector<T>> array_to_vector_2d(const Array& arr) {
	std::vector<std::vector<T>> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(array_to_vector_1d<T>(arr[i]));
	}
	return ans;
}

template <typename T> std::vector<std::vector<std::vector<T>>> array_to_vector_3d(const Array& arr) {
	std::vector<std::vector<std::vector<T>>> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(array_to_vector_2d<T>(arr[i]));
	}
	return ans;
}

template <typename T> Array vector_to_array_1d(const std::vector<T>& vec) {
	Array ans;
	for (T i : vec) {
		ans.push_back(i);
	}
	return ans;
}

template <typename T> Array vector_to_array_2d(const std::vector<std::vector<T>>& vec) {
	Array ans;
	for (const std::vector<T>& row : vec) {
		ans.push_back(vector_to_array_1d<T>(row));
	}
	return ans;
}

void print_path(const Array& path) {
	for (int i=0; i < path.size(); ++i) {
		Vector3 action = path[i];
		std::cout << "(" << action.x << ", " << action.y << ", " << action.z << ")" << std::endl;
	}
}

void print_pos(const Vector2& pos) {
	std::cout << "(" << pos.x << ", " << pos.y << ")" << std::endl;
}

/*
//for debugging
int main(void) {
	//std::ios_base::sync_with_stdio(false);
	//std::cerr << "HERE" << std::flush << std::endl;
	std::vector<std::vector<int>> level_vec = {{17, 0, 0}, {16, 17, 17}};
	Array level_arr = vector_to_array_2d(level_vec);

	Vector2i start(0, 1);
	Vector2i end(1, 0);
	Pathfinder p;
	std::cout << "HA before" << std::flush << std::endl;
	p.generate_hash_keys(Vector2i(3, 2));
	std::cout << "HA Generated" << std::flush << std::endl;
	p.set_tile_pow_max(12);
	p.set_tile_push_limit(1);
	p.set_is_player(true);
	p.set_max_depth(500);
	Array path = p.pathfind(SearchType::IDASTAR, level_arr, start, end);
	print_path(path);
}*/