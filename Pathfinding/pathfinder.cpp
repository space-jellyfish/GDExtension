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
#include <vector>
#include <queue>
#include <unordered_map>
#include <stack>
#include <random>
#include <functional>

using namespace godot;

std::vector<std::vector<std::vector<size_t>>> Pathfinder::level_hash_numbers = {};
std::vector<size_t> Pathfinder::x_hash_numbers = {};
std::vector<size_t> Pathfinder::y_hash_numbers = {};

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
	int end_val = level_vec[end.y][end.x];
	if (end_val < 0 || (end_val >> 5 == 1 && !is_player)) {
		return Array();
	}

	//enclosure check
	if (is_enclosed(level_vec, end, start, is_player)) {
		return Array();
	}

	//hash initial state
	size_t hash = z_hash(level_vec, start);

    switch(search_type) {
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
			return pathfind_idastar(hash, level_vec, start, end);
	}
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
			bool can_split = (curr_val != StuffId::NEG_ONE && curr_val != StuffId::POS_ONE && curr_val != StuffId::ZERO);
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
		bool can_split = (curr_val != StuffId::NEG_ONE && curr_val != StuffId::POS_ONE && curr_val != StuffId::ZERO);

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
//returns updated level if action possible, else empty vector
void Pathfinder::try_action(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector3i action) {
	//std::cout << "start try action" << std::endl;
	//std::cout << "ACTION LEVEL SIZE: " << level.size() << std::endl;
	Vector2i dir = Vector2i(action.x, action.y);

	switch(action.z) {
		case ActionType::SLIDE:
			try_slide(hash, level, pos, dir);
			return;
		case ActionType::SPLIT:
			try_split(hash, level, pos, dir);
			return;
		default:
			try_slide(hash, level, pos, dir);
	}
	//std::cout << "end try action" << std::endl;
	//std::cout << "level size: " << level.size() << std::endl;
}

void Pathfinder::try_slide(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir) {
	//std::cout << "start try slide"; print_pos(pos);
	//std::cout << "level size: " << level.size() << std::endl;
	std::function<bool(Vector2i)> within_bounds;
	if (dir.x) {
		if (dir.x > 0) {
			within_bounds = [&](Vector2i new_pos)->bool { return new_pos.x < level[0].size(); };
		}
		else {
			within_bounds = [&](Vector2i new_pos)->bool { return new_pos.x >= 0; };
		}
	}
	else {
		if (dir.y > 0) {
			within_bounds = [&](Vector2i new_pos)->bool { return new_pos.y < level.size(); };
		}
		else {
			within_bounds = [&](Vector2i new_pos)->bool { return new_pos.y >= 0; };
		}
	}

	Vector2i curr_pos = pos;
	int curr_val = level[pos.y][pos.x];
	int curr_tile_val = curr_val & 0x1f;
	std::vector<int> tile_vals = {curr_tile_val}; //store these bc mod is expensive

	Vector2i next_pos;
	int next_val;
	int next_tile_val;

	int tile_push_count = 0;
	do {
		//std::cout << "start bound check" << std::endl;
		//next_pos, bound check
		next_pos = curr_pos + dir;
		if (!within_bounds(next_pos)) {
			level.clear();
			return; //out of bounds
		}

		next_val = level[next_pos.y][next_pos.x];
		if (next_val < 0) {
			level.clear();
			return; //obstructed by wall
		}
		if ((!is_player || tile_push_count) && next_val >> 5 == 1) {
			level.clear();
			return; //obstructed by membrane
		}
		//std::cout << "start merge check" << std::endl;

		next_tile_val = next_val & 0x1f;
		int curr_tile_pow = abs(curr_tile_val - StuffId::ZERO);
		int next_tile_pow = abs(next_tile_val - StuffId::ZERO);
		bool mergeable_pows = (curr_tile_pow == next_tile_pow && curr_tile_pow != tile_pow_max);
		if (next_tile_val == StuffId::ZERO || (next_tile_val != StuffId::EMPTY && (mergeable_pows || curr_tile_val == StuffId::ZERO))) { //merge
			//std::cout << "start merge" << std::endl;
			//push 0?
			if (tile_push_count != tile_push_limit && next_tile_val == StuffId::ZERO) { //can push if unobstructed
				Vector2i temp_pos = next_pos + dir;
				if (within_bounds(temp_pos)) {
					int temp_val = level[temp_pos.y][temp_pos.x];
					if (temp_val == StuffId::EMPTY || temp_val == StuffId::SAVEPOINT || temp_val == StuffId::GOAL) {
						level[temp_pos.y][temp_pos.x] += StuffId::ZERO;
						update_hash_tile(hash, temp_pos, StuffId::ZERO);
					}
				} //else 0 gets popped
			}

			//set id at merge location
			if (curr_tile_val == StuffId::ZERO) {}
			else if (next_tile_val == StuffId::ZERO) {
				level[next_pos.y][next_pos.x] += curr_tile_val - next_tile_val;
				update_hash_tile(hash, next_pos, next_tile_val);
				update_hash_tile(hash, next_pos, curr_tile_val);
			}
			else if (curr_tile_val == StuffId::POS_ONE || curr_tile_val == StuffId::NEG_ONE) { //ones
				if (curr_tile_val + next_tile_val == StuffId::MEMBRANE) { //opposite sign
					level[next_pos.y][next_pos.x] += StuffId::ZERO - next_tile_val;
					update_hash_tile(hash, next_pos, next_tile_val);
					update_hash_tile(hash, next_pos, StuffId::ZERO);
				}
				else { //same sign
					int dval = (curr_tile_val > StuffId::ZERO) ? -14 : 14;
					level[next_pos.y][next_pos.x] += dval;
					update_hash_tile(hash, next_pos, next_tile_val);
					update_hash_tile(hash, next_pos, next_tile_val + dval);
				}
			}
			else {
				if (curr_tile_pow + next_tile_pow == StuffId::MEMBRANE) { //opposite sign
					level[next_pos.y][next_pos.x] += StuffId::ZERO - next_tile_val;
					update_hash_tile(hash, next_pos, next_tile_val);
					update_hash_tile(hash, next_pos, StuffId::ZERO);
				}
				else { //same sign
					int pow_sign = (curr_tile_val > StuffId::ZERO) ? 1 : -1;
					level[next_pos.y][next_pos.x] += pow_sign;
					update_hash_tile(hash, next_pos, next_tile_val);
					update_hash_tile(hash, next_pos, next_tile_val + pow_sign);
				}
			}

			//shift rest of line
			int temp_val = tile_vals.back();
			tile_vals.pop_back();
			while (!tile_vals.empty()) {
				level[curr_pos.y][curr_pos.x] += tile_vals.back() - temp_val;
				update_hash_tile(hash, curr_pos, temp_val);
				update_hash_tile(hash, curr_pos, tile_vals.back());
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				curr_pos -= dir;
			}

			//curr_pos is now pos
			level[curr_pos.y][curr_pos.x] -= temp_val;
			update_hash_tile(hash, curr_pos, temp_val);
			update_hash_pos(hash, pos, pos + dir);
			return;
		}
		else if ((next_val & 0x1f) == 0) { //slide
			//std::cout << "start slide" << std::endl;
			//shift line
			int temp_val = next_tile_val;
			while (!tile_vals.empty()) {
				level[next_pos.y][next_pos.x] += tile_vals.back() - temp_val;
				update_hash_tile(hash, next_pos, temp_val);
				update_hash_tile(hash, next_pos, tile_vals.back());
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				next_pos -= dir;
			}

			//next_pos is now pos
			level[next_pos.y][next_pos.x] -= temp_val;
			update_hash_tile(hash, next_pos, temp_val);
			update_hash_pos(hash, pos, pos + dir);
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
void Pathfinder::try_split(size_t& hash, std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir) {
	//std::cout << "start try split" << std::endl;
	Vector2i target = pos + dir;
	if (target.x < 0 || target.y < 0 || target.x >= level[0].size() || target.y >= level.size()) { //out of bounds
		level.clear();
		return;
	}

	int tile_val = level[pos.y][pos.x] & 0x1f; //don't care about back layer at pos
	int pow_sign = (tile_val > StuffId::ZERO) ? 1 : -1;
	int tile_pow = abs(tile_val - StuffId::ZERO);
	if (tile_pow == 1) {
		pow_sign *= -14;
	}

	//use try_slide() to handle push logic, then set splitted tile
	level[pos.y][pos.x] -= pow_sign;
	try_slide(hash, level, pos, dir);
	if (level.empty()) { //slide, and by extension split, not possible
		return;
	}

	int split_val = tile_val - pow_sign;
	level[pos.y][pos.x] += split_val;
	update_hash_tile(hash, pos, tile_val);
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
			int next_val = level[next_pos.y][next_pos.x];
			if (next_val < 0 || (next_val >> 5 == 1 && !is_player)) { //can't
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

//generate array of random numbers used for zobrist hashing
//to access random number, use [pos.y][pos.x][s_id - StuffId::RED_WALL]
void Pathfinder::generate_hash_numbers(Vector2i resolution_t) {
	static bool generated = false;
	if (generated) {
		return;
	}

	//random size_t generator
	//std::random_device rd;
	std::mt19937_64 generator(0); //fixed seed is okay
	std::uniform_int_distribution<size_t> distribution(std::numeric_limits<size_t>::min(), std::numeric_limits<size_t>::max()); //inclusive?

	//level hash numbers
	level_hash_numbers.reserve(resolution_t.y);

	for (int y=0; y < resolution_t.y; ++y) {
		std::vector<std::vector<size_t>> row;
		row.reserve(resolution_t.x);

		for (int x=0; x < resolution_t.x; ++x) {
			std::vector<size_t> stuff;
			stuff.reserve(StuffId::MEMBRANE);

			stuff.push_back(0); //no tile at cell
			for (int tile_val=1; tile_val < StuffId::MEMBRANE; ++tile_val) {
				stuff.push_back(distribution(generator));
			}
			row.push_back(stuff);
		}
		level_hash_numbers.push_back(row);
	}

	//pos hash numbers
	x_hash_numbers.reserve(resolution_t.x);
	y_hash_numbers.reserve(resolution_t.y);
	for (int x=0; x < resolution_t.x; ++x) {
		x_hash_numbers.push_back(distribution(generator));
	}
	for (int y=0; y < resolution_t.y; ++y) {
		y_hash_numbers.push_back(distribution(generator));
	}

	generated = true;
	//std::cout << "hash numbers generated" << std::endl;
}

size_t Pathfinder::z_hash(const std::vector<std::vector<int>>& level, const Vector2i pos) {
	size_t hash = 0;
	for (int y=0; y < level.size(); ++y) {
		//Array row = level_hash_numbers[y];
		for (int x=0; x < level[0].size(); ++x) {
			//Array cell = row[x];
			int tile_val = level[y][x] & 0x1f;
			//hash ^= uint64_t(cell[tile_val]);
			hash ^= level_hash_numbers[y][x][tile_val];
		}
	}
	//hash ^= uint64_t(x_hash_numbers[pos.x]);
	//hash ^= uint64_t(y_hash_numbers[pos.y]);
	hash ^= x_hash_numbers[pos.x];
	hash ^= y_hash_numbers[pos.y];
	return hash;
}

void Pathfinder::update_hash_pos(size_t& hash, Vector2i prev, Vector2i next) {
	//std::cout << "start update hash pos" << std::endl;
	//std::cout << "prev: "; print_pos(prev);
	//std::cout << "next: "; print_pos(next);
	if (prev.x != next.x) {
		//hash ^= uint64_t(x_hash_numbers[prev.x]);
		//hash ^= uint64_t(x_hash_numbers[next.x]);
		hash ^= x_hash_numbers[prev.x];
		hash ^= x_hash_numbers[next.x];
	}
	if (prev.y != next.y) {
		//hash ^= uint64_t(y_hash_numbers[prev.y]);
		//hash ^= uint64_t(y_hash_numbers[next.y]);
		hash ^= y_hash_numbers[prev.y];
		hash ^= y_hash_numbers[next.y];
	}
	//std::cout << "end update hash pos" << std::endl;
}

void Pathfinder::update_hash_tile(size_t& hash, Vector2i pos, int tile_val) {
	//std::cout << "start update hash tile" << std::endl;
	hash ^= level_hash_numbers[pos.y][pos.x][tile_val];
	//Array row = level_hash_numbers[pos.y];
	//Array cell = row[pos.x];
	//hash ^= uint64_t(cell[tile_val]);
	//std::cout << "end update hash tile" << std::endl;
}

void Pathfinder::set_gv(Variant _gv) {
	gv = _gv;
	//level_hash_numbers = gv->get(level_hash_numbers_str);
	//x_hash_numbers = gv->get(level_hash_numbers_str);
	//y_hash_numbers = gv->get(level_hash_numbers_str);
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

	ClassDB::bind_static_method("Pathfinder", D_METHOD("generate_hash_numbers", "resolution_t"), &Pathfinder::generate_hash_numbers);
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
	p.generate_hash_numbers(Vector2i(3, 2));
	std::cout << "HA Generated" << std::flush << std::endl;
	p.set_tile_pow_max(12);
	p.set_tile_push_limit(1);
	p.set_is_player(true);
	p.set_max_depth(500);
	Array path = p.pathfind(SearchType::IDASTAR, level_arr, start, end);
	print_path(path);
}*/