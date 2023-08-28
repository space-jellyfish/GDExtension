/*
redo path find when a tile along path changes
get lv array on ready, use set_cell to keep it updated
treat cells outside lv array as wall
non-player tiles can leave, but not enter membrane
two +-2^12 tiles cannot merge
*/

#include "pathfinder.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <stack>

using namespace godot;

std::vector<int> array_to_vector_1d(const Array& arr);
std::vector<std::vector<int>> array_to_vector_2d(const Array& arr);
Array vector_to_array_1d(std::vector<int>& vec);
Array vector_to_array_2d(std::vector<std::vector<int>>& vec);
Array vector_to_array_1d_reversed(std::vector<Vector3i>& vec);
void print_vector_2d(std::vector<std::vector<int>>& v);


//treats ZERO and EMPTY as different level states
//for max performance, use EMPTY in place of ZERO in level (if 0s can be safely ignored)
Array Pathfinder::pathfind(int search_type, const Array& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {

	//wall/membrane check
	std::vector<std::vector<int>> level_vec = array_to_vector_2d(level);
	int end_val = level_vec[end.y][end.x];
	if (end_val < 0 || (end_val >> 5 == 1 && !is_player)) {
		return Array();
	}

	//enclosure check
	if (is_enclosed(level_vec, end, start, is_player)) {
		return Array();
	}

    switch(search_type) {
		case SearchType::IDASTAR:
			return pathfind_idastar(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::ASTAR:
			return pathfind_astar(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::STRAIGHT:
			return pathfind_straight(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::MERGE_GREEDY:
			return pathfind_merge_greedy(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::MERGE_LARGE_TO_SMALL:
			return pathfind_merge_lts(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::MERGE_SMALL_TO_LARGE:
			return pathfind_merge_stl(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		default:
			return pathfind_idastar(level_vec, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
	}
}

Array Pathfinder::pathfind_idastar(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {

	Vector2i level_size(level[0].size(), level.size());
	std::stack<LevelState*, std::vector<LevelState*>> stack;
	std::unordered_map<std::pair<Vector2i, std::vector<std::vector<int>>>, LevelState*, LevelStateHasher, LevelStateEquator> visited;
	int root_h = heuristic(start, end);
	int threshold = root_h;
	int next_threshold = std::numeric_limits<int>::max();

	//add root level state
	LevelState* root = new LevelState(level_size);
	root->pos = start;
	root->g = 0;
	root->h = root_h;
	root->f = root_h;
	root->level = level;
	visited[std::make_pair(start, level)] = root;
	stack.push(root);

	while (!stack.empty() || next_threshold != std::numeric_limits<int>::max()) {
		if (stack.empty()) {
			//delete old level states
			for (auto entry : visited) {
				delete entry.second;
			}
			visited.clear();

			//add first level state
			LevelState* first = new LevelState(level_size);
			first->pos = start;
			first->g = 0;
			first->h = root_h;
			first->f = root_h;
			first->level = level;
			visited[std::make_pair(start, level)] = first;
			stack.push(first);

			//update threshold
			threshold = next_threshold;
			next_threshold = std::numeric_limits<int>::max();
			UtilityFunctions::print("PF NEW ITERATION");
		}
		LevelState* curr = stack.top();
		stack.pop();

		//check if end
		if (curr->pos == end) {
			UtilityFunctions::print("PF FOUND END");
			Array ans = curr->trace_path();
			for (auto entry : visited) {
				delete entry.second;
			}
			return ans;
		}

		//check depth
		if (curr->g >= max_depth) {
			continue;
		}

		//add/update neighbors
		int curr_val = curr->level[curr->pos.y][curr->pos.x] % StuffId::MEMBRANE; //tile value
		bool can_split = (curr_val != StuffId::NEG_ONE && curr_val != StuffId::ZERO && curr_val != StuffId::POW_OFFSET);

		for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
			if (action_type == ActionType::SPLIT && !can_split) {
				continue;
			}

			//dir is (x, y), action is (x, y, *)
			for (Vector2i dir : {Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0), Vector2i(0, 1)}) {
				Vector3i action = Vector3i(dir.x, dir.y, action_type);
				std::vector<std::vector<int>> temp_level = try_action(curr->level, curr->pos, action, tile_push_limit, is_player, tile_pow_max);
				//UtilityFunctions::print("PF TEMP LEVEL SIZE: ", (int)temp_level.size());

				if (temp_level.empty()) { //action not possible
					continue;
				}
				std::pair<Vector2i, std::vector<std::vector<int>>> key(curr->pos + dir, temp_level);

				if (!visited.count(key)) { //create new level state
					LevelState* temp = new LevelState(level_size);
					temp->pos = curr->pos + dir;
					temp->prev = curr;
					temp->prev_action = action;
					temp->g = curr->g + 1;
					temp->h = heuristic(temp->pos, end);
					temp->f = temp->g + temp->h;
					temp->level = temp_level;
					visited[key] = temp;

					if (temp->f <= threshold) { //push level state
						stack.push(temp);
					}
					else { //update next threshold
						next_threshold = std::min(next_threshold, temp->f);
					}
				}
			}
		} //end add/update neighbors
	}

	UtilityFunctions::print("PF NO PATH FOUND");
	for (auto entry : visited) {
		delete entry.second;
	}
	return Array();
}

Array Pathfinder::pathfind_astar(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	
	Vector2i level_size(level[0].size(), level.size());
	std::priority_queue<LevelState*, std::vector<LevelState*>, LevelStateComparer> wavefront;
	std::unordered_map<std::pair<Vector2i, std::vector<std::vector<int>>>, LevelState*, LevelStateHasher, LevelStateEquator> visited;
	
	//add first level state
	LevelState* first = new LevelState(level_size);
	first->pos = start;
	first->g = 0;
	first->h = heuristic(start, end);
	first->f = first->g + first->h;
	first->level = level;
	visited[std::make_pair(start, level)] = first;
	wavefront.push(first);

	//expand until end reaches top of queue
	while (!wavefront.empty()) {
		//get top
		LevelState* curr = wavefront.top();
		wavefront.pop();

		//check if end
		if (curr->pos == end) {
			UtilityFunctions::print("PF FOUND END");
			Array ans = curr->trace_path();
			for (auto entry : visited) {
				delete entry.second;
			}
			return ans;
		}

		//check depth
		if (curr->g >= max_depth) {
			continue;
		}

		//add/update neighbors
		int curr_val = curr->level[curr->pos.y][curr->pos.x] % StuffId::MEMBRANE; //tile value
		bool can_split = (curr_val != StuffId::NEG_ONE && curr_val != StuffId::ZERO && curr_val != StuffId::POW_OFFSET);

		for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
			if (action_type == ActionType::SPLIT && !can_split) {
				continue;
			}

			//dir is (x, y), action is (x, y, *)
			for (Vector2i dir : {Vector2i(1, 0), Vector2i(0, -1), Vector2i(-1, 0), Vector2i(0, 1)}) {
				Vector3i action = Vector3i(dir.x, dir.y, action_type);
				std::vector<std::vector<int>> temp_level = try_action(curr->level, curr->pos, action, tile_push_limit, is_player, tile_pow_max);
				//UtilityFunctions::print("PF TEMP LEVEL SIZE: ", (int)temp_level.size());

				if (temp_level.empty()) { //action not possible
					continue;
				}
				std::pair<Vector2i, std::vector<std::vector<int>>> key(curr->pos + dir, temp_level);

				if (visited.count(key)) { //level state exists
					LevelState* temp = visited.at(key);

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
				else { //create new level state
					LevelState* temp = new LevelState(level_size);
					temp->pos = curr->pos + dir;
					temp->prev = curr;
					temp->prev_action = action;
					temp->g = curr->g + 1;
					temp->h = heuristic(temp->pos, end);
					temp->f = temp->g + temp->h;
					temp->level = temp_level;
					visited[key] = temp;
					wavefront.push(temp);
				}
			}
		} //end add/update neighbors
	} //end search

	UtilityFunctions::print("PF NO PATH FOUND");
	for (auto entry : visited) {
		delete entry.second;
	}
	return Array(); //no path found
}

Array Pathfinder::pathfind_straight(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return Array();
}

Array Pathfinder::pathfind_merge_greedy(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return Array();
}

Array Pathfinder::pathfind_merge_lts(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return Array();
}

Array Pathfinder::pathfind_merge_stl(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return Array();
}

//assume dir is (y, x)
//assume level nonempty
//assume cell at pos is a tile
//returns updated level if action possible, else empty vector
std::vector<std::vector<int>> Pathfinder::try_action(std::vector<std::vector<int>> level, Vector2i pos, Vector3i action, int tile_push_limit, bool is_player, int tile_pow_max) {
	Vector2i dir = Vector2i(action.x, action.y);

	switch(action.z) {
		case ActionType::SLIDE:
			return try_slide(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
		case ActionType::SPLIT:
			return try_split(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
		default:
			return try_slide(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
	}
}

std::vector<std::vector<int>> Pathfinder::try_slide(std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, int tile_push_limit, bool is_player, int tile_pow_max) {
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
	int curr_tile_val = curr_val % StuffId::MEMBRANE;
	std::vector<int> tile_vals = {curr_tile_val}; //store these bc mod is expensive

	Vector2i next_pos;
	int next_val;
	int next_tile_val;

	int tile_push_count = 0;
	do {
		//next_pos, bound check
		next_pos = curr_pos + dir;
		if (!within_bounds(next_pos) || next_val == StuffId::BLACK_WALL || next_val == StuffId::BLUE_WALL || next_val == StuffId::RED_WALL) {
			return std::vector<std::vector<int>>(); //obstructed by wall or out of bounds
		}

		next_val = level[next_pos.y][next_pos.x];
		if ((!is_player || tile_push_count) && next_val >> 5 == 1) {
			return std::vector<std::vector<int>>(); //obstructed by membrane
		}

		next_tile_val = next_val % StuffId::MEMBRANE;
		int curr_tile_pow = abs(curr_tile_val - StuffId::POW_OFFSET);
		int next_tile_pow = abs(next_tile_val - StuffId::POW_OFFSET);
		if (next_tile_val == StuffId::ZERO || (curr_tile_pow == next_tile_pow && curr_tile_pow < tile_pow_max) || curr_tile_val == StuffId::ZERO) { //merge
			//push 0?
			if (tile_push_count != tile_push_limit && next_tile_val == StuffId::ZERO) {
				Vector2i temp_pos = next_pos + dir;
				if (within_bounds(temp_pos)) {
					int temp_val = level[temp_pos.y][temp_pos.x];
					if (temp_val == StuffId::EMPTY || temp_val == StuffId::SAVEPOINT || temp_val == StuffId::GOAL) {
						level[temp_pos.y][temp_pos.x] += StuffId::ZERO;
					}
				} //else 0 gets popped
			}

			//set id at merge location
			if (curr_tile_val == StuffId::ZERO) {}
			else if (next_tile_val == StuffId::ZERO) {
				level[next_pos.y][next_pos.x] += curr_tile_val - next_tile_val;
			}
			else {
				if (curr_tile_pow + next_tile_pow == StuffId::MEMBRANE) { //opposite sign
					level[next_pos.y][next_pos.x] += StuffId::ZERO - next_tile_val;
				}
				else { //same sign
					int pow_sign = (curr_tile_val > StuffId::POW_OFFSET) ? 1 : -1;
					level[next_pos.y][next_pos.x] += pow_sign;
				}
			}

			//shift rest of line
			int temp_val = tile_vals.back();
			tile_vals.pop_back();
			while (!tile_vals.empty()) {
				level[curr_pos.y][curr_pos.x] += tile_vals.back() - temp_val;
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				curr_pos -= dir;
			}

			//curr_pos is now pos
			level[curr_pos.y][curr_pos.x] -= temp_val;
			return level;
		}
		else if (next_val == StuffId::MEMBRANE || next_val == StuffId::EMPTY || next_val == StuffId::SAVEPOINT || next_val == StuffId::GOAL) { //slide
			//shift line
			int temp_val = next_tile_val;
			while (!tile_vals.empty()) {
				level[next_pos.y][next_pos.x] += tile_vals.back() - temp_val;
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				next_pos -= dir;
			}

			//next_pos is now pos
			level[next_pos.y][next_pos.x] -= temp_val;
			return level;
		}

		curr_pos = next_pos;
		curr_val = next_val;
		curr_tile_val = next_tile_val;
		tile_vals.push_back(curr_tile_val);
		++tile_push_count;
	} while (tile_push_count <= tile_push_limit);

	//push limit exceeded
	return std::vector<std::vector<int>>();
}

//assume tile val can split
std::vector<std::vector<int>> Pathfinder::try_split(std::vector<std::vector<int>>& level, Vector2i pos, Vector2i dir, int tile_push_limit, bool is_player, int tile_pow_max) {
	Vector2i target = pos + dir;
	if (target.x < 0 || target.y < 0 || target.x >= level[0].size() || target.y >= level.size()) { //out of bounds
		return std::vector<std::vector<int>>();
	}

	int target_val = level[target.y][target.x];
	int tile_val = level[pos.y][pos.x] % StuffId::MEMBRANE; //don't care about back layer at pos
	int pow_sign = (tile_val > StuffId::POW_OFFSET) ? 1 : -1;

	//use try_slide() to handle push logic, then set splitted tile
	level[pos.y][pos.x] -= pow_sign;
	std::vector<std::vector<int>> ans = try_slide(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
	if (ans.empty()) { //slide, and by extension split, not possible
		return ans;
	}

	int split_val = tile_val - pow_sign;
	ans[pos.y][pos.x] += split_val;
	return ans;
}

//simple floodfill using A*
//assume start/end aren't walls
bool Pathfinder::is_enclosed(std::vector<std::vector<int>>& level, Vector2i start, Vector2i end, bool is_player) {

	std::priority_queue<TileState*, std::vector<TileState*>, TileStateComparer> wavefront;
	std::vector<std::vector<bool>> visited(level.size(), std::vector<bool>(level[0].size(), false));

	TileState* first = new TileState;
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
				TileState* next = new TileState;
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
	while (1);
}

LevelState::LevelState(Vector2i level_size) {
	level.resize(level_size.y);
	for (auto row : level) {
		row.reserve(level_size.x);
	}
}

Array LevelState::trace_path() {
	std::vector<Vector3i> ans_vec;
	LevelState* curr = this;

	while (curr->prev != NULL) {
		ans_vec.push_back(curr->prev_action);
		curr = curr->prev;
	}
	UtilityFunctions::print("PF TRACED PATH SIZE: ", ans_vec.size());
	return vector_to_array_1d_reversed(ans_vec);
}

void Pathfinder::_bind_methods() {
	//pathfind(int search_type, Array& level, Vector2i start, Vector2i end, int tile_push_limit, bool is_player, int tile_pow_max)
	ClassDB::bind_method(D_METHOD("pathfind", "search_type", "level", "start", "end", "tile_push_limit", "is_player", "tile_pow_max"), &Pathfinder::pathfind);
	ClassDB::bind_method(D_METHOD("testing"), &Pathfinder::testing);
}





std::vector<int> array_to_vector_1d(const Array& arr) {
	std::vector<int> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(arr[i]);
	}
	return ans;
}

std::vector<std::vector<int>> array_to_vector_2d(const Array& arr) {
	std::vector<std::vector<int>> ans;
	for (int i=0; i < arr.size(); ++i) {
		ans.push_back(array_to_vector_1d((const Array&) arr[i]));
	}
	return ans;
}

Array vector_to_array_1d(std::vector<int>& vec) {
	Array ans;
	for (int i : vec) {
		ans.push_back(i);
	}
	return ans;
}

Array vector_to_array_2d(std::vector<std::vector<int>>& vec) {
	Array ans;
	for (std::vector<int>& row : vec) {
		ans.push_back(vector_to_array_1d(row));
	}
	return ans;
}

Array vector_to_array_1d_reversed(std::vector<Vector3i>& vec) {
	Array ans;
	ans.resize(vec.size());
	for (int i=0; i < vec.size(); ++i) {
		ans[i] = vec[vec.size() - 1 - i];
	}
	return ans;
}

void print_vector_2d(std::vector<std::vector<int>>& v) {
	std::string s = "[";
	for (std::vector<int>& row : v) {
		s += "[";
		for (int col_itr=0; col_itr < row.size(); ++col_itr) {
			s += std::to_string(row[col_itr]);
			if (col_itr != row.size() - 1) {
				s += ", ";
			}
		}
		s += "]";
	}
	s += "]";
	UtilityFunctions::print(s.c_str());
}

/*
//for debugging
int main(void) {
	std::vector<std::vector<int>> level_vec = {{17, 0, 0}, {16, 17, 17}};
	Array level_arr = vector_to_array_2d(level_vec);

	Vector2i start(0, 1);
	Vector2i end(1, 0);
	Pathfinder p;
	p.pathfind(SearchType::PERFECT, level_arr, start, end, 1, true, 12);
}*/