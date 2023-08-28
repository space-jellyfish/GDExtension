/*
redo path find when a tile along path changes
get lv array on ready, use set_cell to keep it updated
treat cells outside lv array as wall
non-player tiles can leave, but not enter membrane
two +-2^12 tiles cannot merge
*/

#include "pathfinder.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <stack>
#include <cstdlib>

void print_vector_2d(std::vector<std::vector<int>>& v);
void print_vector_2d(std::vector<std::vector<int>>& v, int x0, int y0, int x1, int y1);
void print_dir(std::tuple<int, int> dir, std::string extra);
void print_pos(std::tuple<int, int> dir, std::string extra);
void print_action(std::tuple<int, int, int> action, std::string extra);
void print_path(std::vector<std::tuple<int, int, int>> path);
std::tuple<int, int> add(std::tuple<int, int> a, std::tuple<int, int> b);
std::tuple<int, int> sub(std::tuple<int, int> a, std::tuple<int, int> b);


//treats ZERO and EMPTY as different level states
//for max performance, use EMPTY in place of ZERO in level (if 0s can be safely ignored)
std::vector<std::tuple<int, int, int>> Pathfinder::pathfind(int search_type, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
    
	//generate array of random numbers (for hashing)
	std::vector<std::vector<int>> random;
	random.reserve(level.size());
	for (auto& row : level) {
		std::vector<int> rand_row;
		rand_row.reserve(row.size());
		for (auto& id : row) {
			rand_row.push_back(rand());
		}
		random.push_back(rand_row);
	}
	
	switch(search_type) {
		case SearchType::IDASTAR:
			return pathfind_idastar(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::ASTAR:
			return pathfind_astar(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::STRAIGHT:
			return pathfind_straight(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::MERGE_GREEDY:
			return pathfind_merge_greedy(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::MERGE_LARGE_TO_SMALL:
			return pathfind_merge_lts(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		case SearchType::MERGE_SMALL_TO_LARGE:
			return pathfind_merge_stl(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
		default:
			return pathfind_idastar(random, level, start, end, max_depth, tile_push_limit, is_player, tile_pow_max);
	}
}

std::vector<std::tuple<int, int, int>> Pathfinder::pathfind_idastar(const std::vector<std::vector<int>>& random, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {

	std::tuple<int, int> level_size(level[0].size(), level.size());
	std::stack<LevelStateDFS*, std::vector<LevelStateDFS*>> stack;
	std::vector<std::tuple<int, int, int>> path;
	std::unordered_map<std::pair<std::tuple<int, int>, std::vector<std::vector<int>>>, LevelStateDFS*, LevelStateHasher, LevelStateEquator> in_path;
	int root_h = heuristic(start, end);
	int threshold = root_h;
	int next_threshold = std::numeric_limits<int>::max();
	std::cout << "PF FIRST ITERATION, threshold = " << threshold << std::endl;

	//add root level state
	LevelStateDFS* root = new LevelStateDFS(level_size);
	root->pos = start;
	root->g = 0;
	root->h = root_h;
	root->f = root_h;
	root->level = level;
	in_path[std::make_pair(start, level)] = root;
	stack.push(root);

	while (!stack.empty() || next_threshold != std::numeric_limits<int>::max()) {
		if (stack.empty()) {
			assert(path.size() == 0);
			assert(in_path.size() == 0);

			//add first level state
			LevelStateDFS* first = new LevelStateDFS(level_size);
			first->pos = start;
			first->g = 0;
			first->h = root_h;
			first->f = root_h;
			first->level = level;
			in_path[std::make_pair(start, level)] = first;
			stack.push(first);

			//update threshold
			threshold = next_threshold;
			next_threshold = std::numeric_limits<int>::max();
			std::cout << "PF NEW ITERATION, threshold = " << threshold << std::endl;
		}
		//remove from stack
		LevelStateDFS* curr = stack.top();
		stack.pop();
		//print_pos(curr->pos, "EXPANDED, threshold = "+std::to_string(threshold));

		//add to path
		if (curr->g) {
			path.push_back(curr->prev_action);
		}
		in_path[std::make_pair(curr->pos, curr->level)] = curr;

		//check if end
		if (curr->pos == end) {
			std::cout << "PF FOUND PATH" << std::endl;
			while (!stack.empty()) {
				delete stack.top();
				stack.pop();
			}
			for (auto& entry : in_path) {
				delete entry.second;
			}
			return path;
		}

		//check depth
		if (curr->g < max_depth) {
			//find neighbors
			int curr_val = curr->level[std::get<1>(curr->pos)][std::get<0>(curr->pos)] % StuffId::MEMBRANE; //tile value
			bool can_split = (curr_val != StuffId::NEG_ONE && curr_val != StuffId::ZERO && curr_val != StuffId::POW_OFFSET);
			std::vector<LevelStateDFS*> neighbors;

			for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
				if (action_type == ActionType::SPLIT && !can_split) {
					continue;
				}

				//dir is (x, y), action is (x, y, *)
				for (std::tuple<int, int> dir : {std::tuple<int, int>(1, 0), std::tuple<int, int>(0, -1), std::tuple<int, int>(-1, 0), std::tuple<int, int>(0, 1)}) {
					std::tuple<int, int, int> action = std::tuple<int, int, int>(std::get<0>(dir), std::get<1>(dir), action_type);
					std::vector<std::vector<int>> temp_level = try_action(curr->level, curr->pos, action, tile_push_limit, is_player, tile_pow_max);
					//UtilityFunctions::print("PF TEMP LEVEL SIZE: ", (int)temp_level.size());

					if (temp_level.empty()) { //action not possible
						continue;
					}
					std::pair<std::tuple<int, int>, std::vector<std::vector<int>>> key(add(curr->pos, dir), temp_level);

					if (!in_path.count(key)) { //create new level state
						LevelStateDFS* temp = new LevelStateDFS(level_size);
						temp->pos = add(curr->pos, dir);
						temp->prev = curr;
						temp->prev_action = action;
						temp->g = curr->g + 1;
						temp->h = heuristic(temp->pos, end);
						temp->f = temp->g + temp->h;
						temp->level = temp_level;

						//add level state
						if (temp->f <= threshold) {
							//stack.push(temp);
							neighbors.push_back(temp);
						}
						else if (temp->f < next_threshold) { //update next threshold
							next_threshold = temp->f;
							//print_pos(temp->pos, "update next threshold to "+std::to_string(next_threshold));
						}
					}
				}
			} //end find neighbors

			//add neighbors (in order)
			std::sort(neighbors.begin(), neighbors.end(), [](LevelStateDFS* first, LevelStateDFS* second) {
				if (first->f < second->f) {
					return true;
				}
				if (first->f > second->f) {
					return false;
				}
				return first->g > second->g;
			});
			for (LevelStateDFS* temp : neighbors) {
				stack.push(temp);
				++curr->child_count;
			}
		}

		while (curr != NULL && curr->child_count == 0) { //backtrack (delete childless level states)
			//get parent pointer
			LevelStateDFS* parent = curr->prev;

			//delete curr
			if (!path.empty()) {
				path.pop_back();
			}
			in_path.erase(std::make_pair(curr->pos, curr->level));
			delete curr;

			//decrement parent child_count
			if (parent != NULL) {
				--parent->child_count;
			}

			//update curr
			curr = parent;
		}
	}

	std::cout << "PF NO PATH FOUND" << std::endl;
	return std::vector<std::tuple<int, int, int>>();
}

std::vector<std::tuple<int, int, int>> Pathfinder::pathfind_astar(const std::vector<std::vector<int>>& random, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	
	std::tuple<int, int> level_size(level[0].size(), level.size());
	std::priority_queue<LevelStateBFS*, std::vector<LevelStateBFS*>, LevelStateComparer> wavefront;
	std::unordered_map<std::pair<std::tuple<int, int>, std::vector<std::vector<int>>>, LevelStateBFS*, LevelStateHasher, LevelStateEquator> visited;
	
	//add first level state
	LevelStateBFS* first = new LevelStateBFS(level_size);
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
		LevelStateBFS* curr = wavefront.top();
		wavefront.pop();
		print_pos(curr->pos, "EXPANDED, f = "+std::to_string(curr->f));

		//check if end
		if (curr->pos == end) {
			std::cout << "PF FOUND PATH" << std::endl;
			std::vector<std::tuple<int, int, int>> ans = curr->trace_path();
			for (auto& entry : visited) {
				delete entry.second;
			}
			return ans;
		}

		//check depth
		if (curr->g >= max_depth) {
			continue;
		}

		//add/update neighbors
		int curr_val = curr->level[std::get<1>(curr->pos)][std::get<0>(curr->pos)] % StuffId::MEMBRANE; //tile value
		bool can_split = (curr_val != StuffId::NEG_ONE && curr_val != StuffId::ZERO && curr_val != StuffId::POW_OFFSET);

		for (int action_type=ActionType::SLIDE; action_type != ActionType::END; ++action_type) {
			if (action_type == ActionType::SPLIT && !can_split) {
				continue;
			}

			//dir is (x, y), action is (x, y, *)
			for (std::tuple<int, int> dir : {std::tuple<int, int>(1, 0), std::tuple<int, int>(0, -1), std::tuple<int, int>(-1, 0), std::tuple<int, int>(0, 1)}) {
				std::tuple<int, int, int> action = std::tuple<int, int, int>(std::get<0>(dir), std::get<1>(dir), action_type);
				std::vector<std::vector<int>> temp_level = try_action(curr->level, curr->pos, action, tile_push_limit, is_player, tile_pow_max);
				//UtilityFunctions::print("PF TEMP LEVEL SIZE: ", (int)temp_level.size());

				if (temp_level.empty()) { //action not possible
					continue;
				}
				std::pair<std::tuple<int, int>, std::vector<std::vector<int>>> key(add(curr->pos, dir), temp_level);

				if (visited.count(key)) { //level state exists
					LevelStateBFS* temp = visited.at(key);

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
					LevelStateBFS* temp = new LevelStateBFS(level_size);
					temp->pos = add(curr->pos, dir);
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

	std::cout << "NO PATH FOUND" << std::endl;
	for (auto& entry : visited) {
		delete entry.second;
	}
	return std::vector<std::tuple<int, int, int>>(); //no path found
}

std::vector<std::tuple<int, int, int>> Pathfinder::pathfind_straight(const std::vector<std::vector<int>>& random, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return std::vector<std::tuple<int, int, int>>();
}

std::vector<std::tuple<int, int, int>> Pathfinder::pathfind_merge_greedy(const std::vector<std::vector<int>>& random, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return std::vector<std::tuple<int, int, int>>();
}

std::vector<std::tuple<int, int, int>> Pathfinder::pathfind_merge_lts(const std::vector<std::vector<int>>& random, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return std::vector<std::tuple<int, int, int>>();
}

std::vector<std::tuple<int, int, int>> Pathfinder::pathfind_merge_stl(const std::vector<std::vector<int>>& random, const std::vector<std::vector<int>>& level, std::tuple<int, int> start, std::tuple<int, int> end, int max_depth, int tile_push_limit, bool is_player, int tile_pow_max) {
	return std::vector<std::tuple<int, int, int>>();
}

//assume dir is (y, x)
//assume level nonempty
//assume cell at pos is a tile
//returns updated level if action possible, else empty vector
std::vector<std::vector<int>> Pathfinder::try_action(std::vector<std::vector<int>> level, std::tuple<int, int> pos, std::tuple<int, int, int> action, int tile_push_limit, bool is_player, int tile_pow_max) {
	std::tuple<int, int> dir = std::tuple<int, int>(std::get<0>(action), std::get<1>(action));

	switch(std::get<2>(action)) {
		case ActionType::SLIDE:
			return try_slide(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
		case ActionType::SPLIT:
			return try_split(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
		default:
			return try_slide(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
	}
}

std::vector<std::vector<int>> Pathfinder::try_slide(std::vector<std::vector<int>>& level, std::tuple<int, int> pos, std::tuple<int, int> dir, int tile_push_limit, bool is_player, int tile_pow_max) {
	//print_dir(dir, "SLIDE");
	//print_vector_2d(level);
	std::function<bool(std::tuple<int, int>)> within_bounds;
	if (std::get<0>(dir)) {
		if (std::get<0>(dir) > 0) {
			within_bounds = [&](std::tuple<int, int> new_pos)->bool { return std::get<0>(new_pos) < level[0].size(); };
		}
		else {
			within_bounds = [&](std::tuple<int, int> new_pos)->bool { return std::get<0>(new_pos) >= 0; };
		}
	}
	else {
		if (std::get<1>(dir) > 0) {
			within_bounds = [&](std::tuple<int, int> new_pos)->bool { return std::get<1>(new_pos) < level.size(); };
		}
		else {
			within_bounds = [&](std::tuple<int, int> new_pos)->bool { return std::get<1>(new_pos) >= 0; };
		}
	}

	std::tuple<int, int> curr_pos = pos;
	int curr_val = level[std::get<1>(pos)][std::get<0>(pos)];
	int curr_tile_val = curr_val % StuffId::MEMBRANE;
	std::vector<int> tile_vals = {curr_tile_val}; //store these bc mod is expensive

	std::tuple<int, int> next_pos;
	int next_val;
	int next_tile_val;

	int tile_push_count = 0;
	do {
		//next_pos, bound check
		next_pos = add(curr_pos, dir);
		if (!within_bounds(next_pos) || next_val == StuffId::BLACK_WALL || next_val == StuffId::BLUE_WALL || next_val == StuffId::RED_WALL) {
			//std::cout << "OUT OF BOUNDS" << std::endl;
			return std::vector<std::vector<int>>(); //obstructed by wall or out of bounds
		}

		next_val = level[std::get<1>(next_pos)][std::get<0>(next_pos)];
		if ((!is_player || tile_push_count) && next_val >> 5 == 1) {
			return std::vector<std::vector<int>>(); //obstructed by membrane
		}

		next_tile_val = next_val % StuffId::MEMBRANE;
		int curr_tile_pow = abs(curr_tile_val - StuffId::POW_OFFSET);
		int next_tile_pow = abs(next_tile_val - StuffId::POW_OFFSET);
		if (next_tile_val == StuffId::ZERO || (curr_tile_pow == next_tile_pow && curr_tile_pow < tile_pow_max) || curr_tile_val == StuffId::ZERO) { //merge
			//push 0?
			if (tile_push_count != tile_push_limit && next_tile_val == StuffId::ZERO) {
				std::tuple<int, int> temp_pos = add(next_pos, dir);
				if (within_bounds(temp_pos)) {
					int temp_val = level[std::get<1>(temp_pos)][std::get<0>(temp_pos)];
					if (temp_val == StuffId::EMPTY || temp_val == StuffId::SAVEPOINT || temp_val == StuffId::GOAL) {
						level[std::get<1>(temp_pos)][std::get<0>(temp_pos)] += StuffId::ZERO;
					}
				} //else 0 gets popped
			}

			//set id at merge location
			if (curr_tile_val == StuffId::ZERO) {}
			else if (next_tile_val == StuffId::ZERO) {
				level[std::get<1>(next_pos)][std::get<0>(next_pos)] += curr_tile_val - next_tile_val;
			}
			else {
				if (curr_tile_pow + next_tile_pow == StuffId::MEMBRANE) { //opposite sign
					level[std::get<1>(next_pos)][std::get<0>(next_pos)] += StuffId::ZERO - next_tile_val;
				}
				else { //same sign
					int pow_sign = (curr_tile_val > StuffId::POW_OFFSET) ? 1 : -1;
					level[std::get<1>(next_pos)][std::get<0>(next_pos)] += pow_sign;
				}
			}

			//shift rest of line
			int temp_val = tile_vals.back();
			tile_vals.pop_back();
			while (!tile_vals.empty()) {
				level[std::get<1>(curr_pos)][std::get<0>(curr_pos)] += tile_vals.back() - temp_val;
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				curr_pos = sub(curr_pos, dir);
			}

			//curr_pos is now pos
			level[std::get<1>(curr_pos)][std::get<0>(curr_pos)] -= temp_val;
			//std::cout << "MERGE RESULT: " << std::endl;
			//print_vector_2d(level);
			return level;
		}
		else if (next_val == StuffId::MEMBRANE || next_val == StuffId::EMPTY || next_val == StuffId::SAVEPOINT || next_val == StuffId::GOAL) { //slide
			//shift line
			int temp_val = next_tile_val;
			while (!tile_vals.empty()) {
				level[std::get<1>(next_pos)][std::get<0>(next_pos)] += tile_vals.back() - temp_val;
				temp_val = tile_vals.back();
				tile_vals.pop_back();
				next_pos = sub(next_pos, dir);
			}

			//next_pos is now pos
			level[std::get<1>(next_pos)][std::get<0>(next_pos)] -= temp_val;
			//std::cout << "SLIDE RESULT: " << std::endl;
			//print_vector_2d(level);
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
std::vector<std::vector<int>> Pathfinder::try_split(std::vector<std::vector<int>>& level, std::tuple<int, int> pos, std::tuple<int, int> dir, int tile_push_limit, bool is_player, int tile_pow_max) {
	std::tuple<int, int> target = add(pos, dir);
	if (std::get<0>(target) < 0 || std::get<1>(target) < 0 || std::get<0>(target) >= level[0].size() || std::get<1>(target) >= level.size()) { //out of bounds
		return std::vector<std::vector<int>>();
	}

	int target_val = level[std::get<1>(target)][std::get<0>(target)];
	int tile_val = level[std::get<1>(pos)][std::get<0>(pos)] % StuffId::MEMBRANE; //don't care about back layer at pos
	int pow_sign = (tile_val > StuffId::POW_OFFSET) ? 1 : -1;

	//use try_slide() to handle push logic, then set splitted tile
	level[std::get<1>(pos)][std::get<0>(pos)] -= pow_sign;
	std::vector<std::vector<int>> ans = try_slide(level, pos, dir, tile_push_limit, is_player, tile_pow_max);
	if (ans.empty()) { //slide, and by extension split, not possible
		return ans;
	}

	int split_val = tile_val - pow_sign;
	ans[std::get<1>(pos)][std::get<0>(pos)] += split_val;
	return ans;
}

//use manhattan distance for 4-directional movement
int Pathfinder::heuristic(std::tuple<int, int> pos, std::tuple<int, int> goal) {
	return abs(std::get<0>(pos) - std::get<0>(goal)) + abs(std::get<1>(pos) - std::get<1>(goal));
}

int z_hash(const std::tuple<int, int>& pos, const std::vector<std::vector<int>>& level) {
	int hash = 0;
	hash ^= std::hash<int>{}(std::get<0>(pos)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
	hash ^= std::hash<int>{}(std::get<1>(pos)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);

	return hash;
}

LevelState::LevelState(std::tuple<int, int> level_size) {
	level.resize(std::get<1>(level_size));
	for (auto& row : level) {
		row.reserve(std::get<0>(level_size));
	}
}

std::vector<std::tuple<int, int, int>> LevelStateBFS::trace_path() {
	std::vector<std::tuple<int, int, int>> ans;
	std::vector<std::vector<std::vector<int>>> levels_debug;
	ans.resize(this->g);
	levels_debug.resize(this->g);
	int index = this->g - 1;
	LevelStateBFS* curr = this;

	while (curr->prev != NULL) {
		ans[index] = curr->prev_action;
		levels_debug[index] = curr->prev->level;
		curr = curr->prev;
		--index;
	}
	for (int i=0; i < ans.size(); ++i) {
		//print_action(ans[i], "");
		//print_vector_2d(levels_debug[i],0,0,5,5);
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
		s += "]\n";
	}
	s += "]";
	std::cout << s << std::endl;
}

void print_vector_2d(std::vector<std::vector<int>>& v, int x0, int y0, int x1, int y1) {
	std::string s = "[";
	for (int y=y0; y < y1; ++y) {
		s += "[";
		for (int x=x0; x < x1; ++x) {
			s += std::to_string(v[y][x]);
			if (x != x1 - 1) {
				s += ", ";
			}
		}
		s += "]\n";
	}
	s += "]";
	std::cout << s << std::endl;
}

void print_dir(std::tuple<int, int> dir, std::string extra) {
	std::cout << "DIR: (" << std::get<0>(dir) << ", " << std::get<1>(dir) << ")\t" << extra << std::endl;
}

void print_pos(std::tuple<int, int> dir, std::string extra) {
	std::cout << "POS: (" << std::get<0>(dir) << ", " << std::get<1>(dir) << ")\t" << extra << std::endl;
}

void print_action(std::tuple<int, int, int> action, std::string extra) {
	std::cout << "ACTION: (" << std::get<0>(action) << ", " << std::get<1>(action) << ", " << std::get<2>(action) << ")\t" << extra << std::endl;
}

void print_path(std::vector<std::tuple<int, int, int>> path) {
	for (std::tuple<int, int, int> action : path) {
		print_action(action, "");
	}
}

std::tuple<int, int> add(std::tuple<int, int> a, std::tuple<int, int> b) {
	return std::tuple<int, int>(std::get<0>(a) + std::get<0>(b), std::get<1>(a) + std::get<1>(b));
}

std::tuple<int, int> sub(std::tuple<int, int> a, std::tuple<int, int> b) {
	return std::tuple<int, int>(std::get<0>(a) - std::get<0>(b), std::get<1>(a) - std::get<1>(b));
}

//for debugging
int main(void) {
	//std::vector<std::vector<int>> level = {{16,16,18,21,23}, {31, 0, 18, 21, 23}, {0, 0, 17, 20, 22}, {16, 16, 16, 19, 21}, {-1, 17, 17, 19, 21}};
	std::vector<std::vector<int>> level = {{16, 16, 18, 21, 23, 23, 21, 20, 18, 18, 18, 17, 0, 15, 13, 14, 31, 0, 0, 0, 17, 17, 16, 17, 0, 31, 14, 13, -1, 0, 16, 16, 18, 20, 21, 19, 18, 16, 0, 15}, {31, 0, 18, 21, 23, 23, 21, 20, 18, 18, 18, 17, 0, 31, 13, 13, 31, 0, 0, 16, 18, 19, 18, 18, 17, 15, 13, 12, 32, 0, 0, 0, 17, 19, 22, 21, 18, 17, 0, 15}, {0, 0, 17, 20, 22, 22, 21, 19, 18, 19, 18, 16, 0, 15, 13, 12, 15, 0, 0, 17, 18, 18, 18, 19, 18, 0, 13, 11, 12, 15, 0, 0, 16, 17, 21, 21, 19, 18, 16, 0}, {16, 16, 16, 19, 21, 22, 21, 20, 18, 17, 16, 0, 0, 15, 14, 13, 13, 15, 0, 17, 17, 17, 18, 19, 18, 0, 15, 13, 32, 14, 31, 0, 0, 17, 21, 21, 20, 19, 19, 19}, {-1, 17, 17, 19, 21, 22, 22, 21, 19, 16, 0, 0, 0, 31, 13, 12, 12, 14, 0, 0, 16, 16, 17, 18, 18, 0, 15, 14, -1, 14, 14, 14, 31, 18, 21, 22, 21, 21, 21, 21}, {19, 19, 18, 19, 21, 22, 22, 22, 20, 16, 0, 0, 0, 31, 12, 10, 11, 13, 31, 0, 0, 16, 18, 20, 19, 16, 0, 31, 15, 15, 14, 14, 31, 17, 20, 20, 19, 20, 20, 20}, {19, 19, 18, 18, 19, 20, 21, 21, 20, 17, 0, 0, 0, 15, 12, 10, 12, 14, 31, 31, 0, 0, 18, 20, 20, 18, 16, 0, 31, 31, 31, 15, 0, 17, 18, 16, 0, 17, 18, 19}, {16, 18, 19, 19, 20, 20, 19, 19, 18, 16, 0, 0, 0, 15, 11, 9, 10, 13, 15, 0, 0, 0, 17, 19, 20, 18, 18, 17, 0, 0, 31, 15, 0, 16, 0, 31, 15, 0, 16, 20}, {16, 18, 19, 21, 22, 21, 18, 17, 16, 31, 15, 15, 14, 14, 11, 8, 8, 10, 12, 31, 31, 31, 0, 17, 19, 19, 20, 21, 19, 17, 16, 0, 0, 0, 15, 13, 15, 31, 16, 19}, {0, 17, 18, 19, 21, 20, 16, 0, 31, 14, 13, 12, 13, 13, 11, 8, 8, 10, 11, 14, 31, 31, 31, 0, 18, 20, 20, 32, 22, 20, 19, 18, 16, 0, 15, 13, 15, 0, 16, 18}, {0, 17, 18, 19, 19, 18, 0, 14, 13, 13, 13, 12, 12, 12, 11, 10, 10, 11, 12, 14, 31, 15, 14, 0, 17, 32, -1, 32, 22, 20, 19, 18, 17, 16, 31, 14, 15, 0, 16, 16}, {17, 17, 18, 18, 18, 17, 0, 14, 12, 12, 13, 12, 12, 12, 11, 11, 11, 11, 12, 13, 31, 15, 15, 0, 32, 32, 19, 20, 20, 18, 18, 18, 16, 0, 31, 13, 13, 14, 31, 31}, {20, 19, 17, 17, 16, 16, 0, 15, 14, 12, 11, 11, 12, 13, 12, 12, 12, 12, 12, 13, 15, 15, 15, 15, -1, 16, 19, 19, 17, 17, 18, 17, 0, 0, 15, 12, 11, 12, 13, 14}, {22, 21, 20, 17, 16, 0, 31, 15, 15, 14, 12, 12, 13, 14, 14, 13, 13, 13, 13, 14, 31, 31, 14, 13, -1, 0, 16, 17, 0, 0, 16, 0, 0, 15, 13, 12, 13, 13, 14, 14}, {21, 20, 19, 17, 16, 17, 16, 16, 16, 0, 15, 13, 14, 14, 14, 14, 12, 12, 13, 15, 0, 0, 15, 13, 14, 32, 31, 0, 31, 15, 15, 31, 31, 13, 12, 12, 14, 15, 31, 31}, {20, 18, 16, 0, 17, 18, 18, 19, 17, 17, 16, 0, 0, 31, 14, 13, 14, 13, 13, 15, 0, 31, 14, 12, 13, 32, 14, 15, 14, 13, 13, 15, 15, 12, 11, 11, 13, 15, 31, 31}, {18, 16, 0, 0, 16, 17, 17, 18, 19, 20, 20, 19, 19, 16, 15, 15, 31, 0, 31, 0, 0, 31, 13, 11, -1, 14, 15, 14, 13, 13, 14, 15, 14, 13, 12, 12, 11, 12, 13, 15}, {0, 0, 15, 15, 31, 16, 17, 17, 20, 21, 21, 21, 21, 19, 16, 0, 0, 16, 17, 16, 16, 0, -1, 13, 14, 15, 31, 31, 14, 14, 15, 15, 14, 12, 13, 11, 9, 9, 10, 12}, {31, 14, 13, 12, 14, 31, 0, 16, 19, 20, 19, 20, 21, 20, 19, 17, 18, 19, 20, 18, 17, 0, 15, 14, 14, 15, 15, 14, 13, 14, 31, 31, 14, 13, 13, 11, 9, 9, 9, 10}, {14, 12, 12, 13, 14, 31, 0, 0, 17, 19, 18, 19, 19, 18, 19, 18, 20, 21, 20, 19, 18, 16, 0, 0, 31, 31, 15, 13, 13, 15, 0, 0, 15, 15, 14, 12, 11, 11, 11, 11}, {14, 11, 11, 15, 0, 0, 17, 16, 16, 18, 18, 17, 18, 19, 19, 19, 19, 20, 20, 20, 19, 18, 17, 0, 31, 31, 15, 13, 12, 13, 31, 0, 31, 15, 13, 12, 13, 14, 15, 14}, {14, 10, 11, 15, 0, 16, 17, 16, 0, 17, 16, 16, 18, 20, 19, 17, 16, -1, 20, 21, 20, 18, 17, 0, 0, 31, 15, 13, 12, 11, 14, 31, 15, 15, 14, 13, 15, 0, 0, 31}, {13, 11, 12, 15, 0, 0, 0, 16, 0, 0, 0, 17, 18, 18, 17, 0, 31, 0, 19, 21, 20, 18, 17, 17, 16, 31, 13, 12, 12, 13, 15, 31, 0, 0, 0, 31, 0, 17, 16, 0}, {31, 14, 15, 31, 0, 31, 15, 15, 31, 31, 0, 16, 16, 0, 0, 31, 15, 31, 17, 19, 21, 20, 19, 19, 17, 31, 12, 11, 11, 14, 31, 0, 16, 18, 18, 17, 18, 18, 18, 18}, {0, 0, 0, 0, 0, 31, 14, 13, 15, 31, 0, 0, 31, 14, 15, 15, 31, 0, 16, 18, 20, 20, 19, 19, 16, 31, 14, 12, 12, 14, 31, 0, 17, 19, 18, 18, 20, 20, 20, 21}, {0, 16, 16, 0, 0, 0, 31, 15, 31, 31, 31, 31, 15, 12, -1, 14, 31, 16, 18, 17, 18, 18, 18, 18, 16, 0, 15, 13, 13, 15, 0, 16, 19, 19, 19, 19, 21, 21, 20, 20}, {0, 16, 16, 0, 0, 0, 0, 0, 0, 15, 14, 14, 13, 12, -1, 13, 31, 0, 18, 17, 16, 17, 18, 18, 16, 0, 14, 13, 12, 13, 15, 16, 20, 22, 21, 20, 20, 20, 19, 18}, {0, 0, 16, 0, 0, 0, 17, 18, 16, 31, 15, 15, 15, 15, 32, 31, 0, 16, 17, 17, 0, 0, 16, 16, 0, 31, 13, 11, 11, 12, 14, 0, 19, 22, 22, 19, 19, 18, 17, 17}, {31, 0, 16, 0, 0, 18, 20, 21, 18, 0, 31, 0, 16, 16, 0, 16, 18, 18, 17, 16, 0, 0, 16, 0, 31, 15, 12, 11, 13, 13, 15, 16, 20, 23, 23, 21, 20, 19, 19, 19}, {0, 0, 0, 0, 17, 20, 22, 21, 18, 0, 31, 0, 18, 19, 18, 19, 21, 21, 19, 18, 17, 18, 18, 16, 31, 13, 13, 14, 15, 15, 0, 18, 21, 23, 23, 22, 22, 20, 20, 19}};
	//std::vector<std::vector<int>> level = {{-1, 0}, {16, 31}};
	//std::vector<std::vector<int>> level = {{-1, 0, 0}, {16, 17, 17}};
	//std::vector<std::vector<int>> level = {{16, 17, 17}, {17, 17, 1}, {17, 17, 1}};

	std::tuple<int, int> start(0, 0);
	std::tuple<int, int> end(4, 4);
	Pathfinder p;
	auto path = p.pathfind(SearchType::IDASTAR, level, start, end, 500, 1, true, 12);
	print_path(path);
}