/* from try_split()
	//check if possible without pushing
	if (!is_player && target_val >> 5 == 1) { //blocked by membrane
		return std::vector<std::vector<int>>();
	}
	else if (target_val == StuffId::EMPTY || target_val == StuffId::SAVEPOINT || target_val == StuffId::GOAL || (is_player && target_val == StuffId::MEMBRANE)) {
		//split into tile-free location
		level[std::get<1>(pos)][std::get<0>(pos)] -= pow_sign;
		level[std::get<1>(target)][std::get<0>(target)] += split_val;
	}
	else {
		int split_pow = abs(split_val - StuffId::POW_OFFSET);
		target_val %= StuffId::MEMBRANE;
		int target_pow = abs(target_val - StuffId::POW_OFFSET);
		if (split_pow == target_pow && split_pow < tile_pow_max) { //split-merge
			level[std::get<1>(pos)][std::get<0>(pos)] -= pow_sign;
			if (split_pow + target_pow == StuffId::MEMBRANE) { //opposite sign
				level[std::get<1>(target)][std::get<0>(target)] += StuffId::ZERO - target_val;
			}
			else { //same sign
				level[std::get<1>(target)][std::get<0>(target)] += pow_sign;
			}
		}
	}
*/

/*
	std::vector<LevelState*> test;
	LevelState* first = new LevelState(std::tuple<int, int>(0, 0));
	LevelState* second = new LevelState(std::tuple<int, int>(0, 0));
	first->f = 1;
	second->f = 2;
	test.push_back(first);
	test.push_back(second);
	std::sort(test.begin(), test.end(), [](LevelState* first, LevelState* second) { return first->f < second->f; });
	std::cout << test[0]->f << std::endl;
	std::cout << test[1]->f << std::endl;
*/

/*
	int index = 0;
	for (std::tuple<int, int, int> action : {
		std::tuple<int, int, int>(0,1,0),
		std::tuple<int, int, int>(1,0,0),
		std::tuple<int, int, int>(0,1,0),
		std::tuple<int, int, int>(1,0,0),
		std::tuple<int, int, int>(0,1,1),
		std::tuple<int, int, int>(0,1,0),
		std::tuple<int, int, int>(0,1,0),
		std::tuple<int, int, int>(0,-1,0),
		std::tuple<int, int, int>(0,-1,0),
		std::tuple<int, int, int>(1,0,0),
	}) {
		level = p.try_action(level, start, action, 1, true, 12);
		print_vector_2d(level);
		std::tuple<int, int> dir {std::get<0>(action), std::get<1>(action)};
		start = add(start, dir);
		if (level.size() == 0) {
			std::cout << "MOVE " << index << " IS PROBLEMATIC" << std::endl;
			break;
		}
		++index;
	}
*/

/*
		std::string indentation(2*curr->g, ' ');
		std::string line = indentation + "POS: (" + std::to_string(std::get<0>(curr->pos)) + ", " + std::to_string(std::get<1>(curr->pos)) + ") EXPANDED, f = " + std::to_string(curr->f) + "\n";
		output << line;
*/