#include "Utils.hpp"

#include "Environment.hpp"

// Get all combinations of numbers/agents <n
std::vector<Agent_Combination> get_combinations(Agent_Combination agents) {
	std::vector<size_t> converted;
	for (auto& entry : agents.agents) {
		converted.push_back(entry.id);
	}
	return get_combinations(converted);
}

std::vector<Agent_Combination> get_combinations(size_t n) {
	std::vector<size_t> vec;
	for (size_t i = 0; i < n; ++i) {
		vec.push_back(i);
	}
	return get_combinations(vec);
}

std::vector<Agent_Combination> get_combinations(std::vector<size_t> agents) {
	if (agents.empty()) return {};
	std::vector<bool> status;
	status.push_back(true);
	for (size_t i = 1; i < agents.size(); ++i) {
		status.push_back(false);
	}
	std::vector<Agent_Combination> combinations;

	bool done = false;
	while (!done) {
		
		// Record next entry
		std::vector<Agent_Id> next_combination;
		for (size_t i = 0; i < agents.size(); ++i) {
			if (status.at(i)) next_combination.push_back(agents.at(i));
		}

		// Advance status
		size_t counter = 0;
		while (true) {
			status.at(counter) = !status.at(counter);
			if (status.at(counter)) {
				break;
			} else {
				++counter;
				if (counter == status.size()) {
					done = true;
					break;
				}
			}
		}

		combinations.push_back(Agent_Combination(next_combination));
	}
	return combinations;
}

Direction get_direction(Coordinate from, Coordinate to) {
	auto first_diff = to.first - from.first;
	if (first_diff == 1) return Direction::RIGHT;
	if (first_diff == -1) return Direction::LEFT;

	auto second_diff = to.second - from.second;
	if (second_diff == 1) return Direction::DOWN;
	if (second_diff == -1) return Direction::UP;

	return Direction::NONE;
}