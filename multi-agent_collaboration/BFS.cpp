#include "BFS.hpp"
#include "Search.hpp"

#include <unordered_set>
#include <queue>
#include <iostream>

std::vector<Joint_Action> BFS::search_joint(const State& original_state, const Environment& environment,
	Recipe recipe, const std::vector<Agent_Id>& agents) const {
	return search_joint(original_state, environment, recipe, agents, (size_t)-1);
}

std::vector<Joint_Action> BFS::search_joint(const State& state, const Environment& environment,
	Recipe recipe, const std::vector<Agent_Id>& agents, size_t depth_limit) const {
	Search_Joint_State dummy(state, { { } }, 0, 0);
	std::unordered_set<Search_Joint_State> visited;
	std::vector<Search_Joint_State> path;
	std::deque<Search_Joint_State> frontier;
	path.push_back(dummy);
	visited.insert(dummy);
	frontier.push_back(dummy);
	bool done = false;
	size_t state_id = 1;
	size_t goal_id = 0;

	auto actions = environment.get_joint_actions(agents);
	while (!done) {
		// No possible path
		if (frontier.empty()) {
			return {};
		}
		auto current_state = frontier.front();
		frontier.pop_front();

		for (const auto& action : actions) {
			auto temp_state = current_state;
			environment.act(temp_state.state, action, Print_Level::NOPE);
			auto search_state = Search_Joint_State(temp_state.state, action, current_state.id, state_id);
			if (visited.find(search_state) == visited.end()) {
				visited.insert(search_state);
				path.push_back(search_state);
				frontier.push_back(search_state);
				if (temp_state.state.contains_item(recipe.result)) {
					done = true;
					goal_id = state_id;
					break;
				}
				++state_id;
			}
		}
	}
	return extract_actions<Search_Joint_State>(goal_id, path);
}