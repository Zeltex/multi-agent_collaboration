#include "BFS.hpp"

#include <unordered_set>
#include <queue>

std::vector<Action> BFS::search(const State& state, const Environment& environment, Ingredient goal, Agent_Id agent) const {
	Search_State dummy( state, {Direction::DOWN, {0}}, 0, 0);
	std::unordered_set<Search_State> visited;
	std::vector<Search_State> path;
	std::deque<Search_State> frontier;
	path.push_back(dummy);
	visited.insert(dummy);
	frontier.push_back(dummy);
	bool done = false;
	size_t state_id = 1;
	size_t goal_id = 0;

	while (!done) {
		auto current_state = frontier.front();
		frontier.pop_front();

		auto actions = environment.get_actions(state, agent);
		for (const auto& action : actions) {
			auto temp_state = current_state;
			environment.act(temp_state.state, action, Print_Level::NOPE);
			auto search_state = Search_State(temp_state.state, action, current_state.id, state_id);
			if (visited.find(search_state) == visited.end()) {
				visited.insert(search_state);
				path.push_back(search_state);
				frontier.push_back(search_state);
				if (temp_state.state.contains_item(goal)) {
					done = true;
					goal_id = state_id;
					break;
				}
				++state_id;
			}
		}
	}

	size_t current_id = goal_id;
	std::vector<Action> actions;
	while (current_id != 0) {
		auto temp_state = path.at(current_id);
		actions.push_back(temp_state.action);
		current_id = temp_state.parent_id;
	}

	std::vector<Action> reverse_actions;
	for (auto it = actions.rbegin(); it < actions.rend(); ++it) {
		reverse_actions.push_back(*it);
	}

	return reverse_actions;
}