#include "BFS.hpp"
#include "Search.hpp"

#include <unordered_set>
#include <queue>
#include <iostream>

std::vector<Action> BFS::search(const State& state, const Environment& environment, Ingredient goal, Agent_Id agent) const {
	Search_Joint_State dummy(state, { {} }, 0, 0);
	std::unordered_set<Search_Joint_State> visited;
	std::vector<Search_Joint_State> path;
	std::deque<Search_Joint_State> frontier;
	path.push_back(dummy);
	visited.insert(dummy);
	frontier.push_back(dummy);
	bool done = false;
	size_t state_id = 1;
	size_t goal_id = 0;
	size_t search_depth = 0;
	size_t search_depth_layer = 1;

	std::vector<size_t> hashes;
	hashes.push_back(state.to_hash());

	auto actions = environment.get_joint_actions({ agent });
	while (!done) {
		auto current_state = frontier.front();
		frontier.pop_front();

		for (const auto& action : actions) {
			auto temp_state = current_state;
			environment.act(temp_state.state, action, Print_Level::NOPE);
			auto search_state = Search_Joint_State(temp_state.state, action, current_state.id, state_id);
			if (visited.find(search_state) == visited.end()) {
				hashes.push_back(search_state.state.to_hash());
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
		--search_depth_layer;
		if (search_depth_layer == 0) {
			++search_depth;
			search_depth_layer = frontier.size();
			//std::cout << std::to_string(search_depth) << " : " << path.size() << std::endl;
		}
	}

	size_t current_id = goal_id;
	std::vector<Action> result_actions;
	while (current_id != 0) {
		auto temp_state = path.at(current_id);
		result_actions.push_back(temp_state.action.actions.at(agent.id));
		current_id = temp_state.parent_id;
	}

	std::vector<Action> reverse_actions;
	for (auto it = result_actions.rbegin(); it < result_actions.rend(); ++it) {
		reverse_actions.push_back(*it);
	}

	return reverse_actions;
}


std::vector<Joint_Action> BFS::search_joint(const State& state, const Environment& environment, Ingredient goal) const {
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

	std::vector<Agent_Id> agents;
	for (size_t i = 0; i < environment.get_number_of_agents(); ++i) {
		agents.push_back({ i });
	}

	auto actions = environment.get_joint_actions(agents);
	while (!done) {
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
	std::vector<Joint_Action> result_actions;
	while (current_id != 0) {
		auto temp_state = path.at(current_id);
		result_actions.push_back(temp_state.action);
		current_id = temp_state.parent_id;
	}

	std::vector<Joint_Action> reverse_actions;
	for (auto it = result_actions.rbegin(); it < result_actions.rend(); ++it) {
		reverse_actions.push_back(*it);
	}

	return reverse_actions;
}