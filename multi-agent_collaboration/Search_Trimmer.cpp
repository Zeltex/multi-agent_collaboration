#include "Search_Trimmer.hpp"
#include <ranges>

void Search_Trimmer::trim(std::vector<Joint_Action>& actions, const State& state, const Environment& environment, const Recipe& recipe) const {
	
	std::vector<bool> agent_done;
	for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
		agent_done.push_back(false);
	}


	State baseline_state = state;
	bool done = false;
	for (size_t action_index = 0; action_index < actions.size() && !done; ++action_index) {
		for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
			if (agent_done.at(agent)) continue;
			State current_state = baseline_state;

			auto modified_actions = apply_modified_actions(action_index, agent, current_state, environment, actions);

			if (current_state.contains_item(recipe.result)) {
				actions = modified_actions;
				agent_done.at(agent) = true;

				done |= (std::count(agent_done.begin(), agent_done.end(), true) == environment.get_number_of_agents() - 1);
			}
		}
		environment.act(baseline_state, actions.at(action_index), Print_Level::NOPE);
	}
}

std::vector<Joint_Action> Search_Trimmer::apply_modified_actions(size_t action_index, size_t agent, State& current_state,
	const Environment& environment, const std::vector<Joint_Action>& actions) const {
	auto modified_actions = actions;

	for (; action_index < actions.size(); ++action_index) {
		modified_actions.at(action_index).update_action(agent, Direction::NONE);
		environment.act(current_state, modified_actions.at(action_index), Print_Level::NOPE);
	}
	return modified_actions;
}