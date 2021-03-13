#include "A_Star.hpp"
#include "Search.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>

std::vector<Joint_Action> A_Star::search_joint(const State& original_state, const Environment& environment,
	Recipe recipe, const std::vector<Agent_Id>& agents) const {
	return search_joint(original_state, environment, recipe, agents, (size_t)-1);
}


std::vector<Joint_Action> A_Star::search_joint(const State& original_state, const Environment& environment, 
		Recipe recipe, const std::vector<Agent_Id>& agents, size_t depth_limit) const {
	// TODO - Implement agents
	std::priority_queue<Node, std::vector<Node>, Node_Comparator> frontier; // Open list		f, g, h, state_id, action_count
	std::unordered_set<State_Hash> visited;									// State lookup		state, state_id
	std::vector<State_Info> states;											// State id lookup	state, parent_state_id, action, g, action_count
	std::vector<bool> closed;												// Closed list		[state_id]=is_closed

	Heuristic heuristic(environment, recipe.ingredient1, recipe.ingredient2, agents);

	size_t original_id = 0;
	size_t original_g = 0;
	size_t original_action_count = 0;
	frontier.push({ original_id, original_g, heuristic(original_state), original_action_count });
	visited.insert({ original_state, original_id });
	states.push_back({ original_state, original_id, Joint_Action({ }), original_g, original_action_count });
	closed.push_back(false);

	size_t goal_id = (size_t)-1;

	bool done = false;
	auto actions = environment.get_joint_actions(agents);
	while (!done) {

		// No possible path
		if (frontier.empty()) {
			return {};
		}

		auto current_state_id = frontier.top().state_id;
		frontier.pop();
		if (closed.at(current_state_id)) {
			continue;
		} else {
			closed.at(current_state_id) = true;
		}

		auto current_state = states.at(current_state_id).state;

		if (states.at(current_state_id).g >= depth_limit) {
			continue;
		}

		//std::cout << "\nState " << current_state_id << std::endl;
		//environment.print_state(current_state);

		for (const auto& action : actions) {
			auto new_state = current_state;
			if (!environment.act(new_state, action, Print_Level::NOPE)) {
				continue;
			}
			const auto new_g = states.at(current_state_id).g + 1;
			const auto new_action_cost = states.at(current_state_id).action_count + get_action_cost(action);
			State_Info new_state_info(new_state, current_state_id, action, new_g, new_action_cost);
			auto state_hash = State_Hash(new_state, states.size());
			auto visited_it = visited.find(state_hash);


			if (visited_it != visited.end()) {
				if (new_state_info < states.at(visited_it->state_id)) {
					// Faster path found to already expanded node
					if (closed.at(visited_it->state_id) && new_state_info.g != states.at(visited_it->state_id).g) {
						std::cerr << "Heuristic is not consistent" << std::endl;
						exit(-1);

					// Faster path found
					} else {
						state_hash = *visited_it;
						states.at(state_hash.state_id).g = new_state_info.g;
						states.at(state_hash.state_id).action_count = new_state_info.action_count;
						states.at(state_hash.state_id).parent_id = current_state_id;
					}
				// Found slower path
				} else {
					continue;
				}
			// New state
			} else {
				visited.insert(state_hash);
				states.push_back(new_state_info);
				closed.push_back(false);
				if (state_hash.state.contains_item(recipe.result)) {
					done = true;
					goal_id = state_hash.state_id;
					break;
				}
			}
			frontier.push({ state_hash.state_id, new_g, heuristic(state_hash.state), new_state_info.action_count });
		}
	}

	return extract_actions<State_Info>(goal_id, states);
}

size_t A_Star::get_action_cost(const Joint_Action& joint_action) const {
	size_t result = 0;
	for (const auto& action : joint_action.actions) {
		result += action.direction == Direction::NONE ? 0 : 1;
	}
	return result;
}