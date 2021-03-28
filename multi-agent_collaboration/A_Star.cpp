#include "A_Star.hpp"
#include "Search.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>

std::vector<Joint_Action> A_Star::search_joint(const State& original_state, 
		Recipe recipe, const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent) const {

	//std::cout << "\n\nStarting search " << recipe.result_char() << " " << agents.to_string() << (handoff_agent.has_value() ? "/" + std::to_string(handoff_agent.value().id) : "") << "\n" << std::endl;

	Node_Queue frontier;													// Open list		f, g, h, state_id, action_count
	std::unordered_set<State_Hash> visited;									// State lookup		state, state_id, has_agent_passed
	std::vector<State_Info> states;											// State id lookup	state, parent_state_id, action, g, action_count, has_agent_passed
	std::vector<bool> closed;												// Closed list		[state_id]=is_closed

	Heuristic heuristic(environment, recipe.ingredient1, recipe.ingredient2, agents);
	size_t goal_id = EMPTY_VAL;
	auto actions = get_actions(agents, handoff_agent.has_value());

	initialize_variables(frontier, visited, states, closed, heuristic, original_state);

	while (goal_id == EMPTY_VAL) {

		// No possible path
		auto current_state_id_opt = get_next_state(frontier, closed);
		if (!current_state_id_opt.has_value()) {
			return {};
		}
		auto current_state_id = current_state_id_opt.value();

		// Exceeded depth limit
		auto current_state_info = states.at(current_state_id);
		if (current_state_info.g >= depth_limit) {
			continue;
		}

		//print_current(current_state_id, states, current_state_info);

		for (const auto& action : actions) {

			// Perform action if valid
			auto [action_valid, new_state_info] = check_and_perform(action, current_state_info, current_state_id, handoff_agent);
			if (!action_valid) {
				continue;
			}

			auto state_hash = State_Hash(new_state_info.state, states.size(), new_state_info.has_agent_passed);
			auto visited_it = visited.find(state_hash);


			if (visited_it != visited.end()) {
				if (new_state_info < states.at(visited_it->state_id)) {
					// Faster path found to already expanded node
					if (closed.at(visited_it->state_id) && new_state_info.g != states.at(visited_it->state_id).g) {
						throw std::runtime_error("Heuristic is not consistent");

					// Faster path found
					} else {
						state_hash = *visited_it;
						states.at(state_hash.state_id).update(new_state_info);
					}
				// Found slower path
				} else {
					continue;
				}
			// New state
			} else {
				visited.insert(state_hash);
				states.push_back(new_state_info);
				if (state_hash.state.contains_item(recipe.result)) {
					if (handoff_agent.has_value() && action.is_action_useful(handoff_agent.value())) {
						closed.push_back(true);
						continue;

					// Goal
					} else {
						goal_id = state_hash.state_id;
						break;
					}
				} else {
					closed.push_back(false);
				}
			}
			frontier.push({ state_hash.state_id, new_state_info.g, heuristic(state_hash.state), new_state_info.action_count });
		}
	}

	return extract_actions<State_Info>(goal_id, states);
}

void A_Star::print_current(size_t current_state_id, const std::vector<State_Info>& states, const State_Info& current_state_info) const {
	std::cout << "State " << current_state_id << ", Parent " << states.at(current_state_id).parent_id << " ";
	current_state_info.state.print_compact();
	std::cout << std::endl;
}

std::pair<bool, State_Info> A_Star::check_and_perform(const Joint_Action& action, const State_Info& current_state_info, const size_t& current_state_id, const std::optional<Agent_Id>& handoff_agent) const {
	
	// Useful action from handoff agent after handoff
	if (current_state_info.has_agent_passed && handoff_agent.has_value() && action.is_action_useful(handoff_agent.value())) {
		return { false, {} };
	}

	// Handoff action, simply change handoff status
	if (!action.is_action_valid()) {
		auto new_state_info = current_state_info;
		new_state_info.has_agent_passed = true;
		return { true, new_state_info };
	}

	// Action is illegal or causes no change
	auto new_state_info = current_state_info;
	if (!environment.act(new_state_info.state, action, Print_Level::NOPE)) {
		return { false, {} };
	}

	// Action performed
	new_state_info.parent_id = current_state_id;
	new_state_info.action = action;
	new_state_info.g += 1;
	new_state_info.action_count += get_action_cost(action);
	new_state_info.has_agent_passed |= is_being_passed(handoff_agent, action, current_state_info);
	return { true, new_state_info };
}

size_t A_Star::get_action_cost(const Joint_Action& joint_action) const {
	size_t result = 0;
	for (const auto& action : joint_action.actions) {
		result += action.direction == Direction::NONE ? 0 : 1;
	}
	return result;
}

bool A_Star::is_being_passed(const std::optional<Agent_Id>& handoff_agent, const Joint_Action& action, const State_Info& current_state_info) const {
	return (handoff_agent.has_value() && !action.is_action_useful(handoff_agent.value())) || current_state_info.has_agent_passed;
}

void A_Star::initialize_variables(Node_Queue& frontier,	std::unordered_set<State_Hash>& visited, std::vector<State_Info>& states, 
	std::vector<bool>& closed, Heuristic& heuristic, const State& original_state) const {

	size_t original_id = 0;
	size_t original_g = 0;
	size_t original_action_count = 0;
	frontier.push({ original_id, original_g, heuristic(original_state), original_action_count });
	visited.insert({ original_state, original_id, false });
	states.push_back({ original_state, original_id, {}, original_g, original_action_count, false });
	closed.push_back(false);
}

std::vector<Joint_Action> A_Star::get_actions(const Agent_Combination& agents, bool has_handoff_agent) const {
	auto actions = environment.get_joint_actions(agents);
	if (has_handoff_agent) {
		Joint_Action pass_action;
		actions.push_back(pass_action);
	}
	return actions;
}

std::optional<size_t> A_Star::get_next_state(Node_Queue& frontier, std::vector<bool>& closed) const {
	while (!frontier.empty()) {
		auto current_state_id = frontier.top().state_id;
		frontier.pop();
		if (!closed.at(current_state_id)) {
			closed.at(current_state_id) = true;
			return current_state_id;
		}
	}
	return {};
}