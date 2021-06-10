#include "A_Star.hpp"
#include "Search.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>


A_Star::A_Star(const Environment& environment, size_t depth_limit) 
	: Search_Method(environment, depth_limit), heuristic(environment) {
}
/**
original_state	Initial state to search from
recipe			Recipe to perform
agents			All agents allowed to move
handoff_agent	Agent not allowed to perform the goal action
input_actions	Fixed initial actions for agents not in free_agents
free_agents		Agents allowed any move any time
*/

std::vector<Joint_Action> A_Star::search_joint(const State& original_state,
		Recipe recipe, const Agent_Combination& agents, Agent_Id handoff_agent, 
	const std::vector<Joint_Action>& input_actions, const Agent_Combination& free_agents, const Action& initial_action) {

	heuristic.set(recipe.ingredient1, recipe.ingredient2, agents, handoff_agent);
	PRINT(Print_Category::A_STAR, Print_Level::VERBOSE, std::string("\n\nStarting search ") 
		+ recipe.result_char() + " " + agents.to_string() +(handoff_agent.is_empty() ? "" : "/" 
		+ std::to_string(handoff_agent.id)) + "\n\n");
	
	auto actions = get_actions(agents, false);
	Search_Info si = initialize_variables(recipe, original_state, handoff_agent, agents, input_actions);

	while (!si.has_goal_node()) {

		// No possible path
		auto current_node = get_next_node(si);
		if (current_node == nullptr) {
			return {};
		}

		for (const auto& action : actions) {

			// Fits the requirement for initial actions
			if (!action_conforms_to_input(current_node, input_actions, action, free_agents, initial_action)) {
				continue;
			}

			// Perform action if valid
			auto new_node = check_and_perform(si, action, current_node, input_actions);
			if (new_node == nullptr) {
				continue;
			}

			print_current(new_node);
			if (process_node(si, new_node, action)) {
				auto handoff_node = generate_handoff(si, new_node, input_actions);
				if (handoff_node != nullptr) {
					if (process_node(si, handoff_node, action)) {
						print_current(handoff_node);
					}
				}
			}
		}
	}
	if (si.goal_node != nullptr) {
		print_goal(si.goal_node);
	}
	return extract_actions(si.goal_node);
}

std::pair<size_t, Direction> A_Star::get_dist_direction(Coordinate source, Coordinate dest, size_t walls) {
	return heuristic.get_dist_direction(source, dest, walls);
}

bool A_Star::process_node(Search_Info& si, Node* node, const Joint_Action& action) const {
	auto& visited = si.visited;
	auto& frontier = si.frontier;
	auto& nodes = si.nodes;
	auto visited_it = visited.find(node);

	// Existing state
	if (visited_it != visited.end()) {
		if (node->is_shorter(*visited_it)) {
			(*visited_it)->valid = false;
			visited.erase(visited_it);
			visited.insert(node);
			frontier.push(node);
		} else {
			assert(&nodes.back() == node);
			nodes.pop_back();
			return false;
		}

		// New state
	} else {

		// Goal state which does NOT satisfy handoff_agent
		if (is_invalid_goal(si, node, action)) {
			node->closed = true;
			return false;

			// Goal state which DOES satisfy handoff_agent
		} else if (is_valid_goal(si, node, action)) {
			si.goal_node = node;

			// Non-goal state
		} else {
			visited.insert(node);
			frontier.push(node);
		}
	}
	return true;
}

bool A_Star::action_conforms_to_input(const Node* current_node, const std::vector<Joint_Action>& input_actions,
	const Joint_Action action, const Agent_Combination& free_agents, const Action& initial_action) const {
	if (current_node->g == 0 
		&& initial_action.has_value()
		&& action.get_action(initial_action.agent) != initial_action) {
		return false;
	}

	size_t input_action_size = input_actions.size();

	if (current_node->g < input_action_size) {
		auto& action_ref = input_actions.at(current_node->g);
		for (size_t action_index = 0; action_index < action_ref.actions.size(); ++action_index) {
			if (!free_agents.contains({ action_index })
				&& action_ref.get_action({ action_index }) != action.get_action({ action_index })) {
				return false;
			}
		}
	}
	return true;
}

std::vector<Joint_Action> A_Star::extract_actions(const Node* node) const {
	std::vector<Joint_Action> result;
	while (node->parent != nullptr) {
		if (node->action.is_action_valid()) {
			result.push_back(node->action);
		}
		node = node->parent;
	}
	std::vector<Joint_Action> reversed;
	for (auto it = result.rbegin(); it != result.rend(); ++it) {
		reversed.push_back(*it);
	}
	return reversed;
}

bool A_Star::is_invalid_goal(const Search_Info& si, const Node* node, const Joint_Action& action) const {
	return node->state.contains_item(si.recipe.result) 
		&& si.handoff_agent.is_not_empty() 
		&& action.is_not_none(si.handoff_agent);
}

bool A_Star::is_valid_goal(const Search_Info& si, const Node* node, const Joint_Action& action) const {
	return node->state.contains_item(si.recipe.result)
		&& (!si.handoff_agent.is_not_empty()
			|| (node->has_agent_passed() 
				&& !action.is_not_none(si.handoff_agent)));
}

Node* A_Star::check_and_perform(Search_Info& si, const Joint_Action& action,
	const Node* current_node, const std::vector<Joint_Action>& input_actions) const {
	Node_Ref& nodes = si.nodes;
	auto& handoff_agent = si.handoff_agent;
	
	// Useful action from handoff agent after handoff
	if (current_node->has_agent_passed()
		&& handoff_agent.is_not_empty()
		&& action.is_not_none(handoff_agent)) {

		return nullptr;
	}
	nodes.emplace_back(current_node, nodes.size());
	auto new_node = &nodes.back();

	// Action is illegal or causes no change
	if (!environment.act(new_node->state, action, Print_Level::NOPE)) {
		nodes.pop_back();
		return nullptr;
	}

	// Action performed
	new_node->action = action;
	new_node->g += 1;
	new_node->action_count += get_action_cost(action, handoff_agent);
	new_node->closed = false;
	new_node->h = heuristic(new_node->state, si.agents, handoff_agent);

	if (handoff_agent.is_not_empty() && action.get_action(handoff_agent).is_not_none()) {
		new_node->handoff_first_action = std::min(new_node->g, new_node->handoff_first_action);
		//new_node->pass_time = new_node->g;
	}

	new_node->calculate_hash();

	return new_node;
}

Node* A_Star::generate_handoff(Search_Info& si, Node* node, const std::vector<Joint_Action>& input_actions) const {
	auto& nodes = si.nodes;
	Node* pass_node = nullptr;
	if (si.handoff_agent.is_not_empty() 
		&& !node->has_agent_passed()) {
		auto item = node->state.get_agent(si.handoff_agent).item;
		if (!item.has_value()
			|| (item.value() != si.recipe.ingredient1
				&& item.value() != si.recipe.ingredient2)) {

			nodes.emplace_back(node, nodes.size());
			pass_node = &nodes.back();
			pass_node->parent = node->parent;
			pass_node->pass_time = node->g;
		}
	}
	return pass_node;
}

size_t A_Star::get_action_cost(const Joint_Action& joint_action, const Agent_Id& handoff_agent) const {
	size_t result = 0;
	for (size_t agent = 0; agent < joint_action.size(); ++agent) {
		if (!handoff_agent.is_not_empty() || handoff_agent.id != agent) {
			result += joint_action.get_action(agent).is_none() ? 0 : 1;
		}
	}
	return result;
}

Search_Info A_Star::initialize_variables(Recipe& recipe, const State& original_state, const Agent_Id& handoff_agent, const Agent_Combination& agents, const std::vector<Joint_Action>& input_actions) const {

	Search_Info si(recipe, handoff_agent, agents);

	constexpr size_t id = 0;
	constexpr size_t g = 0;
	constexpr size_t action_count = 0;
	constexpr Node* parent = nullptr;
	constexpr bool closed = false;
	constexpr bool valid = true;
	constexpr size_t handoff_first_action = EMPTY_VAL;
	constexpr bool can_pass = false;
	bool require_handoff = false;
	size_t pass_time = EMPTY_VAL;
	//size_t pass_time = 0;
	Joint_Action action;
	size_t h = 0;
	h = heuristic(original_state, agents, handoff_agent);
	Agent_Id agent;

	// Standard node
	si.nodes.emplace_back(original_state, id, g, h, action_count, pass_time, can_pass, handoff_first_action, parent, action, closed, valid, agent);
	auto node = &si.nodes.back();
	node->calculate_hash();
	si.frontier.push(node);
	si.visited.insert(node);

	// Immediate handoff
	auto handoff_node = generate_handoff(si, node, input_actions);
	if (handoff_node != nullptr) {
		si.frontier.push(handoff_node);
		si.visited.insert(handoff_node);
	}

	return si;
}

std::vector<Joint_Action> A_Star::get_actions(const Agent_Combination& agents, bool has_handoff_agent) const {
	return environment.get_joint_actions(agents);
}

Node* A_Star::get_next_node(Search_Info& si) const {
	while (!si.frontier.empty()) {
		auto current_node = si.frontier.top();
		si.frontier.pop();

		// Exceeded depth limit
		if (current_node->f() >= depth_limit || current_node->h == EMPTY_VAL) {
			current_node->closed = true;
			continue;
		}

		// Unexplored and valid
		if (!current_node->closed && current_node->valid) {
			current_node->closed = true;
			return current_node;
		}
	}
	return nullptr;
}

void A_Star::print_current(const Node* node) const {
	if (!is_print_allowed(Print_Level::VERBOSE)) {
		return;
	}
	std::cout << "Node " << node->id << ", g=" << node->g << ", Parent " << (node->parent == nullptr ? "-" : std::to_string(node->parent->id)) << ", " << node->action.to_string() << ", pt " << (node->pass_time == EMPTY_VAL ? "X" : std::to_string(node->pass_time)) << ", ";
	node->state.print_compact();
	std::cout << std::endl;
}

void A_Star::print_goal(const Node* node) const {
	if (!is_print_allowed(Print_Level::VERBOSE)) {
		return;
	}
	if (node->parent == nullptr) {
		std::cout << "Printing goal" << std::endl;
	} else {
		print_goal(node->parent);
	}
	print_current(node);
}