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
		Recipe recipe, const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent, 
	const std::vector<Joint_Action>& input_actions, const Agent_Combination& free_agents) {

	heuristic.set(recipe.ingredient1, recipe.ingredient2, agents, handoff_agent);
	PRINT(Print_Category::A_STAR, Print_Level::VERBOSE, std::string("\n\nStarting search ") 
		+ recipe.result_char() + " " + agents.to_string() +(handoff_agent.has_value() ? "/" 
		+ std::to_string(handoff_agent.value().id) : "") + "\n\n");

	Node_Queue frontier;
	Node_Set visited;
	Node_Ref nodes;

	Node* goal_node = nullptr;
	auto actions = get_actions(agents, handoff_agent.has_value());
	auto source = original_state;
	size_t input_action_size = input_actions.size();
	//source.purge(agents);

	initialize_variables(frontier, visited, nodes, source, handoff_agent);

	if (nodes.front().h == EMPTY_VAL) {
		return {};
	}

	while (goal_node == nullptr) {

		// No possible path
		auto current_node = get_next_node(frontier);
		if (current_node == nullptr) {
			return {};
		}

		//print_current(current_node);

		for (const auto& action : actions) {

			// Fits the requirement for initial actions
			if (!action_conforms_to_input(current_node, input_actions, action, free_agents)) {
				continue;
			}


			// Perform action if valid
			auto [action_valid, new_node] = check_and_perform(action, nodes, current_node, handoff_agent);
			if (!action_valid) {
				continue;
			}

			auto visited_it = visited.find(new_node);

			// Existing state
			if (visited_it != visited.end()) {
				if (new_node->is_shorter(*visited_it)) {
						(*visited_it)->valid = false;
						visited.erase(visited_it);
						visited.insert(new_node);
						frontier.push(new_node);
				} else {
					nodes.pop_back();
				}

			// New state
			} else {

				// Goal state which does NOT satisfy handoff_agent
				if (is_invalid_goal(new_node, recipe, action, handoff_agent)) {
					new_node->closed = true;

				// Goal state which DOES satisfy handoff_agent
				} else if (is_valid_goal(new_node, recipe, action, handoff_agent)) {
					goal_node = new_node;
					break;

				// Non-goal state
				} else {
					visited.insert(new_node);
					frontier.push(new_node);
				}
			}
		}
	}
	auto result_actions = extract_actions(goal_node);
	
	//if (agents.size() == 1 && agents.contains(Agent_Id{ 1 })) {
	//	for (auto it = nodes.begin(); it != nodes.end(); ++it) {
	//		print_current(&(*it));
	//	}
	//}

	return result_actions;
}

bool A_Star::action_conforms_to_input(const Node* current_node, const std::vector<Joint_Action>& input_actions,
	const Joint_Action action, const Agent_Combination& free_agents) const {
	size_t input_action_size = input_actions.size();

	// Handoff action, true by default
	if (!action.is_action_valid()) {
		return true;
	}

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

bool A_Star::is_invalid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, const std::optional<Agent_Id>& handoff_agent) const {
	return node->state.contains_item(recipe.result) 
		&& handoff_agent.has_value() 
		&& action.is_action_non_trivial(handoff_agent.value());
}

bool A_Star::is_valid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, const std::optional<Agent_Id>& handoff_agent) const {
	return node->state.contains_item(recipe.result)
		&& (!handoff_agent.has_value()
			|| (node->has_agent_passed() 
				&& !action.is_action_non_trivial(handoff_agent.value())));
}

void A_Star::print_current(const Node* node) const {
	std::cout << "Node " << node->id << ", Parent " << (node->parent == nullptr ? "-" : std::to_string(node->parent->id)) << " ";
	node->state.print_compact();
	std::cout << std::endl;
}

std::pair<bool, Node*> A_Star::check_and_perform(const Joint_Action& action, Node_Ref& nodes, const Node* current_node, const std::optional<Agent_Id>& handoff_agent) const {
	
	// Useful action from handoff agent after handoff
	if (current_node->has_agent_passed() 
		&& action.is_action_valid()
		&& handoff_agent.has_value() 
		&& action.is_action_non_trivial(handoff_agent.value())) {

		return { false, nullptr };
	}
	nodes.emplace_back(current_node, nodes.size());
	auto new_node = &nodes.back();

	// Handoff action, simply change handoff status
	if (!action.is_action_valid()) {
		new_node->action = action;
		new_node->g = current_node->g;
		new_node->pass_time = current_node->g;
		new_node->closed = false;
		new_node->calculate_hash();
		return { true, new_node };
	}

	// Action is illegal or causes no change
	if (!environment.act(new_node->state, action, Print_Level::NOPE)) {
		nodes.pop_back();
		return { false, nullptr };
	}

	// Action performed
	new_node->action = action;
	new_node->g += 1;
	new_node->action_count += get_action_cost(action);
	new_node->closed = false;
	new_node->h = heuristic(new_node->state, handoff_agent);

	new_node->calculate_hash();
	return { true, new_node };
}

size_t A_Star::get_action_cost(const Joint_Action& joint_action) const {
	size_t result = 0;
	for (const auto& action : joint_action.actions) {
		result += action.direction == Direction::NONE ? 0 : 1;
	}
	return result;
}

void A_Star::initialize_variables(Node_Queue& frontier, Node_Set& visited, Node_Ref& nodes, const State& original_state, const std::optional<Agent_Id>& handoff_agent) const {

	constexpr size_t id = 0;
	constexpr size_t g = 0;
	constexpr size_t action_count = 0;
	constexpr size_t pass_time = EMPTY_VAL;
	constexpr Node* parent = nullptr;
	constexpr bool closed = false;
	constexpr bool valid = true;
	Joint_Action action;
	size_t h = heuristic(original_state, handoff_agent);

	nodes.emplace_back(original_state, id, g, h, action_count, pass_time, parent, action, closed, valid);
	auto node = &nodes.back();
	node->calculate_hash();
	frontier.push(node);
	visited.insert(node);
}

std::vector<Joint_Action> A_Star::get_actions(const Agent_Combination& agents, bool has_handoff_agent) const {
	auto actions = environment.get_joint_actions(agents);
	if (has_handoff_agent) {
		Joint_Action pass_action;
		actions.push_back(pass_action);
	}
	return actions;
}

Node* A_Star::get_next_node(Node_Queue& frontier) const {
	while (!frontier.empty()) {
		auto current_node = frontier.top();
		frontier.pop();

		// Exceeded depth limit
		if (current_node->f() >= depth_limit) {
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