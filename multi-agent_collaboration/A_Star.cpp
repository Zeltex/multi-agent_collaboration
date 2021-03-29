#include "A_Star.hpp"
#include "Search.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>

std::vector<Joint_Action> A_Star::search_joint(const State& original_state, 
		Recipe recipe, const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent) const {

	//std::cout << "\n\nStarting search " << recipe.result_char() << " " << agents.to_string() << (handoff_agent.has_value() ? "/" + std::to_string(handoff_agent.value().id) : "") << "\n" << std::endl;

	Node_Queue frontier;												// Open list
	Node_Set visited;									// State lookup	

	Heuristic heuristic(environment, recipe.ingredient1, recipe.ingredient2, agents);
	Node* goal_node = nullptr;
	auto actions = get_actions(agents, handoff_agent.has_value());

	initialize_variables(frontier, visited, heuristic, original_state);

	while (goal_node == nullptr) {

		// No possible path
		auto current_node = get_next_node(frontier);
		if (current_node == nullptr) {
			return {};
		}

		// Exceeded depth limit
		if (current_node->g >= depth_limit) {
			continue;
		}

		//print_current(current_state_id, states, current_state_info);

		for (const auto& action : actions) {

			// Perform action if valid
			auto [action_valid, new_node] = check_and_perform(action, current_node, handoff_agent);
			if (!action_valid) {
				continue;
			}

			auto visited_it = visited.find(new_node.get());

			if (visited_it != visited.end()) {
				if (new_node->is_shorter(*visited_it)) {
					// Faster path found to already expanded node
					if ((*visited_it)->closed && new_node->g != (*visited_it)->g) {
						throw std::runtime_error("Heuristic is not consistent");

					// Faster path found
					} else {
						(*visited_it)->update(new_node.get());
					}
				} 

			// New state
			} else {
				auto new_node_ptr = new_node.release();
				visited.insert(new_node_ptr);

				// Goal state which does NOT satisfy handoff_agent
				if (is_invalid_goal(new_node_ptr, recipe, action, handoff_agent)) {
					new_node_ptr->closed = true;

				// Goal state which DOES satisfy handoff_agent
				} else if (is_valid_goal(new_node_ptr, recipe, action, handoff_agent)) {
					goal_node = new_node_ptr;

				// Non-goal state
				} else {
					frontier.push(new_node_ptr);
				}
			}
		}
	}
	auto result_actions = extract_actions(goal_node);
	
	for (auto it : visited) {
		delete it;
	}

	return result_actions;
}

std::vector<Joint_Action> A_Star::extract_actions(const Node* node) const {
	std::vector<Joint_Action> result;
	while (node->parent != nullptr) {
		result.push_back(node->action);
		node = node->parent;
	}
	return result;
}

bool A_Star::is_invalid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, const std::optional<Agent_Id>& handoff_agent) const {
	return node->state.contains_item(recipe.result) 
		&& handoff_agent.has_value() 
		&& action.is_action_useful(handoff_agent.value());
}

bool A_Star::is_valid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, const std::optional<Agent_Id>& handoff_agent) const {
	return node->state.contains_item(recipe.result)
		&& (!handoff_agent.has_value()
			|| (node->has_agent_passed() 
				&& !action.is_action_useful(handoff_agent.value())));
}

void A_Star::print_current(size_t current_state_id, const std::vector<State_Info>& states, const State_Info& current_state_info) const {
	std::cout << "State " << current_state_id << ", Parent " << states.at(current_state_id).parent_id << " ";
	current_state_info.state.print_compact();
	std::cout << std::endl;
}

std::pair<bool, std::unique_ptr<Node>> A_Star::check_and_perform(const Joint_Action& action, const Node* current_node, const std::optional<Agent_Id>& handoff_agent) const {
	
	// Useful action from handoff agent after handoff
	if (current_node->has_agent_passed() && handoff_agent.has_value() && action.is_action_useful(handoff_agent.value())) {
		return { false, std::unique_ptr<Node>() };
	}

	// Handoff action, simply change handoff status
	if (!action.is_action_valid()) {
		std::unique_ptr<Node> new_node = std::make_unique<Node>(current_node, current_node->g);
		new_node->calculate_hash();
		return { true, std::move(new_node) };
	}

	// Action is illegal or causes no change
	auto new_node = std::make_unique<Node>(current_node);
	if (!environment.act(new_node->state, action, Print_Level::NOPE)) {
		return { false, std::unique_ptr<Node>() };
	}

	// Action performed
	new_node->parent = current_node;
	new_node->action = action;
	new_node->g += 1;
	new_node->action_count += get_action_cost(action);
	new_node->closed = false;

	// TODO - Probably don't need this on account of the second if statement in this method
	if (!new_node->has_agent_passed() && is_being_passed(handoff_agent, action, current_node)) {
		new_node->pass_time = new_node->g;
	}
	new_node->calculate_hash();
	return { true, std::move(new_node) };
}

size_t A_Star::get_action_cost(const Joint_Action& joint_action) const {
	size_t result = 0;
	for (const auto& action : joint_action.actions) {
		result += action.direction == Direction::NONE ? 0 : 1;
	}
	return result;
}

bool A_Star::is_being_passed(const std::optional<Agent_Id>& handoff_agent, const Joint_Action& action, const Node* current_node) const {
	return handoff_agent.has_value() && !action.is_action_useful(handoff_agent.value());
}

void A_Star::initialize_variables(Node_Queue& frontier, Node_Set& visited,	Heuristic& heuristic, const State& original_state) const {

	constexpr size_t id = 0;
	constexpr size_t g = 0;
	constexpr size_t action_count = 0;
	constexpr size_t pass_time = EMPTY_VAL;
	constexpr Node* parent = nullptr;
	constexpr bool closed = false;
	Joint_Action action;
	size_t h = heuristic(original_state);

	Node* node = new Node(original_state, id, g, h, action_count, pass_time, parent, action, closed);

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
		if (!current_node->closed) {
			current_node->closed = true;
			return current_node;
		}
	}
	return nullptr;
}