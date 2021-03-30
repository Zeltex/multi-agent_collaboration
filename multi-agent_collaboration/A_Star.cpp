#include "A_Star.hpp"
#include "Search.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>


std::vector<Joint_Action> A_Star::search_joint(const State& original_state, 
		Recipe recipe, const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent) const {

	//std::cout << "\n\nStarting search " << recipe.result_char() << " " << agents.to_string() << (handoff_agent.has_value() ? "/" + std::to_string(handoff_agent.value().id) : "") << "\n" << std::endl;

	Pool pool(sizeof(Node));

	Node_Queue frontier;
	Node_Set visited;
	Node_Ref nodes;

	Heuristic heuristic(environment, recipe.ingredient1, recipe.ingredient2, agents, handoff_agent);
	Node* goal_node = nullptr;
	auto actions = get_actions(agents, handoff_agent.has_value());

	initialize_variables(frontier, visited, nodes, heuristic, original_state, pool);

	while (goal_node == nullptr) {

		// No possible path
		auto current_node = get_next_node(frontier);
		if (current_node == nullptr) {
			return {};
		}

		// Exceeded depth limit
		if (current_node->f() >= depth_limit) {
			continue;
		}

		//print_current(current_node);

		for (const auto& action : actions) {

			// Perform action if valid
			auto [action_valid, new_node] = check_and_perform(action, nodes, current_node, handoff_agent, heuristic, pool);
			if (!action_valid) {
				continue;
			}

			auto visited_it = visited.find(new_node);

			if (visited_it != visited.end()) {
				if (new_node->is_shorter(*visited_it)) {
					// Faster path found to already expanded node
					if ((*visited_it)->closed && new_node->g != (*visited_it)->g) {
						std::cout << "Printing new_node/new_node_parent/it/it_parent" << std::endl;
						print_current(new_node);
						print_current(new_node->parent);
						print_current(*visited_it);
						print_current((*visited_it)->parent);
						throw std::runtime_error("Heuristic is not consistent");

					// Faster path found
					} else {
						(*visited_it)->update(new_node);
					}
				}
				nodes.pop_back();

			// New state
			} else {
				visited.insert(new_node);

				// Goal state which does NOT satisfy handoff_agent
				if (is_invalid_goal(new_node, recipe, action, handoff_agent)) {
					new_node->closed = true;

				// Goal state which DOES satisfy handoff_agent
				} else if (is_valid_goal(new_node, recipe, action, handoff_agent)) {
					goal_node = new_node;

				// Non-goal state
				} else {
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

void A_Star::print_current(const Node* node) const {
	std::cout << "Node " << node->id << ", Parent " << (node->parent == nullptr ? "-" : std::to_string(node->parent->id)) << " ";
	node->state.print_compact();
	std::cout << std::endl;
}

std::pair<bool, Node*> A_Star::check_and_perform(const Joint_Action& action, Node_Ref& nodes, const Node* current_node, const std::optional<Agent_Id>& handoff_agent, Heuristic& heuristic, Pool& pool) const {
	
	// Useful action from handoff agent after handoff
	if (current_node->has_agent_passed() && handoff_agent.has_value() && action.is_action_useful(handoff_agent.value())) {
		return { false, nullptr };
	}
	nodes.emplace_back(current_node, nodes.size());
	auto new_node = &nodes.back();

	// Handoff action, simply change handoff status
	if (!action.is_action_valid()) {
		new_node->g = current_node->g;
		new_node->pass_time = current_node->g;
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
	new_node->h = heuristic(new_node->state);

	// TODO - Probably don't need this on account of the second if statement in this method
	if (!new_node->has_agent_passed() && is_being_passed(handoff_agent, action, current_node)) {
		new_node->pass_time = new_node->g;
	}
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

bool A_Star::is_being_passed(const std::optional<Agent_Id>& handoff_agent, const Joint_Action& action, const Node* current_node) const {
	return handoff_agent.has_value() && !action.is_action_useful(handoff_agent.value());
}

void A_Star::initialize_variables(Node_Queue& frontier, Node_Set& visited, Node_Ref& nodes, Heuristic& heuristic, const State& original_state, Pool& pool) const {

	constexpr size_t id = 0;
	constexpr size_t g = 0;
	constexpr size_t action_count = 0;
	constexpr size_t pass_time = EMPTY_VAL;
	constexpr Node* parent = nullptr;
	constexpr bool closed = false;
	Joint_Action action;
	size_t h = heuristic(original_state);

	nodes.emplace_back(original_state, id, g, h, action_count, pass_time, parent, action, closed);
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
		if (!current_node->closed) {
			current_node->closed = true;
			return current_node;
		}
	}
	return nullptr;
}