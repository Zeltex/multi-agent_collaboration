#include "A_Star.hpp"
#include "Search.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>

std::vector<Joint_Action> A_Star::search_joint(const State& original_state, const Environment& environment, Recipe recipe) const {
	std::priority_queue<Node, std::vector<Node>, Node_Comparator> frontier; // f, g, h, state_id
	std::unordered_set<State_Hash> visited;									// state, state_id
	std::vector<State_Info> states;											// state, parent_state_id, action, g

	Heuristic heuristic(environment, recipe.ingredient1, recipe.ingredient2);

	size_t original_id = 0;
	size_t original_g = 0;
	frontier.push({ original_id, original_g, heuristic(original_state) });
	visited.insert({ original_state, original_id });
	states.push_back({ original_state, original_id, Joint_Action({ }), original_g });

	size_t goal_id = (size_t)-1;

	bool done = false;
	while (!done) {
		auto current_state_id = frontier.top().state_id;
		frontier.pop();
		auto current_state = states.at(current_state_id).state;
		size_t new_g = states.at(current_state_id).g + 1;
		auto actions = environment.get_joint_actions(current_state);

		//std::cout << "\nState " << current_state_id << std::endl;
		//environment.print_state(current_state);

		for (const auto& action : actions) {
			auto new_state = current_state;
			environment.act(new_state, action, Print_Level::NOPE);
			auto state_hash = State_Hash(new_state, states.size());
			auto visited_it = visited.find(state_hash);
			if (visited_it == visited.end()) {


				visited.insert(state_hash);
				frontier.push({ state_hash.state_id, new_g, heuristic(state_hash.state) });
				states.push_back({ state_hash.state, current_state_id, action, new_g });

				if (state_hash.state.contains_item(recipe.result)) {
					done = true;
					goal_id = state_hash.state_id;
				}

			} else if (states.at(visited_it->state_id).g > new_g) {
				std::cerr << "Heuristic is not consistent" << std::endl;
				exit(-1);
			}
		}
		if (frontier.empty()) {
			std::cerr << "Couldn't find path" << std::endl;
			exit(-1);
		}
	}

	size_t current_id = goal_id;
	std::vector<Joint_Action> actions;
	while (current_id != 0) {
		auto temp_state = states.at(current_id);
		actions.push_back(temp_state.action);
		current_id = temp_state.parent_id;
	}

	std::vector<Joint_Action> reverse_actions;
	for (auto it = actions.rbegin(); it < actions.rend(); ++it) {
		reverse_actions.push_back(*it);
	}

	return reverse_actions;
}