#include "Planner.hpp"
#include "BFS.hpp"
#include "A_Star.hpp"
#include "Search_Trimmer.hpp"

#include <chrono>
#include <iostream>
#include <set>

struct Action_Path {

	std::vector<Joint_Action> joint_actions;
	char subtask;
	bool operator<(const Action_Path& other) const {
		return joint_actions.size() < other.joint_actions.size();
	}
};
#define INITIAL_DEPTH_LIMIT 12
Action Planner::get_next_action(const State& state) const {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) return { Direction::NONE, { agent } };

	std::set<Action_Path> paths;

	auto agent_combinations = get_combinations(environment.get_number_of_agents());

	for (const auto& agents : agent_combinations) {
		for (const auto& recipe : recipes) {
			BFS bfs;
			A_Star a_star;

			//auto goal_ingredient = recipe;
			//auto path = bfs.search(state, environment, goal_ingredient, agent);
			//auto path = bfs.search_joint(state, environment, goal_ingredient);
			auto path = a_star.search_joint(state, environment, recipe, agents, INITIAL_DEPTH_LIMIT);

			if (path.empty()) continue;

			auto trim_path = path;
			Search_Trimmer trim;
			trim.trim(trim_path, state, environment, recipe);


			paths.insert({ Action_Path{trim_path, static_cast<char>(recipe.result)} });

			std::cout << "(";
			for (const auto& agent : agents) {
				std::cout << agent.id << ",";
			}
			std::cout << ") : " << trim_path.size() << " : " << static_cast<char>(recipe.result) << std::endl;

		}
	}

	bool done = false;
	for (const auto& action_path : paths) {
		for (const auto& action : action_path.joint_actions) {
			if (action.actions.at(agent.id).direction != Direction::NONE) {
				done = true;
				break;
			}
		}
		if (done) {
			auto& joint_action = action_path.joint_actions.at(0);
			std::cout << "Agent " << agent.id << " chose action " <<
				static_cast<char>(joint_action.actions.at(agent.id).direction) << 
				" for subtask " << action_path.subtask << std::endl;
			return joint_action.actions.at(agent.id);
		}
	}
	std::cout << "Agent " << agent.id << " did not find useful action " << std::endl;

	return { Direction::NONE, { agent } };
	//while (!done) {
	//	while(!done && )
	//}



	//return paths.begin()->joint_actions.at(0).actions.at(agent.id);
	//return environment.convert_to_joint_action(path.at(0), agent);
}

// Get all combinations of numbers/agents <n
std::vector<std::vector<Agent_Id>> Planner::get_combinations(size_t n) const {
	if (n == 0) return {};
	std::vector<bool> status;
	status.push_back(true);
	for (size_t i = 1; i < n; ++i) {
		status.push_back(false);
	}
	std::vector<std::vector<Agent_Id>> combinations;

	bool done = false;
	while (!done) {
		std::vector<Agent_Id> next_combination;
		for (size_t i = 0; i < n; ++i) {
			if (status.at(i)) next_combination.push_back(i);
		}

			size_t counter = 0;
			while (true) {
				status.at(counter) = !status.at(counter);
				if (status.at(counter)) {
					break;
				} else {
					++counter;
					if (counter == status.size()) {
						done = true;
						break;
					}
				}
			}
		
		combinations.push_back(next_combination);
	}
	return combinations;
}