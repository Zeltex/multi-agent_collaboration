#include "Planner.hpp"
#include "BFS.hpp"
#include "A_Star.hpp"
#include "Search_Trimmer.hpp"

#include <chrono>
#include <iostream>
#include <set>

struct Action_Path {

	std::vector<Joint_Action> joint_actions;
	bool operator<(const Action_Path& other) const {
		return joint_actions.size() < other.joint_actions.size();
	}
};

Action Planner::get_next_action(const State& state) const {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) return { Direction::NONE, { agent } };

	std::set<Action_Path> paths;

	for (const auto& recipe : recipes) {
		BFS bfs;
		A_Star a_star;
	
		//auto goal_ingredient = recipe;
		//auto path = bfs.search(state, environment, goal_ingredient, agent);
		//auto path = bfs.search_joint(state, environment, goal_ingredient);
		auto path = a_star.search_joint(state, environment, recipe);

		auto trim_path = path;
		Search_Trimmer trim;
		trim.trim(trim_path, state, environment, recipes.at(0));


		paths.insert({ trim_path });
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
			return joint_action.actions.at(agent.id);
		}
	}
	return { Direction::NONE, { agent } };
	//while (!done) {
	//	while(!done && )
	//}



	//return paths.begin()->joint_actions.at(0).actions.at(agent.id);
	//return environment.convert_to_joint_action(path.at(0), agent);
}

