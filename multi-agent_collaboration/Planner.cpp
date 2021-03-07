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

Joint_Action Planner::get_next_action(const State& state) {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) return { {} };

	std::set<Action_Path> paths;

	for (const auto& recipe : recipes) {
		BFS bfs;
		A_Star a_star;
	
		auto goal_ingredient = recipe;
		//auto path = bfs.search(state, environment, goal_ingredient, agent);
		//auto path = bfs.search_joint(state, environment, goal_ingredient);
		auto path = a_star.search_joint(state, environment, recipe);

		//auto trim_path = path;
		//Search_Trimmer trim;
		//trim.trim(trim_path, state, environment, recipes.at(0));


		paths.insert({ path });
	}





	return paths.begin()->joint_actions.at(0);
	//return environment.convert_to_joint_action(path.at(0), agent);
}

