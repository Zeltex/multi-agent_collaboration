#include "Planner.hpp"
#include "BFS.hpp"
#include "A_Star.hpp"
#include "Search_Trimmer.hpp"

#include <chrono>
#include <iostream>

Joint_Action Planner::get_next_action(const State& state) {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) return { {} };

	BFS bfs;
	A_Star a_star;
	
	auto goal_ingredient = recipes.at(0).result;
	//auto path = bfs.search(state, environment, goal_ingredient, agent);
	//auto path = bfs.search_joint(state, environment, goal_ingredient);
	auto path = a_star.search_joint(state, environment, recipes.at(0));

	auto trim_path = path;
	Search_Trimmer trim;
	trim.trim(trim_path, state, environment, recipes.at(0));

	return trim_path.at(0);
	//return environment.convert_to_joint_action(path.at(0), agent);
}

