#include "Planner.hpp"
#include "BFS.hpp"
#include <chrono>

Action Planner::get_next_action(const State& state) {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) return { Direction::UP, {0} };

	auto time_start = std::chrono::system_clock::now();
	BFS bfs;
	auto goal_ingredient = recipes.at(0).result;
	auto path = bfs.search(state, environment, goal_ingredient, agent);
	auto time_end = std::chrono::system_clock::now();

	auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();


	return path.at(0);
}