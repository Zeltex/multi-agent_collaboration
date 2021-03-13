#include "Planner.hpp"
#include "BFS.hpp"
#include "A_Star.hpp"
#include "Search.hpp"
#include "Search_Trimmer.hpp"

#include <chrono>
#include <iostream>
#include <set>
#include <deque>



#define INITIAL_DEPTH_LIMIT 18
Action Planner::get_next_action(const State& state) const {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) {
		return { Direction::NONE, { agent } };
	}
	auto paths = get_all_paths(recipes, state);
	return get_best_action(paths, recipes);
}

Action Planner::get_best_action(const std::set<Action_Path>& paths, const std::vector<Recipe>& recipes) const {
	
	// Init best solutions
	std::map<Recipe, Recipe_Solution> best_solutions;
 	for (const auto& recipe : recipes) {
		Recipe_Solution solution { {}, (size_t)-1 };
		best_solutions.insert({ recipe, solution });
	}

	// Note best solutions
	for (const auto& action_path : paths) {
		auto& current_solution = best_solutions.at(action_path.recipe);
		if (current_solution > action_path) {
			current_solution.action_count = action_path.joint_actions.size();
			current_solution.agents = action_path.agents;
		}
	}

	// Find best action
	bool done = false;
	for (const auto& action_path : paths) {

		if (!agent_in_best_solution(best_solutions, action_path)) {
			continue;
		}		

		if (contains_useful_action(action_path)) {
			auto& joint_action = action_path.joint_actions.at(0);
			std::cout << "Agent " << agent.id << " chose action " <<
				static_cast<char>(joint_action.actions.at(agent.id).direction) <<
				" for subtask " << static_cast<char>(action_path.recipe.result) << std::endl;
			return joint_action.actions.at(agent.id);
		}
	}
	std::cout << "Agent " << agent.id << " did not find useful action " << std::endl;
	return { Direction::NONE, { agent } };
}

bool Planner::agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, const Action_Path& action_path) const {
	auto& best_agents = best_solutions.at(action_path.recipe).agents;
	return best_agents.contains(agent);
}

bool Planner::contains_useful_action(const Action_Path& action_path) const {
	for (const auto& action : action_path.joint_actions) {
		if (action.actions.at(agent.id).direction != Direction::NONE) {
			return true;
		}
	}
	return false;
}

std::set<Action_Path> Planner::get_all_paths(const std::vector<Recipe>& recipes, const State& state) const {
	std::set<Action_Path> paths;
	auto agent_combinations = get_combinations(environment.get_number_of_agents());

	auto recipe_size = recipes.size();
	for (const auto& agents : agent_combinations) {
		size_t recipe_counter = 0;
		for (size_t i = 0; i < recipe_size; ++i) {
			const auto& recipe = recipes.at(i);

			if (!ingredients_reachable(recipe, agents, state)) {
				continue;
			}

			if (state.items_hoarded(recipe, agents)) {
				continue;
			}

			Search search(std::make_unique<A_Star>());

			auto time_start = std::chrono::system_clock::now();
			auto path = search.search_joint(state, environment, recipe, agents, INITIAL_DEPTH_LIMIT);
			auto time_end = std::chrono::system_clock::now();

			auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();



			auto trim_path = path;
			if (!trim_path.empty()) {
				Search_Trimmer trim;
				trim.trim(trim_path, state, environment, recipe);


				paths.insert({ Action_Path{trim_path, recipe, agents} });
			}


			std::cout << "(";
			for (const auto& agent : agents.get()) {
				std::cout << agent.id << ",";
			}
			std::cout << ") : " << trim_path.size() << " : " << static_cast<char>(recipe.result) <<  " : " << diff << std::endl;

		}
	}
	return paths;
}

bool Planner::ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const {
	auto reachables = agent_reachables.find(agents);
	if (reachables == agent_reachables.end()) {
		std::cerr << "Unknown agent combination" << std::endl;
		exit(-1);
	}

	for (const auto& location : environment.get_locations(state, recipe.ingredient1)) {
		if (!reachables->second.get(location)) {
			return false;
		}
	}

	for (const auto& location : environment.get_locations(state, recipe.ingredient2)) {
		if (!reachables->second.get(location)) {
			return false;
		}
	}
}

// Get all combinations of numbers/agents <n
std::vector<Agent_Combination> Planner::get_combinations(size_t n) const {
	if (n == 0) return {};
	std::vector<bool> status;
	status.push_back(true);
	for (size_t i = 1; i < n; ++i) {
		status.push_back(false);
	}
	std::vector<Agent_Combination> combinations;

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


void Planner::intialize_reachables(const State& initial_state) {
	auto combinations = get_combinations(initial_state.agents.size());
	for (const auto& agents : combinations) {
		Reachables reachables(environment.get_width(), environment.get_height());

		// Initial agent locations
		std::deque<Coordinate> frontier;
		for (const auto& agent : agents.get()) {
			auto location = initial_state.get_location(agent);
			reachables.set(location, true);
			frontier.push_back(location);
		}

		// BFS search
		while (!frontier.empty()) {
			auto next = frontier.front();
			frontier.pop_front();
			
			for (const auto& location : environment.get_neighbours(next)) {
				if (!reachables.get(location)) {
					reachables.set(location, true);
					if (!environment.is_cell_type(location, Cell_Type::WALL)) {
						frontier.push_back(location);
					}
				}
			}
		}
		agent_reachables.insert({ agents, reachables });
	}
}