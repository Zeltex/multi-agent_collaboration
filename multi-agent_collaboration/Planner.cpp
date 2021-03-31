#include "Planner.hpp"
#include "BFS.hpp"
#include "A_Star.hpp"
#include "Search.hpp"
#include "Search_Trimmer.hpp"
#include "Utils.hpp"

#include <chrono>
#include <iostream>
#include <set>
#include <deque>
#include <numeric>
#include <algorithm>



constexpr auto INITIAL_DEPTH_LIMIT = 18;

Planner::Planner(Environment environment, Agent_Id agent, const State& initial_state)
	: agent(agent), environment(environment), time_step(0), search(std::make_unique<A_Star>(environment, INITIAL_DEPTH_LIMIT)) {

	initialize_reachables(initial_state);
	initialize_solutions();
}

Action Planner::get_next_action(const State& state) {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) {
		return { Direction::NONE, { agent } };
	}
	auto paths = get_all_paths(recipes, state);
	update_recipe_solutions(paths);
	recognize_goals();
	auto action = get_best_action(paths, recipes);
	++time_step;
	return action;
}

Action Planner::get_best_action(const std::set<Action_Path>& paths, const std::vector<Recipe>& recipes) const {
	
	std::map<Recipe, Agent_Usefulnes> agent_solutions;

	// Init best solutions
	std::map<Recipe, Recipe_Solution> best_solutions;
 	for (const auto& recipe : recipes) {
		Recipe_Solution solution { Agent_Combination{}, EMPTY_VAL };
		best_solutions.insert({ recipe, solution });
		agent_solutions.insert({ recipe, Agent_Usefulnes{} });
	}

	for (const auto& action_path : paths) {
		// Note best solutions
		auto& current_solution = best_solutions.at(action_path.recipe);
		if (current_solution > action_path) {
			current_solution.action_count = action_path.joint_actions.size();
			current_solution.agents = action_path.agents;
		}

		// Note usefulness of agent for recipe
		agent_solutions.at(action_path.recipe).update(action_path, agent);

		//auto& agent_solution = agent_solutions.at(action_path.recipe);
		//if (action_path.agents.contains(agent)) {
		//	agent_solution.incl_agent = std::min(agent_solution.incl_agent, action_path.size());
		//} else {
		//	agent_solution.excl_agent = std::min(agent_solution.excl_agent, action_path.size());
		//}
	}

	for (const auto& agent_solution : agent_solutions) {
		PRINT(Print_Category::PLANNER, std::to_string(agent_solution.first.result_char()) + " : "
			+ std::to_string(agent_solution.second.incl_length) + "/"
			+ std::to_string(agent_solution.second.excl_length) + " : "
			+ std::to_string(agent_solution.second.get_usefulness()) + '\n');
	}

	for (const auto& agent_solution : agent_solutions) {
		const auto& agent_usefulness = agent_solution.second;
		if (agent_usefulness.is_useful()) {
			return agent_usefulness.action;
		}
		//if (((int)agent_solution.second.excl_agent - agent_solution.second.incl_agent) / 2 > )
	}


	// Find best action
	//bool done = false;
	//for (const auto& action_path : paths) {

	//	if (!agent_in_best_solution(best_solutions, action_path)) {
	//		continue;
	//	}		

	//	if (action_path.contains_useful_action()) {
	//		auto& joint_action = action_path.joint_actions.at(0);
	//		std::cout << "Agent " << agent.id << " chose action " <<
	//			static_cast<char>(joint_action.actions.at(agent.id).direction) <<
	//			" for subtask " << static_cast<char>(action_path.recipe.result) << std::endl;
	//		return joint_action.actions.at(agent.id);
	//	}
	//}
	PRINT(Print_Category::PLANNER, std::string("Agent ") + std::to_string(agent.id) + " did not find useful action \n");
	return { Direction::NONE, { agent } };
}

bool Planner::agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, const Action_Path& action_path) const {
	auto& best_agents = best_solutions.at(action_path.recipe).agents;
	return best_agents.contains(agent);
}

std::set<Action_Path> Planner::get_all_paths(const std::vector<Recipe>& recipes, const State& state) {
	std::set<Action_Path> paths;
	auto agent_combinations = get_combinations(environment.get_number_of_agents());

	auto recipe_size = recipes.size();
	for (const auto& agents : agent_combinations) {
		size_t recipe_counter = 0;
		for (size_t i = 0; i < recipe_size; ++i) {
			const auto& recipe = recipes.at(i);

			if (!ingredients_reachable(recipe, agents, state) || state.items_hoarded(recipe, agents)) {
				continue;
			}


			auto time_start = std::chrono::system_clock::now();
			auto path = search.search_joint(state, recipe, agents, {});
			auto time_end = std::chrono::system_clock::now();

			auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();



			auto trim_path = path;
			if (!trim_path.empty()) {
				Search_Trimmer trim;
				trim.trim(trim_path, state, environment, recipe);


				paths.insert({ Action_Path{trim_path, recipe, agents, agent} });
			}

			std::string debug_string = "(";
			for (const auto& agent : agents.get()) {
				debug_string += std::to_string(agent.id) + ",";
			}
			PRINT(Print_Category::PLANNER, debug_string + ") : " + std::to_string(trim_path.size()) + " : " + static_cast<char>(recipe.result) +  " : " + std::to_string(diff) + '\n');

			//if (agents.size() > 1) {
			//	for (const auto& temp_agent : agents.get()) {
			//		time_start = std::chrono::system_clock::now();
			//		path = search.search_joint(state, recipe, agents, temp_agent);
			//		time_end = std::chrono::system_clock::now();
			//		diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();

			//		Action_Path a_path{ path, recipe, agents, agent };
			//		std::cout << agents.to_string() << "/" 
			//			<< temp_agent.to_string() << " : " 
			//			<< a_path.first_action_string() << "-" 
			//			<< a_path.last_action_string() << " : " 
			//			<< recipe.result_char() << " : " 
			//			<< diff << std::endl;
			//	}
			//}
		}
	}
	return paths;
}

void Planner::update_recipe_solutions(const std::set<Action_Path>& paths) {
	for (const auto& path : paths) {
		recipe_solutions.at(Recipe_Agents{ path.recipe, path.agents }).add(path.joint_actions.size(), time_step);
	}
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
	return true;
}


void Planner::initialize_reachables(const State& initial_state) {
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

void Planner::initialize_solutions() {
	// TODO - Should just maintain one agent_combinations for the class
	auto agent_combinations = get_combinations(environment.get_number_of_agents());
	for (const auto& agents : agent_combinations) {
		for (const auto& recipe : environment.get_all_recipes()) {
			recipe_solutions.insert({ {recipe, agents}, {} });
		}
	}
}

void Planner::recognize_goals() {
	std::vector<std::vector<float>> data_raw(time_step);
	std::vector<std::vector<float>> data_scaled;
	
	// Calculate absolute value for each recipe/timestep
	for (const auto& [recipe_Agents, history] : recipe_solutions) {
		PRINT(Print_Category::PLANNER, std::to_string(static_cast<char>(recipe_Agents.recipe.result)) + recipe_Agents.agents.to_string() + "\t");
		for (float i = 0; i < time_step; ++i) {
			if (history.get(time_step) == 0) {
				data_raw.at(i).push_back(0);
			} else {
				size_t time_diff = time_step - i;
				size_t current_solution = history.get(time_step);
				size_t previous_solution = history.get(i);
				//size_t result = (previous_solution - time_diff) / current_solution;
				//data_raw.at(i).push_back((history.get(time_step) - (time_step - i)) / history.get(i));
				data_raw.at(i).push_back(history.get(i) / (history.get(time_step) + (time_step - i)));
			}
		}
	}
	PRINT(Print_Category::PLANNER, "\n");


	// Normalize per timestep
	for (const auto& entry : data_raw) {
		float max_val = 1.0f;
		//for (const auto& length : entry) max_val = std::max(max_val, length);
		std::vector<float> scaled_row;
		for (const auto& raw_entry : entry) {
			scaled_row.push_back(round((raw_entry / max_val) * 1000) / 1000);
		}
		data_scaled.push_back(scaled_row);
	}

	// Debug print
 	for (const auto& data_row : data_scaled) {
		for (const auto& data_entry : data_row) {
			PRINT(Print_Category::PLANNER, std::to_string(data_entry) + '\t');
		}
		PRINT(Print_Category::PLANNER, "\n");
	}
}