#include "Planner.hpp"
#include "BFS.hpp"
#include "A_Star.hpp"
#include "Search.hpp"
#include "Search_Trimmer.hpp"
#include "Utils.hpp"
#include "Recogniser.hpp"
#include "Sliding_Recogniser.hpp"
#include "Bayesian_Recogniser.hpp"

#include <chrono>
#include <iostream>
#include <set>
#include <deque>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <iomanip>



constexpr auto INITIAL_DEPTH_LIMIT = 30;
constexpr auto GAMMA = 1.01;

Planner::Planner(Environment environment, Agent_Id agent, const State& initial_state)
	: agent(agent), environment(environment), time_step(0), 
		search(std::make_unique<A_Star>(environment, INITIAL_DEPTH_LIMIT)),
		recogniser(std::make_unique<Sliding_Recogniser>(environment, initial_state)) {

	initialize_reachables(initial_state);
	initialize_solutions();
}

Action Planner::get_next_action(const State& state) {
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) {
		return { Direction::NONE, { agent } };
	}
	auto paths = get_all_paths(recipes, state);
	update_recogniser(paths);
	auto goals = recogniser.get_goals();
	recogniser.print_probabilities();
	//auto collaboration = check_for_collaboration(paths);
	auto collaboration = check_for_collaboration(paths, recipes, goals);
	if (collaboration.has_value()) {
		++time_step;
		return collaboration.value().next_action;
	}

	auto action = get_best_action(paths, recipes);
	++time_step;
	return action;
}

std::optional<Collaboration_Info> Planner::check_for_collaboration(const Paths& paths, const std::vector<Recipe>& recipes_in, const std::map<Agent_Id, Goal>& goals) {
	size_t total_agents = environment.get_number_of_agents();
	std::vector<std::vector<std::vector<Recipe>>> all_recipe_combinations;
	size_t recipe_in_size = recipes_in.size();
	for (size_t i = 0; i < total_agents && i < recipe_in_size; ++i) {
		all_recipe_combinations.push_back(get_combinations(recipes_in, i+1));
	}

	std::vector<Collaboration_Info> infos;

	Collaboration_Info result;

	auto agent_combinations = get_combinations(total_agents);
	for (const auto& agents : agent_combinations) {
		//if (!agents.contains(agent)) continue;
		size_t agent_size = agents.size();
		auto agent_permutations = get_permutations(agents);

		// Iterate setups for agent_combination
		for (size_t recipe_combination_index = 0; recipe_combination_index < agent_size && recipe_combination_index < recipe_in_size; ++recipe_combination_index) {
			for (const auto& recipes : all_recipe_combinations.at(recipe_combination_index)) {
				if (agents.size() == 1) {
					Subtask_Entry entry{ recipes.at(0), agents };
					auto info = paths.get_normal(agent, entry);
					if (info.has_value()) {
						auto& info_val = info.value();
						result = { info_val->length, info_val->last_action, agents, recipes, info_val->action(agents.get().at(0)) };
						infos.push_back(result);
					}
				} else {

					size_t recipes_size = recipes.size();
					Agent_Combination best_permutation;
					size_t best_length = HIGH_INIT_VAL;

					// Get value for this setup
					for (const auto& agent_permutation : agent_permutations) {

						bool permutation_valid = true;
						size_t inner_length = 0;
						for (size_t handoff_agent_index = 0; handoff_agent_index < recipes_size; ++handoff_agent_index) {
							Subtask_Entry entry{ recipes.at(handoff_agent_index), agents };
							auto info = paths.get_handoff(agent_permutation.agents.at(handoff_agent_index), entry);
							if (info.has_value()) {
								//inner_length += info.value()->length;
								if (agent_permutation.size() == 2 && recipes_size == 2) {
									size_t other_agent_index = 1 - handoff_agent_index; // exploits |agents|==2
									Subtask_Entry entry_other{ recipes.at(other_agent_index), agents };
									auto other_info = paths.get_handoff(agent_permutation.agents.at(other_agent_index), entry_other);

									// If the other handoff is longer, this task will take longer to finish
									size_t handoff_diff = 0;
									if (other_info.value()->last_action == EMPTY_VAL) {
										// Believe it has to be -1 to account for val vs index
										handoff_diff = other_info.value()->length - 1 - info.value()->last_action;
									} else if (other_info.value()->last_action > info.value()->last_action) {
										handoff_diff = other_info.value()->last_action - info.value()->last_action;
									}

									inner_length = std::max(inner_length, info.value()->length + handoff_diff);
								} else {
									inner_length = std::max(inner_length, info.value()->length);
								}
							} else {
								permutation_valid = false;
								break;
							}
						}

						if (permutation_valid && inner_length < best_length) {
							best_length = inner_length;
							best_permutation = agent_permutation;
						}
					}


					//if (best_length != HIGH_INIT_VAL && (!result.has_value() || best_length < result.length)) {
					if (best_length != HIGH_INIT_VAL) {

						// Get next action for planning agent from best permutation
						auto agent_index = best_permutation.get_index(agent);
						Action action;
						size_t last_action;

						// Get next action from handoff
						if (agent_index < recipes_size) {
							Subtask_Entry entry{ recipes.at(agent_index), agents };
							auto info = paths.get_handoff(agent, entry);
							last_action = info.value()->last_action;

							action = info.value()->action(agent);

							// TODO - This should probably rather check if we are past
							//			handoff time, as a none action could still be useful
							
							// TODO - May have to introduce a special property to mark actions as useless
							//			since a none direction can still be useful
							if (action.is_none()) {
								std::set<size_t> skipped_recipes{ agent_index };
								Action action_inner;
								float highest_prob = 0.0f;
								for (size_t i = 0; i < recipes_size; ++i) {
									if (skipped_recipes.find(i) != skipped_recipes.end()) continue;
									Subtask_Entry entry_inner{ recipes.at(i), agents };
									auto info_inner = paths.get_handoff(i, entry_inner);
									Goal goal_inner{ Agent_Combination{agent}, recipes.at(i) };
									if (info_inner.has_value()
										&& info_inner.value()->action(agent).is_not_none()
										&& recogniser.get_probability(goal_inner) > highest_prob) {

										action_inner = info_inner.value()->action(agent);
										highest_prob = recogniser.get_probability(goal_inner);
									}

								}
								action = action_inner;
							}


							// Get next action from non-handoff
						} else {
							// TODO - Not 100% sure what the agent should do when #agents > #tasks
							// TODO - The 0 index assumes |tasks|==1 when |tasks|!=|agents|
							Subtask_Entry entry{ recipes.at(0), agents };
							auto info = paths.get_normal(agent, entry);
							action = info.value()->action(agent);
							last_action = info.value()->last_action;
						}
						auto sorted_agents = best_permutation.get();
						std::sort(sorted_agents.begin(), sorted_agents.end());
						result = { best_length, last_action, Agent_Combination{sorted_agents}, recipes, action };
						infos.push_back(result);
					}
				}
			}
		}
	}

	Collaboration_Info* best_info = nullptr;
	float lowest_val = HIGH_INIT_VAL;
	std::stringstream buffer1, buffer2;
	buffer2 << std::setprecision(3);

	std::map<Multi_Goal, float> goal_values;

	for (auto& entry : infos) {
		buffer1 << entry.to_string() << "\t";

		//for (auto& recipe : entry.recipes) {
		//	if (!recogniser.is_probable({ entry.combination, recipe })) {
		//		//__debugbreak;
		//		is_probable = false;
		//		break;
		//	}
		//}

		auto penalty = std::pow(GAMMA, entry.combination.size() + entry.recipes.size() - 2);
		entry.value = (penalty * entry.length) / entry.recipes.size();
		goal_values.insert({ Multi_Goal{entry.combination, entry.recipes}, entry.value });
	}

	std::vector<Collaboration_Info> probable_infos;
	probable_infos.reserve(infos.size());

	size_t max_tasks = 0;
	for (auto& info_entry : infos) {
		bool is_probable = true;
		bool is_coop_better = true;

		// TODO - Generalse to more recipes
		//if (entry.recipes.size() == 1 && entry.combination.size() > 1) {
		if (info_entry.combination.size() > 1) {

			// If collaborating on multiple tasks, check if all tasks could be done faster seperately
			if (info_entry.recipes.size() > 1) {
				bool task_faster_coop = false;
				for (auto& recipe_entry : info_entry.recipes) {
					auto it_seperate = goal_values.find(Multi_Goal({ info_entry.combination, {recipe_entry} }));
					assert(it_seperate != goal_values.end());
					if (it_seperate->second > info_entry.value) {
						task_faster_coop = true;
					}
				}
				is_coop_better &= task_faster_coop;
			}

			// TODO - should probably do this with all agent combinations of size |combination|-1
			auto combinations = get_combinations<Agent_Id>(info_entry.combination.get(), info_entry.combination.size() - 1);
			for (const auto& combination : combinations) {
				auto it = goal_values.find(Multi_Goal({ Agent_Combination{combination}, info_entry.recipes}));
				if (it != goal_values.end()) {
					if (it->second <= info_entry.value) {
						is_probable = false;
						break;
					}
				}
			}

			//// Check if a subset(currently single) of agents can solve the combination at least as fast
			//for (auto& recipe_entry : info_entry.recipes) {

			//	// TODO - should probably do this with all agent combinations of size |combination|-1
			//	for (const auto& agent : info_entry.combination.get()) {
			//		auto it = goal_values.find(Multi_Goal({ Agent_Combination{agent}, recipe_entry }));
			//		if (it != goal_values.end()) {
			//			if (it->second <= info_entry.value) {
			//				is_probable = false;
			//				break;
			//			}
			//		}
			//	}
			//}
		}
		is_probable = is_probable && is_coop_better;
		if (is_probable && is_coop_better) {
			probable_infos.push_back(info_entry);
			max_tasks = std::max(max_tasks, info_entry.recipes.size());
		}
		buffer2 << (is_probable ? "" : "X") << info_entry.value << "\t";
	}

	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer1.str() + '\n');
	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer2.str() + "\n\n");

	auto info = get_best_collaboration(probable_infos, max_tasks);


	std::stringstream buffer3;
	if (info.has_value()) {
		buffer3 << "Agent " << agent.id << " chose " << info.to_string() << " action " 
			<< info.next_action.to_string() << "\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return info;
	} else {
		buffer3 << "Agent " << agent.id << " did not find relevant action\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return {};
	}

	//for (auto& entry : infos) {
	//	bool is_probable = true;
	//	if (entry.recipes.size() == 1 && entry.combination.size() > 1) {
	//		auto recipe = entry.recipes.at(0);
	//		for (const auto& agent : entry.combination.get()) {
	//			auto it = goal_values.find(Goal({ Agent_Combination{agent}, recipe }));
	//			if (it != goal_values.end()) {
	//				if (it->second <= entry.value) {
	//					is_probable = false;
	//					break;
	//				}
	//			}
	//		}
	//	}
	//	if (!is_probable) {
	//		buffer2 << "NaN\t";
	//	} else {
	//		buffer2 << entry.value << "\t";
	//		if (entry.combination.contains(agent) && entry.value < lowest_val) {
	//			lowest_val = entry.value;
	//			best_info = &entry;
	//		}
	//	}
	//}
	//PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer1.str() + '\n');
	//PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer2.str() + "\n\n");


	//return (best_info == nullptr ? std::optional<Collaboration_Info>() : *best_info);
}

Collaboration_Info Planner::get_best_collaboration(const std::vector<Collaboration_Info>& infos, 
	const size_t& max_tasks) {

	// TODO - Should sort of infos which are not probable
	auto collection = get_best_collaboration_rec(infos, max_tasks, Colab_Collection{}, infos.begin());
	Collaboration_Info best_info;
	for (const auto& info : collection.infos) {
		if (info.combination.contains(agent) && info.value < best_info.value) {
			best_info = info;
		}
	}
	return best_info;
}

Colab_Collection Planner::get_best_collaboration_rec(const std::vector<Collaboration_Info>& infos, 
	const size_t& max_tasks, Colab_Collection collection_in, 
	std::vector<Collaboration_Info>::const_iterator it_in) {

	Colab_Collection best_collection;

	for (auto it = it_in; it != infos.end(); ++it) {
		if (collection_in.tasks + it->recipes.size() <= max_tasks 
			&& !collection_in.contains(it->recipes)
			//&& !collection_in.contains(it->combination)
			) {
			auto collection = collection_in;
			collection.add(*it);
			
			if (collection.tasks < max_tasks) {
				collection = get_best_collaboration_rec(infos, max_tasks, collection, it);
				if (!collection.has_value()) {
					continue;
				}
			} else {
				collection.calculate_value(environment.get_number_of_agents());
			}

			if (collection.value < best_collection.value) {
				best_collection = collection;
			}
		}
	}
	return best_collection;
}

Action Planner::get_best_action(const Paths& paths, const std::vector<Recipe>& recipes) const {
	
	std::map<Recipe, Agent_Usefulnes> agent_solutions;

	// Init best solutions
	std::map<Recipe, Recipe_Solution> best_solutions;
 	for (const auto& recipe : recipes) {
		Recipe_Solution solution { Agent_Combination{}, EMPTY_VAL };
		best_solutions.insert({ recipe, solution });
		agent_solutions.insert({ recipe, Agent_Usefulnes{} });
	}

	for (const auto& action_path : paths.get_normal()) {
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
		PRINT(Print_Category::PLANNER, Print_Level::VERBOSE, std::to_string(agent_solution.first.result_char()) + " : "
			+ std::to_string(agent_solution.second.incl_length) + "/"
			+ agent_solution.second.excl_length_str() + " : "
			+ agent_solution.second.get_usefulness_str() + '\n');
	}

	//for (const auto& agent_solution : agent_solutions) {
	//	const auto& agent_usefulness = agent_solution.second;
	//	if (agent_usefulness.is_useful()) {
	//		return agent_usefulness.action;
	//	}
	//	//if (((int)agent_solution.second.excl_agent - agent_solution.second.incl_agent) / 2 > )
	//}


	// Find best action
	bool done = false;
	for (const auto& action_path : paths.get_normal()) {

		if (!agent_in_best_solution(best_solutions, action_path)) {
			continue;
		}		

		if (action_path.contains_useful_action()) {
			auto& joint_action = action_path.joint_actions.at(0);
			
			std::stringstream buffer;
			buffer<< "Agent " << agent.id << " chose action " <<
				static_cast<char>(joint_action.actions.at(agent.id).direction) <<
				" for subtask " << static_cast<char>(action_path.recipe.result) << std::endl;
			PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer.str());
			return joint_action.actions.at(agent.id);
		}
	}
	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, std::string("Agent ") + std::to_string(agent.id) + " did not find useful action \n");
	return { Direction::NONE, { agent } };
}

bool Planner::agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, 
		const Action_Path& action_path) const {
	
	auto& best_agents = best_solutions.at(action_path.recipe).agents;
	return best_agents.contains(agent);
}

Paths Planner::get_all_paths(const std::vector<Recipe>& recipes, const State& state) {
	Paths paths(environment.get_number_of_agents());
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


				paths.insert(trim_path, recipe, agents, agent);
			}

			std::string debug_string = "(";
			for (const auto& agent : agents.get()) {
				debug_string += std::to_string(agent.id) + ",";
			}
			PRINT(Print_Category::PLANNER, Print_Level::DEBUG, debug_string + ") : " + std::to_string(trim_path.size()) 
				+ " : " + recipe.result_char() +  " : " + std::to_string(diff) + '\n');

			if (agents.size() > 1) {
				for (const auto& handoff_agent : agents.get()) {
					time_start = std::chrono::system_clock::now();
					path = search.search_joint(state, recipe, agents, handoff_agent);
					time_end = std::chrono::system_clock::now();
					diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();

					Action_Path a_path{ path, recipe, agents, agent };
					
					paths.insert(path, recipe, agents, agent, handoff_agent);
					
					std::stringstream buffer;
					buffer << agents.to_string() << "/"
						<< handoff_agent.to_string() << " : " 
						<< a_path.size() << " ("
						<< a_path.first_action_string() << "-" 
						<< a_path.last_action_string() << ") : " 
						<< recipe.result_char() << " : " 
						<< diff << std::endl;
					PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer.str());
				}
			}
		}
	}
	return paths;
}

void Planner::update_recogniser(const Paths& paths) {
	std::vector<Goal_Length> goal_lengths;
	for (const auto& path : paths.get_normal()) {
		goal_lengths.push_back(Goal_Length{ path.agents, path.recipe, path.joint_actions.size() });
	}
	recogniser.update(goal_lengths);
}

bool Planner::ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const {
	auto reachables = agent_reachables.find(agents);
	if (reachables == agent_reachables.end()) {
		std::cerr << "Unknown agent combination" << std::endl;
		exit(-1);
	}

	for (const auto& location : environment.get_coordinates(state, recipe.ingredient1)) {
		if (!reachables->second.get(location)) {
			return false;
		}
	}

	for (const auto& location : environment.get_coordinates(state, recipe.ingredient2)) {
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

