#include "Planner_Mac.hpp"
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

Planner_Mac::Planner_Mac(Environment environment, Agent_Id planning_agent, const State& initial_state)
	: Planner_Impl(environment, planning_agent), time_step(0), 
		search(std::make_unique<A_Star>(environment, INITIAL_DEPTH_LIMIT)),
		recogniser(std::make_unique<Sliding_Recogniser>(environment, initial_state)) {

	initialize_reachables(initial_state);
	initialize_solutions();
}

Action Planner_Mac::get_next_action(const State& state) {
	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, std::string("Time step: ") + std::to_string(time_step) + "\n");

	initialize_reachables(state);
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) {
		return { Direction::NONE, { planning_agent } };
	}
	auto paths = get_all_paths(recipes, state);
	update_recogniser(paths);
	auto goals = recogniser.get_goals();
	recogniser.print_probabilities();
	auto collaboration = check_for_collaboration(paths, recipes, goals, state);
	if (collaboration.has_value()) {
		++time_step;
		return collaboration.value().next_action;
	} 
	else {
		++time_step;
		return Action{ Direction::NONE, {planning_agent } };
	}
}

std::optional<Collaboration_Info> Planner_Mac::check_for_collaboration(const Paths& paths, const std::vector<Recipe>& recipes_in, 
	const std::map<Agent_Id, Goal>& goals, const State& state) {
	
	auto infos = calculate_infos(paths, recipes_in, state);
	auto goal_values = calculate_goal_values(infos);
	auto probable_infos = calculate_probable_multi_goals(infos, goal_values, state);
	size_t max_tasks = environment.get_number_of_agents();
	auto info = get_best_collaboration(probable_infos, max_tasks, state);


	std::stringstream buffer3;
	if (info.has_value()) {
		buffer3 << "Agent " << planning_agent.id << " chose " << info.to_string() << " action " 
			<< info.next_action.to_string() << "\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return info;
	} else {
		buffer3 << "Agent " << planning_agent.id << " did not find relevant action\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return {};
	}
}

std::vector<Collaboration_Info> Planner_Mac::calculate_infos(const Paths& paths, const std::vector<Recipe>& recipes_in,
	const State& state) {
	size_t total_agents = environment.get_number_of_agents();
	std::vector<Agent_Id> all_agents;

	// Precalculate all recipe combinations
	std::vector<std::vector<std::vector<Recipe>>> all_recipe_combinations;
	size_t recipe_in_size = recipes_in.size();
	for (size_t i = 0; i < total_agents && i < recipe_in_size; ++i) {
		all_recipe_combinations.push_back(get_combinations(recipes_in, i + 1));
	}

	// Precalculate agent permutations for all agent sizes
	std::vector<std::vector<std::vector<Agent_Id>>> agent_permutations;
	for (size_t i = 0; i < total_agents; ++i) {
		all_agents.push_back({ i });
	}
	for (size_t i = 0; i < total_agents; ++i) {
		agent_permutations.push_back(get_combinations_duplicates<Agent_Id>({ all_agents }, i + 1));
	}
	std::vector<Collaboration_Info> infos;

	auto agent_combinations = get_combinations(total_agents);
	for (const auto& agents : agent_combinations) {
		size_t agent_size = agents.size();

		// Iterate setups for agent_combination
		for (size_t recipe_combination_index = 0; recipe_combination_index < agent_size
			&& recipe_combination_index < recipe_in_size; ++recipe_combination_index) {

			for (const auto& recipes : all_recipe_combinations.at(recipe_combination_index)) {
				size_t recipes_size = recipes.size();
				if (agents.size() == 1) {
					Subtask_Entry entry{ recipes.at(0), agents };
					auto info = paths.get_normal(planning_agent, entry);
					if (info.has_value()) {
						auto& info_val = info.value();
						Collaboration_Info result = { info_val->length, agents, recipes,
							info_val->action(agents.get().at(0)), agents };

						infos.push_back(result);
					}
				} else {
					auto info = get_best_permutation(agents, recipes, paths, agent_permutations.at(recipes.size() - 1), state);

					if (info.has_value()) {
						infos.push_back(std::move(info.value()));
					}
				}
			}
		}
	}
	return infos;
}

std::vector<Collaboration_Info> Planner_Mac::calculate_probable_multi_goals(const std::vector<Collaboration_Info>& infos,
	const std::map<Multi_Goal, float>& goal_values, const State& state) {
	std::vector<bool> are_probable;
	std::stringstream buffer1;
	std::stringstream buffer2;
	buffer2 << std::setprecision(3);
	std::vector<Goal> normalisation_goals;


	for (auto& info_entry : infos) {
		bool is_probable = true;
		bool is_coop_better = true;
		buffer1 << info_entry.to_string() << "\t";

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
				is_probable &= task_faster_coop;
			}

			// TODO - should probably do this with all agent combinations of size |combination|-1
			auto combinations = get_combinations<Agent_Id>(info_entry.combination.get(), info_entry.combination.size() - 1);
			for (const auto& combination : combinations) {
				auto it = goal_values.find(Multi_Goal({ Agent_Combination{combination}, info_entry.recipes }));
				if (it != goal_values.end()) {
					if (it->second <= info_entry.value) {
						is_probable = false;
						break;
					}
				}
			}

			// If recipes in combination are mutually exclusive, then not probable
			std::map<Ingredient, size_t> ingredients;
			for (const auto& recipe : info_entry.recipes) {
				// Ingredient 1
				auto it = ingredients.find(recipe.ingredient1);
				if (it == ingredients.end()) {
					ingredients.insert({ recipe.ingredient1, 1 });
				} else {
					++(it->second);
				}

				// Ingredient 2
				it = ingredients.find(recipe.ingredient2);
				if (it == ingredients.end()) {
					ingredients.insert({ recipe.ingredient2, 1 });
				} else {
					++(it->second);
				}
			}
			for (const auto& [ingredient, count] : ingredients) {
				if (!environment.is_type_stationary(ingredient)
					&& state.get_count(ingredient) < count) {

					is_probable = false;
					break;
				}
			}
		}
		are_probable.push_back(is_probable);
		if (is_probable && info_entry.recipes.size() == 1) {
			for (const auto& recipe : info_entry.recipes) {
				normalisation_goals.push_back({ info_entry.combination, recipe });
			}
		}
	}

	// Check if recipes are probable normalised on available tasks
	for (size_t i = 0; i < infos.size(); ++i) {
		const auto& info_entry = infos.at(i);
		std::vector<bool>::reference is_probable = are_probable.at(i);
		
		// If recogniser sees all subtask allocations as probable
		if (is_probable
			&& (info_entry.combination.size() != 1 || info_entry.combination.get().at(0) != planning_agent)) {

			bool inner_probable = false;
			for (const auto& recipe : info_entry.recipes) {
				for (const auto& agent : info_entry.combination.get()) {
					bool use_non_probability = agent != planning_agent;
					if (recogniser.is_probable_normalised(Goal{ info_entry.combination, recipe }, normalisation_goals, agent, use_non_probability)) {
						inner_probable = true;
						break;
					}
				}
			}
				if (!inner_probable) {
					is_probable = false;
				}

		}
		buffer2 << (is_probable ? "" : "X") << info_entry.value << "\t";
	}

	// Copy probable infos
	std::vector<Collaboration_Info> result_infos;
	for (size_t i = 0; i < infos.size(); ++i) {
		if (are_probable.at(i)) {
			result_infos.push_back(infos.at(i));
		}
	}

	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer1.str() + '\n');
	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer2.str() + "\n\n");
	return result_infos;
}

std::map<Multi_Goal, float> Planner_Mac::calculate_goal_values(std::vector<Collaboration_Info>& infos) {
	std::map<Multi_Goal, float> goal_values;
	for (auto& entry : infos) {
		auto penalty = std::pow(GAMMA, entry.combination.size() - entry.recipes.size());
		entry.value = (penalty * entry.length) / entry.recipes.size();
		goal_values.insert({ Multi_Goal{entry.combination, entry.recipes}, entry.value });
	}
	return goal_values;
}

constexpr size_t action_trace_length = 3;
bool Planner_Mac::is_conflict_in_permutation(const State& initial_state, const std::vector<Joint_Action>& actions) {


	// Perform actions, check for invalid/collisions
	auto state = initial_state;
	for (const auto& action : actions) {
		
		// If all actions are none, there is no conflict by default
		bool is_all_none = true;
		for (const auto& single_action : action.actions) {
			if (single_action.is_not_none()) {
				is_all_none = false;
			}
		}
		if (is_all_none) {
			continue;
		}

		// Check if actions are successful
		if (!environment.act(state, action)) {
			return true;
		}
	}
	return false;
}


std::pair<std::vector<Joint_Action>, std::map<Recipe, Agent_Combination>> Planner_Mac::get_actions_from_permutation(
	const Agent_Combination& permutation, const std::vector<Action_Path>& action_paths,
	size_t agent_size, const State& state) {

	std::map<Recipe, Agent_Combination> agent_recipes;
	std::vector<Joint_Action> joint_actions(action_trace_length, agent_size);

	for (size_t agent = 0; agent < agent_size; ++agent) {
		auto [actions, recipe] = get_actions_from_permutation_inner(permutation, { agent }, action_paths, action_trace_length, state);
		auto it = agent_recipes.find(recipe);
		if (it == agent_recipes.end()) {
			agent_recipes.insert({ recipe, Agent_Combination(agent) });
		} else {
			it->second.add({ agent });
		}

		for (size_t action_index = 0; action_index < actions.size(); ++action_index) {
			joint_actions.at(action_index).update_action(agent, actions.at(action_index).direction);
		}
	}
	return { joint_actions, agent_recipes };
}

std::pair<std::vector<Action>, Recipe> Planner_Mac::get_actions_from_permutation_inner(const Agent_Combination& permutation,
	const Agent_Id& acting_agent, const std::vector<Action_Path>& action_paths, size_t action_trace_length,
	const State& state) {

	size_t backup_first_action = HIGH_INIT_VAL;
	const Action_Path* backup_path = nullptr;

	size_t recipes_size = action_paths.size();
	const Action_Path* result_path = nullptr;

	// Agent may have multiple tasks
	auto agent_indices = permutation.get_indices(acting_agent);

	// If responsible for handoff
	size_t best_handoff = EMPTY_VAL;
	size_t agent_index = EMPTY_VAL;
	for (const auto& index : agent_indices) {
		if (index < recipes_size) {
			auto& path = action_paths.at(index);
			if (path.last_action < best_handoff
				&& path.has_useful_action(acting_agent, state, environment)) {

				best_handoff = path.last_action;
				result_path = &path;
			}
		}
	}

	// TODO - Should maybe use the lowest value entry with acceptably high probability
	// If no useful handoff action
	if (result_path == nullptr) {
		float highest_prob = 0.0f;
		for (const auto& action_path : action_paths) {
			auto probability = recogniser.get_probability(Goal(action_path.agents, action_path.recipe));

			if (action_path.has_useful_action(acting_agent, state, environment)
				&& probability > highest_prob) {

				result_path = &action_path;
				highest_prob = probability;
			}

			auto index = action_path.get_first_non_trivial_index(acting_agent);
			if (index != EMPTY_VAL && index < backup_first_action) {
				backup_first_action = index;
				backup_path = &action_path;
			}
		}
	}
	
	if (result_path == nullptr && backup_path != nullptr) {
		result_path = backup_path;
	}

	// Extract actions if found
	if (result_path != nullptr) {
		std::vector<Action> result;
		size_t action_index = 0;
		for (const auto& joint_action : result_path->joint_actions) {
			if (action_index == action_trace_length) break;
			result.push_back(joint_action.get_action(acting_agent));
			++action_index;
		}

		// Fill rest with none actions
		while (action_index < action_trace_length) {
			result.emplace_back( Direction::NONE, acting_agent );
			++action_index;
		}

		return { result, result_path->recipe };
	} else {
		return { {}, EMPTY_RECIPE };
	}

}


struct Permutation_Info {
	//const Subtask_Info* info;
	mutable size_t length;
	mutable size_t handoff_time;
	mutable size_t handoff_agent_index;

	std::vector<Action_Path*> action_paths;

	bool operator<(const Permutation_Info& other) const {
		if (handoff_time != other.handoff_time) return handoff_time < other.handoff_time;
		return handoff_agent_index < other.handoff_agent_index;
	}
};

std::vector<std::set<Temp>> Planner_Mac::get_agent_handoff_infos(const Agent_Combination& agents,
	const std::vector<Agent_Id>& agent_permutation, const std::vector<Action_Path>& action_paths) {

	size_t agent_size = agents.size();
	size_t recipes_size = action_paths.size();

	std::vector<std::set<Temp>> agents_handoffs(agent_size, std::set<Temp>());
	for (size_t handoff_agent_index = 0; handoff_agent_index < recipes_size; ++handoff_agent_index) {
		const auto& path = action_paths.at(handoff_agent_index);
		auto actual_agent = agent_permutation.at(handoff_agent_index);
		agents_handoffs.at(actual_agent.id).insert(
			Temp{ path, handoff_agent_index });
	}
	return agents_handoffs;
}

// Record adjusted handoffs for each agent, and max handoff
// Handoff for agent x entry y is sum(0, ..., y), since an agent
// can only do one subtask at a time
std::vector<size_t> Planner_Mac::calculate_adjusted_agent_handoffs(std::vector<std::set<Temp>>& agents_handoffs) {
	size_t agent_size = environment.get_number_of_agents();

	std::vector<size_t> longest_agent_handoffs(agent_size, 0);
	size_t agent_counter = 0;
	for (auto& agent_handoffs : agents_handoffs) {
		size_t running_handoff_time = 0;
		for (auto& temp : agent_handoffs) {
			if (temp.handoff_time != EMPTY_VAL) {
				temp.length += running_handoff_time;
				temp.handoff_time += running_handoff_time;
				running_handoff_time = temp.handoff_time;

				size_t current_handoff = temp.handoff_time;

				auto& handoff_ref = longest_agent_handoffs.at(agent_counter);
				handoff_ref = std::max(handoff_ref, current_handoff);
			}
		}
		++agent_counter;
	}
	return longest_agent_handoffs;
}

size_t get_expected_length(const std::vector<std::set<Temp>>& agents_handoffs, 
	const std::vector<size_t>& longest_agent_handoffs) {

	size_t length = 0;
	// Record expected lengths with adjusted handoffs
	size_t agent_counter = 0;
	for (auto& agent_handoffs : agents_handoffs) {

		// Find max handoff by other agents
		size_t min_handoff = HIGH_INIT_VAL;
		for (size_t i = 0; i < longest_agent_handoffs.size(); ++i) {
			if (i == agent_counter) {
				continue;
			}
			min_handoff = std::min(min_handoff, longest_agent_handoffs.at(i));
		}

		// Iterate current agent handoffs
		for (auto& temp : agent_handoffs) {
			size_t handoff_penalty = 0;
			if (min_handoff != HIGH_INIT_VAL && temp.handoff_time < min_handoff) {
				handoff_penalty = min_handoff - temp.handoff_time;
			}
			size_t length_inner = temp.length + handoff_penalty;
			length = std::max(length, length_inner);
		}
		++agent_counter;
	}
	return length;
}

bool Planner_Mac::is_agent_abused(const std::vector<Agent_Id>& agent_permutation, const std::vector<Action_Path>& action_paths) const {
	if (agent_permutation.size() == 1) {
		return false;
	}

	bool first = true;
	Agent_Id agent{ EMPTY_VAL };
	for (size_t i = 0; i < agent_permutation.size(); ++i) {
		if (first) {
			first = false;
			agent = agent_permutation.at(i);
		}
		if (agent_permutation.at(i) != agent) {
			return false;
		}
		if (action_paths.at(i).last_action != EMPTY_VAL) {
			return false;
		}
	}
	return true;
}

size_t Planner_Mac::get_permutation_length(const Agent_Combination& agents,
	const std::vector<Agent_Id>& agent_permutation, const std::vector<Action_Path>& action_paths){

	if (is_agent_abused(agent_permutation, action_paths)) {
		return HIGH_INIT_VAL;
	}

	auto agents_handoffs = get_agent_handoff_infos(agents, agent_permutation, action_paths);
	auto longest_agent_handoffs = calculate_adjusted_agent_handoffs(agents_handoffs);
	return get_expected_length(agents_handoffs, longest_agent_handoffs);

}

struct Temp_Info {

	Temp_Info(const std::vector<Action_Path>& action_paths,
		const Agent_Combination& permutation, size_t length, bool planning_agent_has_action)
		: action_paths(action_paths), permutation(permutation), length(length),
		planning_agent_has_action(planning_agent_has_action){};

	std::vector<Action_Path> action_paths;
	Agent_Combination permutation;
	size_t length;
	bool planning_agent_has_action;

	bool operator<(const Temp_Info& other) const {
		if (length != other.length) return length < other.length;
		if (planning_agent_has_action != other.planning_agent_has_action) return planning_agent_has_action;
		return permutation < other.permutation;
	}
};

std::optional<std::vector<Action_Path>> Planner_Mac::get_permutation_action_paths(const Agent_Combination& agents,
	const std::vector<Recipe>& recipes, const Paths& paths, const std::vector<Agent_Id>& agent_permutation) const {

	size_t recipes_size = recipes.size();
	std::vector<Action_Path> action_paths;
	for (size_t handoff_agent_index = 0; handoff_agent_index < recipes_size; ++handoff_agent_index) {

		Subtask_Entry entry{ recipes.at(handoff_agent_index), agents };
		auto info_opt = paths.get_handoff(agent_permutation.at(handoff_agent_index), entry);
		if (info_opt.has_value()) {
			action_paths.push_back(*info_opt.value()->actions);
		} else {
			return {};
		}
	}
	return action_paths;
}

// Calculates expected length of completion for each permutation
// Attempts new search for each recipe if conflict in best permutation
std::optional<Collaboration_Info> Planner_Mac::get_best_permutation(const Agent_Combination& agents,
	const std::vector<Recipe>& recipes, const Paths& paths, const std::vector<std::vector<Agent_Id>>& agent_permutations,
	const State& state) {

	size_t recipes_size = recipes.size();
	size_t agent_size = agents.size();

	std::set<Temp_Info> permutation_infos;
	std::map<Agent_Combination, std::vector<Action_Path>> permutation_paths;

	// Get info on each permutation
	for (const auto& agent_permutation : agent_permutations) {

		auto action_paths = get_permutation_action_paths(agents, recipes, paths, agent_permutation);
		if (action_paths.has_value()) {
			size_t length = get_permutation_length(agents, agent_permutation, action_paths.value());
			bool planning_agent_has_action = false;

			if (agents.contains(planning_agent)) {
				const auto& next_action_path = action_paths.value().at(0);
				planning_agent_has_action = next_action_path.get_next_action(planning_agent).is_not_none();
			}

			permutation_infos.emplace(action_paths.value(), Agent_Combination(agent_permutation), length, planning_agent_has_action);
			permutation_paths.emplace( Agent_Combination{agent_permutation}, std::move(action_paths.value()) );
		}
	}

	// Best collision free
	size_t best_length = HIGH_INIT_VAL;
	std::vector<Collaboration_Info> best_infos;

	while (!permutation_infos.empty()) {

		// Get and erase shortest length permutation 
		auto permutation = permutation_infos.begin()->permutation;
		auto length = permutation_infos.begin()->length;
		permutation_infos.erase(permutation_infos.begin());

		// If collision-free version of earlier entries is faster
		if (best_length < length) {
			return get_random<Collaboration_Info>(best_infos);
		}

		// Get conflict info
		const auto& action_paths = permutation_paths.at(permutation);
		auto [joint_actions, agent_recipes] = get_actions_from_permutation(permutation, action_paths, agent_size, state);

		if (is_conflict_in_permutation(state, joint_actions)) {

			// Perform collision avoidance search
			for (size_t handoff_index = 0; handoff_index < recipes.size(); ++handoff_index) {
				//auto& acting_agent = permutation.get().at(handoff_index);
				auto handoff_agent = permutation.get(handoff_index);
				auto& recipe = recipes.at(handoff_index);
				auto it = agent_recipes.find(recipe);
				if (it == agent_recipes.end()) {
					continue;
				}

				// Erase ending non-actions (handoff agent excluded)
				auto action_it = joint_actions.end();
				--action_it;
				while (true) {
					bool valid_action = false;
					for (size_t j = 0; j < (*action_it).size(); ++j) {
						if (j == handoff_agent.id) {
							continue;
						}
						if (action_it->get_action(j).is_not_none()) {
							valid_action = true;
							break;
						}
					}
					if (valid_action) {
						break;
					} else {
						action_it = joint_actions.erase(action_it);
						if (joint_actions.empty()) {
							break;
						} else {
							--action_it;
						}
					}
				}

				// Perform new search for one recipe
				auto new_path = search.search_joint(state, recipe, agents, permutation.get(handoff_index), joint_actions, it->second);
				if (new_path.empty()) {
					continue;
				}
				auto new_paths = paths;
				new_paths.update(new_path, recipe, agents, planning_agent, handoff_agent, state, environment);

				// Get info using the new search
				auto action_paths = get_permutation_action_paths(agents, recipes, new_paths, permutation.get());
				if (!action_paths.has_value()) {
					continue;
				}
				size_t new_length = get_permutation_length(agents, permutation.get(), action_paths.value());

				// Check if best collision avoidance search so far
				if (new_length <= best_length) {
					auto [joint_actions, agent_recipes] = get_actions_from_permutation(permutation, action_paths.value(), agent_size, state);

					if (!is_conflict_in_permutation(state, joint_actions)) {
						Action planning_agent_action = joint_actions.at(0).get_action(planning_agent);

						if (new_length < best_length) {
							best_infos.clear();
							best_length = new_length;
						}

						best_infos.push_back(Collaboration_Info(new_length, agents, recipes, planning_agent_action, permutation));
					}
				}
			}

		// Return unmodified entry
		} else {
			Action planning_agent_action = joint_actions.at(0).get_action(planning_agent);
			if (length < best_length) {
				best_infos.clear();
				best_length = length;
			}
			best_infos.push_back(Collaboration_Info(length, agents, recipes, planning_agent_action, permutation));
		}
	}
	return !best_infos.empty() ? get_random<Collaboration_Info>(best_infos) : Collaboration_Info{};
}

Collaboration_Info Planner_Mac::get_best_collaboration(const std::vector<Collaboration_Info>& infos, 
	const size_t& max_tasks, const State& state) {

	size_t max_agents = environment.get_number_of_agents();

	// TODO - Should sort of infos which are not probable
	auto collection = get_best_collaboration_rec(infos, max_tasks, max_agents, Colab_Collection{}, 
		infos.begin(), state.get_ingredients_count());
	Collaboration_Info best_info;
	for (const auto& info : collection.infos) {
		if (info.combination.contains(planning_agent) && info.value < best_info.value) {
			best_info = info;
		}
	}
	return best_info;
}

Colab_Collection Planner_Mac::get_best_collaboration_rec(const std::vector<Collaboration_Info>& infos, 
	const size_t& max_tasks, const size_t& max_agents, Colab_Collection collection_in,
	std::vector<Collaboration_Info>::const_iterator it_in,
	const std::map<Ingredient, size_t>& available_ingredients) {

	Colab_Collection best_collection;
	bool found_valid_extension = false;

	for (auto it = infos.begin(); it != infos.end(); ++it) {
		if (collection_in.tasks + it->recipes.size() <= max_tasks 
			&& !collection_in.contains(it->recipes)
			&& collection_in.is_compatible(it->recipes, available_ingredients, environment)) {
			auto collection = collection_in;
			collection.add(*it);
			collection.calculate_value(environment.get_number_of_agents());
			found_valid_extension = true;

			// A different permutation of this collection will already have been processed
			if (it < it_in) {
				continue;
			}

			if (collection.tasks < max_tasks 
				&& collection.agents.size() < max_agents) {
					collection = get_best_collaboration_rec(infos, max_tasks, max_agents,
						collection, it, available_ingredients);

					if (!collection.has_value()) {
						continue;
					}
			} 

			if (collection.value < best_collection.value) {
				best_collection = collection;
			}
		}
	}
	return found_valid_extension ? best_collection : collection_in;
}

Paths Planner_Mac::get_all_paths(const std::vector<Recipe>& recipes, const State& state) {
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
			auto path = search.search_joint(state, recipe, agents, {}, {}, {});
			auto time_end = std::chrono::system_clock::now();

			auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();



			auto trim_path = path;
			if (!trim_path.empty()) {
				Search_Trimmer trim;
				trim.trim(trim_path, state, environment, recipe);


				paths.insert(trim_path, recipe, agents, planning_agent, state, environment);
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
					path = search.search_joint(state, recipe, agents, handoff_agent, {}, {});
					time_end = std::chrono::system_clock::now();
					diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();

						Coordinate agent_coordinate = state.get_agent(planning_agent).coordinate;
						Action_Path a_path{ path, recipe, agents, planning_agent, agent_coordinate, environment };


						std::stringstream buffer;
						buffer << agents.to_string() << "/"
							<< handoff_agent.to_string() << " : "
							<< a_path.size() << " ("
							<< a_path.first_action_string() << "-"
							<< a_path.last_action_string() << ") : "
							<< recipe.result_char() << " : "
							<< diff << std::endl;
						PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer.str());

					if (!path.empty()) {
						paths.insert(path, recipe, agents, planning_agent, handoff_agent, state, environment);
					}
				}
			}
		}
	}
	return paths;
}

void Planner_Mac::update_recogniser(const Paths& paths) {
	std::vector<Goal_Length> goal_lengths;
	for (const auto& path : paths.get_normal()) {
		goal_lengths.push_back(Goal_Length{ path.agents, path.recipe, path.joint_actions.size() });
	}
	recogniser.update(goal_lengths);
}

bool Planner_Mac::ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const {
	auto reachables = agent_reachables.find(agents);
	if (reachables == agent_reachables.end()) {
		std::cerr << "Unknown agent combination" << std::endl;
		exit(-1);
	}
	bool skip1 = false, skip2 = false;
	for (const auto& agent_entry : agents.get()) {
		const auto& agent = state.get_agent(agent_entry);
		if (agent.item.has_value()) {
			if (agent.item.value() == recipe.ingredient1) {
				skip1 = true;
			}
			if (agent.item.value() == recipe.ingredient2) {
				skip2 = true;
			}
		}
	}
	
	if (!skip1) {
		for (const auto& location : environment.get_coordinates(state, recipe.ingredient1)) {
			if (!reachables->second.get(location)) {
				return false;
			}
		}
	}

	if (!skip2) {
		for (const auto& location : environment.get_coordinates(state, recipe.ingredient2)) {
			if (!reachables->second.get(location)) {
				return false;
			}
		}
	}
	return true;
}


void Planner_Mac::initialize_reachables(const State& state) {
	agent_reachables.clear();
	auto combinations = get_combinations(state.agents.size());
	for (const auto& agents : combinations) {
		Reachables reachables(environment.get_width(), environment.get_height());

		// Initial agent locations
		std::deque<Coordinate> frontier;
		for (const auto& agent : agents.get()) {
			auto location = state.get_location(agent);
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
					auto blocking_agent = state.get_agent(location);

					if (!environment.is_cell_type(location, Cell_Type::WALL)
						&& (!blocking_agent.has_value() 
							|| agents.contains(blocking_agent.value()))) {
						frontier.push_back(location);
					}
				}
			}
		}
		agent_reachables.insert({ agents, reachables });
	}
}

void Planner_Mac::initialize_solutions() {
	// TODO - Should just maintain one agent_combinations for the class
	auto agent_combinations = get_combinations(environment.get_number_of_agents());
	for (const auto& agents : agent_combinations) {
		for (const auto& recipe : environment.get_all_recipes()) {
			recipe_solutions.insert({ {recipe, agents}, {} });
		}
	}
}

