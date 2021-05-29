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
constexpr auto GAMMA2 = 1.05;

Planner_Mac::Planner_Mac(Environment environment, Agent_Id planning_agent, const State& initial_state)
	: Planner_Impl(environment, planning_agent), time_step(0), 
		search(std::make_unique<A_Star>(environment, INITIAL_DEPTH_LIMIT)),
		recogniser(std::make_unique<Sliding_Recogniser>(environment, initial_state)) {

	initialize_reachables(initial_state);
	initialize_solutions();
}

Action Planner_Mac::get_next_action(const State& state, bool print_state) {

	if (print_state) environment.print_state(state);
	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, std::string("Time step: ") + std::to_string(time_step) + "\n");

	initialize_reachables(state);
	auto recipes = environment.get_possible_recipes(state);
	if (recipes.empty()) {
		return { Direction::NONE, { planning_agent } };
	}
	auto paths = get_all_paths(recipes, state);
	update_recogniser(paths, state);
	recogniser.print_probabilities();

	auto infos = calculate_infos(paths, recipes, state);
	if (infos.empty()) {
		return Action{ Direction::NONE, {planning_agent } };
	}

	auto goal_values = calculate_goal_values(infos);
	auto probable_infos = calculate_probable_multi_goals(infos, goal_values, state);
	size_t max_tasks = environment.get_number_of_agents();
	auto info = get_best_collaboration(probable_infos, max_tasks, state, true);
	if (!info.has_value()) {
		info = get_best_collaboration(infos, max_tasks, state, false);
	}

	std::stringstream buffer3;
	++time_step;
	if (info.has_value()) {
		buffer3 << "Agent " << planning_agent.id << " chose " << info.to_string() << " action "
			<< info.next_action.to_string() << "\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return info.next_action;
	} else {
		buffer3 << "Agent " << planning_agent.id << " did not find relevant action\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return Action{ Direction::NONE, {planning_agent } };
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
					assert(recipes_size == 1);
					Agent_Id handoff_agent{ EMPTY_VAL };
					Goal goal { agents, recipes.at(0), handoff_agent};
					auto info = paths.get_handoff(goal);
					if (info.has_value()) {
						auto& info_val = info.value();
						Collaboration_Info result { info_val->length(), goal,
							info_val->get_next_action(planning_agent) };

						infos.push_back(result);
					}
				} else {
					Goals goals;
					for (const auto& recipe : recipes) {
						goals.add({ agents, recipe, EMPTY_VAL });
					}

					auto temp_infos = get_collaboration_permutations(goals, paths, agent_permutations.at(recipes.size() - 1), state);
					infos.insert(std::end(infos), std::begin(temp_infos), std::end(temp_infos));
				}
			}
		}
	}
	return infos;
}

std::vector<Collaboration_Info> Planner_Mac::calculate_probable_multi_goals(const std::vector<Collaboration_Info>& infos,
	const std::map<Goals, float>& goal_values, const State& state) {
	std::vector<bool> are_probable;
	std::stringstream buffer1;
	std::stringstream buffer2;
	buffer2 << std::setprecision(3);
	std::vector<Goal> normalisation_goals;
	Ingredients state_ingredients = state.get_ingredients_count();


	for (auto& info_entry : infos) {
		bool is_probable = true;
		bool is_coop_better = true;
		buffer1 << info_entry.to_string() << "\t";

		if (info_entry.agents_size() > 1) {

			// If collaborating on multiple tasks, check if all tasks could be done faster seperately
			if (info_entry.goals_size() > 1) {
				bool task_faster_coop = false;
				for (const auto& goal : info_entry.get_goals_iterable()) {


					Goals goals_reduced_agents(goal);
					goals_reduced_agents.clear_all_handoff_indices();

					auto it_seperate = goal_values.find(goals_reduced_agents);
					//assert(it_seperate != goal_values.end());
					if (it_seperate->second > info_entry.value) {
						task_faster_coop = true;
					}
				}
				is_probable &= task_faster_coop;
			}
			
			if (is_agent_subset_faster(info_entry, goal_values)) {
				is_probable = false;
			}

			// If recipes in combination are mutually exclusive, then not probable
			Ingredients needed_ingredients;
			for (const auto& goal : info_entry.get_goals_iterable()) {
				needed_ingredients.add_ingredients(goal.recipe, environment);
			}
			if (!(needed_ingredients <= state_ingredients)) {
				is_probable = false;
			}
		}
		are_probable.push_back(is_probable);
		if (is_probable && info_entry.goals_size() == 1) {
			for (const auto& goal : info_entry.get_goals_iterable()) {
				normalisation_goals.push_back(goal);
			}
		}
	}

	// Check if recipes are probable normalised on available tasks
	for (size_t i = 0; i < infos.size(); ++i) {
		const auto& info_entry = infos.at(i);
		std::vector<bool>::reference is_probable = are_probable.at(i);
		
		// If recogniser sees all subtask allocations as probable
		if (is_probable && !info_entry.is_only_agent(planning_agent)) {

			bool inner_probable = false;
			for (const auto& goal : info_entry.get_goals_iterable()) {
				for (const auto& agent : goal.agents) {
					//bool use_non_probability = agent != planning_agent;
					bool use_non_probability = (agent != planning_agent || info_entry.goals_size() > 1);
					if (recogniser.is_probable_normalised(goal, normalisation_goals, agent, planning_agent, use_non_probability)) {
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

bool Planner_Mac::is_agent_subset_faster(const Collaboration_Info& info, const std::map<Goals, float>& goal_values) {
	auto combinations = get_combinations<Agent_Id>(info.get_agents().get(), info.agents_size() - 1);
	for (const auto& combination : combinations) {
		Goals goals_reduced_agents(info.get_goals(), Agent_Combination{ combination });
		goals_reduced_agents.clear_all_handoff_indices();
		auto it = goal_values.find(goals_reduced_agents);
		if (it != goal_values.end()) {
			if (it->second <= info.value) {
				return true;
			}
		}
	}
	return false;
}

std::map<Goals, float> Planner_Mac::calculate_goal_values(std::vector<Collaboration_Info>& infos) {
	std::map<Goals, float> goal_values;
	for (auto& entry : infos) {
		auto penalty = std::pow(GAMMA, entry.agents_size() - entry.goals_size());
		//auto penalty = std::pow(GAMMA, entry.agents_size()) / std::pow(GAMMA2, entry.goals_size());
		entry.value = (penalty * entry.length) / entry.goals_size();

		auto goals = entry.get_goals();
		goals.clear_all_handoff_indices();
		auto it = goal_values.find(goals);
		if (it != goal_values.end()) {
			it->second = std::min(it->second, entry.value);
		} else {
			goal_values.insert({ goals, entry.value });
		}
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
	const Goals& goals, const Paths& paths, const State& state) {

	std::map<Recipe, Agent_Combination> agent_recipes;
	std::vector<Joint_Action> joint_actions(action_trace_length, goals.get_agents().size());

	for (const auto& agent : goals.get_agents()) {
		auto [actions, recipe] = get_actions_from_permutation_inner(goals, agent, paths, state);
		agent_recipes[recipe].add(agent);

		for (size_t action_index = 0; action_index < actions.size(); ++action_index) {
			joint_actions.at(action_index).update_action(agent, actions.at(action_index).direction);
		}
	}
	return { joint_actions, agent_recipes };
}

std::pair<std::vector<Action>, Recipe> Planner_Mac::get_actions_from_permutation_inner(const Goals& goals,
	const Agent_Id& acting_agent, const Paths& paths, const State& state) {

	size_t backup_first_action = HIGH_INIT_VAL;
	const Action_Path* backup_path = nullptr;

	//size_t recipes_size = action_paths.size();
	const Action_Path* result_path = nullptr;

	size_t best_handoff = EMPTY_VAL;
	size_t agent_index = EMPTY_VAL;
	for (const auto& goal : goals) {
		if (goal.handoff_agent == acting_agent) {
			auto path_opt = paths.get_handoff(goal);
			auto path = path_opt.value();
			if (path->last_action < best_handoff
				&& path->has_useful_action(acting_agent, state, environment)) {

				best_handoff = path->last_action;
				result_path = path;
			}
		}
	}

	// TODO - Should maybe use the lowest value entry with acceptably high probability
	// If no useful handoff action
	if (result_path == nullptr) {
		float highest_prob = 0.0f;
		for (const auto& goal : goals) {
			auto probability = recogniser.get_probability(goal);
			auto path_opt = paths.get_handoff(goal);
			auto path = path_opt.value();

			if (path->has_useful_action(acting_agent, state, environment)
				&& probability > highest_prob) {

				result_path = path;
				highest_prob = probability;
			}

			auto index = path->get_first_non_trivial_index(acting_agent);
			if (index != EMPTY_VAL && index < backup_first_action) {
				backup_first_action = index;
				backup_path = path;
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

bool Planner_Mac::is_agent_abused(const Goals& goals, const Paths& paths) const {
	if (goals.get_agents().size() == 1 || goals.size() == 1) {
		return false;
	}

	bool first = true;
	Agent_Id agent{ EMPTY_VAL };
	for (const auto& goal : goals.get_iterable()) {
		if (first) {
			first = false;
			agent = goal.handoff_agent;
		}
		if (goal.handoff_agent != agent) {
			return false;
		}

		// Trying out disallowing any collection with the same handoff on all tasks
		//auto path = paths.get_handoff(goal);
		//if (path.has_value() && path.value()->last_action != EMPTY_VAL) {
		//	return false;
		//}
	}
	return true;
}

struct Temp2 {
	Agent_Id agent;			// Agent
	size_t total_length;	// From start(plus penalty if multiple handoffs from same agent) to finish
	size_t extra_length;	// From handoff to finish
	size_t id;
	bool operator<(const Temp2& other) const {
		if (total_length != other.total_length) total_length < other.total_length;
		return id < other.id;
	}
};

struct Temp3 {
	size_t length;
	size_t handoff_time;
	Agent_Id agent;
	size_t id;

	bool operator<(const Temp3& other) const {
		if (handoff_time != other.handoff_time) handoff_time < other.handoff_time;
		return id < other.id;
	}
};

size_t Planner_Mac::get_permutation_length(const Goals& goals, const Paths& paths){

	if (is_agent_abused(goals, paths)) {
		return EMPTY_VAL;
	}

	// Sort tasks by handoff time
	std::set<Temp3> sorted;
	for (const auto& goal : goals) {
		auto path_opt = paths.get_handoff(goal);
		if (!path_opt.has_value()) {
			return EMPTY_VAL;
		}
		auto path = path_opt.value();
		sorted.insert({ path->size(), path->last_action, goal.handoff_agent, sorted.size() });
	}

	// Calculate adjusted handoff time per agent and task
	size_t agent_size = environment.get_number_of_agents();
	std::vector<size_t> handoffs(agent_size, 0);
	std::set<Temp2> tasks;
	for (const auto& entry : sorted) {
		size_t extra_length = entry.length;
		if (entry.handoff_time != EMPTY_VAL) {
			// Note the +1's are from index to length conversion
			handoffs.at(entry.agent.id) += entry.handoff_time + 1;
			extra_length -= (entry.handoff_time + 1);
		}
		tasks.insert({ entry.agent, entry.length, extra_length, tasks.size() });
	}

	// Distribute task completion time across agents
	for (const auto& task : tasks) {

		// Get different agent with lowest handoff time
		size_t lowest_handoff = HIGH_INIT_VAL;
		size_t lowest_index = EMPTY_VAL;
		for (size_t i = 0; i < handoffs.size(); ++i) {
			if (i == task.agent.id) {
				continue;
			}
			if (handoffs.at(i) < lowest_handoff) {
				lowest_handoff = handoffs.at(i);
				lowest_index = i;
			}
		}

		auto& handoff_ref = handoffs.at(lowest_index);
		handoff_ref = std::max(handoff_ref + task.extra_length, task.total_length);
	}

	// Find max completion time
	size_t max_length = 0;
	for (const auto& handoff : handoffs) {
		max_length = std::max(max_length, handoff);
	}
	return max_length == 0 ? EMPTY_VAL : max_length;


	//auto agents_handoffs = get_agent_handoff_infos(agents, agent_permutation, action_paths);
	//auto longest_agent_handoffs = calculate_adjusted_agent_handoffs(agents_handoffs);
	//return get_expected_length(agents_handoffs, longest_agent_handoffs);

}

//struct Temp_Info {
//
//	Temp_Info(const std::vector<Action_Path>& action_paths,
//		const Agent_Combination& permutation, size_t length)
//		: action_paths(action_paths), permutation(permutation), length(length) {};
//
//	std::vector<Action_Path> action_paths;
//	Agent_Combination permutation;
//	size_t length;
//
//	bool operator<(const Temp_Info& other) const {
//		if (length != other.length) return length < other.length;
//		return permutation < other.permutation;
//	}
//};

struct Temp_Info {

	Temp_Info(const Goals& goals, size_t length)
		: goals(goals), length(length) {};

	Goals goals;
	size_t length;

	bool operator<(const Temp_Info& other) const {
		if (length != other.length) return length < other.length;
		return goals < other.goals;
	}
};

// TODO - Would be best to return pointers, but paths is a local variable in one of the calls
std::optional<std::vector<Action_Path>> Planner_Mac::get_permutation_action_paths(const Goals& goals,
	const Paths& paths) const {

	size_t goals_size = goals.size();
	std::vector<Action_Path> action_paths;
	for (const auto& goal : goals.get_iterable()) {
		auto info_opt = paths.get_handoff(goal);
		if (info_opt.has_value()) {
			action_paths.push_back(*info_opt.value());
		} else {
			return {};
		}
	}
	return action_paths;
}

std::vector<Collaboration_Info> Planner_Mac::get_collaboration_permutations(const Goals& goals_in,
	const Paths& paths, const std::vector<std::vector<Agent_Id>>& agent_permutations,
	const State& state) {

	std::vector<Collaboration_Info> infos;

	// Get info on each permutation
	for (const auto& agent_permutation : agent_permutations) {

		auto goals = goals_in;
		goals.update_handoffs(agent_permutation);

		size_t length = get_permutation_length(goals, paths);
		if (length == EMPTY_VAL) {
			continue;
		}

		// Get conflict info
		auto [original_joint_actions, agent_recipes] = get_actions_from_permutation(goals, paths, state);

		if (is_conflict_in_permutation(state, original_joint_actions)) {

			// Perform collision avoidance search
			size_t best_length = HIGH_INIT_VAL;
			Collaboration_Info best_collaboration;
			for (const auto& goal : goals) {

				// Skip if no agent chose to act on this recipe
				if (agent_recipes.find(goal.recipe) == agent_recipes.end()) {
					continue;
				}

				auto joint_actions = original_joint_actions;
				trim_trailing_non_actions(joint_actions, goal.handoff_agent);

				// Perform new search for one recipe
				auto new_paths = perform_new_search(state, goal, paths, joint_actions, agent_recipes.at(goal.recipe));
				if (new_paths.empty()) {
					continue;
				}

				// Get info using the new search
				size_t new_length = get_permutation_length(goals, new_paths);

				// Check if best collision avoidance search so far
				// TODO - Not sure if should introduce randomness between equal choices of collision avoidance
				if (new_length < best_length) {
					auto [joint_actions, agent_recipes] = get_actions_from_permutation(goals, new_paths, state);

					if (!is_conflict_in_permutation(state, joint_actions)) {
						Action planning_agent_action = joint_actions.at(0).get_action(planning_agent);
						best_length = new_length;
						best_collaboration = { new_length, goals, planning_agent_action };
					}
				}
			}
			if (best_length != HIGH_INIT_VAL) {
				infos.push_back(best_collaboration);
			}

		// Return unmodified entry
		} else {
			Action planning_agent_action = original_joint_actions.at(0).get_action(planning_agent);
			infos.push_back(Collaboration_Info(length, goals, planning_agent_action));
		}
	}
	return infos;
}

Paths Planner_Mac::perform_new_search(const State& state, const Goal& goal, const Paths& paths, const std::vector<Joint_Action>& joint_actions, const Agent_Combination& acting_agents) {
	auto new_path = search.search_joint(state, goal.recipe, goal.agents, goal.handoff_agent, joint_actions, acting_agents);
	if (new_path.empty()) {
		return {};
	}
	Search_Trimmer trim;
	trim.trim_forward(new_path, state, environment, goal.recipe);
	auto new_paths = paths;
	new_paths.update(new_path, goal, state, environment);
	return new_paths;
}

void Planner_Mac::trim_trailing_non_actions(std::vector<Joint_Action>& joint_actions, const Agent_Id& handoff_agent) {
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
}

Collaboration_Info Planner_Mac::get_best_collaboration(const std::vector<Collaboration_Info>& infos, 
	const size_t& max_tasks, const State& state, bool track_compatibility) {

	auto infos_copy = infos;
	std::sort(infos_copy.begin(), infos_copy.end(), [](const Collaboration_Info& lhs, const Collaboration_Info& rhs) {
		return lhs.value < rhs.value;
		});

	const Collaboration_Info* backup_result = nullptr;

	auto ingredients = state.get_ingredients_count();
	for (const auto& info : infos_copy) {
		if (track_compatibility) {
			// Probable
			bool ingredients_available = true;
			auto new_ingredients = ingredients;
			for (const auto& goal : info.get_goals()) {
				if (!new_ingredients.have_ingredients(goal.recipe, environment)) {
					ingredients_available = false;
					break;
				} else {
					new_ingredients.perform_recipe(goal.recipe, environment);
				}
			}
			if (ingredients_available) {
				ingredients = new_ingredients;
				if (info.get_agents().contains(planning_agent)) return info;
			}
		} else {

			// Improbable, planning_agent is handoff
			const auto& goals = info.get_goals();
			const auto& goal = *goals.begin();
				std::stringstream buffer;
				buffer << "Considering " << info.to_string() << " with action " << info.next_action.to_string() << " ";
				if (goals.size() == 1
					&& goal.handoff_agent == planning_agent) {
					if (info.next_action.is_not_none()) {
						buffer << " ACCEPTED\n";
						PRINT(Print_Category::PLANNER, Print_Level::VERBOSE, buffer.str());
						return info;
					} else {
						buffer << " REJECTED\n";
					}
				}
				PRINT(Print_Category::PLANNER, Print_Level::VERBOSE, buffer.str());
			

			// Improbable
			if (backup_result == nullptr) {
				backup_result = &info;
			}
		}
	}
	if (backup_result != nullptr) return *backup_result;
	return {};
}

Paths Planner_Mac::get_all_paths(const std::vector<Recipe>& recipes, const State& state) {
	Paths paths;
	auto agent_combinations = get_combinations(environment.get_number_of_agents());

	auto recipe_size = recipes.size();
	for (const auto& agents : agent_combinations) {
		size_t recipe_counter = 0;
		for (size_t i = 0; i < recipe_size; ++i) {
			const auto& recipe = recipes.at(i);

			if (!ingredients_reachable(recipe, agents, state) || state.items_hoarded(recipe, agents)) {
				continue;
			}

			if (agents.size() == 1) {
				auto time_start = std::chrono::system_clock::now();
				auto path = search.search_joint(state, recipe, agents, {}, {}, {});
				auto time_end = std::chrono::system_clock::now();

				auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();


				// Trim and insert normal path
				auto trim_path = path;
				if (!trim_path.empty()) {
					Search_Trimmer trim;
					trim.trim(trim_path, state, environment, recipe);

					Goal goal(agents, recipe, EMPTY_VAL);
					paths.insert(trim_path, goal, state, environment);
				}

				// Debug print
				std::string debug_string = "(";
				for (const auto& agent : agents.get()) {
					debug_string += std::to_string(agent.id) + ",";
				}
				PRINT(Print_Category::PLANNER, Print_Level::DEBUG, debug_string + ") : " + std::to_string(trim_path.size())
					+ " : " + recipe.result_char() + " : " + std::to_string(diff) + '\n');
			} else {
				for (const auto& handoff_agent : agents.get()) {
					auto time_start = std::chrono::system_clock::now();
					auto path = search.search_joint(state, recipe, agents, handoff_agent, {}, {});
					auto time_end = std::chrono::system_clock::now();
					auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();

					Goal goal(agents, recipe, handoff_agent);
					Action_Path a_path{ path, goal, state, environment };


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

						Search_Trimmer trim;
						trim.trim_forward(path, state, environment, recipe);
						paths.insert(path, goal, state, environment);
					}
				}
			}
		}
	}
	return paths;
}

void Planner_Mac::update_recogniser(const Paths& paths, const State& state) {
	std::map<Goal, size_t> goal_lengths;
	for (const auto& [goal, path] : paths.get_handoff()) {
		goal_lengths.insert({ goal, path->size() });
	}
	recogniser.update(goal_lengths, state);
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

