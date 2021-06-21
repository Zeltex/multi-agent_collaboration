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
constexpr auto GAMMA2 = 1.02;

Planner_Mac::Planner_Mac(Environment environment, Agent_Id planning_agent, const State& initial_state, size_t seed)
	: Planner_Impl(environment, planning_agent), time_step(0), 
		search(std::make_unique<A_Star>(environment, INITIAL_DEPTH_LIMIT)),
		recogniser(std::make_unique<Sliding_Recogniser>(environment, initial_state)) {
	set_random_seed(0);
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
	auto info = get_best_collaboration(infos, probable_infos, state);

	std::stringstream buffer3;
	++time_step;
	Action result_action{};
	if (info.has_value()
		&& info.chosen_goal.has_value()
		&& info.next_action.is_not_none()) {
		result_action = get_random_good_action(info, paths, state);
		//result_action = info.next_action;
	}

	if (info.has_value()) {
		buffer3 << "Agent " << planning_agent.id << " chose " << info.to_string() << " action "
			<< result_action.to_string() << "\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return result_action;

	//if (info.has_value()) {
	//	buffer3 << "Agent " << planning_agent.id << " chose " << info.to_string() << " action "
	//		<< info.next_action.to_string() << "\n";
	//	PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
	//	return info.next_action;
	} else {
		buffer3 << "Agent " << planning_agent.id << " did not find relevant action\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer3.str());
		return Action{ Direction::NONE, {planning_agent } };
	}
}

Action Planner_Mac::get_random_good_action(const Collaboration_Info& info, const Paths& paths_in, const State& state) {

	auto& goals = info.get_goals();

	std::vector<Action> result_actions;
	size_t result_length = HIGH_INIT_VAL;
	size_t result_path_length = HIGH_INIT_VAL;
	size_t result_handoff = HIGH_INIT_VAL;
	//size_t original_path_length = paths_in.get_handoff(info.chosen_goal).value()->size();

	for (const auto& action : environment.get_actions(planning_agent)) {
		if (action.is_none()) {
			continue;
		}
		auto paths = perform_new_search(state, info.chosen_goal, paths_in, {}, {}, action);
		if (paths.empty()) {
			continue;
		}
		

		size_t length = get_permutation_length(goals, paths);
		if (length == EMPTY_VAL) {
			continue;
		}
		//if (path_length > info.path_length) {
		//	continue;
		//}

		// Get conflict info
		auto [original_joint_actions, goal_agents] = get_actions_from_permutation(goals, paths, state);

		if (is_conflict_in_permutation(state, original_joint_actions)) {

			// Perform collision avoidance search
			size_t best_length = HIGH_INIT_VAL;
			size_t best_path_length = HIGH_INIT_VAL;
			size_t best_handoff = HIGH_INIT_VAL;
			Collaboration_Info best_collaboration;
			for (const auto& goal : goals) {

				// Skip if no agent chose to act on this recipe
				if (goal_agents.empty(goal)) {
					continue;
				}

				auto joint_actions = original_joint_actions;
				trim_trailing_non_actions(joint_actions, goal.handoff_agent);

				// Perform new search for one recipe
				auto new_paths = perform_new_search(state, goal, paths, joint_actions, goal_agents.get(goal), action);
				if (new_paths.empty()) {
					continue;
				}

				// Get info using the new search
				size_t new_length = get_permutation_length(goals, new_paths);
				size_t path_length = new_paths.get_handoff(info.chosen_goal).value()->size();
				size_t new_handoff = new_paths.get_handoff(info.chosen_goal).value()->last_action;

				// Check if best collision avoidance search so far
				if (new_length < best_length 
					|| (new_length == best_length && path_length < best_path_length)
					|| (new_length == best_length && path_length == result_path_length && new_handoff < best_handoff && best_handoff != EMPTY_VAL)) {
					auto [joint_actions, goal_agents] = get_actions_from_permutation(goals, new_paths, state);

					if (!is_conflict_in_permutation(state, joint_actions)) {
						//Action planning_agent_action = joint_actions.at(0).get_action(planning_agent);
						Action planning_agent_action = action;
						best_length = new_length;
						best_path_length = path_length;
						best_handoff = new_handoff;
						auto chosen_goal = goal_agents.get_chosen_goal();
						best_collaboration = { new_length, goals, planning_agent_action,  chosen_goal, new_paths.get_handoff(chosen_goal).value()->size()};
					}
				}
			}
			if (best_length != HIGH_INIT_VAL) {
				if (best_length < result_length 
					|| (best_length == result_length && best_path_length < result_path_length)
					|| (best_length == result_length && best_path_length == result_path_length && best_handoff < result_handoff && result_handoff != EMPTY_VAL)) {
					result_length = best_length;
					result_path_length = best_path_length;
					result_handoff = best_handoff;
					result_actions.clear();
				}
				if (best_length == result_length && best_path_length == result_path_length && best_handoff == result_handoff) {
					result_actions.push_back(best_collaboration.next_action);
				}
			}

			// Return unmodified entry
		} else {
			size_t path_length = paths.get_handoff(info.chosen_goal).value()->size();
			size_t handoff = paths.get_handoff(info.chosen_goal).value()->last_action;

			if (length < result_length 
				|| (length == result_length && path_length < result_path_length)
				|| (length == result_length && path_length == result_path_length && handoff < result_handoff && result_handoff != EMPTY_VAL)) {
				result_length = length;
				result_path_length = path_length;
				result_handoff = handoff;
				result_actions.clear();
			}
			if (length == result_length && path_length == result_path_length && handoff == result_handoff) {
				//Action planning_agent_action = original_joint_actions.at(0).get_action(planning_agent);
				Action planning_agent_action = action;
				result_actions.push_back(planning_agent_action);
			}
		}
	}

	auto result_action = get_random<Action>(result_actions);
	if (result_action != info.next_action) {
		std::stringstream buffer;
		buffer << "Changed from " << info.next_action.to_string() << " to " << result_action.to_string() << " for goal " << info.chosen_goal.to_string() << "\n";
		PRINT(Print_Category::PLANNER, Print_Level::DEBUG, buffer.str());
	}
	return result_action;
}

std::vector<Collaboration_Info> Planner_Mac::calculate_infos(const Paths& paths, const std::vector<Recipe>& recipes_in,
	const State& state) {
	size_t total_agents = environment.get_number_of_agents();

	// Precalculate all recipe combinations
	std::vector<std::vector<std::vector<Recipe>>> all_recipe_combinations;
	size_t recipe_in_size = recipes_in.size();
	for (size_t i = 0; i < total_agents && i < recipe_in_size; ++i) {
		all_recipe_combinations.push_back(get_combinations(recipes_in, i + 1));
	}

	auto agent_permutations = get_handoff_permutations();
	std::vector<Collaboration_Info> infos;

	auto agent_combinations = get_combinations(total_agents);
	for (const auto& agents : agent_combinations) {
		size_t agent_size = agents.size();

		// Iterate setups for agent_combination
		for (size_t recipe_combination_index = 0; 
			//recipe_combination_index < agent_size && recipe_combination_index < recipe_in_size; 
			recipe_combination_index < 1;
			++recipe_combination_index) {

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
							info_val->get_next_action(planning_agent), goal, paths.get_handoff(goal).value()->size() };

						infos.push_back(result);
					}
				} else {
					Goals goals;
					for (const auto& recipe : recipes) {
						goals.add({ agents, recipe, EMPTY_VAL });
					}

					auto temp_infos = get_collaboration_permutations(goals, paths, agent_permutations.get(recipes.size()), state);
					infos.insert(std::end(infos), std::begin(temp_infos), std::end(temp_infos));
				}
			}
		}
	}
	return infos;
}

Permutations Planner_Mac::get_handoff_permutations() const {
	// -1 for including none, 0 for not including none
	int min_index = 0;
	std::vector<Agent_Id> all_agents;
	size_t max_size = environment.get_number_of_agents();
	for (size_t i = 0; i < environment.get_number_of_agents(); ++i) {
		all_agents.push_back({ i });
	}

	Permutations result(max_size);
	size_t agent_size = all_agents.size();

	// None-handoff for single agent
	if (min_index > -1) {
		result.insert(Agent_Combination(Agent_Id(EMPTY_VAL)));
	}

	for (size_t current_size = 1; current_size <= max_size; ++current_size) {
		std::vector<int> counters(current_size, min_index);
		bool done = false;
		while (!done) {
			std::vector<Agent_Id> next_permutation;
			bool valid_permutation = true;
			std::vector<bool> seen(agent_size, false);
			for (size_t i = 0; i < counters.size(); i++) {
				if (counters[i] == -1) {
					next_permutation.push_back(Agent_Id(EMPTY_VAL));
				} else {
					// TODO - Testing without this constraint
					//if (seen.at(counters[i])) {
					//	valid_permutation = false;
					//}
					seen.at(counters[i]) = true;
					next_permutation.push_back(all_agents.at(counters[i]));
				}
			}
			if (valid_permutation) {
				result.insert(next_permutation);
			}

			// Advance indices
			size_t index = 0;
			while (!done && ++counters.at(index) >= agent_size) {
				counters.at(index++) = min_index;
				done = index >= counters.size();
			}
		}
	}
	return result;
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

			//// If collaborating on multiple tasks, check if all tasks could be done faster seperately
			//if (info_entry.goals_size() > 1) {
			//	bool task_faster_coop = false;
			//	for (const auto& goal : info_entry.get_goals_iterable()) {


			//		Goals goals_reduced_agents(goal);
			//		goals_reduced_agents.clear_all_handoff_indices();

			//		auto it_seperate = goal_values.find(goals_reduced_agents);
			//		//assert(it_seperate != goal_values.end());
			//		if (it_seperate->second > info_entry.value) {
			//			task_faster_coop = true;
			//		}
			//	}
			//	is_probable &= task_faster_coop;
			//}
			
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
		//if (is_probable && info_entry.goals_size() == 1) {
		if (is_probable) {
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
		//auto penalty = std::pow(GAMMA, entry.agents_size() - entry.goals_size());
		auto penalty = std::pow(GAMMA, entry.agents_size()) / std::pow(GAMMA2, entry.goals_size());
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
	std::sort(infos.begin(), infos.end(), [](const Collaboration_Info& lhs, const Collaboration_Info& rhs) {
		return lhs.value < rhs.value;
		});
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


std::pair<std::vector<Joint_Action>, Goal_Agents> Planner_Mac::get_actions_from_permutation(
	const Goals& goals, const Paths& paths, const State& state) {

	Goal_Agents goal_agents;
	std::vector<Joint_Action> joint_actions(action_trace_length, goals.get_agents());

	for (const auto& agent : goals.get_agents()) {
		auto [actions, goal] = get_actions_from_permutation_inner(goals, agent, paths, state);
		goal_agents.add(goal, agent);

		if (agent == planning_agent) {
			goal_agents.set_chosen_goal(goal);
		}

		for (size_t action_index = 0; action_index < actions.size(); ++action_index) {
			joint_actions.at(action_index).update_action(agent, actions.at(action_index).direction);
		}
	}
	return { joint_actions, goal_agents };
}

std::pair<std::vector<Action>, Goal> Planner_Mac::get_actions_from_permutation_inner(const Goals& goals,
	const Agent_Id& acting_agent, const Paths& paths, const State& state) {

	size_t backup_first_action = HIGH_INIT_VAL;
	const Action_Path* backup_path = nullptr;
	const Goal* backup_goal = nullptr;

	//size_t recipes_size = action_paths.size();
	const Action_Path* result_path = nullptr;
	const Goal* chosen_goal = nullptr;

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
				chosen_goal = &goal;
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
				chosen_goal = &goal;
				highest_prob = probability;
			}

			auto index = path->get_first_non_trivial_index(acting_agent);
			if (index != EMPTY_VAL && index < backup_first_action) {
				backup_first_action = index;
				backup_path = path;
				backup_goal = &goal;
			}
		}
	}
	
	if (result_path == nullptr && backup_path != nullptr) {
		result_path = backup_path;
		chosen_goal = backup_goal;
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

		return { result, *chosen_goal };
	} else {
		return { {}, {} };
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
	Recipe recipe;

	bool operator<(const Temp3& other) const {
		if (handoff_time != other.handoff_time) handoff_time < other.handoff_time;
		return id < other.id;
	}
};

size_t Planner_Mac::get_permutation_length(const Goals& goals, const Paths& paths){

	// TODO - Testing without this constraint
	//if (is_agent_abused(goals, paths)) {
	//	return EMPTY_VAL;
	//}

	// Sort tasks by handoff time
	std::set<Temp3> sorted;
	for (const auto& goal : goals) {
		auto path_opt = paths.get_handoff(goal);
		if (!path_opt.has_value()) {
			return EMPTY_VAL;
		}
		auto path = path_opt.value();
		sorted.insert({ path->size(), path->last_action, goal.handoff_agent, sorted.size(), goal.recipe });
	}

	// Calculate adjusted handoff time per agent and task
	size_t agent_size = environment.get_number_of_agents();
	std::vector<size_t> handoffs(agent_size, 0);
	std::set<Temp2> tasks;
	for (const auto& entry : sorted) {
		size_t extra_length = entry.length;
		size_t total_length = entry.length;
		if (entry.handoff_time != EMPTY_VAL) {
			auto& handoff_ref = handoffs.at(entry.agent.id);

			extra_length += handoff_ref;
			total_length += handoff_ref;


			// Note the +1's are from index to length conversion
			handoffs.at(entry.agent.id) += entry.handoff_time + 1;
			//if (environment.is_type_stationary(entry.recipe.ingredient1)
			//	|| environment.is_type_stationary(entry.recipe.ingredient2)) {
				
				extra_length -= (entry.handoff_time + 1);
			//}
		}
		tasks.insert({ entry.agent, total_length, extra_length, tasks.size() });
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
		handoff_ref = std::max(handoff_ref + task.extra_length + 1, task.total_length);
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
	const Paths& paths, const std::vector<Agent_Combination>& agent_permutations,
	const State& state) {

	std::vector<Collaboration_Info> infos;

	// Get info on each permutation
	for (const auto& agent_permutation : agent_permutations) {
		if (goals_in.get_agents().size() == 1) {
			if (agent_permutation != Agent_Combination(Agent_Id(EMPTY_VAL))) {
				continue;
			}
		} else {
			bool invalid_agent = false;
			for (const auto& agent : agent_permutation) {
				if (
					// If we allow none-handoff agents
					//!agent.is_empty() && 
					!goals_in.get_agents().contains(agent)) {

					invalid_agent = true;
					break;
				}
			}
			if (invalid_agent) {
				continue;
			}
		}

		auto goals = goals_in;
		goals.update_handoffs(agent_permutation);

		size_t length = get_permutation_length(goals, paths);
		if (length == EMPTY_VAL) {
			continue;
		}

		// Get conflict info
		auto [original_joint_actions, goal_agents] = get_actions_from_permutation(goals, paths, state);

		if (is_conflict_in_permutation(state, original_joint_actions)) {

			// Perform collision avoidance search
			size_t best_length = HIGH_INIT_VAL;
			Collaboration_Info best_collaboration;
			for (const auto& goal : goals) {

				// Skip if no agent chose to act on this recipe
				if (goal_agents.empty(goal)) {
					continue;
				}

				auto joint_actions = original_joint_actions;
				trim_trailing_non_actions(joint_actions, goal.handoff_agent);

				// Perform new search for one recipe
				auto new_paths = perform_new_search(state, goal, paths, joint_actions, goal_agents.get(goal));
				if (new_paths.empty()) {
					continue;
				}

				// Get info using the new search
				size_t new_length = get_permutation_length(goals, new_paths);

				// Check if best collision avoidance search so far
				// TODO - Not sure if should introduce randomness between equal choices of collision avoidance
				if (new_length < best_length) {
					auto [joint_actions, goal_agents] = get_actions_from_permutation(goals, new_paths, state);

					if (!is_conflict_in_permutation(state, joint_actions)) {
						Action planning_agent_action = joint_actions.at(0).get_action(planning_agent);
						best_length = new_length;
						auto chosen_goal = goal_agents.get_chosen_goal();
						size_t path_length = EMPTY_VAL;
						if (chosen_goal.has_value()) {
							path_length = paths.get_handoff(chosen_goal).value()->size();
						}
						best_collaboration = { new_length, goals, planning_agent_action, chosen_goal, path_length };
					}
				}
			}
			if (best_length != HIGH_INIT_VAL) {
				infos.push_back(best_collaboration);
			}

		// Return unmodified entry
		} else {
			Action planning_agent_action = original_joint_actions.at(0).get_action(planning_agent);
			auto chosen_goal = goal_agents.get_chosen_goal();
			auto path_length = EMPTY_VAL;
			if (chosen_goal.has_value()) {
				path_length = paths.get_handoff(chosen_goal).value()->size();
			}
			infos.push_back(Collaboration_Info(length, goals, planning_agent_action, chosen_goal, path_length));
		}
	}
	return infos;
}

Paths Planner_Mac::perform_new_search(const State& state, const Goal& goal, const Paths& paths, 
	const std::vector<Joint_Action>& joint_actions, const Agent_Combination& acting_agents, const Action& initial_action) {


	auto new_path = search.search_joint(state, goal.recipe, goal.agents, goal.handoff_agent, joint_actions, acting_agents, initial_action);
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
	const std::vector<Collaboration_Info>& probable_infos, const State& state) {

	//auto infos_copy = infos;
	//std::sort(infos_copy.begin(), infos_copy.end(), [](const Collaboration_Info& lhs, const Collaboration_Info& rhs) {
	//	return lhs.value < rhs.value;
	//	});

	const Collaboration_Info* backup_result = nullptr;
	//std::map<Agent_Id, size_t> assignments;
	//for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
	//	assignments[agent] = EMPTY_VAL;
	//}
	
	// Probable
	Agent_Combination used_agents;
	auto ingredients = state.get_ingredients_count();
	for (const auto& info : probable_infos) {
		auto new_agents = used_agents.get_new_agents(info.get_agents());
		if (new_agents.empty()) {
			continue;
		}
		auto recipes = info.get_goals().get_recipes();
		if (!ingredients.have_ingredients(recipes, environment)) {
			continue;
		}

		ingredients.perform_recipes(recipes, environment);
		used_agents.add(new_agents);
		if (info.get_agents().contains(planning_agent)) return info;

	}



	//// Probable
	//Agent_Combination used_agents;
	//auto ingredients = state.get_ingredients_count();
	//for (const auto& info : probable_infos) {
	//	auto new_agents = used_agents.get_new_agents(info.get_agents());
	//	if (new_agents.empty()) {
	//		continue;
	//	}
	//	auto recipes = info.get_goals().get_recipes();
	//	if (!ingredients.have_ingredients(recipes, environment)) {
	//		continue;
	//	}

	//	ingredients.perform_recipes(recipes, environment);
	//	used_agents.add(new_agents);
	//	if (info.get_agents().contains(planning_agent)) return info;
	//}

	// Improbable
	for (const auto& info : infos) {
		if (info.next_action.is_not_none()) {
			auto recipes = info.get_goals().get_recipes();
			if (ingredients.have_ingredients(recipes, environment)) {
				return info;
			}

		}
	}

	// Any, we are active
	for (const auto& info : infos) {
		auto recipes = info.get_goals().get_recipes();
		auto& agents = info.get_agents();
		if (ingredients.have_ingredients(recipes, environment)) {
			return info;
			//// Check if contains relevant goal
			//Goal chosen_goal;
			//if (agents.is_only_agent(planning_agent)) {
			//	chosen_goal = *info.get_goals().begin();
			//} else {
			//	for (const auto& goal : info.get_goals()) {
			//		if (goal.handoff_agent == planning_agent) {
			//			chosen_goal = goal;
			//			break;
			//		}
			//	}
			//}

			//// If goal found, find direction to reduce distance to nearest ingredient
			//if (chosen_goal.has_value()) {
			//	Action best_action;
			//	size_t best_dist = EMPTY_VAL;
			//	Coordinate agent_location = state.get_agent(planning_agent).coordinate;
			//	size_t walls_penetrated = 0;

			//	// Find best direction, ingredient 1
			//	if (!environment.is_type_stationary(chosen_goal.recipe.ingredient1)) {
			//		for (const auto& ingredient_location : environment.get_non_wall_locations(state, chosen_goal.recipe.ingredient1)) {
			//			auto [dist, direction] = search.get_dist_direction(agent_location, ingredient_location.coordinate, walls_penetrated);
			//			if (dist < best_dist || best_dist == EMPTY_VAL) {
			//				best_dist = dist;
			//				best_action = { direction, planning_agent };
			//			}
			//		}
			//	}
			//	// Find best direction, ingredient 2
			//	for (const auto& ingredient_location : environment.get_non_wall_locations(state, chosen_goal.recipe.ingredient2)) {
			//		auto [dist, direction] = search.get_dist_direction(agent_location, ingredient_location.coordinate, walls_penetrated);
			//		if (dist < best_dist || best_dist == EMPTY_VAL) {
			//			best_dist = dist;
			//			best_action = { direction, planning_agent };
			//		}
			//	}
			//	// Return best
			//	if (best_dist != EMPTY_VAL) {
			//		auto info_copy = info;
			//		info_copy.next_action = best_action;
			//		return info_copy;
			//	}
			//}
		}
	}




	//auto ingredients = state.get_ingredients_count();
	//for (const auto& info : infos_copy) {
	//	if (track_compatibility) {
	//		// Probable
	//		bool ingredients_available = true;
	//		bool contains_new_agent = false;

	//		auto new_used_agents = used_agents;
	//		for (const auto& agent : info.get_agents()) {
	//			if (!used_agents.contains(agent)) {
	//				new_used_agents.add(agent);
	//				contains_new_agent = true;
	//			}
	//		}

	//		auto new_ingredients = ingredients;
	//		for (const auto& goal : info.get_goals()) {
	//			if (!new_ingredients.have_ingredients(goal.recipe, environment)) {
	//				ingredients_available = false;
	//				break;
	//			} else {
	//				new_ingredients.perform_recipe(goal.recipe, environment);
	//			}
	//		}
	//		if (ingredients_available && contains_new_agent) {
	//			ingredients = new_ingredients;
	//			used_agents = new_used_agents;
	//			if (info.get_agents().contains(planning_agent)) return info;
	//		}
	//	} else {

	//		// Improbable, planning_agent is handoff
	//		const auto& goals = info.get_goals();
	//		const auto& goal = *goals.begin();
	//			std::stringstream buffer;
	//			buffer << "Considering " << info.to_string() << " with action " << info.next_action.to_string() << " ";
	//			if (goals.size() == 1
	//				&& goal.handoff_agent == planning_agent) {
	//				if (info.next_action.is_not_none()) {
	//					buffer << " ACCEPTED\n";
	//					PRINT(Print_Category::PLANNER, Print_Level::VERBOSE, buffer.str());
	//					return info;
	//				} else {
	//					buffer << " REJECTED\n";
	//				}
	//			}
	//			PRINT(Print_Category::PLANNER, Print_Level::VERBOSE, buffer.str());
	//		

	//		// Improbable
	//		//if (backup_result == nullptr) {
	//		//	backup_result = &info;
	//		//}
	//	}
	//}
	//if (backup_result != nullptr) return *backup_result;
	return {};
}

bool Planner_Mac::temp(const Agent_Combination& agents, const Agent_Id& handoff_agent, const Recipe& recipe, const State& state) {
	bool reachable1 = false, reachable2 = false;
	for (const auto& agent : agents) {
		if (agent == handoff_agent) continue;
		auto reduced_agents = agents;
		reduced_agents.remove(agent);
		if (!reachable1 && ingredient_reachable(recipe.ingredient1, agent, reduced_agents, state)) {
			reachable1 = true;
			if (reachable2) break;
		}
		if (!reachable2 && ingredient_reachable(recipe.ingredient2, agent, reduced_agents, state)) {
			reachable2 = true;
			if (reachable1) break;
		}
	}
	if (reachable1 && reachable2) {
		return true;
	}
	auto reduced_agents = agents;
	reduced_agents.remove(handoff_agent);
	if (reachable2
		&& !environment.is_type_stationary(recipe.ingredient1) 
		&& ingredient_reachable(recipe.ingredient1, handoff_agent, reduced_agents, state)) {
		return true;
	}
	if (reachable1
		&& ingredient_reachable(recipe.ingredient2, handoff_agent, reduced_agents, state)) {
		return true;
	}
	return false;
}

Paths Planner_Mac::get_all_paths(const std::vector<Recipe>& recipes, const State& state) {
	Paths paths;
	auto agent_combinations = get_combinations(environment.get_number_of_agents());

	auto recipe_size = recipes.size();
	for (const auto& agents : agent_combinations) {
		size_t recipe_counter = 0;
		for (size_t i = 0; i < recipe_size; ++i) {
			const auto& recipe = recipes.at(i);

			bool reachable1 = false, reachable2 = false;
			for (const auto& agent : agents) {
				auto reduced_agents = agents;
				reduced_agents.remove(agent);
				if (!reachable1 && ingredient_reachable(recipe.ingredient1, agent, reduced_agents, state)) {
					reachable1 = true;
					if (reachable2) break;
				}
				if (!reachable2 && ingredient_reachable(recipe.ingredient2, agent, reduced_agents, state)) {
					reachable2 = true;
					if (reachable1) break;
				}
			}
			if (!reachable1 || !reachable2) {
				continue;
			}


			Agent_Combination handoff_agents;
			if (agents.size() > 1) {
				handoff_agents.add(agents);
			} else {
				handoff_agents.add(Agent_Id(EMPTY_VAL));
			}

			for (const auto& handoff_agent : handoff_agents) {
				if (handoff_agent.is_empty()) {
					bool reachable = false;
					for (const auto& agent : agents) {
						auto reduced_agents = agents;
						reduced_agents.remove(agent);
						if (ingredients_reachable(recipe, agent, reduced_agents, state)) {
							reachable = true;
							break;
						}
					}
					if (!reachable) {
						continue;
					}

				} else {
					if (!temp(agents, handoff_agent, recipe, state)) {
						continue;
					}
					//auto reduced_agents = agents;
					//reduced_agents.remove(handoff_agent);
					//for (const auto& agent : agents) {
					//	if (ingredient_reachable(recipe.ingredient1, agent, reduced_agents, state)
					//		|| (environment.is_type_stationary(recipe. ) {
					//		reachable = true;
					//		break;
					//	}
					//}
				}

				auto time_start = std::chrono::system_clock::now();
				auto path = search.search_joint(state, recipe, agents, handoff_agent, {}, {}, {});
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
					//trim.trim(trim_path, state, environment, recipe);
					paths.insert(path, goal, state, environment);
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
bool Planner_Mac::ingredients_reachable(const Recipe& recipe, const Agent_Id agent, const Agent_Combination& agents, const State& state) const {
	if (!ingredient_reachable(recipe.ingredient1, agent, agents, state)) return false;
	return ingredient_reachable(recipe.ingredient2, agent, agents, state);
}

bool Planner_Mac::ingredient_reachable(const Ingredient& ingredient_in, const Agent_Id agent, const Agent_Combination& agents, const State& state) const {
	auto reachables = agent_reachables.find({ agent, agents });
	if (reachables == agent_reachables.end()) {
		throw std::runtime_error("Unknown agent combination");

	}

	const auto& agent_item = state.get_agent(agent).item;
	if (agent_item.has_value() && agent_item.value() == ingredient_in) {
		return true;
	}

	for (const auto& location : environment.get_coordinates(state, ingredient_in, false)) {
		if (reachables->second.get(location)) {
			return true;
		}
	}
	return false;
}


void Planner_Mac::initialize_reachables(const State& state) {
	agent_reachables.clear();
	std::vector<size_t> all_agents;
	for (size_t i = 0; i < state.agents.size(); ++i) {
		all_agents.push_back(i);
	}

	for (size_t agent = 0; agent < state.agents.size(); ++agent) {
		auto reduced_agents = all_agents;
		reduced_agents.erase(reduced_agents.begin() + agent);

		auto combinations = get_combinations(reduced_agents);
		combinations.push_back({});
		for (const auto& agents : combinations) {
			Reachables reachables(environment.get_width(), environment.get_height());

			// Initial agent locations
			std::deque<Coordinate> frontier;
			//for (const auto& agent : agents.get()) {
				auto location = state.get_location(agent);
				reachables.set(location, true);
				frontier.push_back(location);
			//}

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
								|| agents.contains(blocking_agent.value())
								|| agent == blocking_agent.value().id)) {
							frontier.push_back(location);
						}
					}
				}
			}
			agent_reachables.insert({ {agent, agents}, reachables });
		}
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

