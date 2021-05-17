#pragma once

#include "Environment.hpp"
#include "Core.hpp"
#include "Search.hpp"
#include "State.hpp"
#include "Recogniser.hpp"
#include "Planner.hpp"

#include <vector>
#include <set>
#include <algorithm> 
#include <deque>

struct Action_Path {
	Action_Path(std::vector<Joint_Action> joint_actions,
		const Goal goal,
		const State& state,
		const Environment& environment)
		: joint_actions(joint_actions), recipe(goal.recipe), agents(goal.agents),
		handoff_agent(goal.handoff_agent),
		first_action(EMPTY_VAL), last_action(EMPTY_VAL) {

		//const auto& handoff_agent = goal.handoff_agent;

		if (joint_actions.empty() || handoff_agent == EMPTY_VAL) {
			return;
		}
		auto initial_coordinate = state.get_agent(handoff_agent).coordinate;
		
		// Stop at first action which interacts with a wall
		first_action = 0;
		auto coordinate = initial_coordinate;
		while (true) {
			coordinate = environment.move_noclip(coordinate, joint_actions.at(first_action).get_action(handoff_agent).direction);
			if (environment.is_cell_type(coordinate, Cell_Type::WALL)) {
				break;
			}
			if (first_action == joint_actions.size() - 1) {
				first_action = EMPTY_VAL;
				break;
			}
			++first_action;
		}

		// Note last action which interacts with a wall 
		// (note wall check is using noclip, but coordinate tracking is using regular move)
		coordinate = initial_coordinate;
		last_action = EMPTY_VAL;
		size_t action_counter = 0;
		while (action_counter < joint_actions.size()) {
			const auto& direction = joint_actions.at(action_counter).get_action(handoff_agent).direction;
			if (environment.is_cell_type(environment.move_noclip(coordinate, direction), Cell_Type::WALL)) {
				last_action = action_counter;
			}

			coordinate = environment.move(coordinate, direction);
			++action_counter;
		}
	}

	bool operator<(const Action_Path& other) const {
		if (joint_actions.size() != other.joint_actions.size()) return joint_actions.size() < other.joint_actions.size();
		if (agents.size() != other.agents.size()) return agents.size() < other.agents.size();
		if (recipe != other.recipe) return recipe < other.recipe;
		if (agents != other.agents) return agents < other.agents;
	}

	size_t size() const {
		return joint_actions.size();
	}

	size_t length() const {
		return joint_actions.size();
	}

	bool empty() const {
		return joint_actions.empty();
	}

	bool contains_useful_action() const {
		return last_action != EMPTY_VAL;
	}

	bool has_useful_action(const Agent_Id& agent, const State& state, const Environment& environment) const {

		auto coordinate = state.get_agent(agent).coordinate;
		size_t action_counter = 0;
		while (action_counter < joint_actions.size()) {
			coordinate = environment.move_noclip(coordinate, joint_actions.at(action_counter).get_action(agent).direction);
			if (environment.is_cell_type(coordinate, Cell_Type::WALL)) {
				return true;
			}
			++action_counter;
		}
		return false;
	}

	size_t get_first_non_trivial_index(Agent_Id agent) const {
		size_t index = 0;
		for (const auto& action : joint_actions) {
			if (action.is_action_non_trivial(agent)) {
				return index;
			}
			++index;
		}
		return EMPTY_VAL;
	}

	Action get_next_action(Agent_Id agent) const {
		if (joint_actions.empty()) return {Direction::NONE, agent};
		return joint_actions.at(0).get_action(agent);
	}

	std::string first_action_string() {
		return first_action == EMPTY_VAL ? "X" : std::to_string(first_action);
	}

	std::string last_action_string() {
		return last_action == EMPTY_VAL ? "X" : std::to_string(last_action);
	}
	
	//	bool is_valid() const {
//		return length != 0 && length != EMPTY_VAL;
//	}
//	Action action(const Agent_Id& agent) const {
//		return next_action.get_action(agent);
//	}
//	bool has_useful_action(const Agent_Id& agent) const {
//		for (const auto& action : actions->joint_actions) {
//			if (action.is_action_non_trivial(agent)) {
//				return true;
//			}
//		}
//		return false;
//	}

	std::vector<Joint_Action> joint_actions;
	Recipe recipe;
	Agent_Combination agents;
	size_t first_action;	// First useful action by handoff_agent
	size_t last_action;		// Last useful action by handoff_agent

	Agent_Id handoff_agent;
};

struct Paths {
	Paths() : handoff_map(), handoff_paths(){};

	Paths(const Paths& other) {
		for (const auto& [goal, path_ptr] : other.handoff_map) {
			this->insert(goal, *path_ptr);
		}
	}

	void insert(const Goal& goal, const Action_Path& path) {
		handoff_paths.push_back(path);
		handoff_map.insert({ goal, &handoff_paths.back() });
	}
	
	void insert(const std::vector<Joint_Action>& actions, const Goal& goal,
		const State& state, const Environment& environment) {

		handoff_paths.push_back({ actions, goal, state, environment });
		Action_Path* path_ptr = &handoff_paths.back();
		handoff_map.insert({ goal, path_ptr });

	}

	void update(const std::vector<Joint_Action>& actions,
		const Goal& goal, const State& state, const Environment& environment) {

		auto it = handoff_map.find(goal);
		if (it == handoff_map.end()) {
			insert(actions, goal, state, environment);
		} else {
			(*it->second) = Action_Path(actions, goal, state, environment);
		}
	}

	const std::map<Goal, Action_Path*>& get_handoff() const {
		return handoff_map;
	}

	std::optional<const Action_Path*> get_handoff(const Goal& goal) const { 
		auto it = handoff_map.find(goal); 
		if (it == handoff_map.end()) {
			return {};
		} else {
			return it->second;
		}
	}

	bool empty() const {
		return handoff_paths.empty();
	}

private:
	std::deque<Action_Path> handoff_paths;
	std::map<Goal, Action_Path*> handoff_map;
};

struct Collaboration_Info {
	Collaboration_Info() : length(EMPTY_VAL), 
		next_action(), value(EMPTY_VAL), goals() {};

	//Collaboration_Info(size_t length, Agent_Combination combination, const std::vector<Recipe> recipes,
	//	Action next_action, Agent_Combination permutation)
	//	: length(length), 
	//		next_action(next_action), value(EMPTY_VAL), goals() {};

	Collaboration_Info(size_t length, const Goals& goals, const Action& next_action)
		: length(length), goals(goals), next_action(next_action) {}

	std::string to_string() const {
		if (goals.has_same_agents()) {
			std::string result;
			for (const auto& goal : goals.get_iterable()) {
				result += goal.recipe.result_char();
			}
			result += ":" + goals.get_agents().to_string_raw() 
				+ ":" + goals.get_handoff_string();
			
			return result;
		} else {
			return "String todo - planner_mac.hpp";
		}
	}

	bool has_value() const {
		return length != EMPTY_VAL;
	}

	bool is_only_agent(Agent_Id agent) const {
		const auto& agents = goals.get_agents();
		return agents.size() == 1 && *agents.begin() == agent;
	}

	size_t goals_size() const {
		return goals.size();
	}

	size_t agents_size() const {
		return get_agents().size();
	}

	const Agent_Combination& get_agents() const {
		return goals.get_agents();
	}

	const Goals& get_goals() const {
		return goals;
	}
	
	const std::vector<Goal>& get_goals_iterable() const {
		return goals.get_iterable();
	}

	Action next_action;
	size_t length;
	float value;
	
private:
	Goals goals;
};

struct Recipe_Solution {
	Agent_Combination agents;
	size_t action_count;
	bool operator>(const Action_Path& other) const {
		if (action_count != other.joint_actions.size()) return action_count > other.joint_actions.size();
		return agents.size() > other.agents.size();
	}
};

struct Agent_Usefulnes {
	Agent_Usefulnes() : 
		incl_length(EMPTY_VAL), incl_first_action(EMPTY_VAL), incl_last_action(EMPTY_VAL),
		excl_length(EMPTY_VAL), excl_first_action(EMPTY_VAL), excl_last_action(EMPTY_VAL),
		action(Direction::NONE, {EMPTY_VAL}) {}

	size_t incl_length;
	size_t incl_first_action;
	size_t incl_last_action;
	size_t excl_length;
	size_t excl_first_action;
	size_t excl_last_action;
	Action action;

	void update(const Action_Path& action_path, Agent_Id agent) {
		if (action_path.empty()) return;
		if (action_path.agents.contains(agent)) {
			if (action_path.size() < incl_length) {
				incl_length = action_path.size();
				incl_first_action = action_path.first_action;
				incl_last_action = action_path.last_action;
				action = action_path.get_next_action(agent);
			}
		} else {
			if (action_path.size() < excl_length) {
				excl_length = action_path.size();
				excl_first_action = action_path.first_action;
				excl_last_action = action_path.last_action;
			}
		}
	}

	bool is_useful() const {
		return ((float)excl_length - incl_length) / 2 > incl_last_action;
	}

	float get_usefulness() const {
		return ((float)excl_length - incl_length) / 2;
	}

	std::string get_usefulness_str() const {
		if (excl_length == EMPTY_VAL) return "-";
		return convert(get_usefulness());
	}
	std::string excl_length_str() const {
		return convert(excl_length);
	}

	std::string convert(size_t val) const {
		if (val == EMPTY_VAL) {
			return "-";
		} else {
			return std::to_string(val);
		}
	}
};

struct Reachables {
	Reachables(size_t width, size_t height) : data((width * height), false), width(width) {}
	void set(Coordinate location, bool value) {
		data.at(location.second * width + location.first) = value;
	}
	bool get(Coordinate location) const {
		return data.at(location.second * width + location.first);
	}
	std::vector<bool> data;
	size_t width;
};

struct Solution_History {
	Solution_History() : history() {}
	std::vector<size_t> history;
	void add(size_t solution_length, size_t time_step) { 
		while (history.size() < time_step) {
			history.push_back(0);
		}
		history.push_back(solution_length); 
	}
	size_t get(size_t i) const {
		if (history.size() < i + 1) {
			return 0;
		} else {
			return history.at(i);
		}
	}
};

struct Recipe_Agents {
	Recipe_Agents(Recipe recipe, Agent_Combination agents) : agents(agents), recipe(recipe) {}
	Agent_Combination agents;
	Recipe recipe;
	bool operator<(const Recipe_Agents& other) const {
		if (recipe != other.recipe) return recipe < other.recipe;
		return agents < other.agents;
	}
};

struct Colab_Collection {
	Colab_Collection() : infos(), tasks(0), value(EMPTY_VAL) {};

	void add(const Collaboration_Info& info) {
		tasks += info.goals_size();
		infos.push_back(info);
		for (const auto& agent : info.get_agents()) {
			agents.insert(agent);
		}
	}

	bool has_value() const {
		return tasks != 0;
	}

	bool contains(const Agent_Combination& agent_combination) const {
		for (const auto& agent : agent_combination.get()) {
			for (const auto& info : infos) {
				if (info.get_agents().contains(agent)) {
					return true;
				}
			}
		}
		return false;
	}

	//bool contains(const std::vector<Recipe>& recipes_in) const {
	//	for (const auto& recipe_in : recipes_in) {
	//		for (const auto& info : infos) {
	//			for (const auto& goal : info.get_goals()) {
	//				if (recipe_in.result == goal.recipe.result) {
	//					return true;
	//				}
	//			}
	//		}
	//	}
	//	return false;
	//}

	//bool contains(const Recipe& recipe_in) const {
	//	for (const auto& info : infos) {
	//		for (const auto& goal : info.get_goals()) {
	//			if (recipe_in.result == goal.recipe.result) {
	//				return true;
	//			}
	//		}
	//	}
	//	return false;
	//}

	void calculate_value(size_t max_agents) {
		std::vector<float> values(max_agents, 0);
		for (const auto& info : infos) {
			for (const auto& agent : info.get_agents()) {
				//values.at(agent.id) += info.value * info.recipes.size();
				values.at(agent.id) += info.value;
			}
		}
		value = 0;
		for (const auto& entry_value : values) {
			value = std::max(value, entry_value);
		}
	}

	bool is_compatible(const Goals& goals_in,
		const Ingredients& available_ingredients_in,
		const Environment& environment) const {

		// Check if new goals contains at least 1 unassigned agent
		bool found_unassigned_agent = false;
		for (const auto& goal : goals_in) {
			for (const auto& agent : goal.agents) {
				if (agents.find(agent) == agents.end()) {
					found_unassigned_agent = true;
					break;
				}
			}
			if (found_unassigned_agent) {
				break;
			}
		}
		if (!found_unassigned_agent) {
			return false;
		}
		
		// Check if enough ingredients are present
		Ingredients recipe_ingredients;
		for (const auto& info : infos) {
			for (const auto& goal : info.get_goals_iterable()) {
				recipe_ingredients.add_ingredients(goal.recipe, environment);
			}
		}
		for (const auto& goal : goals_in.get_iterable()) {
			recipe_ingredients.add_ingredients(goal.recipe, environment);
		}

		if (!(recipe_ingredients <= available_ingredients_in)) {
			return false;
		}

		// Check if recipes can still lead to goal
		auto available_ingredients = available_ingredients_in;
		for (const auto& info : infos) {
			for (const auto& goal : info.get_goals_iterable()) {
				available_ingredients.perform_recipe(goal.recipe, environment);
			}
		}

		for (const auto& goal : goals_in.get_iterable()) {
			available_ingredients.perform_recipe(goal.recipe, environment);
		}
		if (!environment.do_ingredients_lead_to_goal(available_ingredients)) {
			return false;
		}


		return true;
	}

	std::vector<Collaboration_Info> infos;
	size_t tasks;
	std::set<Agent_Id> agents;
	float value;
};

struct Temp {
	Temp(const Action_Path& path, size_t handoff_agent_index)
		: path(path), handoff_agent_index(handoff_agent_index), length(path.size()),
		handoff_time(path.last_action) {}

	Action_Path path;
	size_t handoff_agent_index;
	mutable size_t length;
	mutable size_t handoff_time;

	bool operator<(const Temp& other) const {
		if (handoff_time != other.handoff_time) return handoff_time < other.handoff_time;
		return handoff_agent_index < other.handoff_agent_index;
	}
};


class Planner_Mac : public Planner_Impl {


public:
	Planner_Mac(Environment environment, Agent_Id agent, const State& initial_state);
	virtual Action get_next_action(const State& state, bool print_state) override;

private:
	Paths get_all_paths(const std::vector<Recipe>& recipes, const State& state);
	bool ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const;
	void initialize_reachables(const State& initial_state);
	void initialize_solutions();
	std::optional<Collaboration_Info> check_for_collaboration(const Paths& paths, 
		const std::vector<Recipe>& recipes, const std::map<Agent_Id, Goal>& goals, 
		const State& state);

	void update_recogniser(const Paths& paths);

	Collaboration_Info get_best_collaboration(const std::vector<Collaboration_Info>& infos, 
		const size_t& max_tasks, const State& state, bool track_compatibility);
	Colab_Collection get_best_collaboration_rec(const std::vector<Collaboration_Info>& infos, 
		const size_t& max_tasks, const size_t& max_agents, const Colab_Collection& collection,
		std::vector<Collaboration_Info>::const_iterator it_in,
		const Ingredients& available_ingredients);

	std::optional<Collaboration_Info> get_best_permutation(const Goals& goals, 
		const Paths& paths, const std::vector<std::vector<Agent_Id>>& agent_permutations,
		const State& state);

	std::vector<Collaboration_Info> get_collaboration_permutations(const Goals& goals,
		const Paths& paths, const std::vector<std::vector<Agent_Id>>& agent_permutations,
		const State& state);

	bool is_conflict_in_permutation(const State& initial_state, 
		const std::vector<Joint_Action>& actions);

	Collaboration_Info get_action_from_permutation(const Agent_Combination& best_permutation,
		const std::vector<Recipe>& recipes, const Paths& paths, const Agent_Combination& agents,
		const size_t& best_length);

	std::pair<std::vector<Joint_Action>, std::map<Recipe, Agent_Combination>> get_actions_from_permutation(
		const Goals& goals, const Paths& paths, size_t agent_size, const State& state);

	std::pair<std::vector<Action>, Recipe> get_actions_from_permutation_inner(const Goals& goals,
		const Agent_Id& acting_agent, const Paths& paths, const State& state);

	std::vector<std::set<Temp>> get_agent_handoff_infos( 
		const Agent_Combination& agents, const std::vector<Agent_Id>& agent_permutation, 
		const std::vector<Action_Path>& action_paths);
	std::vector<size_t> calculate_adjusted_agent_handoffs(std::vector<std::set<Temp>>& agents_handoffs);

	size_t get_permutation_length(const Goals& goals, const Paths& paths);

	std::optional<std::vector<Action_Path>> get_permutation_action_paths(const Goals& goals,
		const Paths& paths) const;

	bool is_agent_abused(const Goals& goals, const Paths& paths) const;

	std::map<Goals, float> calculate_goal_values(std::vector<Collaboration_Info>& infos);

	std::vector<Collaboration_Info> calculate_probable_multi_goals(const std::vector<Collaboration_Info>& infos,
		const std::map<Goals, float>& goal_values, const State& state);

	std::vector<Collaboration_Info> calculate_infos(const Paths& paths, const std::vector<Recipe>& recipes_in,
		const State& state);

	void trim_trailing_non_actions(std::vector<Joint_Action>& joint_actions, const Agent_Id& handoff_agent);

	Paths perform_new_search(const State& state, const Goal& goal, const Paths& paths, const std::vector<Joint_Action>& joint_actions, const Agent_Combination& acting_agents);

	bool is_agent_subset_faster(const Collaboration_Info& info, const std::map<Goals, float>& goal_values);

	Recogniser recogniser;
	Search search;
	std::map<Agent_Combination, Reachables> agent_reachables;
	std::map<Recipe_Agents, Solution_History> recipe_solutions;
	size_t time_step;
};