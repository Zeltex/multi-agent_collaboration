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
		Recipe recipe,
		Agent_Combination agents,
		Agent_Id main_agent,
		const Coordinate& initial_coordinate,
		const Environment& environment)
		: joint_actions(joint_actions), recipe(recipe), agents(agents),
		first_action(EMPTY_VAL), last_action(EMPTY_VAL) {

		if (joint_actions.empty()) {
			return;
		}
		
		// Stop at first action which interacts with a wall
		first_action = 0;
		auto coordinate = initial_coordinate;
		while (true) {
			coordinate = environment.move_noclip(coordinate, joint_actions.at(first_action).get_action(main_agent).direction);
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
			const auto& direction = joint_actions.at(action_counter).get_action(main_agent).direction;
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

	std::vector<Joint_Action> joint_actions;
	Recipe recipe;
	Agent_Combination agents;
	size_t first_action;	// First useful action by main_agent
	size_t last_action;		// Last useful action by main_agent
};

struct Subtask_Info {
	size_t first_action;
	size_t last_action;
	size_t length;
	Joint_Action next_action;
	Action_Path* actions;
	bool is_valid() const {
		return length != 0 && length != EMPTY_VAL;
	}
	Action action(const Agent_Id& agent) const {
		return next_action.get_action(agent);
	}
	bool has_useful_action(const Agent_Id& agent) const {
		for (const auto& action : actions->joint_actions) {
			if (action.is_action_non_trivial(agent)) {
				return true;
			}
		}
		return false;
	}
};
struct Subtask_Entry {
	Recipe recipe;
	Agent_Combination agents;
	bool operator<(const Subtask_Entry& other) const {
		if (recipe != other.recipe) return recipe < other.recipe;
		if (agents != other.agents) return agents < other.agents;
		return false;
	}
};

struct Paths {
	Paths(size_t number_of_agents) 
		: normal_paths(), handoff_info(number_of_agents), normal_info(number_of_agents), 
		handoff_paths(number_of_agents){};

	void insert(const std::vector<Joint_Action>& actions, Recipe recipe,
		Agent_Combination agents, Agent_Id main_agent,
		const State& state, const Environment& environment) {

		Coordinate agent_coordinate = state.get_agent(main_agent).coordinate;

		Action_Path temp(actions, recipe, agents, main_agent,
			agent_coordinate, environment);
		for (size_t i = 0; i < normal_info.size(); ++i) {
			normal_info.at(i).insert({
				Subtask_Entry{recipe, agents},
				Subtask_Info{temp.first_action, temp.last_action,
					actions.size(), actions.at(0)} });
		}
		normal_paths.insert(temp);
	}
	
	void insert(const std::vector<Joint_Action>& actions, Recipe recipe,
		Agent_Combination agents, Agent_Id main_agent, Agent_Id handoff_agent,
		const State& state, const Environment& environment) {

		Coordinate agent_coordinate = state.get_agent(main_agent).coordinate;

		// TODO - A little inefficient to create an Action_Path just to get last_action
		Action_Path action_path(actions, recipe, agents, handoff_agent, 
			agent_coordinate, environment);
		handoff_paths.at(handoff_agent.id).push_back(action_path);
		Action_Path* path_ptr = &handoff_paths.at(handoff_agent.id).back();

		handoff_info.at(handoff_agent.id).insert({ 
			Subtask_Entry{recipe, agents}, 
			Subtask_Info{action_path.first_action, action_path.last_action, actions.size(), 
				actions.at(0), path_ptr} });

	}

	void update(const std::vector<Joint_Action>& actions, Recipe recipe,
		Agent_Combination agents, Agent_Id main_agent, Agent_Id handoff_agent,
		const State& state, const Environment& environment) {

		// Erase info
		auto& handoff_ref = handoff_info.at(handoff_agent.id);
		Subtask_Entry entry{ recipe, agents };
		auto it = handoff_ref.find(entry);
		if (it != handoff_ref.end()) {
			handoff_ref.erase(it);
		}

		// Erase path
		auto& handoff_path_ref = handoff_paths.at(handoff_agent.id);
		for (auto path = handoff_path_ref.begin(); path != handoff_path_ref.end(); ++path) {
			if (agents == path->agents && recipe == path->recipe) {
				handoff_path_ref.erase(path);
				break;
			}
		}

		// Insert info and path
		insert(actions, recipe, agents, main_agent, handoff_agent, state, environment);
	}
	
	const std::set<Action_Path>& get_normal() const { 
		return normal_paths; 
	}

	std::optional<const Subtask_Info*> get_handoff(Agent_Id agent, const Subtask_Entry& entry) const { 
		auto it = handoff_info.at(agent.id).find(entry); 
		if (it == handoff_info.at(agent.id).end()) {
			return {};
		} else {
			return &(it->second);
		}
	}

	std::optional<const Subtask_Info*> get_normal(Agent_Id agent, const Subtask_Entry& entry) const {
		auto it = normal_info.at(agent.id).find(entry);
		if (it == normal_info.at(agent.id).end()) {
			return {};
		} else {
			return &(it->second);
		}
	}
private:
	std::vector<std::deque<Action_Path>> handoff_paths;
	std::set<Action_Path> normal_paths;
	std::vector<std::map<Subtask_Entry, Subtask_Info>> handoff_info;
	std::vector<std::map<Subtask_Entry, Subtask_Info>> normal_info;
};

struct Collaboration_Info {
	Collaboration_Info() : length(EMPTY_VAL), 
		combination(), recipes(), 
		next_action(), value(EMPTY_VAL), permutation() {};

	Collaboration_Info(size_t length, Agent_Combination combination, const std::vector<Recipe> recipes,
		Action next_action, Agent_Combination permutation)
		: length(length), combination(combination), recipes(recipes), 
			next_action(next_action), value(EMPTY_VAL), permutation(permutation){};

	std::string to_string() const {
		std::string result;
		for (const auto& recipe : recipes) {
			result += recipe.result_char();
		}
		result += ":" + combination.to_string_raw() 
			+ ":" + permutation.to_string_raw();
		
		return result;
	}

	bool has_value() const {
		return length != EMPTY_VAL;
	}

	size_t length;
	//size_t last_action;
	Agent_Combination combination;
	Agent_Combination permutation;
	std::vector<Recipe> recipes;
	Action next_action;

	float value;
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
		tasks += info.recipes.size();
		infos.push_back(info);
		for (const auto& agent : info.combination.get()) {
			agents.insert(agent);
		}
	}

	bool has_value() const {
		return tasks != 0;
	}

	bool contains(const Agent_Combination& agent_combination) const {
		for (const auto& agent : agent_combination.get()) {
			for (const auto& info : infos) {
				if (info.combination.contains(agent)) {
					return true;
				}
			}
		}
		return false;
	}

	bool contains(const std::vector<Recipe>& recipes_in) const {
		for (const auto& recipe_in : recipes_in) {
			for (const auto& info : infos) {
				for (const auto& recipe : info.recipes) {
					if (recipe_in.result == recipe.result) {
						return true;
					}
				}
			}
		}
		return false;
	}

	bool contains(const Recipe& recipe_in) const {
		for (const auto& info : infos) {
			for (const auto& recipe : info.recipes) {
				if (recipe_in.result == recipe.result) {
					return true;
				}
			}
		}
		return false;
	}

	void calculate_value(size_t max_agents) {
		std::vector<float> values(max_agents, 0);
		for (const auto& info : infos) {
			for (const auto& agent : info.combination.get()) {
				//values.at(agent.id) += info.value * info.recipes.size();
				values.at(agent.id) += info.value;
			}
		}
		value = 0;
		for (const auto& entry_value : values) {
			value = std::max(value, entry_value);
		}
	}

	bool is_compatible(const std::vector<Recipe>& recipes_in, 
		const std::map<Ingredient, size_t>& available_ingredients,
		const Environment& environment) const {
		
		std::map<Ingredient, size_t> recipe_ingredients;
		Recipes recipes;

		for (const auto& info : infos) {
			recipes.get_ingredient_counts(recipe_ingredients, info.recipes);
		}
		recipes.get_ingredient_counts(recipe_ingredients, recipes_in);

		for (const auto& [ingredient, count] : recipe_ingredients) {
			if (environment.is_type_stationary(ingredient)) continue;

			auto it = available_ingredients.find(ingredient);
			if (it == available_ingredients.end()){ 
				if (count > 0) return false;
			} else {
				if (count > it->second) return false;
			}
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
	virtual Action get_next_action(const State& state) override;

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
		const size_t& max_tasks, const State& state);
	Colab_Collection get_best_collaboration_rec(const std::vector<Collaboration_Info>& infos, 
		const size_t& max_tasks, const size_t& max_agents, Colab_Collection collection,
		std::vector<Collaboration_Info>::const_iterator it_in,
		const std::map<Ingredient, size_t>& available_ingredients);

	std::optional<Collaboration_Info> get_best_permutation(const Agent_Combination& agents, 
		const std::vector<Recipe>& recipes, const Paths& paths,
		const std::vector<std::vector<Agent_Id>>& agent_permutations,
		const State& state);

	bool is_conflict_in_permutation(const State& initial_state, 
		const std::vector<Joint_Action>& actions);

	Collaboration_Info get_action_from_permutation(const Agent_Combination& best_permutation,
		const std::vector<Recipe>& recipes, const Paths& paths, const Agent_Combination& agents,
		const size_t& best_length);

	std::pair<std::vector<Joint_Action>, std::map<Recipe, Agent_Combination>> get_actions_from_permutation(
		const Agent_Combination& permutation, const std::vector<Action_Path>& action_paths, size_t agent_size,
		const State& state);

	std::pair<std::vector<Action>, Recipe> get_actions_from_permutation_inner(
		const Agent_Combination& permutation, const Agent_Id& acting_agent,
		const std::vector<Action_Path>& action_paths, size_t action_trace_length,
		const State& state);

	std::vector<std::set<Temp>> get_agent_handoff_infos( 
		const Agent_Combination& agents, const std::vector<Agent_Id>& agent_permutation, 
		const std::vector<Action_Path>& action_paths);
	std::vector<size_t> calculate_adjusted_agent_handoffs(std::vector<std::set<Temp>>& agents_handoffs);

	size_t get_permutation_length(const Agent_Combination& agents,
		const std::vector<Agent_Id>& agent_permutation, const std::vector<Action_Path>& action_paths);

	std::optional<std::vector<Action_Path>> get_permutation_action_paths(const Agent_Combination& agents,
		const std::vector<Recipe>& recipes, const Paths& paths, const std::vector<Agent_Id>& agent_permutation) const;

	bool is_agent_abused(const std::vector<Agent_Id>& agent_permutation, const std::vector<Action_Path>& action_paths) const;

	Recogniser recogniser;
	Search search;
	std::map<Agent_Combination, Reachables> agent_reachables;
	std::map<Recipe_Agents, Solution_History> recipe_solutions;
	size_t time_step;
};