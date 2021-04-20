#pragma once

#include "Environment.hpp"
#include "Core.hpp"
#include "Search.hpp"
#include "State.hpp"
#include "Recogniser.hpp"

#include <vector>
#include <set>
#include <algorithm> 
#include <deque>

struct Action_Path {
	Action_Path(std::vector<Joint_Action> joint_actions,
		Recipe recipe,
		Agent_Combination agents,
		Agent_Id main_agent)
		: joint_actions(joint_actions), recipe(recipe), agents(agents),
		first_action(EMPTY_VAL), last_action(EMPTY_VAL) {

		if (joint_actions.empty()) {
			return;
		}
		
		first_action = 0;
		while (!joint_actions.at(first_action).is_action_useful(main_agent)) {
			if (first_action == joint_actions.size() - 1) {
				first_action = EMPTY_VAL;
				break;
			}
			++first_action;
		}

		last_action = joint_actions.size() - 1;
		while (!joint_actions.at(last_action).is_action_useful(main_agent)) {
			if (last_action == 0) {
				last_action = EMPTY_VAL;
				break;
			}
			--last_action;
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
		Agent_Combination agents, Agent_Id main_agent) {
		Action_Path temp(actions, recipe, agents, main_agent);
		for (size_t i = 0; i < normal_info.size(); ++i) {
			normal_info.at(i).insert({
				Subtask_Entry{recipe, agents},
				Subtask_Info{temp.first_action, temp.last_action,
					actions.size(), actions.at(0)} });
		}
		normal_paths.insert(temp);
	}
	
	void insert(const std::vector<Joint_Action>& actions, Recipe recipe,
		Agent_Combination agents, Agent_Id main_agent, Agent_Id handoff_agent) {
		// TODO - A little inefficient to create an Action_Path just to get last_action
		Action_Path action_path(actions, recipe, agents, handoff_agent);
		handoff_paths.at(handoff_agent.id).push_back(action_path);
		Action_Path* path_ptr = &handoff_paths.at(handoff_agent.id).back();

		handoff_info.at(handoff_agent.id).insert({ 
			Subtask_Entry{recipe, agents}, 
			Subtask_Info{action_path.first_action, action_path.last_action, actions.size(), 
				actions.at(0), path_ptr} });

		Action_Path temp(actions, recipe, agents, main_agent);
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
	Collaboration_Info() : length(EMPTY_VAL), last_action(EMPTY_VAL), combination(), recipes(), 
		next_action(), value(EMPTY_VAL) {};

	Collaboration_Info(size_t length, size_t last_action, Agent_Combination combination, const std::vector<Recipe> recipes,
		Action next_action)
		: length(length), last_action(last_action), combination(combination), recipes(recipes), 
			next_action(next_action), value(EMPTY_VAL) {};

	std::string to_string() const {
		std::string result;
		for (const auto& recipe : recipes) {
			result += recipe.result_char();
		}
		result += combination.to_string();
		return result;
	}

	bool has_value() const {
		return length != EMPTY_VAL;
	}

	size_t length;
	size_t last_action;
	Agent_Combination combination;
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
				values.at(agent.id) += info.value * info.recipes.size();
			}
		}
		value = 0;
		for (const auto& entry_value : values) {
			value = std::max(value, entry_value);
		}
	}

	std::vector<Collaboration_Info> infos;
	size_t tasks;
	float value;
};

class Planner {


public:
	Planner(Environment environment, Agent_Id agent, const State& initial_state);
	Action get_next_action(const State& state);

private:
	Paths get_all_paths(const std::vector<Recipe>& recipes, const State& state);
	Action get_best_action(const Paths& paths, const std::vector<Recipe>& recipes) const;
	bool agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, const Action_Path& action_path) const;
	bool ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const;
	void initialize_reachables(const State& initial_state);
	void initialize_solutions();
	std::optional<Collaboration_Info> check_for_collaboration(const Paths& paths, 
		const std::vector<Recipe>& recipes, const std::map<Agent_Id, Goal>& goals, 
		const State& state);

	void update_recogniser(const Paths& paths);

	Collaboration_Info get_best_collaboration(const std::vector<Collaboration_Info>& infos, 
		const size_t& max_tasks);
	Colab_Collection get_best_collaboration_rec(const std::vector<Collaboration_Info>& infos, 
		const size_t& max_tasks, Colab_Collection collection,
		std::vector<Collaboration_Info>::const_iterator it_in);

	std::pair<size_t, Agent_Combination> get_best_permutation(const Agent_Combination& agents, 
		const std::vector<Recipe>& recipes, const Paths& paths,
		const std::vector<std::vector<Agent_Id>>& agent_permutations,
		const State& state);

	bool is_conflict_in_permutation(const Agent_Combination& best_permutation,
		const std::vector<Recipe>& recipes, const Paths& paths, const Agent_Combination& agents,
		const State& state);
	Collaboration_Info get_action_from_permutation(const Agent_Combination& best_permutation,
		const std::vector<Recipe>& recipes, const Paths& paths, const Agent_Combination& agents,
		const size_t& best_length);
	std::pair<size_t, const Subtask_Info*> get_actions_from_permutation_inner(
		const Agent_Combination& best_permutation, const std::vector<Recipe>& recipes, 
		const Paths& paths, const Agent_Combination& agents, const Agent_Id& acting_agent);

	Recogniser recogniser;
	Search search;
	Agent_Id planning_agent;
	Environment environment;
	std::map<Agent_Combination, Reachables> agent_reachables;
	std::map<Recipe_Agents, Solution_History> recipe_solutions;
	size_t time_step;
};