#pragma once

#include "Environment.hpp"
#include "Core.hpp"
#include "Search.hpp"
#include "State.hpp"

#include <vector>
#include <set>
#include <algorithm> 

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
		if (agents.size() < other.agents.size()) return agents.size() < other.agents.size();
		return recipe < other.recipe;
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
		return ((int)excl_length - incl_length) / 2 > incl_last_action;
	}

	float get_usefulness() const {
		return ((int)excl_length - incl_length) / 2;
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

class Planner {


public:
	Planner(Environment environment, Agent_Id agent, const State& initial_state);
	Action get_next_action(const State& state);

private:
	std::set<Action_Path> get_all_paths(const std::vector<Recipe>& recipes, const State& state);
	Action get_best_action(const std::set<Action_Path>& paths, const std::vector<Recipe>& recipes) const;
	bool agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, const Action_Path& action_path) const;
	bool ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const;
	void initialize_reachables(const State& initial_state);
	void initialize_solutions();

	void update_recipe_solutions(const std::set<Action_Path>& paths);
	void recognize_goals();

	Search search;
	Agent_Id agent;
	Environment environment;
	std::map<Agent_Combination, Reachables> agent_reachables;
	std::map<Recipe_Agents, Solution_History> recipe_solutions;
	size_t time_step;
};