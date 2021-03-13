#pragma once

#include "Environment.hpp"
#include <vector>
#include <set>

struct Action_Path {

	std::vector<Joint_Action> joint_actions;
	Recipe recipe;
	Agent_Combination agents;
	bool operator<(const Action_Path& other) const {
		if (joint_actions.size() != other.joint_actions.size()) return joint_actions.size() < other.joint_actions.size();
		if (agents.size() < other.agents.size()) return agents.size() < other.agents.size();
		return recipe < other.recipe;
	}
};

struct Recipe_Solution {
	Agent_Combination agents;
	size_t action_count;
	bool operator>(const Action_Path& other) const {
		if (action_count != other.joint_actions.size()) return action_count > other.joint_actions.size();
		return agents.size() > other.agents.size();
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

class Planner {


public:
	Planner(Environment environment, Agent_Id agent, const State& initial_state) : agent(agent), environment(environment) {
		intialize_reachables(initial_state);
	};
	Action get_next_action(const State& state) const;

private:
	std::vector<Agent_Combination> get_combinations(size_t n) const;
	std::set<Action_Path> get_all_paths(const std::vector<Recipe>& recipes, const State& state) const;
	Action get_best_action(const std::set<Action_Path>& paths, const std::vector<Recipe>& recipes) const;
	bool agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, const Action_Path& action_path) const;
	bool contains_useful_action(const Action_Path& action_path) const;
	bool ingredients_reachable(const Recipe& recipe, const Agent_Combination& agents, const State& state) const;
	void intialize_reachables(const State& initial_state);

	Agent_Id agent;
	Environment environment;
	std::map<Agent_Combination, Reachables> agent_reachables;
};