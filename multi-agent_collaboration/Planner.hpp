#pragma once

#include "Environment.hpp"
#include <vector>
#include <set>

struct Action_Path {

	std::vector<Joint_Action> joint_actions;
	Recipe recipe;
	std::vector<Agent_Id> agents;
	bool operator<(const Action_Path& other) const {
		if (joint_actions.size() != other.joint_actions.size()) return joint_actions.size() < other.joint_actions.size();
		if (agents.size() < other.agents.size()) return agents.size() < other.agents.size();
		return recipe < other.recipe;
	}
};

struct Recipe_Solution {
	std::vector<Agent_Id> agents;
	size_t action_count;
	bool operator>(const Action_Path& other) const {
		if (action_count != other.joint_actions.size()) return action_count > other.joint_actions.size();
		return agents.size() > other.agents.size();
	}
};

class Planner {


public:
	Planner(Environment environment, Agent_Id agent) : agent(agent), environment(environment) {};
	Action get_next_action(const State& state) const;

private:
	std::vector<std::vector<Agent_Id>> get_combinations(size_t n) const;
	std::set<Action_Path> get_all_paths(const std::vector<Recipe>& recipes, const State& state) const;
	Action get_best_action(const std::set<Action_Path>& paths, const std::vector<Recipe>& recipes) const;
	bool agent_in_best_solution(const std::map<Recipe, Recipe_Solution>& best_solutions, const Action_Path& action_path) const;
	bool contains_useful_action(const Action_Path& action_path) const;

	Agent_Id agent;
	Environment environment;
};