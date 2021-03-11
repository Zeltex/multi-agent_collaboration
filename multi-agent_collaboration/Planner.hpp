#pragma once

#include "Environment.hpp"
#include <vector>

class Planner {
public:
	Planner(Environment environment, Agent_Id agent) : agent(agent), environment(environment) {};
	Action get_next_action(const State& state) const;

private:
	std::vector<std::vector<Agent_Id>> get_combinations(size_t n) const;

	Agent_Id agent;
	Environment environment;
};