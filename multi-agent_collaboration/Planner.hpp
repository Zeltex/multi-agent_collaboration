#pragma once

#include "Environment.hpp"

class Planner {
public:
	Planner(Environment environment, Agent_Id agent) : agent(agent), environment(environment) {};
	Action get_next_action(const State& state) const;

private:
	Agent_Id agent;
	Environment environment;
};