#pragma once

#include "Environment.hpp"

class Planner {
public:
	Planner(Environment environment, Agent_Id agent) : agent(agent), environment(environment) {};
	Joint_Action get_next_action(const State& state);

private:
	Agent_Id agent;
	Environment environment;
};