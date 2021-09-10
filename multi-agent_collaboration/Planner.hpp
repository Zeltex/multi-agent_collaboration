#pragma once

#include "Environment.hpp"

enum class Planner_Types {
	MAC='m',
	MAC_ONE='n',
	STILL='s'
};

class Planner_Impl {
public:
	Planner_Impl(Environment environment, Agent_Id planning_agent)
		: environment(environment), planning_agent(planning_agent) {}
	virtual Action get_next_action(const State& state, bool print_state) = 0;
protected:
	Environment environment;
	Agent_Id planning_agent;
};

class Planner {
public:
	// TODO - Probably delete copy constructor and more
	Planner(std::unique_ptr<Planner_Impl> planner_impl) : planner_impl(std::move(planner_impl)) {};

	Action get_next_action(const State& state, bool print_state = false) {
		return planner_impl->get_next_action(state, print_state);
	}

private:
	std::unique_ptr<Planner_Impl> planner_impl;
};