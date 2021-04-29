#pragma once

#include "Environment.hpp"

enum class Planner_Types {
	MAC,
	STILL
};

class Planner_Impl {
public:
	Planner_Impl(Environment environment, Agent_Id planning_agent)
		: environment(environment), planning_agent(planning_agent) {}
	virtual Action get_next_action(const State& state) = 0;
protected:
	Environment environment;
	Agent_Id planning_agent;
};

class Planner {
public:
	// TODO - Probably delete copy constructor and more
	Planner(std::unique_ptr<Planner_Impl> planner_impl) : planner_impl(std::move(planner_impl)) {};

	Action get_next_action(const State& state) {
		return planner_impl->get_next_action(state);
	}

private:
	std::unique_ptr<Planner_Impl> planner_impl;
};