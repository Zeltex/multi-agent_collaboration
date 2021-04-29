#pragma once
#include "State.hpp"
#include "Planner.hpp"


class Planner_Still : public Planner_Impl {
public:
	Planner_Still(Environment environment, Agent_Id agent, const State& initial_state)
		:Planner_Impl(environment, agent) {}

	virtual Action get_next_action(const State& state) override {
		return { Direction::NONE, planning_agent };
	}

private:
};