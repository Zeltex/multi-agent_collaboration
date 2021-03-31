#pragma once
#include <vector>
#include "Environment.hpp"
#include "State.hpp"


class Search_Trimmer {
public:
	/*
	Transforms an agents remaining actions into no-ops as early as possible
	*/
	void trim(std::vector<Joint_Action>& actions, const State& state, const Environment& environment, const Recipe& recipe) const;
private:
	std::vector<Joint_Action> apply_modified_actions(size_t action_index, size_t agent, State& current_state, const Environment& environment, const std::vector<Joint_Action>& actions) const;
};