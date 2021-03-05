#pragma once
#include <vector>
#include "Environment.hpp"



class BFS {
public:
	std::vector<Action> search(const State& state, const Environment& environment, Ingredient goal, Agent_Id agent) const;
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, Ingredient goal) const;
private:
};