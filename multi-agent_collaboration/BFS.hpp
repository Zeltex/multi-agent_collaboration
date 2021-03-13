#pragma once
#include <vector>
#include "Environment.hpp"
#include "Search.hpp"



class BFS : public Search_Method {
public:
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, 
		Recipe recipe, const std::vector<Agent_Id>& agents) const override;
	
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, 
		Recipe recipe, const std::vector<Agent_Id>& agents, size_t depth_limit) const override;
private:
};