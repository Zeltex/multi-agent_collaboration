#pragma once
#include <vector>
#include "Environment.hpp"
#include "Search.hpp"



class BFS : public Search_Method {
public:
	using Search_Method::Search_Method;
	std::vector<Joint_Action> search_joint(const State& state, 
		Recipe recipe, const Agent_Combination& agents, 
		std::optional<Agent_Id> handoff_agent) override;
private:
};