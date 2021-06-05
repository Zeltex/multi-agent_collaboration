#pragma once
#include <vector>
#include "Environment.hpp"
#include "Search.hpp"



class BFS : public Search_Method {
public:
	using Search_Method::Search_Method;
	std::vector<Joint_Action> search_joint(const State& state, 
		Recipe recipe, const Agent_Combination& agents, 
		Agent_Id handoff_agent,
		const std::vector<Joint_Action>& input_actions, 
		const Agent_Combination& free_agents, const Action& initial_action) override;
	std::pair<size_t, Direction> get_dist_direction(Coordinate source, Coordinate dest, size_t walls) override;
private:
};