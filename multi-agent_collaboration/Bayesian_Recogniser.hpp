#pragma once

#include "Recogniser.hpp"

class Bayesian_Recogniser : public Recogniser_Method {
	Bayesian_Recogniser(const Environment& environment, const State& initial_state);
	void update(const std::vector<Goal_Length>& goal_lengths) override;
	Goal get_goal(Agent_Id agent) override;
	std::map<Agent_Id, Goal> get_goals() override;
public:
private:
};