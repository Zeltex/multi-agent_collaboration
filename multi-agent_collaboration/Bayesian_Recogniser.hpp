#pragma once

#include "Recogniser.hpp"

class Bayesian_Recogniser : public Recogniser_Method {
public:
	Bayesian_Recogniser(const Environment& environment, const State& initial_state);
	void update(const std::map<Goal, size_t>& goal_lengths) override;
	Goal get_goal(Agent_Id agent) override;
	std::map<Agent_Id, Goal> get_goals() const override;
	void print_probabilities() const override;
private:
};