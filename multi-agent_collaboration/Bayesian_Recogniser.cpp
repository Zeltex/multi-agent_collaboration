#include "Bayesian_Recogniser.hpp"

Bayesian_Recogniser::Bayesian_Recogniser(const Environment& environment, const State& initial_state)
	: Recogniser_Method(environment, initial_state) {};


void Bayesian_Recogniser::update(const std::map<Goal, size_t>& goal_lengths) {
	throw std::runtime_error("Not implemented");
}

Goal Bayesian_Recogniser::get_goal(Agent_Id agent) {
	throw std::runtime_error("Not implemented");
}

std::map<Agent_Id, Goal> Bayesian_Recogniser::get_goals() const {
	throw std::runtime_error("Not implemented");
}


void Bayesian_Recogniser::print_probabilities() const {
	throw std::runtime_error("Not implemented");
}