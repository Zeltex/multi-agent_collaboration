#include "Sliding_Recogniser.hpp"

Sliding_Recogniser::Sliding_Recogniser(const Environment& environment, const State& initial_state)
	: Recogniser_Method(environment, initial_state) {};


void Sliding_Recogniser::update(const std::vector<Goal_Length>& goal_lengths) {
	throw std::runtime_error("Not implemented");
}

Goal Sliding_Recogniser::get_goal(Agent_Id agent) {
	throw std::runtime_error("Not implemented");
}

std::map<Agent_Id, Goal> Sliding_Recogniser::get_goals() {
	throw std::runtime_error("Not implemented");
}