#pragma once

#include <memory>

#include "Environment.hpp"
#include "State.hpp"

struct Goal_Length {
	Agent_Combination agents;
	Recipe recipe;
	size_t length;
};

struct Goal {
	Agent_Combination agents;
	Recipe recipe;
};

class Recogniser_Method {
public:
	Recogniser_Method(const Environment& environment, const State& initial_state) 
		: environment(environment), state(initial_state) {}

	virtual void update(const std::vector<Goal_Length>& goal_lengths) = 0;
	virtual Goal get_goal(Agent_Id agent) = 0;
	virtual std::map<Agent_Id, Goal> get_goals() = 0;

protected:
	Environment environment;
	State state;
};

class Recogniser {
public:
	// TODO - Probably delete copy constructor and more
	Recogniser(std::unique_ptr<Recogniser_Method> recogniser_method) 
		: recogniser_method(std::move(recogniser_method)) {};
	
private:
	std::unique_ptr<Recogniser_Method> recogniser_method;
};