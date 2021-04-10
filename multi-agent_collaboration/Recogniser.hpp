#pragma once

#include <memory>

#include "Environment.hpp"
#include "State.hpp"

constexpr auto EMPTY_RECIPE = Recipe{ Ingredient::DELIVERY, Ingredient::DELIVERY, Ingredient::DELIVERY };
constexpr auto EMPTY_PROB = 0.0f;
struct Goal_Length {
	Agent_Combination agents;
	Recipe recipe;
	size_t length;
};

struct Goal {
	Goal(const Agent_Combination& agents, const Recipe& recipe) : agents(agents), recipe(recipe) {};
	Goal() : agents(), recipe(EMPTY_RECIPE) {};
	bool operator<(const Goal& other) const {
		if (agents != other.agents) return agents < other.agents;
		return recipe < other.recipe;
	}
	Agent_Combination agents;
	Recipe recipe;
};

class Recogniser_Method {
public:
	Recogniser_Method(const Environment& environment, const State& initial_state) 
		: environment(environment), state(initial_state) {}

	virtual void update(const std::vector<Goal_Length>& goal_lengths) = 0;
	virtual Goal get_goal(Agent_Id agent) = 0;
	virtual std::map<Agent_Id, Goal> get_goals() const = 0;
	virtual void print_probabilities() const = 0;

protected:
	Environment environment;
	State state;
};

class Recogniser {
public:
	// TODO - Probably delete copy constructor and more
	Recogniser(std::unique_ptr<Recogniser_Method> recogniser_method) 
		: recogniser_method(std::move(recogniser_method)) {};
	
	void update(const std::vector<Goal_Length>& goal_lengths) { 
		recogniser_method->update(goal_lengths); 
	}
	
	Goal get_goal(Agent_Id agent) { 
		return recogniser_method->get_goal(agent); 
	}
	
	std::map<Agent_Id, Goal> get_goals() const { 
		return recogniser_method->get_goals(); 
	}

	void print_probabilities() const {
		recogniser_method->print_probabilities();
	}
	
private:
	std::unique_ptr<Recogniser_Method> recogniser_method;
};