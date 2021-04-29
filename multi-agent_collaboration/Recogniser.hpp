#pragma once

#include <memory>
#include <algorithm>

#include "Environment.hpp"
#include "State.hpp"

constexpr auto EMPTY_RECIPE = Recipe{ Ingredient::DELIVERY, Ingredient::DELIVERY, Ingredient::DELIVERY };
constexpr auto EMPTY_PROB = 0.0f;
struct Goal_Length {
	Agent_Combination agents;
	Recipe recipe;
	size_t length;
};

struct Multi_Goal {
	Multi_Goal(const Agent_Combination& agents, const std::vector<Recipe>& recipes) 
		: agents(agents), recipes(recipes) {};
	Multi_Goal() : agents(), recipes() {};
	bool operator<(const Multi_Goal& other) const {
		if (agents != other.agents) return agents < other.agents;
		if (recipes.size() != other.recipes.size()) return recipes.size() < other.recipes.size();
		auto recipes1 = recipes;
		auto recipes2 = other.recipes;
		std::sort(recipes1.begin(), recipes1.end());
		std::sort(recipes2.begin(), recipes2.end());
		for (size_t i = 0; i < recipes.size(); ++i) {
			if (recipes1.at(i) != recipes2.at(i)) return recipes1.at(i) < recipes2.at(i);
		}
		return false;
	}
	Agent_Combination agents;
	std::vector<Recipe> recipes;
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
	virtual std::map<Goal, float> get_raw_goals() const = 0;
	virtual bool is_probable(Goal goal) const = 0;
	virtual bool is_probable_normalised(Goal goal, const std::vector<Goal>& available_goals, Agent_Id agent) const = 0;
	virtual void print_probabilities() const = 0;
	virtual float get_probability(const Goal& goal) const = 0;

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
	
	// Returns single most probable goal for each agent
	std::map<Agent_Id, Goal> get_goals() const { 
		return recogniser_method->get_goals(); 
	}

	std::map<Goal, float> get_raw_goals() const {
		return recogniser_method->get_raw_goals();
	}

	bool is_probable(Goal goal) const {
		return recogniser_method->is_probable(goal);
	}

	virtual bool is_probable_normalised(Goal goal, const std::vector<Goal>& available_goals, Agent_Id agent) const {
		return recogniser_method->is_probable_normalised(goal, available_goals, agent);
	}

	void print_probabilities() const {
		recogniser_method->print_probabilities();
	}

	float get_probability(const Goal& goal) const {
		return recogniser_method->get_probability(goal);
	}


	
private:
	std::unique_ptr<Recogniser_Method> recogniser_method;
};