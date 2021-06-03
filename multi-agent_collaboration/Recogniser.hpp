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
	Agent_Id handoff_agent;
	size_t length;
};

//struct Multi_Goal {
//	Multi_Goal(const Agent_Combination& agents, const std::vector<Recipe>& recipes) 
//		: agents(agents), recipes(recipes) {};
//
//	Multi_Goal(const Goal& goal) : agents(goal.agents), recipes(1, goal.recipe) {};
//
//	Multi_Goal() : agents(), recipes() {};
//
//	bool operator<(const Multi_Goal& other) const {
//		if (agents != other.agents) return agents < other.agents;
//		if (recipes.size() != other.recipes.size()) return recipes.size() < other.recipes.size();
//		auto recipes1 = recipes;
//		auto recipes2 = other.recipes;
//		std::sort(recipes1.begin(), recipes1.end());
//		std::sort(recipes2.begin(), recipes2.end());
//		for (size_t i = 0; i < recipes.size(); ++i) {
//			if (recipes1.at(i) != recipes2.at(i)) return recipes1.at(i) < recipes2.at(i);
//		}
//		return false;
//	}
//	Agent_Combination agents;
//	std::vector<Recipe> recipes;
//};

struct Goal {
	Goal(const Agent_Combination& agents, const Recipe& recipe, const Agent_Id& handoff_agent)
		: agents(agents), recipe(recipe), handoff_agent(handoff_agent) {};

	Goal(const Agent_Id& agent, const Recipe& recipe, const Agent_Id& handoff_agent)
		: agents(Agent_Combination{ agent }), recipe(recipe), handoff_agent(handoff_agent) {};

	Goal() : agents(), recipe(EMPTY_RECIPE), handoff_agent(EMPTY_VAL) {};

	bool operator<(const Goal& other) const {
		if (agents != other.agents) return agents < other.agents;
		if (handoff_agent != other.handoff_agent) return handoff_agent < other.handoff_agent;
		return recipe < other.recipe;
	}

	bool operator==(const Goal& other) const {
		if (*this < other) return false;
		if (other < *this) return false;
		return true;
	}

	bool operator!=(const Goal& other) const {
		return !(*this == other);
	}

	bool has_value() const {
		return !(agents.empty() && recipe == EMPTY_RECIPE);
	}

	Agent_Combination agents;
	Recipe recipe;
	Agent_Id handoff_agent;
};

struct Goals {

	Goals() : goals(), same_agents(true) {};
	Goals(const Goal& goal) : goals(1, goal), same_agents(true) {};
	Goals(const Goals& other, const Agent_Combination& new_agents)
		: goals(other.goals), same_agents(true) {

		for (auto& goal : goals) {
			goal.agents = new_agents;
		}
	}

	void update_handoffs(const Agent_Combination& handoff_agents) {
		assert(handoff_agents.size() == goals.size());
		size_t goals_size = goals.size();
		for (size_t i = 0; i < goals_size; ++i) {
			goals.at(i).handoff_agent = handoff_agents.get(i);
		}
	}

	bool operator<(const Goals& other) const {
		if (goals.size() != other.goals.size()) return goals.size() < other.goals.size();
		for (auto it1 = goals.begin(), it2 = other.goals.begin(); it1 != goals.end(); ++it1, ++it2) {
			if (*it1 != *it2) return *it1 < *it2;
		}
		return false;
	}

	void add(const Goal& goal) {
		if (!goals.empty()
			&& goals.begin()->agents != goal.agents) {
			same_agents = false;
		}
		goals.push_back(goal);
		std::sort(goals.begin(), goals.end());
	}

	const Agent_Combination& get_agents() const {
		assert(same_agents);
		if (goals.empty()) {
			return {};
		} else {
			return goals.at(0).agents;
		}
	}

	size_t size() const {
		return goals.size();
	}

	const std::vector<Goal>& get_iterable() const {
		return goals;
	}

	std::vector<Goal>::const_iterator begin() const {
		return goals.begin();
	}
	std::vector<Goal>::const_iterator end() const {
		return goals.end();
	}
	bool has_same_agents() const {
		return same_agents;
	}
	std::string get_handoff_string() const {
		std::string result;
		for (const auto& goal : goals) {
			result += goal.handoff_agent.to_string();
		}
		return result;
	}

	void clear_all_handoff_indices() {
		for (auto& goal : goals) {
			goal.handoff_agent = EMPTY_VAL;
		}
	}

	std::vector<Recipe> get_recipes() const {
		std::vector<Recipe> recipes;
		for (const auto& goal : goals) {
			recipes.push_back(goal.recipe);
		}
		return recipes;
	}

private:
	bool same_agents;		 // If all goals contain same agents
	std::vector<Goal> goals;
};

class Recogniser_Method {
public:
	Recogniser_Method(const Environment& environment, const State& initial_state) 
		: environment(environment), state(initial_state) {}

	virtual void update(const std::map<Goal, size_t>& goal_lengths, const State& state) = 0;
	virtual Goal get_goal(Agent_Id agent) = 0;
	virtual std::map<Agent_Id, Goal> get_goals() const = 0;
	virtual std::map<Goal, float> get_raw_goals() const = 0;
	virtual bool is_probable(Goal goal) const = 0;
	virtual bool is_probable_normalised(Goal goal, const std::vector<Goal>& available_goals, Agent_Id agent, Agent_Id planning_agent, bool use_non_probability) const = 0;
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
	
	void update(const std::map<Goal, size_t>& goal_lengths, const State& state) {
		recogniser_method->update(goal_lengths, state); 
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

	virtual bool is_probable_normalised(Goal goal, const std::vector<Goal>& available_goals, Agent_Id agent, Agent_Id planning_agent, bool use_non_probability) const {
		return recogniser_method->is_probable_normalised(goal, available_goals, agent, planning_agent, use_non_probability);
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