#pragma once
#include <vector>
#include <algorithm>
#include "Environment.hpp"

struct Node {
	Node(size_t state_id, size_t g, size_t h, size_t action_count) : 
		state_id(state_id), g(g), h(h), action_count(action_count){};
		
	size_t state_id;
	size_t g;
	size_t h;
	size_t f() const { return g + h; }
	size_t action_count;
};

struct State_Hash {
	State_Hash(State state, size_t state_id) : state(state), state_id(state_id) {};
	State state;
	size_t state_id;
	bool operator==(const State_Hash& other) const {
		return state == other.state;
	}
};

namespace std {
	template<>
	struct hash<State_Hash>
	{
		size_t
			operator()(const State_Hash& obj) const
		{
			return obj.state.to_hash();
		}
	};
}

struct State_Info {
	State_Info(State state, size_t parent_id, Joint_Action action, size_t g, size_t action_count) : 
		state(state), parent_id(parent_id), action(action), g(g), action_count(action_count) {};
	
	State state;
	size_t parent_id;
	Joint_Action action;
	size_t g;
	size_t action_count;

	bool operator<(const State_Info& other) const {
		if (this->g != other.g) return this->g < other.g;
		if (this->action_count != other.action_count) return this->action_count < other.action_count;
		return false;
	}
};

struct Node_Comparator {
	bool operator()(const Node& lhs, const Node& rhs) const {
		if (lhs.f() != rhs.f()) return lhs.f() > rhs.f();
		if (lhs.g != rhs.g) return lhs.g > rhs.g;
		if (lhs.action_count != rhs.action_count) return lhs.action_count > rhs.action_count;
		return false;
	}
};

#define INFINITE_HEURISTIC 1000
struct Heuristic {
	Heuristic(Environment environment, Ingredient ingredient1, Ingredient ingredient2)
		: environment(environment), ingredient1(ingredient1), ingredient2(ingredient2) {};
	size_t operator()(const State& state) {
		auto locations1 = environment.get_locations(state, ingredient1);
		auto locations2 = environment.get_locations(state, ingredient2);
		//if (locations1.empty()) locations1 = environment.get_recipe_locations(state, ingredient1);
		//if (locations2.empty()) locations2 = environment.get_recipe_locations(state, ingredient2);
		if (locations1.empty() or locations2.empty()) {
			return INFINITE_HEURISTIC;
		}
		size_t min_dist = (size_t)-1;
		for (const auto& location1 : locations1) {
			for (const auto& location2 : locations2) {
				min_dist = std::min(min_dist, (size_t)std::abs((int)location1.first - (int)location2.first) + std::abs((int)location1.second - (int)location2.second));
			}
		}
		return min_dist == (size_t)-1 ? 0 : min_dist;
	}

	Environment environment; 
	Ingredient ingredient1; 
	Ingredient ingredient2;
};

class A_Star {
public:
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, 
		Recipe recipe, const std::vector<Agent_Id>& agents) const;
	
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, 
		Recipe recipe, const std::vector<Agent_Id>& agents, size_t depth_limit) const;
private:
	size_t get_action_cost(const Joint_Action& action) const;
	std::vector<Joint_Action> extract_actions(size_t goal_id, const std::vector<State_Info>& states) const;
};