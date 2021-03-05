#pragma once
#include <vector>
#include <algorithm>
#include "Environment.hpp"

struct Node {
	Node(size_t state_id, size_t g, size_t h) : state_id(state_id), g(g), h(h) {};
	size_t state_id;
	size_t g;
	size_t h;
	size_t f() const { return g + h; }
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
	State_Info(State state, size_t parent_id, Joint_Action action, size_t g) : state(state), parent_id(parent_id), action(action), g(g) {};
	State state;
	size_t parent_id;
	Joint_Action action;
	size_t g;
};

struct Node_Comparator {
	bool operator()(const Node& lhs, const Node& rhs) const {
		return lhs.f() > rhs.f() || (lhs.f() == rhs.f() && lhs.g > rhs.g);
	}
};

struct Heuristic {
	Heuristic(Environment environment, Ingredient ingredient1, Ingredient ingredient2)
		: environment(environment), ingredient1(ingredient1), ingredient2(ingredient2) {};
	size_t operator()(const State& state) {
		auto locations1 = environment.get_locations(state, ingredient1);
		auto locations2 = environment.get_locations(state, ingredient2);
		size_t min_dist = (size_t)-1;
		for (const auto& location1 : locations1) {
			for (const auto& location2 : locations2) {
				min_dist = std::min(min_dist, (size_t)std::abs((int)location1.first - (int)location2.first) + std::abs((int)location1.second - (int)location2.second));
			}
		}
		return min_dist;
	}

	Environment environment; 
	Ingredient ingredient1; 
	Ingredient ingredient2;
};

class A_Star {
public:
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, Recipe recipe) const;
private:
};