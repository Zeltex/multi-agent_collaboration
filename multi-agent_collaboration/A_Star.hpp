#pragma once
#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_set>

#include "Environment.hpp"
#include "Search.hpp"

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
	State_Hash(State state, size_t state_id, bool has_passed) 
		: state(state), state_id(state_id), has_passed(has_passed) {};
	State state;
	bool has_passed;
	size_t state_id;
	bool operator==(const State_Hash& other) const {
		if (has_passed != other.has_passed) return false;
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
	State_Info() : state(), parent_id(EMPTY_VAL), action(), g(EMPTY_VAL),
		action_count(EMPTY_VAL), has_agent_passed(true) {};

	State_Info(State state, size_t parent_id, Joint_Action action, size_t g, 
		size_t action_count, bool has_agent_passed) 
		: state(state), parent_id(parent_id), action(action), g(g), 
		action_count(action_count), has_agent_passed(has_agent_passed){};
	
	State state;
	size_t parent_id;
	Joint_Action action;
	size_t g;
	size_t action_count;
	bool has_agent_passed;

	bool operator<(const State_Info& other) const {
		if (this->g != other.g) return this->g < other.g;
		if (this->action_count != other.action_count) return this->action_count < other.action_count;
		return false;
	}

	void update(size_t g, size_t action_count, size_t parent_id, const Joint_Action& action) {
		this->g = g;
		this->action_count = action_count;
		this->parent_id = parent_id;
		this->action = action;
	}
	void update(const State_Info& other) {
		this->g = other.g;
		this->action_count = other.action_count;
		this->parent_id = other.parent_id;
		this->action = other.action;
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

// Could optimise some by calculating distance from all ingredients to nearest agent
// and using this in combination with the location1/location2 nested for loops
#define INFINITE_HEURISTIC 1000
struct Heuristic {
	Heuristic(Environment environment, Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents)
		: environment(environment), ingredient1(ingredient1), ingredient2(ingredient2),  agents(agents) {};

	size_t euclidean(Coordinate location1, Coordinate location2) const {
		return (size_t) (std::abs((int)location1.first - (int)location2.first)
			+ std::abs((int)location1.second - (int)location2.second));
	}
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
				min_dist = std::min(min_dist, euclidean(location1, location2));
			}
		}
		size_t min_agent_dist = (size_t)-1;
		if (!environment.is_type_stationary(ingredient1)) {
			for (const auto& location1 : locations1) {
				for (const auto& agent_id : agents.get()) {
					const auto& agent = state.agents.at(agent_id.id);
					min_agent_dist = std::min(min_agent_dist, euclidean(location1, agent.coordinate));
				}
			}
		}
		if (!environment.is_type_stationary(ingredient2)) {
			for (const auto& location2 : locations2) {
				for (const auto& agent_id : agents.get()) {
					const auto& agent = state.agents.at(agent_id.id);
					min_agent_dist = std::min(min_agent_dist, euclidean(location2, agent.coordinate));
				}
			}
		}
		if (min_dist == (size_t)-1) min_dist = 0;
		if (min_agent_dist == (size_t)-1) min_agent_dist = 0;


		return min_dist + min_agent_dist;
	}

	Environment environment; 
	Ingredient ingredient1; 
	Ingredient ingredient2;
	Agent_Combination agents;
};

using Node_Queue = std::priority_queue<Node, std::vector<Node>, Node_Comparator>;

class A_Star : public Search_Method {
public:
	using Search_Method::Search_Method;
	std::vector<Joint_Action> search_joint(const State& state, 
		Recipe recipe, const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent) const override;
private:
	size_t get_action_cost(const Joint_Action& action) const;
	void print_current(size_t current_state_id, const std::vector<State_Info>& states, const State_Info& current_state) const;
	void initialize_variables(std::priority_queue<Node, std::vector<Node>, Node_Comparator>& frontier,
		std::unordered_set<State_Hash>& visited, std::vector<State_Info>& states, std::vector<bool>& closed, Heuristic& heuristic, const State& original_state) const;
	bool is_being_passed(const std::optional<Agent_Id>& handoff_agent, const Joint_Action& action, const State_Info& current_state_info) const;
	std::vector<Joint_Action> get_actions(const Agent_Combination& agents, bool has_handoff_agent) const;
	std::optional<size_t> get_next_state(Node_Queue& frontier, std::vector<bool>& closed) const;
	std::pair<bool, State_Info > check_and_perform(const Joint_Action& action, const State_Info& current_state_info, const size_t& current_state_id, const std::optional<Agent_Id>& handoff_agent) const;

	//std::vector<Joint_Action> extract_actions(size_t goal_id, const std::vector<State_Info>& states) const;
};