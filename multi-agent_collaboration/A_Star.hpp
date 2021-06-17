#pragma once

#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING 1
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1

#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <memory>
#include <cassert>
#include <array>

#include "Environment.hpp"
#include "Search.hpp"
#include "Utils.hpp"
#include "Heuristic.hpp"

struct Node {
	Node() {};

	Node(const Node* other, const size_t& id) {
		init(other);
		this->id = id;
		this->parent = other;
	}

	Node(const Node* other) {
		init(other);
		this->parent = other;
	}

	Node(State state, size_t id, size_t g, size_t h, size_t action_count,
		size_t pass_time, bool can_pass, size_t handoff_first_action,
		Node* parent, Joint_Action action, bool closed, bool valid, Agent_Id agent)
		: state(state), id(id), g(g), h(h), action_count(action_count),
		pass_time(pass_time), can_pass(can_pass), handoff_first_action(handoff_first_action),
		parent(parent), action(action), closed(closed), valid(valid), agent(agent) {};
	
	void init(const Node* other) {
		this->state = other->state;
		this->id = other->id;
		this->g = other->g;
		this->h = other->h;
		this->action_count = other->action_count;
		this->parent = other->parent;
		this->action = other->action;
		this->closed = other->closed;
		this->valid = other->valid;
		this->pass_time = other->pass_time;
		this->can_pass = other->can_pass;
		this->handoff_first_action = other->handoff_first_action;
		this->hash = EMPTY_VAL;
		this->agent = other->agent;
	}

	size_t g;
	float h;
	float f() const { return g + h; }
	State state;
	size_t id;
	size_t action_count;
	size_t pass_time;
	bool can_pass;
	const Node* parent;
	Joint_Action action;
	bool closed;
	bool valid;
	size_t handoff_first_action;
	Agent_Id agent;

	// For debug purposes
	size_t hash;
	void calculate_hash() {
		this->hash = to_hash();
	}

	bool set_equals(const Node* other) const {
		return this->state == other->state 
			&& (this->pass_time == other->pass_time
				|| (this->pass_time != EMPTY_VAL && other->pass_time != EMPTY_VAL));
	}

	size_t to_hash() const {
		std::string pass_string = (pass_time == EMPTY_VAL ? "0" : "1");
		return std::hash<std::string>()(state.to_hash_string() + pass_string);
	}

	bool has_agent_passed() const {
		return pass_time != EMPTY_VAL;
	}

	bool operator<(const Node* other) const {
		std::cout << "<node" << std::endl;
		return this->state < other->state;
	}

	// Used to determine if a shorter path has been found (assumes this->state == other->state)
	bool is_shorter(const Node* other) const {
		if (this->g != other->g) {
			return this->g < other->g;
		}

		if (this->action_count != other->action_count) {
			return this->action_count < other->action_count;
		}

		if (this->pass_time != EMPTY_VAL && other->pass_time != EMPTY_VAL) {
			return this->pass_time < other->pass_time;
		}

		return false;
	}

	bool is_agent_compatible(Agent_Id agent) {
		return this->agent == EMPTY_VAL || this->agent == agent;
	}
};

namespace std {
	template<>
	struct hash<Node*>
	{
		size_t
			operator()(const Node* obj) const
		{
			return obj->to_hash();
		}
	};
}



struct Node_Queue_Comparator {
	bool operator()(const Node* lhs, const Node* rhs) const {
		if (lhs->f() != rhs->f()) return lhs->f() > rhs->f();
		if (lhs->g != rhs->g) return lhs->g < rhs->g;
		if (lhs->action_count != rhs->action_count) return lhs->action_count > rhs->action_count;
		//if (lhs->pass_time != rhs->pass_time) return lhs->pass_time > rhs->pass_time;

		//if (lhs->handoff_first_action != rhs->handoff_first_action) 
		//	return lhs->handoff_first_action > rhs->handoff_first_action;

		return false;
	}
};

struct Node_Hasher {
	bool operator()(const Node* node) const {
		return node->to_hash();
	}
};

struct Node_Set_Comparator {
	bool operator()(const Node* lhs, const Node* rhs) const {
		return lhs->set_equals(rhs);
	}
};

using Node_Queue = std::priority_queue<Node*, std::vector<Node*>, Node_Queue_Comparator>;
using Node_Set = std::unordered_set<Node*, Node_Hasher, Node_Set_Comparator>;
using Node_Ref = std::deque<Node>;

struct Search_Info {
	Search_Info(const Recipe& recipe, const Agent_Id& handoff_agent, const Agent_Combination& agents)
		: frontier(), visited(), nodes(), goal_node(nullptr), recipe(recipe), 
		handoff_agent(handoff_agent), agents(agents) {}
	bool has_goal_node() const {
		return goal_node != nullptr;
	}

	Node_Queue frontier;
	Node_Set visited;
	Node_Ref nodes;
	Node* goal_node;
	Recipe recipe;
	Agent_Id handoff_agent;
	Agent_Combination agents;
};

class Manhattan_Heuristic {
public:
	Manhattan_Heuristic(const Environment& environment) : environment(environment),
		ingredient1(Ingredient::DELIVERY), ingredient2(Ingredient::DELIVERY) { }

	size_t operator()(const State& state, const Agent_Combination& agents, const Agent_Id& handoff_agent) const {
		bool require_handoff_action = false;
		std::vector<Location> locations1;
		locations1 = environment.get_locations(state, ingredient1);
		if (environment.is_type_stationary(ingredient1)) {
		} else {
			locations1 = environment.get_non_wall_locations(state, ingredient1);
		}
		auto locations2 = environment.get_non_wall_locations(state, ingredient2);

		size_t min_dist = EMPTY_VAL;
		size_t number_of_agents = environment.get_number_of_agents();

		// Search all location combinations using all agents, and all agents minus handoff_agent
		for (const auto& location1 : locations1) {
			for (const auto& location2 : locations2) {
				auto first = (size_t) std::abs((int)location1.coordinate.first - (int)location2.coordinate.first);
				auto second = (size_t) std::abs((int)location1.coordinate.second - (int)location2.coordinate.second);
				min_dist = std::min(min_dist, first + second);
			}
		}
		return min_dist;
	}

	void set(Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents,
		const Agent_Id& handoff_agent) {

		this->ingredient1 = ingredient1;
		this->ingredient2 = ingredient2;
	}

	Environment environment;
	Ingredient ingredient1;
	Ingredient ingredient2;
};


class A_Star : public Search_Method {
public:
	A_Star(const Environment& environment, size_t depth_limit);
	std::vector<Joint_Action> search_joint(const State& state, Recipe recipe, 
		const Agent_Combination& agents, Agent_Id handoff_agent,
		const std::vector<Joint_Action>& input_actions, 
		const Agent_Combination& free_agents, const Action& initial_action = {}) override;
	std::pair<size_t, Direction> get_dist_direction(Coordinate source, Coordinate dest, size_t walls) override;
private:
	
	
	
	bool						action_conforms_to_input(const Node* current_node, const std::vector<Joint_Action>& input_actions,
									const Joint_Action action, const Agent_Combination& agents, const Action& initial_action) const;
	Node*						check_and_perform(Search_Info& si, const Joint_Action& action, const Node* current_node, const std::vector<Joint_Action>& input_actions) const;
	std::vector<Joint_Action>	extract_actions(const Node* node) const;
	Node*						generate_handoff(Search_Info& si, Node* node, const std::vector<Joint_Action>& input_actions) const;
	size_t						get_action_cost(const Joint_Action& action, const Agent_Id& handoff_agent) const;
	std::vector<Joint_Action>	get_actions(const Agent_Combination& agents, bool has_handoff_agent) const;
	Node*						get_next_node(Search_Info& si) const;
	Search_Info					initialize_variables(Recipe& recipe, const State& original_state, 
									const Agent_Id& handoff_agent, const Agent_Combination& agents, const std::vector<Joint_Action>& input_actions) const;
	bool						is_invalid_goal(const Search_Info& si, const Node* node, const Joint_Action& action) const;
	bool						is_valid_goal(const Search_Info& si, const Node* node, const Joint_Action& action) const;
	void						print_current(const Node* node) const;
	void						print_goal(const Node* node) const;
	bool						process_node(Search_Info& si, Node* node, const Joint_Action& action) const;

	
	
		
	



	Heuristic dist_heuristic; 
	Heuristic heuristic;
};