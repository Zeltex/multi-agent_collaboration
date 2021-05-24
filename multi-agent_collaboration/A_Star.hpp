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
		size_t pass_time, size_t handoff_first_action, 
		Node* parent, Joint_Action action, bool closed, bool valid)
		: state(state), id(id), g(g), h(h), action_count(action_count),
		pass_time(pass_time), handoff_first_action(handoff_first_action),
		parent(parent), action(action), closed(closed), valid(valid) {};
	
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
		this->handoff_first_action = other->handoff_first_action;
		this->hash = EMPTY_VAL;
	}

	size_t g;
	float h;
	float f() const { return g + h; }
	State state;
	size_t id;
	size_t action_count;
	size_t pass_time;
	const Node* parent;
	Joint_Action action;
	bool closed;
	bool valid;
	size_t handoff_first_action;

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
		if (lhs->pass_time != rhs->pass_time) return lhs->pass_time > rhs->pass_time;
		if (lhs->action_count != rhs->action_count) return lhs->action_count > rhs->action_count;

		if (lhs->handoff_first_action != rhs->handoff_first_action) 
			return lhs->handoff_first_action > rhs->handoff_first_action;

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

class A_Star : public Search_Method {
public:
	A_Star(const Environment& environment, size_t depth_limit);
	std::vector<Joint_Action> search_joint(const State& state, Recipe recipe, 
		const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent,
		const std::vector<Joint_Action>& input_actions, 
		const Agent_Combination& free_agents) override;
private:
	size_t	get_action_cost(const Joint_Action& action, const std::optional<Agent_Id>& handoff_agent) const;
	
	Node*	get_next_node(Node_Queue& frontier) const;
	
	void	initialize_variables(Node_Queue& frontier, Node_Set& visited, Node_Ref& nodes,
		const State& original_state, const std::optional<Agent_Id>& handoff_agent) const;
	
	bool	is_invalid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, 
		const std::optional<Agent_Id>& handoff_agent) const;

	bool	is_valid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, 
		const std::optional<Agent_Id>& handoff_agent) const;
	
	void	print_current(const Node* node) const;
	
	std::vector<Joint_Action> get_actions(const Agent_Combination& agents, 
		bool has_handoff_agent) const;
		
	std::pair<bool, std::array<Node*, 2>> check_and_perform(const Joint_Action& action, Node_Ref& nodes,
		const Node* current_node, const std::optional<Agent_Id>& handoff_agent) const;
	
	std::vector<Joint_Action> extract_actions(const Node* node) const;

	bool action_conforms_to_input(const Node* current_node, const std::vector<Joint_Action>& input_actions,
		const Joint_Action action, const Agent_Combination& agents) const;

	Heuristic heuristic;
};