#pragma once

#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING 1
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1

#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <memory>

#include "Environment.hpp"
#include "Search.hpp"
#include "boost/pool/pool.hpp"

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
		size_t pass_time, Node* parent, Joint_Action action, bool closed)
		: state(state), id(id), g(g), h(h), action_count(action_count),
		pass_time(pass_time), parent(parent), action(action), closed(closed){};
	//Node(size_t state_id, size_t g, size_t h, size_t action_count) : 
	//	state_id(state_id), g(g), h(h), action_count(action_count){};
		
	void init(const Node* other) {
		this->state = other->state;
		this->id = other->id;
		this->g = other->g;
		this->h = other->h;
		this->action_count = other->action_count;
		this->parent = other->parent;
		this->action = other->action;
		this->closed = other->closed;
		this->pass_time = other->pass_time;
		this->hash = EMPTY_VAL;
	}

	//void init(State state, size_t id, size_t g, size_t h, size_t action_count,
	//	size_t pass_time, Node* parent, Joint_Action action, bool closed) {
	//	this->state = state;
	//	this->id = id;
	//	this->g = g; 
	//	this->h = h;
	//	this->action_count = action_count;
	//	this->pass_time = pass_time;
	//	this->parent = parent;
	//	this->action = action;
	//	this->closed = closed;
	//}

	State state;
	size_t id;
	size_t g;
	size_t h;
	size_t f() const { return g + h; }
	size_t action_count;
	size_t pass_time;
	const Node* parent;
	Joint_Action action;
	bool closed;

	// For debug purposes
	size_t hash;
	void calculate_hash() {
		this->hash = to_hash();
	}

	bool set_equals(const Node* other) const {
		return this->state == other->state && this->pass_time == other->pass_time;
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

	// Used to determine if a shorter path has been found (assumes this->stae == other->state)
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

	// TODO - Check if more fields should be updated
	// Used to update node when shorter path has been found
	void update(const Node* other) {
		if (this->id == other->parent->id) {
			size_t dummy = 0;
		}
		this->g = other->g;
		this->action_count = other->action_count;
		this->parent = other->parent;
		this->action = other->action;
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

// Could optimise some by calculating distance from all ingredients to nearest agent
// and using this in combination with the location1/location2 nested for loops
#define INFINITE_HEURISTIC 1000
struct Heuristic {
	Heuristic(Environment environment, Ingredient ingredient1, Ingredient ingredient2, 
		const Agent_Combination& agents, const std::optional<Agent_Id>& handoff_agent)
		: environment(environment), ingredient1(ingredient1), ingredient2(ingredient2),  
		agents(agents), handoff_agent(handoff_agent) {};

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
		for (const auto& agent_id : agents.get()) {
			if (handoff_agent.has_value() && agent_id == handoff_agent.value()) {
				continue;
			}
			const auto& agent = state.agents.at(agent_id.id);
			if (!environment.is_type_stationary(ingredient1)) {
				for (const auto& location1 : locations1) {
					min_agent_dist = std::min(min_agent_dist, euclidean(location1, agent.coordinate));

				}
			}
			if (!environment.is_type_stationary(ingredient2)) {
				for (const auto& location2 : locations2) {
					min_agent_dist = std::min(min_agent_dist, euclidean(location2, agent.coordinate));

				}
			}
		}
		if (min_dist == (size_t)-1) min_dist = 0;
		if (min_agent_dist == (size_t)-1) min_agent_dist = 0;
		else if (min_agent_dist > 0) --min_agent_dist;


		return min_dist + min_agent_dist;
	}

	Environment environment; 
	Ingredient ingredient1; 
	Ingredient ingredient2;
	Agent_Combination agents;
	std::optional<Agent_Id> handoff_agent;
};


struct Node_Queue_Comparator {
	bool operator()(const Node* lhs, const Node* rhs) const {
		if (lhs->f() != rhs->f()) return lhs->f() > rhs->f();
		if (lhs->g != rhs->g) return lhs->g > rhs->g;
		if (lhs->action_count != rhs->action_count) return lhs->action_count > rhs->action_count;
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
using Pool = boost::pool<>;

class A_Star : public Search_Method {
public:
	using Search_Method::Search_Method;
	std::vector<Joint_Action> search_joint(const State& state, Recipe recipe, 
		const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent) const override;
private:
	size_t	get_action_cost(const Joint_Action& action) const;
	
	Node*	get_next_node(Node_Queue& frontier) const;
	
	void	initialize_variables(Node_Queue& frontier, Node_Set& visited, Node_Ref& nodes,
		Heuristic& heuristic, const State& original_state, Pool& pool) const;
	
	bool	is_being_passed(const std::optional<Agent_Id>& handoff_agent, const Joint_Action& action, 
		const Node* current_node) const;
	
	bool	is_invalid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, 
		const std::optional<Agent_Id>& handoff_agent) const;

	bool	is_valid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, 
		const std::optional<Agent_Id>& handoff_agent) const;
	
	void	print_current(const Node* node) const;
	
	std::vector<Joint_Action> get_actions(const Agent_Combination& agents, 
		bool has_handoff_agent) const;
		
	std::pair<bool, Node*> check_and_perform(const Joint_Action& action, Node_Ref& nodes,
		const Node* current_node, const std::optional<Agent_Id>& handoff_agent,
		Heuristic& heuristic, Pool& pool) const;
	
	std::vector<Joint_Action> extract_actions(const Node* node) const;

	//std::vector<Joint_Action> extract_actions(size_t goal_id, const std::vector<State_Info>& states) const;
};