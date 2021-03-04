#pragma once
#include <vector>
#include "Environment.hpp"

class Search_State {
public:
	Search_State(State state, Action action, size_t parent_id, size_t id) 
		: state(state), action(action), parent_id(parent_id), id(id) {};
	State state;
	Action action;
	size_t parent_id;
	size_t id;
	bool operator==(const Search_State& other) const {
		return  state == other.state;
	}
};


class Search_Joint_State {
public:
	Search_Joint_State(State state, Joint_Action action, size_t parent_id, size_t id)
		: state(state), action(action), parent_id(parent_id), id(id) {};
	State state;
	Joint_Action action;
	size_t parent_id;
	size_t id;
	bool operator==(const Search_Joint_State& other) const {
		return  state == other.state;
	}
};

namespace std {
	template<>
	struct hash<Search_State>
	{
		size_t
			operator()(const Search_State& obj) const
		{
			return obj.state.to_hash();
		}
	};
}

namespace std {
	template<>
	struct hash<Search_Joint_State>
	{
		size_t
			operator()(const Search_Joint_State& obj) const
		{
			return obj.state.to_hash();
		}
	};
}

class BFS {
public:
	std::vector<Action> search(const State& state, const Environment& environment, Ingredient goal, Agent_Id agent) const;
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment, Ingredient goal) const;
private:
};