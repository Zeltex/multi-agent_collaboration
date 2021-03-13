#pragma once

#include <memory>
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

class Search_Method {
public:
	virtual std::vector<Joint_Action> search_joint(const State& state, const Environment& environment,
		Recipe recipe, const std::vector<Agent_Id>& agents, size_t depth_limit) const = 0;
	virtual std::vector<Joint_Action> search_joint(const State& state, const Environment& environment,
		Recipe recipe, const std::vector<Agent_Id>& agents) const = 0;
protected:
		template<typename T>
		std::vector<Joint_Action> extract_actions(size_t goal_id, const std::vector<T>& states) const;
};

class Search {
public:
	Search(std::unique_ptr<Search_Method> search_method) :search_method(std::move(search_method)) {};
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment,
		Recipe recipe, const std::vector<Agent_Id>& agents, size_t depth_limit) const {
		return search_method->search_joint(state, environment, recipe, agents, depth_limit);
	}
	std::vector<Joint_Action> search_joint(const State& state, const Environment& environment,
		Recipe recipe, const std::vector<Agent_Id>& agents) const {
		return search_method->search_joint(state, environment, recipe, agents);
	}
private:
	std::unique_ptr<Search_Method> search_method;
};

#include "Search.ipp"