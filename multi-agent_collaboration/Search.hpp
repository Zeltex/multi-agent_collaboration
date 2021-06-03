#pragma once

#include <memory>
#include "Environment.hpp"
#include "State.hpp"

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
	Search_Method(const Environment& environment, size_t depth_limit) : environment(environment), depth_limit(depth_limit) {}
	virtual std::vector<Joint_Action> search_joint(const State& state,
		Recipe recipe, const Agent_Combination& agents, Agent_Id handoff_agent,
		const std::vector<Joint_Action>& input_actions, const Agent_Combination& free_agents) = 0;
	virtual std::pair<size_t, Direction> get_dist_direction(Coordinate source, Coordinate dest, size_t walls) = 0;
protected:
		template<typename T>
		std::vector<Joint_Action> extract_actions(size_t goal_id, const std::vector<T>& states) const;

		Environment environment;
		size_t depth_limit;
};

class Search {
public:
	// TODO - Probably delete copy constructor and more
	Search(std::unique_ptr<Search_Method> search_method) : search_method(std::move(search_method)) {};
	std::vector<Joint_Action> search_joint(const State& state, Recipe recipe, const Agent_Combination& agents, 
		Agent_Id handoff_agent, const std::vector<Joint_Action>& input_actions, 
		const Agent_Combination& free_agents) {
		
		return search_method->search_joint(state, recipe, agents, handoff_agent, input_actions, free_agents);
	}
	std::pair<size_t, Direction> get_dist_direction(Coordinate source, Coordinate dest, size_t walls) {
		return search_method->get_dist_direction(source, dest, walls);
	}
private:
	std::unique_ptr<Search_Method> search_method;
};

#include "Search.ipp"