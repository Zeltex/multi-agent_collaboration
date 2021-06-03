#pragma once

#include "Environment.hpp"


struct Distance_Entry {
	Distance_Entry() :g(EMPTY_VAL), parent(EMPTY_VAL, EMPTY_VAL), wall_g(0) {}
	Distance_Entry(size_t g) :g(g), parent(EMPTY_VAL, EMPTY_VAL), wall_g(0) {}
	Distance_Entry(size_t g, Coordinate parent) : g(g), parent(parent), wall_g(0) {}
	Distance_Entry(size_t g, Coordinate parent, size_t wall_g) : g(g), parent(parent), wall_g(wall_g) {}
	size_t g;
	Coordinate parent;
	size_t wall_g;
};
struct Distances {
	Distances(size_t width, size_t height)
		: distances(width* height, std::vector<Distance_Entry>(width* height)),
		width(width), height(height) {};
	std::vector<std::vector<Distance_Entry>> distances;
	size_t width;
	size_t height;
	constexpr size_t convert(const Coordinate& coord1) const {
		return coord1.first * height + coord1.second;
	}

	const Distance_Entry& const_at(Coordinate coord1, Coordinate coord2) const {
		return distances.at(convert(coord1)).at(convert(coord2));
	}

	Distance_Entry& at(Coordinate coord1, Coordinate coord2) {
		return distances.at(convert(coord1)).at(convert(coord2));
	}
};

struct Helper_Agent_Distance {
	Helper_Agent_Distance()
		: dist(EMPTY_VAL), was_handed_off(false) {}
	Helper_Agent_Distance(size_t dist, bool was_handed_off) 
		: dist(dist), was_handed_off(was_handed_off) {}
	
	size_t dist;
	bool was_handed_off;

	bool has_value() {
		return dist != EMPTY_VAL;
	}
};

struct Helper_Agent_Request {

};

struct Helper_Agent_Info {
	size_t help_to_dest;	// The distance from the spot where the helper agent  
							//	is needed to the destination 

	size_t agent_to_help;	// The distance from the helper agent to the
							//	spot where it is needed (plus optional holding penalty)

	Agent_Id agent;

	bool has_value() {
		return help_to_dest != EMPTY_VAL && agent_to_help != EMPTY_VAL;
	}
};

#define INFINITE_HEURISTIC 1000
class Heuristic {
public:
	Heuristic(Environment environment);
	size_t operator()(const State& state, const Agent_Combination& agents, const Agent_Id& handoff_agent) const;
	void set(Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents,
		const Agent_Id& handoff_agent);
	std::pair<size_t, Direction> get_dist_direction(Coordinate source, Coordinate dest, size_t walls) const;

private:
	Helper_Agent_Distance  get_helper_agents_distance(Coordinate source, Coordinate destination, const State& state,
		const Agent_Id handoff_agent, const Agent_Combination& local_agents, const bool& require_handoff_action,
		size_t walls_to_penetrate) const;
	
	size_t get_heuristic_distance(const Location& location1, const Location& location2, const State& state, 
		const Agent_Id& handoff_agent, const Agent_Combination& local_agents, const bool& require_handoff_action) const;
	
	size_t get_nearest_agent_distance(const State& state, Location location, const Agent_Id& handoff_agent, const Agent_Combination& local_agents) const;
	bool are_items_available (const Location& location1, const Location& location2, const State& state, const Agent_Combination& local_agents) const;
	size_t convert(const Coordinate& coord1) const;
	void print_distances(Coordinate coordinate, size_t agent_number) const;
	void init();
	size_t get_distance_to_nearest_wall(Coordinate agent_coord, Coordinate blocked, const State& state) const;
	Helper_Agent_Info find_helper(const std::vector<Helper_Agent_Info>& helpers, const Agent_Id handoff_agent, const Agent_Combination& local_agents, const bool first,
		const State& state, const Coordinate& prev, const Coordinate& next, const size_t path_length) const;

	std::vector<Distances> distances;	// Vector index is the amount of walls intersected on the path
	Environment environment;
	Ingredient ingredient1;
	Ingredient ingredient2;
	//Agent_Combination agents;
	std::optional<Agent_Id> handoff_agent;
	std::vector<Agent_Combination> agent_combinations;
};