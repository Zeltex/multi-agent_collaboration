#pragma once

#include "Environment.hpp"


struct Distance_Entry {
	Distance_Entry() :g(EMPTY_VAL), parent(EMPTY_VAL, EMPTY_VAL) {}
	Distance_Entry(size_t g) :g(g), parent(EMPTY_VAL, EMPTY_VAL) {}
	Distance_Entry(size_t g, Coordinate parent) :g(g), parent(parent) {}
	size_t g;
	Coordinate parent;
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

#define INFINITE_HEURISTIC 1000
class Heuristic {
public:
	Heuristic(Environment environment);
	size_t operator()(const State& state, const std::optional<Agent_Id>& handoff_agent) const;
	void set(Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents,
		const std::optional<Agent_Id>& handoff_agent);

private:
	std::tuple<size_t, size_t, bool>  get_helper_agents_distance(Coordinate source, Coordinate destination, const State& state,
		const std::optional<Agent_Id> handoff_agent, const Agent_Combination& local_agents) const;
	
	size_t get_heuristic_distance(const Location& location1, const Location& location2, const State& state, 
		const std::optional<Agent_Id>& handoff_agent, const Agent_Combination& local_agents) const;
	
	size_t get_nearest_agent_distance(const State& state, Location location, const std::optional<Agent_Id>& handoff_agent) const;
	size_t get_non_participating_agent_holding_penalty(const Location& location1,
		const Location& location2, const State& state, const Agent_Combination& local_agents) const;
	size_t convert(const Coordinate& coord1) const;
	void print_distances(Coordinate coordinate, size_t agent_number) const;
	void init();
	size_t get_distance_to_nearest_wall(Coordinate agent_coord, Coordinate blocked, const State& state) const;

	std::vector<Distances> distances;	// Vector index is the amount of walls intersected on the path
	Environment environment;
	Ingredient ingredient1;
	Ingredient ingredient2;
	Agent_Combination agents;
	std::optional<Agent_Id> handoff_agent;
	std::vector<Agent_Combination> agent_combinations;
};