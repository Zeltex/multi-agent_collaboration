#pragma once
#include <optional>
#include <string>
#include <vector>

#include "Environment.hpp"


struct State {
		bool operator<(const State& other) const;
	std::optional<Ingredient> get_ingredient_at_position(Coordinate coordinate) const;
	bool items_hoarded(const Recipe& recipe, const Agent_Combination& available_agents) const;
	Coordinate get_location(Agent_Id agent) const;
	bool contains_item(Ingredient ingredient) const;
	void add(Coordinate coordinate, Ingredient ingredient);
	void add_goal_item(Coordinate coordinate, Ingredient ingredient);
	void remove(Coordinate coordinate);
	std::string to_hash_string() const;
	size_t to_hash() const;
	bool operator==(const State& other) const;
	std::vector<Coordinate> get_locations(Ingredient ingredient) const;
	std::vector<Coordinate> get_non_wall_locations(Ingredient ingredient,
		const Environment& environment) const;
	void print_compact() const;

	std::map<Coordinate, Ingredient> items;
	std::vector<std::pair<Coordinate, Ingredient>> goal_items;
	std::vector<Agent> agents;
};

namespace std {
	template<>
	struct hash<State>
	{
		size_t
			operator()(const State& obj) const
		{
			return obj.to_hash();
		}
	};
}