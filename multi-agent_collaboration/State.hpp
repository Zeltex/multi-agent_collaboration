#pragma once
#include <optional>
#include <string>
#include <vector>
#include <map>

#include "Environment.hpp"


struct State {
		bool operator<(const State& other) const;
	void						add(Coordinate coordinate, Ingredient ingredient);
	void						add_goal_item(Coordinate coordinate, Ingredient ingredient);
	bool						contains_item(Ingredient ingredient) const;
	std::optional<Agent_Id>		get_agent(Coordinate coordinate) const;
	Agent						get_agent(Agent_Id agent) const;
	std::vector<Coordinate>		get_coordinates(Ingredient ingredient) const;
	size_t						get_count(Ingredient ingredient) const;
	std::optional<Ingredient>	get_ingredient_at_position(Coordinate coordinate) const;
	std::map<Ingredient, size_t>get_ingredients_count() const;
	Coordinate					get_location(Agent_Id agent) const;
	std::vector<Location>		get_locations(Ingredient ingredient) const;
	std::vector<Location>		get_non_wall_locations(Ingredient ingredient, const Environment& environment) const;
	bool						items_hoarded(const Recipe& recipe, const Agent_Combination& available_agents) const;
	void						print_compact() const;
	void						purge(const Agent_Combination& agents);
	void						remove(Coordinate coordinate);
	std::string					to_hash_string() const;
	size_t						to_hash() const;
	bool operator==(const State& other) const;

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