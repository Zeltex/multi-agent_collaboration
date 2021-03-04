#pragma once

#include "Core.hpp"

#include <vector>
#include <map>
#include <utility>
#include <string>
#include <optional>

using Coordinate = std::pair<size_t, size_t> ;

struct Agent_Id {
	Agent_Id(size_t id) :id(id) {};
	size_t id;
};

enum class Cell_Type {
	WALL='-',
	CUTTING_STATION='/',
	DELIVERY_STATION='*'
};

enum class Direction {
	UP='u',
	RIGHT='r',
	DOWN='d',
	LEFT='l',
	NONE='n'
};

enum class Ingredient {
	TOMATO='t',
	CHOPPED_TOMATO='T',
	LETTUCE='l',
	CHOPPED_LETTUCE='L',
	PLATE='p',
	PLATED_TOMATO='a',
	PLATED_LETTUCE='b',
	PLATED_SALAD='c',
	DELIVERED_TOMATO='A',
	DELIVERED_LETTUCE='B',
	DELIVERED_SALAD='C',
	SALAD='s',
	CUTTING='x',
	DELIVERY='y'
};

struct Recipe {
	Recipe(Ingredient ingredient1, Ingredient ingredient2, Ingredient result) :
		ingredient1(ingredient1), ingredient2(ingredient2), result(result) {};
	Ingredient ingredient1;
	Ingredient ingredient2;
	Ingredient result;
};

struct Action {
	Action(Direction direction, Agent_Id agent) : direction(direction), agent(agent) {};
	Direction direction;
	Agent_Id agent;
};

struct State {
	std::map<Coordinate, Ingredient> items;
	std::vector<Coordinate> agents;
	std::optional<Ingredient> get_ingredient_at_position(Coordinate coordinate) const {
		auto it = items.find(coordinate);
		if (it != items.end()) {
			return it->second;
		} else {
			return {};
		}
	}

	bool contains_item(Ingredient ingredient) const {
		for (const auto& item : items) {
			if (item.second == ingredient) return true;
		}
		return false;
	}

	void add(Coordinate coordinate, Ingredient ingredient) {
		items.insert({ coordinate, ingredient });
	}

	void remove(Coordinate coordinate) {
		items.erase(items.find(coordinate));
	}

	size_t to_hash() const {
		std::string hash;
		hash += std::to_string(items.size() * 128 + agents.size());
		for (const auto& item : items) {
			hash += std::to_string(item.first.first * 128 + item.first.second)
				+ static_cast<char>(item.second);
		}

		for (const auto& agent : agents) {
			hash += std::to_string(agent.first * 128 + agent.second);
		}
		return std::hash<std::string>()(hash);
	}

	bool operator==(const State& other) const {
		if (items.size() != other.items.size()) return false;
		if (agents.size() != other.agents.size()) return false;
		for (size_t i = 0; i < agents.size(); ++i) {
			if (agents.at(i) != other.agents.at(i)) return false;
		}
		for (const auto& item : items) {
			auto it = other.items.find(item.first);
			if (it == other.items.end()) return false;
			if (item.second != it->second) return false;
		}
		return true;
	}
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

class Environment {

public:

	Environment(size_t number_of_agents) :
		number_of_agents(number_of_agents), goal_name(), agents_initial_positions(), walls(), cutting_stations(), delivery_stations() {};

	bool is_cell_type(const Coordinate& coordinate, const Cell_Type& type) const;
	void act(State& state, const Action& action) const;
	void act(State& state, const Action& action, Print_Level print_level) const;
	std::vector<Action> get_actions(const State& state, Agent_Id agent) const;
	State load(const std::string& path);
	void print_state() const;
	void print_state(const State& state) const;
	void play(State& state) const;
	std::vector<Recipe> get_possible_recipes(const State& state) const;
	Ingredient get_goal() const;
	bool is_done(const State& state) const;

private:
	void load_map_line(State& state, size_t& line_counter, const std::string& line, size_t width);
	void flip_walls_array();
	Ingredient goal_name_to_ingredient(const std::string& name) const;
	std::optional<Ingredient> get_recipe(Ingredient ingredient1, Ingredient ingredient2) const;
	const std::map<std::pair<Ingredient, Ingredient>, Ingredient>& get_recipes() const;

	size_t number_of_agents;
	std::string goal_name;
	Ingredient goal_ingredient;
	std::vector<Coordinate> agents_initial_positions;

	std::vector<std::vector<bool>> walls;
	std::vector<Coordinate> cutting_stations;
	std::vector<Coordinate> delivery_stations;

	
};

