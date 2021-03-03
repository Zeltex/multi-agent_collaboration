#pragma once

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
	LEFT='l'
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
	SALAD='s',
	CUTTING='x'
};


//std::map<Ingredient, Ingredient> recipes = { {Ingredient::CUTTING, Ingredient::CHOPPED_TOMATO} };
//std::map<std::pair<Ingredient, Ingredient>, Ingredient> recipes2 = {
//	{ {Ingredient::CUTTING, Ingredient::TOMATO}, Ingredient::CHOPPED_TOMATO},
//	{ {Ingredient::CUTTING, Ingredient::LETTUCE}, Ingredient::CHOPPED_LETTUCE},
//
//	{ {Ingredient::PLATE, Ingredient::CHOPPED_LETTUCE}, Ingredient::PLATED_LETTUCE},
//	{ {Ingredient::PLATE, Ingredient::CHOPPED_TOMATO}, Ingredient::PLATED_TOMATO},
//	{ {Ingredient::PLATE, Ingredient::SALAD}, Ingredient::PLATED_SALAD},
//
//	{ {Ingredient::CHOPPED_LETTUCE, Ingredient::CHOPPED_TOMATO}, Ingredient::SALAD},
//	{ {Ingredient::PLATED_LETTUCE, Ingredient::CHOPPED_TOMATO}, Ingredient::PLATED_SALAD},
//	{ {Ingredient::PLATED_TOMATO, Ingredient::CHOPPED_LETTUCE}, Ingredient::PLATED_SALAD},
//};
//
////constexpr std::map<std::pair<Ingredient, Ingredient>, Ingredient> recipes = {
////	{Ingredient::CUTTING, Ingredient::TOMATO}, Ingredient::CHOPPED_TOMATO }
////};
//
//class Recipes {
//public:
//	std::optional<Ingredient> get_recipe(Ingredient ingredient1, Ingredient ingredient2) {
//		auto recipe_it = recipes.find({ ingredient1, ingredient2 });
//		if ( recipe_it != recipes.end()) {
//			return recipe_it->second;
//		} else {
//			return {};
//		}
//	}
//private:
//	std::map<std::pair<Ingredient, Ingredient>, Ingredient> recipes;
//};

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

	void add(Coordinate coordinate, Ingredient ingredient) {
		items.insert({ coordinate, ingredient });
	}

	void remove(Coordinate coordinate) {
		items.erase(items.find(coordinate));
	}

};

class Environment {

public:

	Environment(size_t number_of_agents) :
		number_of_agents(number_of_agents), goal_name(), agents_initial_positions(), walls(), cutting_stations(), delivery_stations() {};

	bool is_cell_type(const Coordinate& coordinate, const Cell_Type& type) const;
	void act(State& state, const Action& action) const;
	std::vector<Action> get_actions(const State& state) const;
	State load(const std::string& path);
	void print_state() const;
	void print_state(const State& state) const;
	void play(State& state) const;

private:
	void load_map_line(State& state, size_t& line_counter, const std::string& line, size_t width);
	void flip_walls_array();
	Ingredient goal_name_to_ingredient(const std::string& name) const;
	std::optional<Ingredient> get_recipe(Ingredient ingredient1, Ingredient ingredient2) const;

	size_t number_of_agents;
	std::string goal_name;
	Ingredient goal_ingredient;
	std::vector<Coordinate> agents_initial_positions;

	std::vector<std::vector<bool>> walls;
	std::vector<Coordinate> cutting_stations;
	std::vector<Coordinate> delivery_stations;
};

