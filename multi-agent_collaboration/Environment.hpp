#pragma once

#include "Core.hpp"

#include <vector>
#include <map>
#include <utility>
#include <string>
#include <optional>
#include <iostream>
#include <cassert>

using Coordinate = std::pair<size_t, size_t> ;

struct Location {
	Coordinate coordinate;
	Coordinate original;
	bool from_wall; // If this was generated from neighbouring wall(original), used by heuristic
};

struct Agent_Id {
	Agent_Id(size_t id) :id(id) {};
	size_t id;
	bool operator==(const Agent_Id& other) const {
		return this->id == other.id;
	}
	bool operator!=(const Agent_Id& other) const {
		return !(this->id == other.id);
	}
	bool operator<(const Agent_Id& other) const {
		return this->id < other.id;
	}
	std::string to_string() const {
		return std::to_string(id);
	}
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
	constexpr Recipe(Ingredient ingredient1, Ingredient ingredient2, Ingredient result) :
		ingredient1(ingredient1), ingredient2(ingredient2), result(result) {};
	Ingredient ingredient1;
	Ingredient ingredient2;
	Ingredient result;
	bool operator<(const Recipe& other) const {
		if (ingredient1 != other.ingredient1) return ingredient1 < other.ingredient1;
		if (ingredient2 != other.ingredient2) return ingredient2 < other.ingredient2;
		if (result != other.result) return result < other.result;
		return false;
	}
	bool operator!=(const Recipe& other) const {
		return (ingredient1 != other.ingredient1 || ingredient2 != other.ingredient2 || result != other.result);
	}
	bool operator==(const Recipe& other) const {
		return (ingredient1 == other.ingredient1 && ingredient2 == other.ingredient2 && result == other.result);
	}
	char result_char() const {
		return static_cast<char>(result);
	}
};

struct Recipes {
	std::vector<Recipe> recipes;

	std::map<Ingredient, size_t> get_ingredient_counts() const {
		std::map<Ingredient, size_t> ingredients;
		get_ingredient_counts(ingredients, recipes);
		return ingredients;
	}
	
	void get_ingredient_counts(std::map<Ingredient, size_t>& ingredients, const std::vector<Recipe>& recipes) const {
		for (const auto& recipe : recipes) {
			// Ingredient 1
			auto it = ingredients.find(recipe.ingredient1);
			if (it == ingredients.end()) {
				ingredients.insert({ recipe.ingredient1, 1 });
			} else {
				++(it->second);
			}

			// Ingredient 2
			it = ingredients.find(recipe.ingredient2);
			if (it == ingredients.end()) {
				ingredients.insert({ recipe.ingredient2, 1 });
			} else {
				++(it->second);
			}
		}

		//for (const auto& [ingredient, count] : ingredients) {
		//	if (!environment.is_type_stationary(ingredient)
		//		&& state.get_count(ingredient) < count) {

		//		is_probable = false;
		//		break;
		//	}
		//}
	}

};

struct Action {
	Action() : direction(Direction::NONE), agent(EMPTY_VAL) {};
	Action(Direction direction, Agent_Id agent) : direction(direction), agent(agent) {};
	Direction direction;
	Agent_Id agent;

	std::string to_string() const {
		return std::string(1, static_cast<char>(direction)) + ":" + std::to_string(agent.id);
	}

	bool is_not_none() const {
		return direction != Direction::NONE;
	}
	bool is_none() const {
		return direction == Direction::NONE;
	}
};

struct Joint_Action {
	Joint_Action() : actions() {};
	Joint_Action(std::vector<Action> actions) : actions(actions) {};

	std::vector<Action> actions;
	
	// TODO - I believe actions are ordered by agent, and therefore the agent id is 
	// not needed in the actions, or this function, but need to verify
	void update_action(size_t agent, Direction direction) {
		for (auto& action : actions) {
			if (action.agent.id == agent) {
				action.direction = direction;
				return;
			}
		}
	}

	bool is_action_useful(Agent_Id agent) const {
		assert(actions.size() > agent.id);
		return actions.at(agent.id).direction != Direction::NONE;
	}

	Action get_action(const Agent_Id& agent) const {
		if (actions.size() <= agent.id) {
			std::cerr << "Attempting to get action for agent " << agent.id << ", but action length is " << actions.size() << std::endl;
			for (const auto& action : actions) {
				std::cerr << action.to_string() << " ";
			}
			std::cerr << std::endl;
			throw std::runtime_error("Invalid get action");
		}
		return actions.at(agent.id);
	}

	// false if handoff action
	bool is_action_valid() const {
		return !actions.empty();
	}
};

struct Agent {
	Agent(Coordinate coordinate)
		: coordinate(coordinate), item() {};
	Agent(Coordinate coordinate, Ingredient item) 
		: coordinate(coordinate), item(item) {};
	Coordinate coordinate;
	std::optional<Ingredient> item;
	bool operator== (const Agent& other) const {
		if (coordinate != other.coordinate) return false;
		if (item != other.item) return false;
		return true;
	}
	bool operator!= (const Agent& other) const {
		return !(*this == other);
	}
	bool operator< (const Agent& other) const {
		if (coordinate < other.coordinate) return true;
		if (coordinate > other.coordinate) return false;
		if (item.has_value() && !other.item.has_value()) return true;
		if (!item.has_value() && other.item.has_value()) return true;
		if (item.value() < other.item.value()) return true;
		if (item.value() > other.item.value()) return false;
		return false;
	}
	void clear_item() {
		item = {};
	}
	void set_item(Ingredient item) {
		this->item = item;
	}
	void move_to(Coordinate coordinate) {
		this->coordinate = coordinate;
	}
	void print_compact(Agent_Id id) const {
		std::cout << "(" << id.id << ", " << coordinate.first << ", " << coordinate.second << ") ";
		if (item.has_value()) {
			std::cout << "(" << static_cast<char>(item.value()) << ", " << coordinate.first << ", " << coordinate.second << ") ";
		}
	}
};


struct Agent_Combination {
	Agent_Combination() : agents() { generate_pretty_print(); }
	explicit Agent_Combination(std::vector<Agent_Id> agents) : agents(agents) { generate_pretty_print(); }
	explicit Agent_Combination(Agent_Id agent) : agents(1, agent) { generate_pretty_print(); }

	std::vector<Agent_Id> agents;
	std::string pretty_print;

	bool operator<(const Agent_Combination& other) const {
		if (agents.size() != other.agents.size()) return agents.size() < other.agents.size();
		for (size_t i = 0; i < agents.size(); ++i) {
			if (agents.at(i) != other.agents.at(i)) return agents.at(i) < other.agents.at(i);
		}
		return false;
	}

	bool operator!=(const Agent_Combination& other) const {
		if (agents.size() != other.agents.size()) return true;
		for (size_t i = 0; i < agents.size(); ++i) {
			if (agents.at(i) != other.agents.at(i)) return true;
		}
		return false;
	}

	bool contains(Agent_Id agent) const {
		return std::find(agents.begin(), agents.end(), agent) != agents.end();
	}

	size_t get_index(Agent_Id agent) const {
		size_t counter = 0;
		for (const auto& entry : agents) {
			if (entry == agent) return counter;
			else ++counter;
		}
		return EMPTY_VAL;
	}
	Agent_Id get(size_t index) const {
		assert(index < agents.size());
		return agents.at(index);
	}

	Agent_Id get_largest() const {
		Agent_Id largest = agents.at(0);
		for (const auto& agent : agents) {
			if (largest < agent) {
				largest = agent;
			}
		}
		return largest;
	}

	const std::vector<Agent_Id>& get() const {
		return agents;
	}

	size_t size() const {
		return agents.size();
	}

	bool empty() const {
		return agents.empty();
	}

	const std::string& to_string() const {
		return pretty_print;
	}

	void remove(Agent_Id agent) {
		auto it = std::find(agents.begin(), agents.end(), agent);
		if (it != agents.end())	agents.erase(it);
	}

private:
	void generate_pretty_print() {
		pretty_print = "(";
		bool first = true;
		for (const auto& agent : agents) {
			if (first) first = false;
			else pretty_print += ",";
			pretty_print += std::to_string(agent.id);
		}
		pretty_print += ")";
	}
};


struct State;
class Environment {

public:

	Environment(size_t number_of_agents) :
		number_of_agents(number_of_agents), goal_names(), agents_initial_positions(), walls(), 
		cutting_stations(), delivery_stations(), width(), height() {};

	bool is_inbound(const Coordinate& coordiante) const;
	bool is_cell_type(const Coordinate& coordinate, const Cell_Type& type) const;
	bool is_type_stationary(Ingredient ingredient) const;
	bool is_done(const State& state) const;
	bool act(State& state, const Action& action) const;
	bool act(State& state, const Action& action, Print_Level print_level) const;
	bool act(State& state, const Joint_Action& action) const;
	bool act(State& state, const Joint_Action& action, Print_Level print_level) const;
	State load(const std::string& path);
	void print_state() const;
	void print_state(const State& state) const;
	void play(State& state) const;
	Joint_Action convert_to_joint_action(const Action& action, Agent_Id agent) const;

	std::vector<Action>			get_actions(Agent_Id agent) const;
	const std::vector<Recipe>&	get_all_recipes() const;
	std::vector<Ingredient>		get_goal() const;
	size_t						get_height() const;
	std::vector<Joint_Action>	get_joint_actions(const Agent_Combination& agents) const;
	std::vector<Coordinate>		get_coordinates(const State& state, Ingredient ingredient) const;
	std::vector<Coordinate>		get_neighbours(Coordinate location) const;
	std::vector<Location>		get_locations(const State& state, Ingredient ingredient) const;
	std::vector<Location>		get_non_wall_locations(const State& state, Ingredient ingredient) const;
	size_t						get_number_of_agents() const;
	std::vector<Recipe>			get_possible_recipes(const State& state) const; 
	std::vector<Coordinate>		get_recipe_locations(const State& state, Ingredient ingredient) const;
	size_t						get_width() const;
	

private:
	void load_map_line(State& state, size_t& line_counter, const std::string& line, size_t width);
	void flip_walls_array();
	Ingredient goal_name_to_ingredient(const std::string& name) const;
	std::optional<Ingredient> get_recipe(Ingredient ingredient1, Ingredient ingredient2) const;
	const std::map<std::pair<Ingredient, Ingredient>, Ingredient>& get_recipes() const;
	void check_collisions(const State& state, Joint_Action& joint_action) const;
	Coordinate move(const Coordinate& coordinate, Direction direction) const;
	Coordinate move_noclip(const Coordinate& coordinate, Direction direction) const;
	void reset();
	void calculate_recipes();

	size_t width;
	size_t height;

	size_t number_of_agents;
	std::vector<std::string> goal_names;
	std::vector<Ingredient> goal_ingredients;
	std::vector<Coordinate> agents_initial_positions;

	std::vector<std::vector<bool>> walls;
	std::vector<Coordinate> cutting_stations;
	std::vector<Coordinate> delivery_stations;
	std::vector<Recipe> goal_related_recipes;
	
};

