#pragma once

#include "Core.hpp"

#include <vector>
#include <map>
#include <utility>
#include <string>
#include <optional>
#include <iostream>
#include <cassert>
#include <set>
#include <algorithm>

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
		return id == EMPTY_VAL ? "X" : std::to_string(id);
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

class Environment;

struct Ingredients {
	Ingredients() : ingredients() {}

	void add_ingredients(const std::vector<Recipe>& recipes, const Environment& environment);
	void add_ingredients(const Recipe& recipe, const Environment& environment);
	void add_ingredient(const Ingredient& ingredient) {
		auto it = ingredients.find(ingredient);
		if (it == ingredients.end()) {
			ingredients.insert({ ingredient, 1 });
		} else {
			++(it->second);
		}
	}

	void perform_recipes(const std::vector<Recipe>& recipes, const Environment& environment);
	void perform_recipe(const Recipe& recipe, const Environment& environment);
	bool have_ingredients(const Recipe& recipe, const Environment& environment) const;

	size_t get_count(Ingredient ingredient) const {
		auto it = ingredients.find(ingredient);
		return it == ingredients.end() ? 0 : it->second;
	}

	void clear() {
		ingredients.clear();
	}

	std::set<Ingredient> get_types() const {
		std::set<Ingredient> set;
		for (const auto& [ingredient, count] : ingredients) {
			set.insert(ingredient);
		}
		return set;
	}

	bool operator<=(const Ingredients& other) const {
		for (const auto& [ingredient, count] : ingredients) {
			if (count > other.get_count(ingredient)) return false;
		}
		return true;
	}

	bool operator>(const Ingredients& other) const {
		for (const auto& [ingredient, count] : ingredients) {
			if (count > other.get_count(ingredient)) return true;
		}
		return false;
	}
private:

	std::map<Ingredient, size_t> ingredients;
};



struct Action {
	Action() : direction(Direction::NONE), agent(EMPTY_VAL) {};
	Action(Direction direction, Agent_Id agent) : direction(direction), agent(agent) {};
	Direction direction;
	Agent_Id agent;

	bool operator!=(const Action& other) const {
		return !(*this == other);
	}

	bool operator==(const Action& other) const {
		return direction == other.direction && agent == other.agent;
	}

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
	Joint_Action(size_t number_of_agents) : actions() {
		for (size_t action_index = 0; action_index < number_of_agents; ++action_index) {
			actions.emplace_back(Direction::NONE, Agent_Id{ action_index });
		}
	};
	Joint_Action(std::vector<Action> actions) : actions(actions) {};

	std::vector<Action> actions;
	
	void update_action(Agent_Id agent, Direction direction) {
		for (auto& action : actions) {
			if (action.agent == agent) {
				action.direction = direction;
				return;
			}
		}
	}

	bool is_action_non_trivial(Agent_Id agent) const {
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

	size_t size() const {
		return actions.size();
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

	void add(Agent_Id agent) {
		agents.push_back(agent);
		generate_pretty_print();
	}

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

	bool operator==(const Agent_Combination& other) const {
		if (agents.size() != other.agents.size()) return false;
		for (size_t i = 0; i < agents.size(); ++i) {
			if (agents.at(i) != other.agents.at(i)) return false;
		}
		return true;
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

	std::vector<size_t> get_indices(Agent_Id agent) const {
		std::vector<size_t> result;
		size_t counter = 0;
		for (const auto& entry : agents) {
			if (entry == agent) result.push_back(counter);
			++counter;
		}
		return result;
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

	std::string to_string_raw() const {
		std::string result;
		for (const auto& agent : agents) {
			result += std::to_string(agent.id);
		}
		return result;
	}

	void remove(Agent_Id agent) {
		auto it = std::find(agents.begin(), agents.end(), agent);
		if (it != agents.end())	agents.erase(it);
	}

	std::vector<Agent_Id>::const_iterator begin() const {
		return agents.begin();
	}

	std::vector<Agent_Id>::const_iterator end() const {
		return agents.end();
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
		cutting_stations(), delivery_stations(), width(), height() {
		load_recipes();
	};

	bool			act(State& state, const Action& action) const;
	bool			act(State& state, const Action& action, Print_Level print_level) const;
	bool			act(State& state, const Joint_Action& action) const;
	bool			act(State& state, const Joint_Action& action, Print_Level print_level) const;
	Joint_Action	convert_to_joint_action(const Action& action, Agent_Id agent) const;
	bool			do_ingredients_lead_to_goal(const Ingredients& ingredients_count) const;
	bool			is_cell_type(const Coordinate& coordinate, const Cell_Type& type) const;
	bool			is_cell_type(const Coordinate& coordinate, const Direction& direction, const Cell_Type& type) const;
	bool			is_done(const State& state) const;
	bool			is_inbound(const Coordinate& coordiante) const;
	bool			is_type_stationary(Ingredient ingredient) const;
	State			load(const std::string& path);
	Coordinate		move(const Coordinate& coordinate, Direction direction) const;
	Coordinate		move_noclip(const Coordinate& coordinate, Direction direction) const;
	void			play(State& state) const;
	void			print_state() const;
	void			print_state(const State& state) const;

	std::vector<Action>			get_actions(Agent_Id agent) const;
	const std::vector<Recipe>&	get_all_recipes() const;
	std::vector<Coordinate>		get_coordinates(const State& state, Ingredient ingredient) const;
	size_t						get_height() const;
	std::vector<Joint_Action>	get_joint_actions(const Agent_Combination& agents) const;
	std::vector<Location>		get_locations(const State& state, Ingredient ingredient) const;
	std::vector<Coordinate>		get_neighbours(Coordinate location) const;
	std::vector<Location>		get_non_wall_locations(const State& state, Ingredient ingredient) const;
	size_t						get_number_of_agents() const;
	std::vector<Recipe>			get_possible_recipes(const State& state) const; 
	std::vector<Coordinate>		get_recipe_locations(const State& state, Ingredient ingredient) const;
	size_t						get_width() const;
	

private:
	void						calculate_recipes();
	bool						contains_collisions(const State& state, const Joint_Action& joint_action) const;
	bool						does_recipe_lead_to_goal(const Ingredients& ingredients_count, const Recipe& recipe_in) const;
	void						flip_walls_array();
	std::optional<Ingredient>	get_recipe(Ingredient ingredient1, Ingredient ingredient2) const;
	Ingredient					goal_name_to_ingredient(const std::string& name) const;
	void						load_map_line(State& state, size_t& line_counter, const std::string& line, size_t width);
	void						load_recipes();
	void						reset();



	size_t width;
	size_t height;
	size_t number_of_agents;

	std::vector<Coordinate>										agents_initial_positions;
	std::vector<Recipe>											all_recipes;
	std::vector<Coordinate>										cutting_stations;
	std::vector<Coordinate>										delivery_stations;
	Ingredients													goal_ingredients;
	std::vector<std::string>									goal_names;
	std::vector<Recipe>											goal_related_recipes;
	std::map<std::pair<Ingredient, Ingredient>, Ingredient>		recipes_map;
	std::vector<std::vector<bool>>								walls;


};

