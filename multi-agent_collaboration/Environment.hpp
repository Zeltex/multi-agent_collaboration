#pragma once

#include "Core.hpp"

#include <vector>
#include <map>
#include <utility>
#include <string>
#include <optional>
#include <iostream>

using Coordinate = std::pair<size_t, size_t> ;

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
	Recipe(Ingredient ingredient1, Ingredient ingredient2, Ingredient result) :
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
	char result_char() const {
		return static_cast<char>(result);
	}
};

struct Action {
	Action() : direction(Direction::NONE), agent(EMPTY_VAL) {};
	Action(Direction direction, Agent_Id agent) : direction(direction), agent(agent) {};
	Direction direction;
	Agent_Id agent;

	std::string to_string() const {
		return std::to_string(static_cast<char>(direction)) + std::to_string(agent.id);
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
		if (actions.size() <= agent.id)  return false;
		return actions.at(agent.id).direction != Direction::NONE;
	}

	Action get_action(Agent_Id agent) const {
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
	Agent_Combination(std::vector<Agent_Id> agents) : agents(agents) { generate_pretty_print(); }
	std::vector<Agent_Id> agents;
	std::string pretty_print;

	bool operator<(const Agent_Combination& other) const {
		if (agents.size() != other.agents.size()) return agents.size() < other.agents.size();
		for (size_t i = 0; i < agents.size(); ++i) {
			if (agents.at(i) != other.agents.at(i)) return agents.at(i) < other.agents.at(i);
		}
		return false;
	}

	bool contains(Agent_Id agent) const {
		return std::find(agents.begin(), agents.end(), agent) != agents.end();
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


struct State {
	std::map<Coordinate, Ingredient> items;
	std::vector<std::pair<Coordinate, Ingredient>> goal_items;
	std::vector<Agent> agents;

	bool operator<(const State& other) const {
		std::cout << "<state" << std::endl;
		if (this->items.size() != other.items.size()) return this->items.size() < other.items.size();
		if (this->goal_items.size() != other.goal_items.size()) return this->goal_items.size() < other.goal_items.size();
		if (this->agents.size() != other.agents.size()) return this->agents.size() < other.agents.size();

		if (this->items < other.items) return true;
		if (this->items > other.items) return false;
		if (this->goal_items < other.goal_items) return true;
		if (this->goal_items > other.goal_items) return false;
		if (this->agents < other.agents) return true;
		//if (this->agents > other.agents) return false;
		return false;
	}

	std::optional<Ingredient> get_ingredient_at_position(Coordinate coordinate) const {
		auto it = items.find(coordinate);
		if (it != items.end()) {
			return it->second;
		} else {
			return {};
		}
	}

	bool items_hoarded(const Recipe& recipe, const Agent_Combination& available_agents) const {
		std::vector<Ingredient> items_needed{ recipe.ingredient1, recipe.ingredient2 };
		for (const auto& item : items) {
			auto it = std::find(items_needed.begin(), items_needed.end(), item.second);
			if (it != items_needed.end()) {
				items_needed.erase(it);
				if (items_needed.empty()) false;
			}
		}

		for (size_t i = 0; i < agents.size(); ++i) {
			if (agents.at(i).item.has_value()) {
				if (std::find(items_needed.begin(), items_needed.end(), agents.at(i).item.value()) != items_needed.end()
					&& !available_agents.contains({ i })) {
					return true;
				}
			}
		}
		return false;
	}

	Coordinate get_location(Agent_Id agent) const {
		return agents.at(agent.id).coordinate;
	}

	bool contains_item(Ingredient ingredient) const {
		for (const auto& item : items) {
			if (item.second == ingredient) return true;
		}
		for (const auto& item : goal_items) {
			if (item.second == ingredient) return true;
		}
		for (const auto& agent : agents) {
			if (agent.item == ingredient) return true;
		}
		return false;
	}

	void add(Coordinate coordinate, Ingredient ingredient) {
		items.insert({ coordinate, ingredient });
	}

	void add_goal_item(Coordinate coordinate, Ingredient ingredient) {
		goal_items.push_back({ coordinate, ingredient });
	}

	void remove(Coordinate coordinate) {
		items.erase(items.find(coordinate));
	}

	std::string to_hash_string() const {
		std::string hash;
		hash += std::to_string(items.size() * 128 + agents.size());
		for (const auto& item : items) {
			hash += std::to_string(item.first.first * 128 + item.first.second)
				+ static_cast<char>(item.second);
		}

		for (const auto& agent : agents) {
			hash += std::to_string(agent.coordinate.first * 128 + agent.coordinate.second);
			if (agent.item.has_value()) {
				hash += static_cast<char>(agent.item.value());
			}
			hash += "0";
		}
		return hash;
	}

	size_t to_hash() const {
		return std::hash<std::string>()(this->to_hash_string());
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

	std::vector<Coordinate> get_locations(Ingredient ingredient) const {
		std::vector<Coordinate> result;
		for (const auto& item : items) {
			if (item.second == ingredient) {
				result.push_back(item.first);
			}
		}
		for (const auto& agent : agents) {
			if (agent.item == ingredient) {
				result.push_back(agent.coordinate);
			}
		}
		return result;
	}

	void print_compact() const {
		for (const auto& item : items) {
			std::cout << "(" << static_cast<char>(item.second) << ", " << item.first.first << ", " << item.first.second << ") ";
		}
		size_t agent_counter = 0;
		for (const auto& agent : agents) {
			agent.print_compact({ agent_counter });
			++agent_counter;
		}
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
		number_of_agents(number_of_agents), goal_names(), agents_initial_positions(), walls(), 
		cutting_stations(), delivery_stations(), width(), height() {};

	bool is_inbound(const Coordinate& coordiante) const;
	bool is_cell_type(const Coordinate& coordinate, const Cell_Type& type) const;
	bool is_type_stationary(Ingredient ingredient) const;
	bool act(State& state, const Action& action) const;
	bool act(State& state, const Action& action, Print_Level print_level) const;
	bool act(State& state, const Joint_Action& action) const;
	bool act(State& state, const Joint_Action& action, Print_Level print_level) const;
	std::vector<Action> get_actions(Agent_Id agent) const;
	std::vector<Joint_Action> get_joint_actions(const Agent_Combination& agents) const;
	State load(const std::string& path);
	void print_state() const;
	void print_state(const State& state) const;
	void play(State& state) const;
	std::vector<Recipe> get_possible_recipes(const State& state) const; 
	const std::vector<Recipe>& get_all_recipes() const;
	std::vector<Ingredient> get_goal() const;
	bool is_done(const State& state) const;
	size_t get_number_of_agents() const;
	Joint_Action convert_to_joint_action(const Action& action, Agent_Id agent) const;
	std::vector<Coordinate> get_locations(const State& state, Ingredient ingredient) const;
	std::vector<Coordinate> get_recipe_locations(const State& state, Ingredient ingredient) const;
	size_t get_width() const;
	size_t get_height() const;
	std::vector<Coordinate> get_neighbours(Coordinate location) const;
	

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

