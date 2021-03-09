#include "Environment.hpp"
#include "Core.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <set>

bool Environment::is_cell_type(const Coordinate& coordinate, const Cell_Type& type) const {
	switch (type) {
	case Cell_Type::WALL: {
		return walls.at(coordinate.first).at(coordinate.second);
		break;
	}
	case Cell_Type::CUTTING_STATION: {
		return std::find(cutting_stations.begin(), cutting_stations.end(), coordinate) != cutting_stations.end();
		break;
	}
	case Cell_Type::DELIVERY_STATION: {
		return std::find(delivery_stations.begin(), delivery_stations.end(), coordinate) != delivery_stations.end();
		break;
	}
	}
	std::cerr << "Unknown cell type" << std::endl;
	exit(-1);
	return false;
}

void Environment::act(State& state, const Joint_Action& action) const {
	return act(state, action, Print_Level::DEBUG);
}

void Environment::act(State& state, const Joint_Action& joint_action, Print_Level print_level) const {
	auto temp_action = joint_action; // Need to modify the action, potentially take non-const as argument instead
	check_collisions(state, temp_action);
	for (const auto& action : temp_action.actions) {
		act(state, action, print_level);
	}
}

void Environment::act(State& state, const Action& action) const {
	return act(state, action, Print_Level::DEBUG);
}

void Environment::act(State& state, const Action& action, Print_Level print_level) const {
	auto& agent = state.agents.at(action.agent.id);
	Coordinate old_position = agent.coordinate;
	Coordinate new_position = move_noclip(old_position, action.direction);	

	auto item_old_position = agent.item;
	auto item_new_position = state.get_ingredient_at_position(new_position);

	// Simple move
	if (!is_cell_type(new_position, Cell_Type::WALL)) {
		state.agents.at(action.agent.id).move_to(new_position);
		PRINT(print_level, std::string("Moved ") + static_cast<char>(action.direction));

	// Goal condition
	} else if (is_cell_type(new_position, Cell_Type::DELIVERY_STATION)) {
		if (item_old_position.has_value()) {
			auto recipe = get_recipe(Ingredient::DELIVERY, item_old_position.value());
			if (recipe.has_value()) {
				agent.clear_item();
				state.add_goal_item(new_position, recipe.value());
				PRINT(print_level, std::string("goal ingredient delivered ") + static_cast<char>(recipe.value()));
			}
		}

	// Combine
	} else if (item_new_position.has_value() && item_old_position.has_value()) {
		auto recipe = get_recipe(item_old_position.value(), item_new_position.value());
		if (recipe.has_value()) {
			state.remove(new_position);
			agent.clear_item();
			state.add(new_position, recipe.value());
			PRINT(print_level, std::string("Combine ") + static_cast<char>(recipe.value()));
		} else {
			auto recipe_reverse = get_recipe(item_new_position.value(), item_old_position.value());
			if (recipe_reverse.has_value()) {
				state.remove(new_position);
				agent.clear_item();
				state.add(new_position, recipe_reverse.value());
				PRINT(print_level, std::string("Combine ") + static_cast<char>(recipe_reverse.value()));
			}
		}


	// Chop chop / pickup
	} else if (is_cell_type(new_position, Cell_Type::CUTTING_STATION) && item_new_position.has_value()) {
		auto recipe = get_recipe(Ingredient::CUTTING, item_new_position.value());
		if (recipe.has_value()) {
			state.remove(new_position);
			state.add(new_position, recipe.value());
			PRINT(print_level, std::string("chop chop ") + static_cast<char>(recipe.value()));
		} else if (!item_old_position.has_value()){
			state.remove(new_position);
			agent.set_item(item_new_position.value());
			PRINT(print_level, std::string("pickup"));
		}


	// Place
	} else if (item_old_position.has_value()) {
		agent.clear_item();
		state.add(new_position, item_old_position.value());
		PRINT(print_level, std::string("place ") + static_cast<char>(item_old_position.value()));

	// Pickup
	} else if (item_new_position.has_value()) {
		state.remove(new_position);
		agent.set_item(item_new_position.value());
		PRINT(print_level, std::string("pickup ") + static_cast<char>(item_new_position.value()));
	} else {
		// They are simply humping a wall
		PRINT(print_level, std::string("Nothing happened"));
	}
}

// This collision check is basically a one to one from the BD, but I believe it allows two agents to collide if one is no'opping, need to check this
// UPDATE: the original code did allow collision with stationary agents, changed it to disallow this even though I believe that BD allows it
void Environment::check_collisions(const State& state, Joint_Action& joint_action) const {
	std::vector<Coordinate> current_coordinates;
	std::vector<Coordinate> next_coordinates;
	std::vector<Coordinate> action_coordinates;

	std::set<size_t> cancelled_agents;
	
	for (size_t agent = 0; agent < joint_action.actions.size(); ++agent) {
		current_coordinates.push_back(state.agents.at(agent).coordinate);
		next_coordinates.push_back(move(state.agents.at(agent).coordinate, joint_action.actions.at(agent).direction));
		action_coordinates.push_back(move_noclip(state.agents.at(agent).coordinate, joint_action.actions.at(agent).direction));
	}

	for (size_t agent1 = 0; agent1 < joint_action.actions.size(); ++agent1) {
		for (size_t agent2 = agent1+1; agent2 < joint_action.actions.size(); ++agent2) {
			if (next_coordinates.at(agent1) == next_coordinates.at(agent2)) {
				
				// Agent1 still, agent2 invalid
				if (current_coordinates.at(agent1) == next_coordinates.at(agent1)) {//&& joint_action.actions.at(agent1).direction != Direction::NONE) {
					cancelled_agents.insert(agent2);

				// Agent2 still, agent1 invalid
				} else if (current_coordinates.at(agent2) == next_coordinates.at(agent2)){// && joint_action.actions.at(agent2).direction != Direction::NONE) {
					cancelled_agents.insert(agent1);
				} else {
					cancelled_agents.insert(agent1);
					cancelled_agents.insert(agent2);
				}
			// Swap (invalid)
			} else if (current_coordinates.at(agent1) == next_coordinates.at(agent2) 
				&& current_coordinates.at(agent2) == next_coordinates.at(agent1)) {
				cancelled_agents.insert(agent1);
				cancelled_agents.insert(agent2);

			// Accessing same counter space
			} else if (action_coordinates.at(agent1) == action_coordinates.at(agent2)) {
				cancelled_agents.insert(agent1);
				cancelled_agents.insert(agent2);
			}
		}
	}

	for (const auto& agent : cancelled_agents) {
		joint_action.actions.at(agent).direction = Direction::NONE;
	}

}

std::vector<Action> Environment::get_actions(const State& state, Agent_Id agent) const {
	return { {Direction::UP, agent}, {Direction::RIGHT, agent}, {Direction::DOWN, agent}, {Direction::LEFT, agent}, {Direction::NONE, agent} };
}


#define NONE_INDEX 4
std::vector<Joint_Action> Environment::get_joint_actions(const State& state) const {
	std::vector<std::vector<Action>> single_actions;
	std::vector<size_t> counters;
	for (size_t agent = 0; agent < number_of_agents; ++agent) {
		single_actions.push_back(get_actions(state, agent));
		counters.emplace_back(0);
	}

	std::vector<Joint_Action> joint_actions;

	bool done = false;
	while (!done) {
		std::vector<Action> actions;
		for (size_t i = 0; i < counters.size(); i++) {
			actions.push_back(single_actions[i][counters[i]]);
		}
		joint_actions.push_back(std::move(actions));

		// Advance indices
		size_t index = 0;
		while (!done && ++counters.at(index) >= single_actions.at(index).size()) {
			counters.at(index++) = 0;
			done = index >= counters.size();
		}
	}
	return joint_actions;
}

State Environment::load(const std::string& path) {
	reset();
	std::ifstream file;
	file.open(path);
	std::string line;
	
	if (!file) {
		std::cerr << "Could not load file at path " << path << std::endl;
		exit(-1);
	}

	State state;

	size_t load_status = 0;
	size_t width;
	size_t line_counter = 0;
	while (std::getline(file, line)) {
		if (line_counter == 0) {
			width = line.size();
		}

		if (line.empty()) {
			++load_status;
			continue;
		}

		// Level file as defined by BD paper is split in 3 sections
		switch (load_status) {
		case 0: {
			load_map_line(state, line_counter, line, width);
			break;
		}
		case 1: { 
			goal_names.push_back(line);
			goal_ingredients.push_back(goal_name_to_ingredient(line));
			break; 
		}
		case 2: {
			int x = atoi(&line[0]);
			int y = atoi(&line[2]);
			agents_initial_positions.push_back({ x, y });
			break;
		}
		}

		std::cout << line_counter << ": " << line << std::endl;
		++line_counter;
	}

	for (size_t agent = 0; agent < number_of_agents; ++agent) {
		state.agents.push_back(agents_initial_positions.at(agent));
	}

	calculate_recipes();

	flip_walls_array();
	file.close();
	return state;
}

void Environment::load_map_line(State& state, size_t& line_counter, const std::string& line, size_t width) {

	std::vector<bool> wall_line;
	wall_line.reserve(width);
	size_t index_counter = 0;
	for (const auto& c : line) {

		// Environment related
		if (c != ' ') wall_line.push_back(true);
		else wall_line.push_back(false);
		if (c == static_cast<char>(Cell_Type::CUTTING_STATION)) cutting_stations.push_back({ index_counter, line_counter });
		if (c == static_cast<char>(Cell_Type::DELIVERY_STATION)) delivery_stations.push_back({ index_counter, line_counter });

		// State related
		if (c == static_cast<char>(Ingredient::TOMATO)) 
			state.items.insert({ {index_counter, line_counter}, Ingredient::TOMATO });

		if (c == static_cast<char>(Ingredient::LETTUCE)) state.items.insert({ {index_counter, line_counter}, Ingredient::LETTUCE });
		if (c == static_cast<char>(Ingredient::PLATE)) state.items.insert({ {index_counter, line_counter}, Ingredient::PLATE });

		++index_counter;
	}
	walls.push_back(wall_line);
}

void Environment::flip_walls_array() {

	// Flips walls array so the first coordinate is x, not y
	auto walls_copy = walls;
	for (size_t y = 0; y < walls_copy.size(); ++y) {
		for (size_t x = 0; x < walls_copy.at(0).size(); ++x) {
			walls.at(x).at(y) = walls_copy.at(y).at(x);
		}
	}
}

void Environment::print_state() const {
	State state;
	print_state(state);
}
void Environment::print_state(const State& state) const {
	std::string buffer;
	for (size_t y = 0; y < walls.size(); ++y) {
		for (size_t x = 0; x < walls.at(0).size(); ++x) {
			auto agent_it = std::find_if(state.agents.begin(), state.agents.end(), [x, y](Agent agent)->bool {return agent.coordinate == Coordinate{x, y}; });
			//auto agent_it = std::find_if(state.agents.begin(), state.agents.end(), Coordinate{ x, y }, [x, y](Agent agent)->bool {return agent.coordinate == {x, y}; });

			if (state.items.find({ x, y }) != state.items.end()) {
				buffer += static_cast<char>(state.items.at({ x, y }));

			} else if (agent_it != state.agents.end()) {
				if (agent_it->item.has_value()) {
					buffer += static_cast<char>(agent_it->item.value());
				} else {
					char buf[2];
					_itoa_s(std::distance(state.agents.begin(), agent_it), buf, 10);
					buffer += buf[0];
				}
			
			} else if (std::find(cutting_stations.begin(), cutting_stations.end(), Coordinate{ x, y }) != cutting_stations.end()) {
				buffer += static_cast<char>(Cell_Type::CUTTING_STATION);
			
			} else if (std::find(delivery_stations.begin(), delivery_stations.end(), Coordinate{ x, y }) != delivery_stations.end()) {
				buffer += static_cast<char>(Cell_Type::DELIVERY_STATION);
			
			} else if (walls.at(x).at(y)) {
				buffer += static_cast<char>(Cell_Type::WALL);
			
			} else {
				buffer += ' ';
			}
		
		}
		buffer += '\n';
	}
	std::cout << buffer << std::endl;
}

void Environment::play(State& state) const {
	bool done = false;
	Agent_Id agent = 0;
	std::cout << "type {w, a, s, d} to move single agent, {w, a, s, d, n}* to move multiple agents, {0-9} to switch single agent, and {q} to quit" << std::endl;
	for (std::string line; std::getline(std::cin, line) && !done;) {
		char c = line[0];
		if (c >= '0' && c <= '9') {
			agent = { static_cast<size_t>(atoi(&c)) };
			PRINT(Print_Level::DEBUG, std::string("Switcharoo ") + c);
		} else {

			std::vector<Action> actions;
			for (size_t i = 0; i < number_of_agents; ++i) {
				actions.push_back({ Direction::NONE, i });
			}

			for (size_t i = 0; i < line.size(); ++i) {
				Direction dir = Direction::NONE;
				switch (line.at(i)) {
				case 'q': done = true; break;
				case 'w': dir = Direction::UP; break;
				case 'a': dir = Direction::LEFT; break;
				case 's': dir = Direction::DOWN; break;
				case 'd': dir = Direction::RIGHT; break;
				}
				if (line.size() > 1) {
					actions.at(i).direction = dir;
				} else {
					actions.at(agent.id).direction = dir;
				}
			}
			act(state, { actions });
		}

		print_state(state);
	}
}

Ingredient Environment::goal_name_to_ingredient(const std::string& name) const {
	if (name == "Salad") {
		return Ingredient::DELIVERED_SALAD;
	} else if (name == "SimpleTomato") {
		return Ingredient::DELIVERED_TOMATO;
	} else if (name == "SimpleLettuce") {
		return Ingredient::DELIVERED_LETTUCE;
	} else {
		std::cerr << "Unknown goal ingredient " << name << std::endl;
		exit(-1);
	}
}

const std::map<std::pair<Ingredient, Ingredient>, Ingredient>& Environment::get_recipes() const {
	static const std::map<std::pair<Ingredient, Ingredient>, Ingredient> recipes = {
	{ {Ingredient::CUTTING, Ingredient::TOMATO}, Ingredient::CHOPPED_TOMATO},
	{ {Ingredient::CUTTING, Ingredient::LETTUCE}, Ingredient::CHOPPED_LETTUCE},

	{ {Ingredient::PLATE, Ingredient::CHOPPED_LETTUCE}, Ingredient::PLATED_LETTUCE},
	{ {Ingredient::PLATE, Ingredient::CHOPPED_TOMATO}, Ingredient::PLATED_TOMATO},
	{ {Ingredient::PLATE, Ingredient::SALAD}, Ingredient::PLATED_SALAD},

	{ {Ingredient::CHOPPED_LETTUCE, Ingredient::CHOPPED_TOMATO}, Ingredient::SALAD},
	{ {Ingredient::PLATED_LETTUCE, Ingredient::CHOPPED_TOMATO}, Ingredient::PLATED_SALAD},
	{ {Ingredient::PLATED_TOMATO, Ingredient::CHOPPED_LETTUCE}, Ingredient::PLATED_SALAD},

	{ {Ingredient::DELIVERY, Ingredient::PLATED_SALAD}, Ingredient::DELIVERED_SALAD},
	{ {Ingredient::DELIVERY, Ingredient::PLATED_TOMATO}, Ingredient::DELIVERED_TOMATO},
	{ {Ingredient::DELIVERY, Ingredient::PLATED_LETTUCE}, Ingredient::DELIVERED_LETTUCE},
	};
	return recipes;
}

// Not a great way to define recipes, but functional for now
std::optional<Ingredient> Environment::get_recipe(Ingredient ingredient1, Ingredient ingredient2) const {
	auto& recipes = get_recipes();
	auto recipe_it = recipes.find({ ingredient1, ingredient2 });
	if (recipe_it != recipes.end()) {
		return recipe_it->second;
	} else {
		return {};
	}
}


std::vector<Recipe> Environment::get_possible_recipes(const State& state) const {
	//auto& recipes = get_recipes();
	std::vector<Recipe> possible_recipes;

	for (const auto& recipe : goal_related_recipes) {

		// TODO - Should handle cutting/delivery stations in a little less hardcoded way
		if ((state.contains_item(recipe.ingredient1) || recipe.ingredient1 == Ingredient::CUTTING || recipe.ingredient1 == Ingredient::DELIVERY) && state.contains_item(recipe.ingredient2)) {
			possible_recipes.push_back(recipe);
		}
	}
	return possible_recipes;
}

std::vector<Ingredient> Environment::get_goal() const {
	return goal_ingredients;
}

bool Environment::is_done(const State& state) const {
	for (const auto& item : goal_ingredients) {
		if (!state.contains_item(item)) return false;
	}
	return true;
}

Coordinate Environment::move(const Coordinate& coordinate, Direction direction) const {
	Coordinate new_coordinate = coordinate;
	switch (direction) {
	case Direction::UP:		new_coordinate = { coordinate.first, coordinate.second - 1 }; break;
	case Direction::RIGHT:	new_coordinate = { coordinate.first + 1, coordinate.second }; break; 
	case Direction::DOWN:	new_coordinate = { coordinate.first, coordinate.second + 1 }; break; 
	case Direction::LEFT:	new_coordinate = { coordinate.first - 1, coordinate.second }; break; 
	default: return coordinate;
	}
	if (walls.at(new_coordinate.first).at(new_coordinate.second)) {
		return coordinate;
	} else {
		return new_coordinate;
	}
}

Coordinate Environment::move_noclip(const Coordinate& coordinate, Direction direction) const {
	switch (direction) {
	case Direction::UP:		return { coordinate.first, coordinate.second - 1 };
	case Direction::RIGHT:	return { coordinate.first + 1, coordinate.second };
	case Direction::DOWN:	return { coordinate.first, coordinate.second + 1 };
	case Direction::LEFT:	return { coordinate.first - 1, coordinate.second };
	default: return coordinate;
	}
}

size_t Environment::get_number_of_agents() const {
	return number_of_agents;
}


Joint_Action Environment::convert_to_joint_action(const Action& action, Agent_Id agent) const {
	std::vector<Action> actions;
	for (size_t i = 0; i < number_of_agents; ++i) {
		if (i == agent.id) {
			actions.push_back(action);
		} else {
			actions.push_back({ Direction::NONE, {i} });
		}
	}
	return { actions };
}

std::vector<Coordinate> Environment::get_locations(const State& state, Ingredient ingredient) const {
	switch (ingredient) {
	case Ingredient::CUTTING: return cutting_stations;
	case Ingredient::DELIVERY: return delivery_stations;
	default: return state.get_locations(ingredient);
	}
}

std::vector<Coordinate> Environment::get_recipe_locations(const State& state, Ingredient ingredient) const {
	exit(-1);
}

void Environment::reset() {
	goal_names.clear();
	goal_ingredients.clear();
	agents_initial_positions.clear();

	walls.clear();
	cutting_stations.clear();
	delivery_stations.clear();

}


void Environment::calculate_recipes() {
	auto& recipes = get_recipes();

	std::map<Ingredient, std::pair<Ingredient, Ingredient>> reversed_recipes;
	for (const auto& recipe : recipes) {
		reversed_recipes.insert({ recipe.second, {recipe.first.first, recipe.first.second} });
	}

	std::set<Ingredient> ingredients;
	for (const auto& ingredient : goal_ingredients) {
		ingredients.insert(ingredient);
	}


	bool done = false;
	while (!done) {
		auto ingredients_size = ingredients.size();
		for (const auto& recipe : recipes) {
			if (ingredients.find(recipe.second) != ingredients.end()) {
				ingredients.insert(recipe.first.first);
				ingredients.insert(recipe.first.second);
			}
		}
		if (ingredients_size == ingredients.size()) {
			break;
		}
	}

	std::vector<Recipe> result;
	for (const auto& recipe : recipes) {
		if (ingredients.find(recipe.second) != ingredients.end()) {
			result.push_back({ recipe.first.first, recipe.first.second, recipe.second });
		}
	}
	goal_related_recipes = result;
}