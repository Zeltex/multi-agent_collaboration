#include "Environment.hpp"
#include "Core.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>

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

// TODO - Need to finish act
void Environment::act(State& state, const Action& action) const {
	Coordinate old_position = state.agents.at(action.agent.id);
	Coordinate new_position = old_position;
	switch (action.direction) {
	case Direction::UP: new_position = { new_position.first, new_position.second - 1 }; break;
	case Direction::RIGHT: new_position = { new_position.first + 1, new_position.second }; break;
	case Direction::DOWN: new_position = { new_position.first, new_position.second + 1 }; break;
	case Direction::LEFT: new_position = { new_position.first - 1, new_position.second }; break;
	}

	auto item_old_position = state.get_ingredient_at_position(old_position);
	auto item_new_position = state.get_ingredient_at_position(new_position);

	// Simple move
	if (!is_cell_type(new_position, Cell_Type::WALL)) {
		state.agents.at(action.agent.id) = new_position;
		if (item_old_position.has_value()) {
			state.remove(old_position);
			state.add(new_position, item_old_position.value());
		}
		PRINT(Print_Level::DEBUG, std::string("Moved ") + static_cast<char>(action.direction));

	// Goal condition
	} else if (is_cell_type(new_position, Cell_Type::DELIVERY_STATION)) {
		if (item_old_position.has_value() && item_old_position == goal_ingredient) {
			state.remove(old_position);
			state.add(new_position, item_old_position.value());
			PRINT(Print_Level::DEBUG, std::string("goal ingredient delivered ") + static_cast<char>(item_old_position.value()));
		} else {
			return;
		}

	// Combine
	} else if (item_new_position.has_value() && item_old_position.has_value()) {
		auto recipe = get_recipe(item_old_position.value(), item_new_position.value());
		if (recipe.has_value()) {
			state.remove(new_position);
			state.remove(old_position);
			state.add(new_position, recipe.value());
			PRINT(Print_Level::DEBUG, std::string("Combine ") + static_cast<char>(recipe.value()));
		} else {
			auto recipe_reverse = get_recipe(item_new_position.value(), item_old_position.value());
			if (recipe_reverse.has_value()) {
				state.remove(new_position);
				state.remove(old_position);
				state.add(new_position, recipe_reverse.value());
				PRINT(Print_Level::DEBUG, std::string("Combine ") + static_cast<char>(recipe_reverse.value()));
			}
		}


	// Chop chop / pickup
	} else if (is_cell_type(new_position, Cell_Type::CUTTING_STATION) && item_new_position.has_value()) {
		auto recipe = get_recipe(Ingredient::CUTTING, item_new_position.value());
		if (recipe.has_value()) {
			state.remove(new_position);
			state.add(new_position, recipe.value());
			PRINT(Print_Level::DEBUG, std::string("chop chop ") + static_cast<char>(recipe.value()));
		} else if (!item_old_position.has_value()){
			state.remove(new_position);
			state.add(old_position, item_new_position.value());
			PRINT(Print_Level::DEBUG, std::string("pickup"));
		}


	// Place
	} else if (item_old_position.has_value()) {
		state.remove(old_position);
		state.add(new_position, item_old_position.value());
		PRINT(Print_Level::DEBUG, std::string("place ") + static_cast<char>(item_old_position.value()));

	// Pickup
	} else if (item_new_position.has_value()) {
		state.remove(new_position);
		state.add(old_position, item_new_position.value());
		PRINT(Print_Level::DEBUG, std::string("pickup ") + static_cast<char>(item_new_position.value()));
	} else {
		// They are simply humping a wall
	}
}

std::vector<Action> Environment::get_actions(const State& state) const {
	return {};
}

State Environment::load(const std::string& path) {
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
			goal_name = line;
			goal_ingredient = goal_name_to_ingredient(line);
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
			auto agent_it = std::find(state.agents.begin(), state.agents.end(), Coordinate{ x, y });

			if (state.items.find({ x, y }) != state.items.end()) {
				buffer += static_cast<char>(state.items.at({ x, y }));

			} else if (agent_it != state.agents.end()) {
				char buf[2];
				_itoa_s(std::distance(state.agents.begin(), agent_it), buf, 10);
				buffer += buf[0];
			
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
	std::cout << "type {w, a, s, d} to move, {0-9} to switch agent, and {q} to quit" << std::endl;
	for (std::string line; std::getline(std::cin, line) && !done;) {
		char c = line[0];
		switch (c) {
		case 'q': done = true; break;
		case 'w': act(state, { Direction::UP, agent }); break;
		case 'a': act(state, { Direction::LEFT, agent }); break;
		case 's': act(state, { Direction::DOWN, agent }); break;
		case 'd': act(state, { Direction::RIGHT, agent }); break;
		}
		if (c >= '0' && c <= '9') {
			agent = { static_cast<size_t>(atoi(&c)) };
			PRINT(Print_Level::DEBUG, std::string("Switcharoo ") + c);
		}
		print_state(state);
	}
}

Ingredient Environment::goal_name_to_ingredient(const std::string& name) const {
	if (name == "Salad") {
		return Ingredient::PLATED_SALAD;
	} else {
		std::cerr << "Unknown goal ingredient " << name << std::endl;
		exit(-1);
	}
}

// Not a great way to define recipes, but functional for now
std::optional<Ingredient> Environment::get_recipe(Ingredient ingredient1, Ingredient ingredient2) const {
	static const std::map<std::pair<Ingredient, Ingredient>, Ingredient> recipes = {
		{ {Ingredient::CUTTING, Ingredient::TOMATO}, Ingredient::CHOPPED_TOMATO},
		{ {Ingredient::CUTTING, Ingredient::LETTUCE}, Ingredient::CHOPPED_LETTUCE},

		{ {Ingredient::PLATE, Ingredient::CHOPPED_LETTUCE}, Ingredient::PLATED_LETTUCE},
		{ {Ingredient::PLATE, Ingredient::CHOPPED_TOMATO}, Ingredient::PLATED_TOMATO},
		{ {Ingredient::PLATE, Ingredient::SALAD}, Ingredient::PLATED_SALAD},

		{ {Ingredient::CHOPPED_LETTUCE, Ingredient::CHOPPED_TOMATO}, Ingredient::SALAD},
		{ {Ingredient::PLATED_LETTUCE, Ingredient::CHOPPED_TOMATO}, Ingredient::PLATED_SALAD},
		{ {Ingredient::PLATED_TOMATO, Ingredient::CHOPPED_LETTUCE}, Ingredient::PLATED_SALAD},
	};
	auto recipe_it = recipes.find({ ingredient1, ingredient2 });
	if (recipe_it != recipes.end()) {
		return recipe_it->second;
	} else {
		return {};
	}
}