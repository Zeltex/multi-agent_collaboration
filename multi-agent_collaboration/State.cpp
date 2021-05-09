#include "State.hpp"

bool State::operator<(const State& other) const {
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

std::optional<Agent_Id> State::get_agent(Coordinate coordinate) const {
	size_t agent_index = 0;
	for (const auto& agent : agents) {
		if (agent.coordinate == coordinate) {
			return { agent_index };
		}
		++agent_index;
	}
	return {};
}

Agent State::get_agent(Agent_Id agent) const {
	assert(agent.id < agents.size());
	return agents.at(agent.id);
}

size_t State::get_count(Ingredient ingredient) const {
	size_t count = 0;
	for (const auto& [coord, ingredient_entry] : items) {
		if (ingredient == ingredient_entry) {
			++count;
		}
	}

	for (size_t i = 0; i < agents.size(); ++i) {
		auto& agent_item = agents.at(i).item;
		if (agent_item.has_value() && agent_item.value() == ingredient) {
			++count;
		}
	}
	return count;
}

std::optional<Ingredient> State::get_ingredient_at_position(Coordinate coordinate) const {
	auto it = items.find(coordinate);
	if (it != items.end()) {
		return it->second;
	} else {
		return {};
	}
}

Ingredients State::get_ingredients_count() const {
	Ingredients ingredients;
	for (const auto& [coord, ingredient] : items) {
		ingredients.add_ingredient(ingredient);
	}
	for (const auto& [coord, ingredient] : goal_items) {
		ingredients.add_ingredient(ingredient);
	}
	for (const auto& agent : agents) {
		if (agent.item.has_value()) {
			ingredients.add_ingredient(agent.item.value());
		}
	}
	return ingredients;
}

bool State::items_hoarded(const Recipe& recipe, const Agent_Combination& available_agents) const {
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

Coordinate State::get_location(Agent_Id agent) const {
	return agents.at(agent.id).coordinate;
}

bool State::contains_item(Ingredient ingredient) const {
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

void State::add(Coordinate coordinate, Ingredient ingredient) {
	items.insert({ coordinate, ingredient });
}

void State::add_goal_item(Coordinate coordinate, Ingredient ingredient) {
	goal_items.push_back({ coordinate, ingredient });
}

void State::remove(Coordinate coordinate) {
	items.erase(items.find(coordinate));
}

std::string State::to_hash_string() const {
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

size_t State::to_hash() const {
	return std::hash<std::string>()(this->to_hash_string());
}

bool State::operator==(const State& other) const {
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


std::vector<Coordinate> State::get_coordinates(Ingredient ingredient) const {
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

std::vector<Location> State::get_locations(Ingredient ingredient) const {
	std::vector<Location> result;
	for (const auto& item : items) {
		if (item.second == ingredient) {
			result.push_back({ item.first, item.first, false });
		}
	}
	for (const auto& agent : agents) {
		if (agent.item == ingredient) {
			result.push_back({ agent.coordinate, agent.coordinate, false });
		}
	}
	return result;
}

std::vector<Location> State::get_non_wall_locations(Ingredient ingredient, const Environment& environment) const {
	std::vector<Location> result;
	for (const auto& item : items) {
		if (item.second == ingredient) {
			if (environment.is_cell_type(item.first, Cell_Type::WALL)) {
				for (const auto& coord : environment.get_neighbours(item.first)) {
					if (environment.is_inbound(coord) && !environment.is_cell_type(coord, Cell_Type::WALL)) {
						result.push_back({ coord, item.first, true });
					}
				}
			} else {
				result.push_back({ item.first, item.first, false });
			}
		}
	}
	for (const auto& agent : agents) {
		if (agent.item == ingredient) {
			if (environment.is_cell_type(agent.coordinate, Cell_Type::WALL)) {
				for (const auto& coord : environment.get_neighbours(agent.coordinate)) {
					if (environment.is_inbound(coord) && !environment.is_cell_type(coord, Cell_Type::WALL)) {
						result.push_back({ coord, agent.coordinate, true });
					}
				}
			} else {
				result.push_back({ agent.coordinate, agent.coordinate, false });
			}

		}
	}
	return result;
}

void State::purge(const Agent_Combination& agents_keep) {
	for (size_t i = 0; i < agents.size(); ++i) {
		if (!agents_keep.contains({ i })) {
			agents.at(i).coordinate = { EMPTY_VAL, EMPTY_VAL };
		}
	}
}

void State::print_compact() const {
	for (const auto& item : items) {
		std::cout << "(" << static_cast<char>(item.second) << ", " << item.first.first << ", " << item.first.second << ") ";
	}
	size_t agent_counter = 0;
	for (const auto& agent : agents) {
		agent.print_compact({ agent_counter });
		++agent_counter;
	}
}
