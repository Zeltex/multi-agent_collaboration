#include "Heuristic.hpp"
#include "Utils.hpp"
#include "State.hpp"

#include <deque>
#include <cassert>

#define INFINITE_HEURISTIC 1000
struct Location_Info {
	Coordinate coordinate;
	size_t wall_penalty;
};

struct Search_Entry {
	Search_Entry(Coordinate coord, size_t dist, size_t walls) : coord(coord), dist(dist), walls(walls) {}
	Coordinate coord;
	size_t dist;
	size_t walls;
};

Heuristic::Heuristic(Environment environment) : environment(environment),
	ingredient1(Ingredient::DELIVERY), ingredient2(Ingredient::DELIVERY), agents(),
	handoff_agent(), agent_combinations(get_combinations(environment.get_number_of_agents())) {

	init();
}

void Heuristic::set(Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents,
	const std::optional<Agent_Id>& handoff_agent) {

	this->ingredient1 = ingredient1;
	this->ingredient2 = ingredient2;
	this->agents = agents;
	this->agent_combinations = get_combinations(agents);
	this->handoff_agent = handoff_agent;
}

// Returns the optimal path length, and total distance agents must move to assist in across-wall transfers
std::pair<size_t, size_t> Heuristic::get_helper_agents_distance(Coordinate source, Coordinate destination, 
	const State& state, const std::optional<Agent_Id> handoff_agent, const Agent_Combination& local_agents) const {
	
	if (source == destination) {
		return { 0, 0 };
	}

	auto agent_index = local_agents.size() - 1;
	const auto& dist_ref = distances.at(agent_index);
	const auto& dist_agent_ref = distances.at(0);
	auto prev = destination;
	size_t total_agent_dist = 0;
	size_t path_length = 1;
	bool first = true;
	while (true) {
		auto& prev_dist = dist_ref.const_at(source, prev);
		if (prev_dist.parent == source) break;
		if (prev_dist.parent == Coordinate{EMPTY_VAL, EMPTY_VAL}) {
			return { EMPTY_VAL, EMPTY_VAL };
		}

		auto next = prev_dist.parent;

		++path_length;
		if (environment.is_cell_type(next, Cell_Type::WALL)) {
			size_t min_dist = EMPTY_VAL;
			for (const auto& agent_id : agents.get()) {

				// First will be the last agent to move, i.e. should not be handoff_agent
				if (first && handoff_agent.has_value() && handoff_agent.value() == agent_id) {
					continue;
				}
				const auto& agent_coord = state.agents.at(agent_id.id).coordinate;
				const auto& dist = dist_agent_ref.const_at(agent_coord, prev);
				if (dist.g < min_dist) {
					min_dist = dist.g;
				}
			}
			first = false;
			total_agent_dist += min_dist;
		}
		prev = next;
	}
	assert((distances.at(agent_index).const_at(source, destination).g == path_length));
	return { total_agent_dist, path_length };
}

// Simply distance to nearest (non-handoff)agent
size_t Heuristic::get_nearest_agent_distance(const State& state, Coordinate location, const std::optional<Agent_Id>& handoff_agent) const {
	size_t min_dist = EMPTY_VAL;

	// Penalty if handoff agent is holding item
	size_t handoff_agent_penalty = 0;
	if (handoff_agent.has_value()) {
		if (state.agents.at(handoff_agent.value().id).coordinate == location) {
			handoff_agent_penalty = 1;
		}
	}

	for (const auto& agent : agents.get()) {

		// Invalid agent
		if (handoff_agent.has_value() && agent == handoff_agent.value().id) {
			continue;
		}
		const auto& agent_coord = state.agents.at(agent.id).coordinate;

		// Already holding
		if (location == agent_coord) {
			return 0;
		}
		auto temp_dist = distances.at(0).const_at(agent_coord, location).g;

		// Penalty for holding item
		if (state.agents.at(agent.id).item.has_value()) {
			++temp_dist;
		}

		min_dist = std::min(min_dist, temp_dist);
	}
	return min_dist + handoff_agent_penalty;
}

// Main heuristic calculation
size_t Heuristic::get_heuristic_distance(const Location& location1, const Location& location2, const State& state, const std::optional<Agent_Id>& handoff_agent, const Agent_Combination& local_agents) const {
	if (location1.original == location2.original) {
		return get_nearest_agent_distance(state, location1.original, handoff_agent);
	}

	size_t min_dist = EMPTY_VAL;
	size_t wall_penalty = (size_t)0
		+ (location1.from_wall ? 1 : 0)
		+ (location2.from_wall ? 1 : 0);
		//+ (environment.is_type_stationary(ingredient1)
		//	&& location1.coordinate != location2.coordinate ? 1 : 0);


	if (!environment.is_type_stationary(ingredient1)) {
		auto [agent_dist, path_length] = get_helper_agents_distance(location1.coordinate, location2.coordinate, state, handoff_agent, local_agents);
		if (agent_dist == EMPTY_VAL || path_length == EMPTY_VAL) return EMPTY_VAL;
		if (agent_dist == 0) {
			agent_dist += get_nearest_agent_distance(state, location1.coordinate, handoff_agent);
		} else {
			agent_dist += get_nearest_agent_distance(state, location1.coordinate, {});
		}
		min_dist = std::min(min_dist, agent_dist + path_length + wall_penalty);
	}
	auto [agent_dist, path_length] = get_helper_agents_distance(location2.coordinate, location1.coordinate, state, handoff_agent, local_agents);
	if (agent_dist == EMPTY_VAL || path_length == EMPTY_VAL) return EMPTY_VAL;
	if (agent_dist == 0) {
		agent_dist += get_nearest_agent_distance(state, location2.coordinate, handoff_agent);
	} else {
		agent_dist += get_nearest_agent_distance(state, location2.coordinate, {});
	}
	min_dist = std::min(min_dist, agent_dist + path_length + wall_penalty);
	return min_dist;
}

// Main heuristic function, setup
size_t Heuristic::operator()(const State& state, const std::optional<Agent_Id>& handoff_agent) const {
	std::vector<Location> locations1;
	if (environment.is_type_stationary(ingredient1)) {
		locations1 = environment.get_locations(state, ingredient1);
	} else {
		locations1 = environment.get_non_wall_locations(state, ingredient1);
	}
	auto locations2 = environment.get_non_wall_locations(state, ingredient2);

	// Agent set without handoff_agent
	auto reduced_agents = agents;
	if (handoff_agent.has_value()) {
		for (size_t i = 0; i < reduced_agents.size(); ++i) {
			if (reduced_agents.agents.at(i) == handoff_agent.value().id) {
				reduced_agents.agents.erase(reduced_agents.agents.begin() + i);
				break;
			}
		}
	}

	size_t min_dist = EMPTY_VAL;
	size_t number_of_agents = environment.get_number_of_agents();

	// Search all location combinations using all agents, and all agents minus handoff_agent
	for (const auto& location1 : locations1) {
		for (const auto& location2 : locations2) {
			min_dist = std::min(min_dist, get_heuristic_distance(location1, location2, state, handoff_agent, agents));
			if (handoff_agent.has_value()) {
				min_dist = std::min(min_dist, get_heuristic_distance(location1, location2, state, handoff_agent, reduced_agents));
			}
		}
	}
	return min_dist;
}

size_t Heuristic::convert(const Coordinate& coord1) const {
	return coord1.first * environment.get_height() + coord1.second;
}

void Heuristic::print_distances(Coordinate coordinate, size_t agent_number) const {
	std::cout << "\nPrinting distances for " << agent_number << " agents from (" << coordinate.first << ", " << coordinate.second << ")" << std::endl;
	for (size_t y = 0; y < environment.get_height(); ++y) {
		for (size_t x = 0; x < environment.get_width(); ++x) {
			const auto& temp = distances.at(agent_number - 1).const_at(coordinate, { x,y });
			std::cout << (temp.g == EMPTY_VAL ? "--" : (temp.g < 10 ? "0" : "") + std::to_string(temp.g)) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "\nPrinting directions for " << agent_number << " agents from (" << coordinate.first << ", " << coordinate.second << ")" << std::endl;
	for (size_t y = 0; y < environment.get_height(); ++y) {
		for (size_t x = 0; x < environment.get_width(); ++x) {
			const auto& temp = distances.at(agent_number - 1).const_at(coordinate, { x,y });
			std::cout << (temp.g == EMPTY_VAL ? '-' : static_cast<char>(get_direction({ x, y }, temp.parent))) << " ";
		}
		std::cout << std::endl;
	}
}

// All pairs shortest path for all amounts of agents, taking wall-handover in to account
void Heuristic::init() {
	// Get all possible coordinates
	std::vector<Coordinate> coordinates;
	for (size_t x = 0; x < environment.get_width(); ++x) {
		for (size_t y = 0; y < environment.get_height(); ++y) {
			coordinates.emplace_back(x, y);
		}
	}

	// Loop agent sizes
	for (size_t current_agents = 0; current_agents < environment.get_number_of_agents(); ++current_agents) {
		size_t max_walls = current_agents;
		distances.emplace_back(environment.get_width(), environment.get_height());
		auto& final_dist = distances.back();

		// Loop all source coordiantes
		for (const auto& source : coordinates) {
			std::vector<std::vector<Distance_Entry>> temp_distances(
				environment.get_number_of_agents(), std::vector<Distance_Entry>(environment.get_width() * environment.get_height()));

			Search_Entry origin{ source, 0, 0 };
			std::deque<Search_Entry> frontier;
			frontier.push_back(origin);
			temp_distances.at(0).at(convert(source)) = 0;
			while (!frontier.empty()) {
				auto& current = frontier.front();
				auto is_current_wall = environment.is_cell_type(current.coord, Cell_Type::WALL);

				// Check all directions
				for (const auto& destination : environment.get_neighbours(current.coord)) {
					if (!environment.is_inbound(destination)) {
						continue;
					}

					auto is_next_wall = environment.is_cell_type(destination, Cell_Type::WALL);

					// Check if path is valid
					if (is_current_wall && is_next_wall) {
						continue;
					}

					// Record
					auto& recorded_dist = temp_distances.at(current.walls).at(convert(destination));
					if (recorded_dist.g == EMPTY_VAL || current.dist + 1 < recorded_dist.g) {
						recorded_dist.g = current.dist + 1;
						recorded_dist.parent = current.coord;
						size_t wall_count = current.walls + (is_next_wall ? 1 : 0);
						if (wall_count <= max_walls) {
							frontier.emplace_back(destination, current.dist + 1, wall_count);
						}
					}
				}
				frontier.pop_front();
			}

			// Recorded the smallest dist among the distances from different wall values
			for (const auto& dist : temp_distances) {
				for (const auto& destination : coordinates) {
					auto& ref_dist = final_dist.at(source, destination);
					if (ref_dist.g == EMPTY_VAL || dist.at(convert(destination)).g < ref_dist.g) {
						ref_dist = dist.at(convert(destination));
					}
				}
			}

		}
	}

	//print_distances({ 5,1 }, 1);
	//print_distances({ 5,1 }, 2);
	//print_distances({ 0,1 }, 2);
	//print_distances({ 5,0 }, 2);
}