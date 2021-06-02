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
	Search_Entry(Coordinate coord, size_t dist, size_t walls, size_t wall_g) 
		: coord(coord), dist(dist), walls(walls), wall_g(wall_g) {}
	Coordinate coord;
	size_t dist;
	size_t walls;
	size_t wall_g;
};

Heuristic::Heuristic(Environment environment) : environment(environment),
	ingredient1(Ingredient::DELIVERY), ingredient2(Ingredient::DELIVERY),
	handoff_agent(), agent_combinations(get_combinations(environment.get_number_of_agents())) {

	init();
}

void Heuristic::set(Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents,
	const Agent_Id& handoff_agent) {

	this->ingredient1 = ingredient1;
	this->ingredient2 = ingredient2;
	this->agent_combinations = get_combinations(agents);
	this->handoff_agent = handoff_agent;
}

size_t Heuristic::get_distance_to_nearest_wall(Coordinate agent_coord, Coordinate blocked, const State& state) const {
	if (agent_coord != blocked && environment.is_cell_type(agent_coord, Cell_Type::WALL)) {
		return 0;
	}
	constexpr size_t max_search_dist = 3;
	for (size_t i = 1; i <= max_search_dist; ++i) {
		for (size_t j = 0; j <= i; ++j) {
			size_t a = j;		// Dimension 1 offset
			size_t b = i - j;	// Dimension 2 offset

			std::vector<Coordinate> coords;
			coords.emplace_back(agent_coord.first + a, agent_coord.second + b);
			coords.emplace_back(agent_coord.first + a, agent_coord.second - b);
			coords.emplace_back(agent_coord.first - a, agent_coord.second + b);
			coords.emplace_back(agent_coord.first - a, agent_coord.second - b);
			for (const auto& coord : coords) {
				if (environment.is_inbound(coord)
					&& environment.is_cell_type(coord, Cell_Type::WALL) 
					&& !state.is_wall_occupied(coord) 
					&& coord != blocked) {
					
					return i;
				}
			}
		}
	}
	return max_search_dist + 1;
}

Helper_Agent_Info Heuristic::find_helper(const std::vector<Helper_Agent_Info>& helpers, const Agent_Id handoff_agent, const Agent_Combination& local_agents, const bool first,
	const State& state, const Coordinate& prev, const Coordinate& next, const size_t path_length) const {

	size_t min_dist = EMPTY_VAL;
	Agent_Id min_agent = {};
	const auto& dist_agent_ref = distances.at(0);
	for (const auto& agent_id : local_agents) {

		// First will be the last agent to move, i.e. should not be handoff_agent
		if (first && handoff_agent.is_not_empty() && handoff_agent == agent_id) {
			continue;
		}
		const auto& agent_ref = state.agents.at(agent_id.id);
		const auto& agent_coord = agent_ref.coordinate;
		const auto& dist = dist_agent_ref.const_at(agent_coord, prev);
		size_t holding_penalty = 0;

		if (agent_ref.item.has_value() 
			&& agent_ref.item.value() != ingredient1 
			&& agent_ref.item.value() != ingredient2) {
			
			holding_penalty = get_distance_to_nearest_wall(agent_coord, next, state);
		}

		if (dist.g + holding_penalty < min_dist) {
			min_dist = dist.g + holding_penalty;
			min_agent = agent_id;
		}
	}
	// The -1 is to account for the wall (i.e. path_length is linked to next, 
	// but helper spot is linked to prev)
	return { path_length - 1, min_dist, min_agent};
}

// Returns the optimal path length, and total distance agents must move to assist in across-wall transfers
Helper_Agent_Distance Heuristic::get_helper_agents_distance(Coordinate source, Coordinate destination,
	const State& state, const Agent_Id handoff_agent, const Agent_Combination& agents_in, const bool& require_handoff_action) const {
	
	if (source == destination) {
		return { 0, false };
	}

	auto local_agents = agents_in;
	bool skip_source = false, skip_dest = false;
	std::vector<Helper_Agent_Info> helpers;

	// TODO - This is not very pretty, this and find_helper should be reorganised
	// Assign handoff agent to source or dest if we require handoff to have action
	if (require_handoff_action) {
		size_t dest_dist = EMPTY_VAL;
		auto& agent_ref = state.agents.at(handoff_agent.id);
		auto dest_agent = state.get_agent(destination);
		if (!environment.is_type_stationary(ingredient1)
			&& !environment.is_type_stationary(ingredient2)
			&& (!dest_agent.has_value() || dest_agent.value() == handoff_agent)) {
			dest_dist = distances.at(0).const_at(agent_ref.coordinate, destination).g;
		}
		size_t source_dist = EMPTY_VAL;
		auto source_agent = state.get_agent(source);
		if (!source_agent.has_value() || source_agent.value() == handoff_agent) {
			source_dist = distances.at(0).const_at(agent_ref.coordinate, source).g;
		}

		if (source_dist == EMPTY_VAL && dest_dist == EMPTY_VAL) {
			return {};
		}

		size_t holding_penalty = 0;
		if (dest_dist != EMPTY_VAL && dest_dist < source_dist) {
			skip_dest = true;
			if (agent_ref.item.has_value() && source_dist != 0 && dest_dist != 0) {
				holding_penalty = get_distance_to_nearest_wall(agent_ref.coordinate, destination, state);
			}
			helpers.push_back({ 0, dest_dist });
		} else {
			skip_source = true;
			if (agent_ref.item.has_value() && source_dist != 0 && dest_dist != 0) {
				holding_penalty = get_distance_to_nearest_wall(agent_ref.coordinate, source, state);
			}
			helpers.push_back({ EMPTY_VAL, source_dist, handoff_agent });
		}
		local_agents.remove(handoff_agent);
	}

	// Find helpers
	auto agent_index = local_agents.size() - 1;
	auto prev = destination;
	size_t path_length = 1;
	bool first = true;
	while (true) {
		auto& prev_dist = distances.at(agent_index).const_at(source, prev);
		if (prev_dist.parent == source) {
			if (!skip_source) {
				auto helper = find_helper(helpers, handoff_agent, local_agents, first, state, source, { EMPTY_VAL, EMPTY_VAL }, path_length);
				if (!helper.has_value()) {
					return { };
				}
				helpers.push_back(helper);
			}
			break;
		}
		if (prev_dist.parent == Coordinate{EMPTY_VAL, EMPTY_VAL}) {
			return { };
		}

		auto next = prev_dist.parent;

		++path_length;
		if (environment.is_cell_type(next, Cell_Type::WALL)) {
			if (!first || !skip_dest) {
				auto helper = find_helper(helpers, handoff_agent, local_agents, first, state, prev, next, path_length);
				if (!helper.has_value()) {
					return { };
				}
				helpers.push_back(helper);
				local_agents.remove(helper.agent);
			}
			first = false;
		}
		prev = next;
	}

	// Calculate distance from helpers
	first = true;
	size_t forward_length = path_length;
	size_t reverse_length = 0;
	for (auto& helper : helpers) {
		if (helper.help_to_dest == 0
			&& !environment.is_type_stationary(ingredient1)
			&& !environment.is_type_stationary(ingredient2)) {
			reverse_length = helper.help_to_dest + helper.agent_to_help;
			assert(helper.help_to_dest <= forward_length);
			forward_length -= helper.help_to_dest;

		} else {
			// To account for, if we assign the handoff agent at the beginning of this method,
			//		we do not know what the path length is, so it is set to temporaryt EMPTY_VAL
			if (helper.help_to_dest == EMPTY_VAL) {
				helper.help_to_dest = path_length - 1;
			}

			int additional_length = (int)helper.agent_to_help - (path_length - helper.help_to_dest);
			if (additional_length > 0) {
				forward_length += additional_length;
			}
		}
		first = false;
	}
	bool was_handed_off = helpers.size() > 1;
	assert((distances.at(agent_index).const_at(source, destination).g == path_length));
	return { std::max(forward_length, reverse_length), was_handed_off };
}

bool Heuristic::are_items_available(const Location& location1,
	const Location& location2, const State& state, const Agent_Combination& local_agents) const {

	if (!location1.from_wall) {
		auto agent = state.get_agent(location1.coordinate);
		if (agent.has_value() && !local_agents.contains(agent.value())) {
			return false;
		}
	}

	if (!location2.from_wall) {
		auto agent = state.get_agent(location2.coordinate);
		if (agent.has_value() && !local_agents.contains(agent.value())) {
			return false;
		}
	}
	return true;
}

// Main heuristic calculation
size_t Heuristic::get_heuristic_distance(const Location& location1, const Location& location2, const State& state, 
	const Agent_Id& handoff_agent, const Agent_Combination& local_agents, const bool& require_handoff_action) const {

	size_t min_dist = EMPTY_VAL;
	size_t wall_penalty = (size_t)0
		+ (location1.from_wall ? 1 : 0)
		+ (location2.from_wall ? 1 : 0);

	if (!are_items_available(location1, location2, state, local_agents)) {
		return EMPTY_VAL;
	}

	if (environment.is_type_stationary(ingredient1) && environment.is_type_stationary(ingredient2)) {
		wall_penalty += std::min(
			get_distance_to_nearest_wall(location1.coordinate, { EMPTY_VAL, EMPTY_VAL }, state),
			get_distance_to_nearest_wall(location2.coordinate, { EMPTY_VAL, EMPTY_VAL }, state));
	}

	if (!environment.is_type_stationary(ingredient1)) {
		auto had = get_helper_agents_distance(location1.coordinate, location2.coordinate, state, handoff_agent, local_agents, require_handoff_action);
		if (had.has_value()) {
			min_dist = std::min(min_dist, had.dist + wall_penalty);
		}
	}
	auto had = get_helper_agents_distance(location2.coordinate, location1.coordinate, state, handoff_agent, local_agents, require_handoff_action);
	if (had.has_value()) {
		min_dist = std::min(min_dist, had.dist + wall_penalty);
	}
	return min_dist;
}

// Main heuristic function, setup
size_t Heuristic::operator()(const State& state, const Agent_Combination& agents, const Agent_Id& handoff_agent, const bool&  require_handoff_action) const {
	std::vector<Location> locations1;
	if (environment.is_type_stationary(ingredient1)) {
		locations1 = environment.get_locations(state, ingredient1);
	} else {
		locations1 = environment.get_non_wall_locations(state, ingredient1);
	}
	auto locations2 = environment.get_non_wall_locations(state, ingredient2);

	size_t min_dist = EMPTY_VAL;
	size_t number_of_agents = environment.get_number_of_agents();

	// Search all location combinations using all agents, and all agents minus handoff_agent
	for (const auto& location1 : locations1) {
		for (const auto& location2 : locations2) {
			min_dist = std::min(min_dist, get_heuristic_distance(location1, location2, state, handoff_agent, agents, require_handoff_action));
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

			Search_Entry origin{ source, 0, 0, 0 };
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
					if (recorded_dist.g == EMPTY_VAL 
						|| current.dist + 1 < recorded_dist.g
						|| (current.dist +1 == recorded_dist.g 
							&& recorded_dist.wall_g < current.wall_g)) {
						
						recorded_dist.g = current.dist + 1;
						recorded_dist.parent = current.coord;
						recorded_dist.wall_g = (is_next_wall ? current.dist + 1 : current.wall_g);
						size_t wall_count = current.walls + (is_next_wall ? 1 : 0);
						if (wall_count <= max_walls) {
							frontier.emplace_back(destination, current.dist + 1, wall_count, (is_next_wall ? current.dist + 1 : current.wall_g));
						}
					}
				}
				frontier.pop_front();
			}

			// Recorded the smallest dist among the distances from different wall values
			for (const auto& dist : temp_distances) {
				for (const auto& destination : coordinates) {
					auto& ref_dist = final_dist.at(source, destination);
					if (ref_dist.g == EMPTY_VAL || dist.at(convert(destination)).g <= ref_dist.g) {
						ref_dist = dist.at(convert(destination));
					}
				}
			}

		}
	}

	print_distances({ 5,2 }, 2);
	print_distances({ 1,1 }, 2);
	print_distances({ 2,1 }, 2);
	//print_distances({ 5,0 }, 2);
}