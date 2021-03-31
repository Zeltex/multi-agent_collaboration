#pragma once

#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING 1
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1

#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <memory>

#include "Environment.hpp"
#include "Search.hpp"
#include "boost/pool/pool.hpp"

struct Node {
	Node() {};

	Node(const Node* other, const size_t& id) {
		init(other);
		this->id = id;
		this->parent = other;
	}

	Node(const Node* other) {
		init(other);
		this->parent = other;
	}

	Node(State state, size_t id, size_t g, size_t h, size_t action_count,
		size_t pass_time, Node* parent, Joint_Action action, bool closed, bool valid)
		: state(state), id(id), g(g), h(h), action_count(action_count),
		pass_time(pass_time), parent(parent), action(action), closed(closed), valid(valid) {};
	//Node(size_t state_id, size_t g, size_t h, size_t action_count) : 
	//	state_id(state_id), g(g), h(h), action_count(action_count){};
		
	void init(const Node* other) {
		this->state = other->state;
		this->id = other->id;
		this->g = other->g;
		this->h = other->h;
		this->action_count = other->action_count;
		this->parent = other->parent;
		this->action = other->action;
		this->closed = other->closed;
		this->valid = other->valid;
		this->pass_time = other->pass_time;
		this->hash = EMPTY_VAL;
	}

	//void init(State state, size_t id, size_t g, size_t h, size_t action_count,
	//	size_t pass_time, Node* parent, Joint_Action action, bool closed) {
	//	this->state = state;
	//	this->id = id;
	//	this->g = g; 
	//	this->h = h;
	//	this->action_count = action_count;
	//	this->pass_time = pass_time;
	//	this->parent = parent;
	//	this->action = action;
	//	this->closed = closed;
	//}
	size_t g;
	float h;
	float f() const { return g + h; }
	State state;
	size_t id;
	size_t action_count;
	size_t pass_time;
	const Node* parent;
	Joint_Action action;
	bool closed;
	bool valid;

	// For debug purposes
	size_t hash;
	void calculate_hash() {
		this->hash = to_hash();
	}

	bool set_equals(const Node* other) const {
		return this->state == other->state && this->pass_time == other->pass_time;
	}

	size_t to_hash() const {
		std::string pass_string = (pass_time == EMPTY_VAL ? "0" : "1");
		return std::hash<std::string>()(state.to_hash_string() + pass_string);
	}

	bool has_agent_passed() const {
		return pass_time != EMPTY_VAL;
	}

	bool operator<(const Node* other) const {
		std::cout << "<node" << std::endl;
		return this->state < other->state;
	}

	// Used to determine if a shorter path has been found (assumes this->state == other->state)
	bool is_shorter(const Node* other) const {
		if (this->g != other->g) {
			return this->g < other->g;
		}

		if (this->action_count != other->action_count) {
			return this->action_count < other->action_count;
		}

		if (this->pass_time != EMPTY_VAL && other->pass_time != EMPTY_VAL) {
			return this->pass_time < other->pass_time;
		}

		return false;
	}
};

namespace std {
	template<>
	struct hash<Node*>
	{
		size_t
			operator()(const Node* obj) const
		{
			return obj->to_hash();
		}
	};
}

struct Distances {
	Distances(size_t width, size_t height) 
		: distances(width*height, std::vector<size_t>(width*height, EMPTY_VAL)), 
		width(width), height(height) {};
	std::vector<std::vector<size_t>> distances;
	size_t width;
	size_t height;
	constexpr size_t convert(const Coordinate& coord1) const {
		return coord1.first * height + coord1.second;
	}

	const size_t& const_at(Coordinate coord1, Coordinate coord2) const {
		return distances.at(convert(coord1)).at(convert(coord2));
	}

	size_t& at(Coordinate coord1, Coordinate coord2) {
		return distances.at(convert(coord1)).at(convert(coord2));
	}
};

// Could optimise some by calculating distance from all ingredients to nearest agent
// and using this in combination with the location1/location2 nested for loops
#define INFINITE_HEURISTIC 1000
struct Heuristic {
	Heuristic(Environment environment) : environment(environment),
		ingredient1(Ingredient::DELIVERY), ingredient2(Ingredient::DELIVERY), agents(), handoff_agent() {
		init();
	}

	void set(Ingredient ingredient1, Ingredient ingredient2, const Agent_Combination& agents, 
		const std::optional<Agent_Id>& handoff_agent) {
		
		this->ingredient1 = ingredient1;
		this->ingredient2 = ingredient2;
		this->agents = agents;
		this->handoff_agent = handoff_agent;
	}

	size_t euclidean(Coordinate location1, Coordinate location2) const {
		return (size_t)(std::abs((int)location1.first - (int)location2.first)
			+ std::abs((int)location1.second - (int)location2.second));
	}
	size_t operator()(const State& state) const {
		auto locations1 = environment.get_non_wall_locations(state, ingredient1);
		auto locations2 = environment.get_non_wall_locations(state, ingredient2);
		//if (locations1.empty()) locations1 = environment.get_recipe_locations(state, ingredient1);
		//if (locations2.empty()) locations2 = environment.get_recipe_locations(state, ingredient2);
		if (locations1.empty() or locations2.empty()) {
			return INFINITE_HEURISTIC;
		}
		size_t min_dist = EMPTY_VAL;
		size_t agents_index = environment.get_number_of_agents() - 1;
		if (handoff_agent.has_value()) {
			--agents_index;
		}
		auto& distances_coop_ref = distances.at(agents_index);
		//auto& distances_single_ref = distances.at(0);
		for (const auto& agent_id : agents.get()) {
			if (handoff_agent.has_value() && agent_id == handoff_agent.value()) {
				continue;
			}
			const auto& agent = state.agents.at(agent_id.id);
			for (const auto& location1 : locations1) {
				for (const auto& location2 : locations2) {
					auto& locations_distance = distances_coop_ref.const_at(location1, location2);
					min_dist = std::min(min_dist, 
						distances_coop_ref.const_at(agent.coordinate, location1) + locations_distance);
					
					min_dist = std::min(min_dist, 
						distances_coop_ref.const_at(agent.coordinate, location2) + locations_distance);
				}
			}
		}
		// NOTE: Must divide by 3 in case 
		// 1) non-handoff agent is moving is moving towards goal with item, 
		//		and handoff agent is moving from goal to non-handoff agent
		return min_dist == EMPTY_VAL ? INFINITE_HEURISTIC : min_dist;
	}
	struct Search_Entry {
		Search_Entry(Coordinate coord, size_t dist, size_t walls) : coord(coord), dist(dist), walls(walls) {}
		Coordinate coord;
		size_t dist;
		size_t walls;
	};
	size_t convert(const Coordinate& coord1) const {
		return coord1.first * environment.get_height() + coord1.second;
	}

	void print_distances(Coordinate coordinate, size_t agent_number) const {
		std::cout << "\nPrinting distances for " << agent_number << " agents from (" << coordinate.first << ", " << coordinate.second << ")" << std::endl;
		for (size_t y = 0; y < environment.get_height(); ++y) {
			for (size_t x = 0; x < environment.get_width(); ++x) {
				const auto& temp = distances.at(agent_number-1).const_at(coordinate, { x,y });
				std::cout << (temp == EMPTY_VAL ? "--" : (temp < 10 ? "0" : "") + std::to_string(temp)) << " ";
			}
			std::cout << std::endl;
		}
	}

	// All pairs shortest path for all amounts of agents, taking wall-handover in to account
	void init() {
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
				std::vector<std::vector<size_t>> temp_distances(
					environment.get_number_of_agents(), std::vector<size_t>(environment.get_width() * environment.get_height(), EMPTY_VAL));

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
						if (recorded_dist == EMPTY_VAL || current.dist + 1 < recorded_dist) {
							recorded_dist = current.dist + 1;
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
						if (ref_dist == EMPTY_VAL || dist.at(convert(destination)) < ref_dist) {
							ref_dist = dist.at(convert(destination));
						}
					}
				}

			}
		}

		//print_distances({ 5,1 }, 1);
		//print_distances({ 5,1 }, 2);
		//print_distances({ 5,0 }, 2);
	}

	std::vector<Distances> distances;
	Environment environment;

	Ingredient ingredient1; 
	Ingredient ingredient2;
	Agent_Combination agents;
	std::optional<Agent_Id> handoff_agent;
};


struct Node_Queue_Comparator {
	bool operator()(const Node* lhs, const Node* rhs) const {
		if (lhs->f() != rhs->f()) return lhs->f() > rhs->f();
		if (lhs->g != rhs->g) return lhs->g < rhs->g;
		if (lhs->action_count != rhs->action_count) return lhs->action_count > rhs->action_count;
		return false;
	}
};

struct Node_Hasher {
	bool operator()(const Node* node) const {
		return node->to_hash();
	}
};

struct Node_Set_Comparator {
	bool operator()(const Node* lhs, const Node* rhs) const {
		return lhs->set_equals(rhs);
	}
};


using Node_Queue = std::priority_queue<Node*, std::vector<Node*>, Node_Queue_Comparator>;
using Node_Set = std::unordered_set<Node*, Node_Hasher, Node_Set_Comparator>;
using Node_Ref = std::deque<Node>;
using Pool = boost::pool<>;

class A_Star : public Search_Method {
public:
	A_Star(const Environment& environment, size_t depth_limit);
	std::vector<Joint_Action> search_joint(const State& state, Recipe recipe, 
		const Agent_Combination& agents, std::optional<Agent_Id> handoff_agent) override;
private:
	size_t	get_action_cost(const Joint_Action& action) const;
	
	Node*	get_next_node(Node_Queue& frontier) const;
	
	void	initialize_variables(Node_Queue& frontier, Node_Set& visited, Node_Ref& nodes,
		const State& original_state, Pool& pool) const;
	
	bool	is_being_passed(const std::optional<Agent_Id>& handoff_agent, const Joint_Action& action, 
		const Node* current_node) const;
	
	bool	is_invalid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, 
		const std::optional<Agent_Id>& handoff_agent) const;

	bool	is_valid_goal(const Node* node, const Recipe& recipe, const Joint_Action& action, 
		const std::optional<Agent_Id>& handoff_agent) const;
	
	void	print_current(const Node* node) const;
	
	std::vector<Joint_Action> get_actions(const Agent_Combination& agents, 
		bool has_handoff_agent) const;
		
	std::pair<bool, Node*> check_and_perform(const Joint_Action& action, Node_Ref& nodes,
		const Node* current_node, const std::optional<Agent_Id>& handoff_agent, Pool& pool) const;
	
	std::vector<Joint_Action> extract_actions(const Node* node) const;

	Heuristic heuristic;

	//std::vector<Joint_Action> extract_actions(size_t goal_id, const std::vector<State_Info>& states) const;
};