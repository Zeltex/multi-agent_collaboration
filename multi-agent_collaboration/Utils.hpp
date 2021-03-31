#pragma once

#include <vector>

#include "Environment.hpp"

//struct Agent_Combination;
//enum class Direction;
//struct Coordinate;
// Get all combinations of numbers/agents <n
std::vector<Agent_Combination> get_combinations(size_t n);
std::vector<Agent_Combination> get_combinations(std::vector<size_t> agents);
std::vector<Agent_Combination> get_combinations(Agent_Combination agents);
Direction get_direction(Coordinate from, Coordinate to);