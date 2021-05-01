#pragma once

#include <vector>
#include <random>

#include "Environment.hpp"

std::vector<Agent_Combination> get_combinations(size_t n);
std::vector<Agent_Combination> get_combinations(std::vector<size_t> agents);
std::vector<Agent_Combination> get_combinations(Agent_Combination agents);
std::vector<Agent_Combination> get_permutations(Agent_Combination agents);
Direction get_direction(Coordinate from, Coordinate to);

template <typename T>
std::vector<std::vector<T>> get_combinations(const std::vector<T>& recipes, size_t combination_size);

template <typename T>
std::vector<std::vector<T>> get_combinations_duplicates(const std::vector<T>& input, size_t combination_size);


extern size_t random_seed;
extern std::default_random_engine random_engine;
template <typename T>
T get_random(const std::vector<T>& input);

void set_random_seed(size_t seed);

#include "Utils.ipp"