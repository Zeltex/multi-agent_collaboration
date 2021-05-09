#pragma once
#include <cassert>
#include <algorithm>
#include <vector>
#include <random>

// All combinations on input (NOT including duplicates) of size combination_size
template <typename T>
std::vector<std::vector<T>> get_combinations(const std::vector<T>& input, size_t combination_size) {
	auto recipe_size = input.size();
	assert(recipe_size >= combination_size);
	std::string bitmask(combination_size, 1);
	bitmask.resize(recipe_size, 0);
	std::vector<std::vector<T>> combinations;
	do {
		std::vector<T> next_combination;
		for (size_t i = 0; i < recipe_size; ++i) {
			if (bitmask[i]) next_combination.push_back(input.at(i));
		}
		combinations.push_back(next_combination);

	} while (std::prev_permutation(bitmask.begin(), bitmask.end()));
	return combinations;
}

// All combinations of input (including duplicate entries) of size combination_size
template <typename T>
std::vector<std::vector<T>> get_combinations_duplicates(const std::vector<T>& input, size_t combination_size) {
	size_t input_size = input.size();
	std::vector<std::vector<T>> result;
	std::vector<size_t> counters(combination_size, 0);
	bool done = false;
	while (!done) {
		std::vector<T> data;
		for (size_t i = 0; i < counters.size(); i++) {
			data.push_back(input.at(counters[i]));
		}
		result.push_back(std::move(data));

		// Advance indices
		size_t index = 0;
		while (!done && ++counters.at(index) >= input_size) {
			counters.at(index++) = 0;
			done = index >= counters.size();
		}
	}

	return result;
}

template <typename T>
T get_random(const std::vector<T>& input) {
	//static std::random_device device;
	//static std::default_random_engine engine(random_seed);
	//engine.seed(random_seed);
	std::vector<T> out;
	std::sample(input.begin(), input.end(), std::back_inserter(out), 1, random_engine);
	return out.at(0);
}
