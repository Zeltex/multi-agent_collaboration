#pragma once
#include <cassert>
#include <algorithm>
#include <vector>

template <typename T>
std::vector<std::vector<T>> get_combinations(const std::vector<T>& recipes, size_t combination_size) {
	auto recipe_size = recipes.size();
	assert(recipe_size >= combination_size);
	std::string bitmask(combination_size, 1);
	bitmask.resize(recipe_size, 0);
	std::vector<std::vector<T>> combinations;
	do {
		std::vector<T> next_combination;
		for (size_t i = 0; i < recipe_size; ++i) {
			if (bitmask[i]) next_combination.push_back(recipes.at(i));
		}
		combinations.push_back(next_combination);

	} while (std::prev_permutation(bitmask.begin(), bitmask.end()));
	return combinations;
}