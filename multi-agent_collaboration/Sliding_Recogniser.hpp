#pragma once

#include "Recogniser.hpp"

#include <cassert>

struct Goal_Entry {
	Goal_Entry() : probability(EMPTY_PROB), lengths() {};
	void add(size_t length, size_t time_step) {
		if (!time_step == 0) {
			while (lengths.size() < time_step - 1) {
				lengths.push_back(EMPTY_VAL);
			}
		}
		lengths.push_back(length);
	}

	size_t get_non_empty_index(size_t index) const {
		while (index < lengths.size() && lengths.at(index) == EMPTY_VAL) {
			++index;
		}
		return index >= lengths.size() || lengths.empty() ? EMPTY_VAL : index;
	}

	bool is_current(size_t time_step) const {
		assert(lengths.size() <= time_step);
		return lengths.size() == time_step || lengths.size() == 0; // NONE goal has size==0
	}

	float probability;
	std::vector<size_t> lengths;
};

class Sliding_Recogniser : public Recogniser_Method {
public:
	Sliding_Recogniser(const Environment& environment, const State& initial_state);
	void update(const std::vector<Goal_Length>& goal_lengths) override;
	Goal get_goal(Agent_Id agent) override;
	std::map<Agent_Id, Goal> get_goals() const override;
	void print_probabilities() const override;
private:

	void init(Goal goal);
	void insert(const std::vector<Goal_Length>& goal_lengths);
	float update_standard_probabilities(size_t base_window_index);
	float update_non_probabilities(size_t base_window_index, size_t number_of_agents);
	void normalise(float max_prob);

	std::map<Goal, Goal_Entry> goals;
	size_t time_step;
};