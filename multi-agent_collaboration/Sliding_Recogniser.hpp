#pragma once

#include "Recogniser.hpp"

#include <cassert>
#include <map>
#include <vector>

struct Goal_Entry {
	Goal_Entry() : probability(EMPTY_PROB), lengths(),
		length_prob(EMPTY_PROB), progress_prob(EMPTY_PROB) {};

	Goal_Entry(size_t length, size_t time_step) : probability(EMPTY_PROB), lengths(),
		length_prob(EMPTY_PROB), progress_prob(EMPTY_PROB) {
		add(length, time_step);
	};
	
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

	// For debug
	float length_prob;
	float progress_prob;
};

class Sliding_Recogniser : public Recogniser_Method {
public:
	Sliding_Recogniser(const Environment& environment, const State& initial_state);
	void update(const std::map<Goal, size_t>& goal_lengths) override;
	Goal get_goal(Agent_Id agent) override;
	std::map<Goal, float> get_raw_goals() const override;
	bool is_probable(Goal goal) const override;
	bool is_probable_normalised(Goal goal, const std::vector<Goal>& available_goals, Agent_Id agent, bool use_non_probability) const override;
	std::map<Agent_Id, Goal> get_goals() const override;
	void print_probabilities() const override;
	float get_probability(const Goal& goal) const override;

private:
	float get_non_probability(Agent_Id agent) const;
	void insert(const std::map<Goal, size_t>& goal_lengths);
	float update_standard_probabilities(size_t base_window_index);
	float update_non_probabilities(size_t base_window_index, size_t number_of_agents);
	void normalise(float max_prob);

	std::vector<std::vector<bool>> agents_active_status;
	std::map<Goal, Goal_Entry> goals;
	size_t time_step;
};