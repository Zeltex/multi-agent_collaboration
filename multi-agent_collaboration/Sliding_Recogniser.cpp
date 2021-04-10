#include "Sliding_Recogniser.hpp"
#include "Environment.hpp"
#include "Core.hpp"
#include "Utils.hpp"

#include <sstream>
#include <iomanip>

constexpr auto WINDOW_SIZE = 5; 
constexpr auto alpha = 100.0f;			// Inverse weight of solution length in goal probability


void Sliding_Recogniser::init(Goal goal) {
	goals.insert({ goal,  {} }); 
}

void Sliding_Recogniser::insert(const std::vector<Goal_Length>& goal_lengths) {
	for (const auto& entry : goal_lengths) {
		Goal goal{ entry.agents, entry.recipe };
		auto it = goals.find(goal);
		if (it == goals.end()) {
			bool dummy;
			std::tie(it, dummy) = goals.insert({ goal, {} });
		}
		it->second.add(entry.length, time_step);
	}
}

float Sliding_Recogniser::update_standard_probabilities(size_t base_window_index){
	float max_prob = 0.0f;
	for (auto& [key, val] : goals) {
		if (!val.is_current(time_step)) {
			continue;
		}

		size_t window_index = val.get_non_empty_index(base_window_index);
		size_t window_length = time_step - window_index - 1;
		if (window_index == EMPTY_VAL) {
			val.probability = EMPTY_PROB;
		} else {
			auto old_length = val.lengths.at(window_index);
			auto length_prob = (alpha / (old_length + alpha));
			//val.probability = (alpha * 1.0f / val.lengths.at(window_index))
			auto progress_prob = ((float)val.lengths.at(window_index)) / (val.lengths.at(time_step - 1) + window_length);

			if (window_length == 0) {
				val.probability = length_prob;
			} else {
				val.probability = length_prob * progress_prob;
			}
		}
		if (val.probability > max_prob) max_prob = val.probability;
	}
	return max_prob;
}


float Sliding_Recogniser::update_non_probabilities(size_t base_window_index, size_t number_of_agents){
	std::vector<float> max_diff(number_of_agents, 0.0f);
	std::vector<bool> agents_useful(number_of_agents, false);
	for (auto& [key, val] : goals) {
		if (!val.is_current(time_step) || time_step == 1) {
			continue;
		}
		size_t window_index = val.get_non_empty_index(base_window_index);
		if (window_index == EMPTY_VAL) {
			continue;
		}
		size_t window_length = time_step - window_index - 1;
		float progress = (float)val.lengths.at(window_index) - (val.lengths.at(time_step - 1));
		float diff = 1 - (progress / window_length);

		diff = std::min(diff, 1.0f);
		diff = std::max(diff, 0.0f);

		if (key.agents.size() > 1) {
			for (auto& agent : key.agents.get()) {
				bool is_useful = true;
				auto agent_prob = goals.at(Goal{ Agent_Combination{agent}, key.recipe }).probability;
				auto agents = key.agents;
				agents.remove(agent);
				auto combinations = get_combinations(agents);
				for (auto& combination : combinations) {
					if (goals.at(Goal(combination, key.recipe)).probability >= agent_prob) {
						is_useful = false;
						break;
					}
				}
				if (is_useful) {
					agents_useful.at(agent.id) = true;
					auto& ref = max_diff.at(agent.id);
					ref = std::max(ref, diff);
				}
			}
		}

	}

	// Update NONE probabilities
	float max_prob = 0.0f;
	for (size_t agent = 0; agent < number_of_agents; ++agent) {
		auto& diff_ref = max_diff.at(agent);
		if (!agents_useful.at(agent) && time_step > 1) {
			diff_ref = 1.0f;	// Set NONE probability to 100%
		}

		max_prob = std::max(max_prob, diff_ref);

		Goal goal{ Agent_Combination{ agent }, EMPTY_RECIPE };
		goals.at(goal).probability = diff_ref;
	}

	return max_prob;
}

void Sliding_Recogniser::normalise(float max_prob) {
	for (auto& [key, val] : goals) {
		val.probability /= max_prob;
	}
}

Sliding_Recogniser::Sliding_Recogniser(const Environment& environment, const State& initial_state)
	: Recogniser_Method(environment, initial_state), goals(), time_step(0) {
	for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
		Goal goal{ Agent_Combination{ agent }, EMPTY_RECIPE };
		init(goal);
	}
}


void Sliding_Recogniser::update(const std::vector<Goal_Length>& goal_lengths) {
	++time_step;
	insert(goal_lengths);


	size_t base_window_index = time_step >= WINDOW_SIZE ? time_step - WINDOW_SIZE : 0;
	auto prob1 = update_standard_probabilities(base_window_index);
	auto prob2 = update_non_probabilities(base_window_index, environment.get_number_of_agents());
	normalise(std::max(prob1, prob2));

}

Goal Sliding_Recogniser::get_goal(Agent_Id agent) {
	float best_prob = EMPTY_PROB;
	Goal best_goal = {};
	for (const auto& [key, val]: goals) {
		if (val.probability > best_prob && key.agents.contains(agent)) {
			best_prob = val.probability;
			best_goal = key;
		}
	}
	return best_goal;
}

std::map<Agent_Id, Goal> Sliding_Recogniser::get_goals() const {
	std::map<Agent_Id, Goal> result;
	std::map<Agent_Id, float> probs;
	for (const auto& [key, val] : goals) {
		for (const auto& agent : key.agents.get()) {
			auto it = result.find(agent);
			if (it == result.end()) {
				result.insert({ agent, key });
				probs.insert({ agent, val.probability });
			} else if (probs.at(agent) < val.probability) {
				result.at(agent) = key;
				probs.at(agent) = val.probability;
			}
		}
	}
	return result;
}

void Sliding_Recogniser::print_probabilities() const {
	for (const auto& [key, val] : goals) {
		if (!val.is_current(time_step)) continue;
		PRINT(Print_Category::RECOGNISER, Print_Level::DEBUG, static_cast<char>(key.recipe.result) + key.agents.to_string() + "\t");
	}

	std::stringstream buffer;
	buffer << std::fixed << std::setprecision(3) << '\n';
	for (const auto& [key, val] : goals) {
		if (!val.is_current(time_step)) continue;
		buffer << val.probability << "\t";
	}
	PRINT(Print_Category::RECOGNISER, Print_Level::DEBUG, buffer.str() + '\n');
}
