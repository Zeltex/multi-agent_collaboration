#include "Sliding_Recogniser.hpp"
#include "Environment.hpp"
#include "Core.hpp"
#include "Utils.hpp"

#include <sstream>
#include <iomanip>

constexpr auto WINDOW_SIZE = 3; 
constexpr auto alpha = 10.0f;			// Inverse weight of solution length in goal probability

Sliding_Recogniser::Sliding_Recogniser(const Environment& environment, const State& initial_state)
	: Recogniser_Method(environment, initial_state), goals(), time_step(0) {
	for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
		Goal goal{ Agent_Combination{ agent }, EMPTY_RECIPE };
		goals.insert({ goal,  {} }); 
		agents_active_status.emplace_back();
	}
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
			auto new_length = val.lengths.at(time_step - 1);
			auto length_prob = (alpha / (new_length + alpha));
			//val.probability = (alpha * 1.0f / val.lengths.at(window_index))
			auto progress_prob = ((float)val.lengths.at(window_index)) / (val.lengths.at(time_step - 1) + window_length);

			if (window_length == 0) {
				val.probability = length_prob;
				val.length_prob = length_prob;		// debug
			} else {
				val.probability = length_prob * progress_prob;
				val.length_prob = length_prob;		// debug
				val.progress_prob = progress_prob;	// debug
			}
		}
		if (val.probability > max_prob) max_prob = val.probability;
	}
	return max_prob;
}


float Sliding_Recogniser::update_non_probabilities(size_t base_window_index, size_t number_of_agents){
	std::vector<float> max_progress(number_of_agents, 0.0f);
	std::vector<bool> agents_useful(number_of_agents, false);

	// Record largest progression/diff towards a single goal/combination
	for (auto& [goal, goal_entry] : goals) {
		// Skip irrelevant goals and initial state
		if (!goal_entry.is_current(time_step) || time_step == 1) {
			continue;
		}
		size_t window_index = goal_entry.get_non_empty_index(base_window_index);
		
		// Skip non-goals
		if (window_index == EMPTY_VAL) {
			continue;
		}
		size_t window_length = time_step - window_index - 1;
		float absolute_progress = (float)goal_entry.lengths.at(window_index) - (goal_entry.lengths.at(time_step - 1));
		float progress = absolute_progress / window_length;

		progress = std::max(std::min(progress, 1.0f), 0.0f);
		// TODO - Progress should probably also be recorded for single agent goals, even though this should
		//		be covered by the multi agent cases

		if (goal.agents.size() > 1) {
			for (auto& agent : goal.agents.get()) {
				bool is_useful = true;
				//auto agent_goal_it = goals.find(Goal{ Agent_Combination{agent}, goal.recipe });
				//if (agent_goal_it == goals.end()) {
				//	exit(-1);
				//}
				//auto agent_prob = agent_goal_it->second.probability;
				auto agent_prob = goal_entry.probability;
				auto agents = goal.agents;
				agents.remove(agent);
				auto combinations = get_combinations(agents);
				for (auto& combination : combinations) {

					auto it = goals.find(Goal(combination, goal.recipe));
					if (it != goals.end() && it->second.is_current(time_step) && it->second.probability >= agent_prob) {
						is_useful = false;
						break;
					}
				}
				if (is_useful) {
					agents_useful.at(agent.id) = true;
					auto& ref = max_progress.at(agent.id);
					ref = std::max(ref, progress);
				}
			}
		}

	}

	// Update NONE probabilities
	float max_prob = 0.0f;
	for (size_t agent = 0; agent < number_of_agents; ++agent) {
		auto progress_prob = 1 - max_progress.at(agent);
		if (time_step == 1) {
			// Not NONE on first round by default
			progress_prob = 0.0f;
		}
		// Note if previous window was optimal
		bool previous_active_status = time_step == 1 ? true : agents_active_status.at(agent).back();
		
		// If entire window has been optimal
		agents_active_status.at(agent).push_back(progress_prob == 0.0f);

		// non-probability is 0% the step after having entire optimal window by default
		if (previous_active_status) {
			progress_prob = 0.0f;
		}

		max_prob = std::max(max_prob, progress_prob);

		Goal goal{ Agent_Combination{ agent }, EMPTY_RECIPE };
		goals.at(goal).probability = progress_prob;
	}

	return max_prob;
}

void Sliding_Recogniser::normalise(float max_prob) {
	for (auto& [key, val] : goals) {
		val.probability /= max_prob;
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

	std::vector<std::stringstream> buffers(3);
	for (auto& buffer : buffers) {
		buffer << std::fixed << std::setprecision(3) << '\n';
	}
	for (const auto& [key, val] : goals) {
		if (!val.is_current(time_step)) continue;
		buffers.at(0) << val.length_prob << "\t";
		buffers.at(1) << val.progress_prob << "\t";
		buffers.at(2) << val.probability << "\t";
	}
	buffers.at(2) << "\n\n";
	for (auto& buffer : buffers) {
		PRINT(Print_Category::RECOGNISER, Print_Level::DEBUG, buffer.str());
	}
}
