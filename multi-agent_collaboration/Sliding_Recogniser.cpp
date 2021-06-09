#include "Sliding_Recogniser.hpp"
#include "Environment.hpp"
#include "Core.hpp"
#include "Utils.hpp"

#include <sstream>
#include <iomanip>

constexpr auto WINDOW_SIZE = 4; 
constexpr auto alpha = 100.0f;			// Inverse weight of solution length in goal probability
constexpr auto beta = 0.9f;				// Adjust NONE probability scale
constexpr auto charlie = 0.8;			// Threshold for goal being probable
constexpr auto delta = 1.05;				// Collaboration penalty

Sliding_Recogniser::Sliding_Recogniser(const Environment& environment, const State& initial_state)
	: Recogniser_Method(environment, initial_state), goals(), time_step(0) {
	for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
		Goal goal{ agent , EMPTY_RECIPE, EMPTY_VAL };
		goals.insert({ goal,  {} }); 
		//agents_active_status.emplace_back();
	}
}

void Sliding_Recogniser::insert(const std::map<Goal, size_t>& goal_lengths, const State& state) {
	for (const auto& [goal, length] : goal_lengths) {
		auto it = goals.find(goal);
		if (it == goals.end()) {
			goals.insert({ goal, Goal_Entry{length, time_step} });
		} else {
			it->second.add(length, time_step);
		}
	}

	for (auto& [key, val] : goals) {
		if (val.is_current(time_step)) {
			continue;
		}
		if (state.contains_item(key.recipe.result)) {
			val.add(0, time_step);
		} else {
			val.repeat(time_step);
		}
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
			progress_prob = std::pow(progress_prob, 1 + (key.agents.size() - 1) * 0.5);
			progress_prob = std::max(std::min(progress_prob, 1.0f), 0.0f);

			if (window_length == 0) {
				constexpr float new_goal_penalty = 0.8f;
				val.probability = length_prob * new_goal_penalty;
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
	//std::vector<bool> agents_useful(number_of_agents, false);

	std::vector<Agent_Id> all_agents;
	std::vector<std::vector<Agent_Combination>> agent_permutations(number_of_agents, std::vector<Agent_Combination>());
	for (size_t i = 0; i < number_of_agents; ++i) {
		all_agents.push_back({ i });
	}
	for (size_t i = 0; i < number_of_agents; ++i) {
		for (auto& combination : get_combinations<Agent_Id>({ all_agents }, i + 1)) {
			agent_permutations.at(i).push_back(Agent_Combination{ combination });
		}
	}

	// Record largest progression/diff towards a single goal/combination
	for (auto& [key, val] : goals) {
		// Skip irrelevant goals and initial state
		// TODO - The skip irrelevant part should be replaced by new system which replaces
		// empty entries with the last non-empty entry
		if (!val.is_current(time_step) || time_step == 1) {
			continue;
		}
		size_t window_index = val.get_non_empty_index(base_window_index);
		
		// Skip non-goals
		if (window_index == EMPTY_VAL) {
			continue;
		}
		size_t window_length = time_step - window_index - 1;

		// To make sure e.g. T(0) does not make T(0,1) appear useless after handoff
		if (window_index == time_step - 1) {
			window_length += 1;
		}

		float absolute_progress = (float)val.lengths.at(window_index) - (val.lengths.at(time_step - 1));
		float progress = absolute_progress / window_length;

		progress = std::max(std::min(progress, 1.0f), 0.0f);

		for (auto& agent : key.agents.get()) {
			bool is_useful = true;
			auto agent_prob = val.probability;
			for (size_t i = 0; i < agent_permutations.size(); ++i) {
				for (const auto& permutation : agent_permutations.at(i)) {
					
					if (permutation.contains(agent)) {
						continue;
					}
					auto possible_handoff_agents = permutation.get();
					possible_handoff_agents.push_back(EMPTY_VAL);
					for (const auto& handoff_agent : possible_handoff_agents) {
						auto it = goals.find(Goal(Agent_Combination{ permutation }, key.recipe, handoff_agent));
						//if (it != goals.end() && it->second.is_current(time_step) && it->second.probability * delta >= agent_prob) {
						if (it != goals.end() && it->second.is_current(time_step) && it->second.probability >= agent_prob * delta) {
							is_useful = false;
							break;
						}
					}
				}
			}
			if (is_useful) {
				//agents_useful.at(agent.id) = true;
				auto& ref = max_progress.at(agent.id);
				ref = std::max(ref, progress);
			}
		}
	}

	// Update NONE probabilities
	float max_prob = 0.0f;
	for (size_t agent = 0; agent < number_of_agents; ++agent) {
		auto progress_prob = 1 - (max_progress.at(agent) / 3);
		if (time_step == 1) {
			// Not NONE on first round by default
			progress_prob = 0.0f;
		}
		// Note if previous window was optimal
		// TODO - May be replaced by new system of replacing empty length entries
		//bool previous_active_status = time_step == 1 ? true : agents_active_status.at(agent).back();
		
		// If entire window has been optimal
		//agents_active_status.at(agent).push_back(progress_prob == 0.0f);

		// non-probability is 0% the step after having entire optimal window by default
		//if (previous_active_status) {
		//	progress_prob = 0.0f;
		//}

		progress_prob *= beta;

		max_prob = std::max(max_prob, progress_prob);

		Goal goal{  agent, EMPTY_RECIPE, EMPTY_VAL};
		goals.at(goal).probability = progress_prob;
		goals.at(goal).progress_prob = progress_prob;
	}

	return max_prob;
}

void Sliding_Recogniser::normalise(float max_prob) {
	for (auto& [key, val] : goals) {
		val.probability /= max_prob;
	}
}



void Sliding_Recogniser::update(const std::map<Goal, size_t>& goal_lengths, const State& state) {
	++time_step;
	insert(goal_lengths, state);


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


std::map<Goal, float> Sliding_Recogniser::get_raw_goals() const {
	throw std::runtime_error("Not implemented");
}

// False if subset of agents is as likely, true otherwise
bool Sliding_Recogniser::is_probable(Goal goal_input) const {
	auto it_input = goals.find(goal_input);
	if (it_input == goals.end()) {
		return false;
	}
	auto probability = it_input->second.probability;

	if (probability < charlie) {
		return false;
	}

	//for (size_t i = 0; i < goal_input.agents.size() - 1; ++i) {
	//	auto combinations = get_combinations<Agent_Id>(goal_input.agents.get(), i+1);
	//	for (auto& combination : combinations) {
	//		Goal goal{ Agent_Combination{combination}, goal_input.recipe };
	//		auto it = goals.find(goal);
	//		if (it != goals.end() && it->second.is_current(time_step) && it->second.probability >= probability) {
	//			return false;
	//		}
	//	}
	//}
	return true;
}

float Sliding_Recogniser::get_non_probability(Agent_Id agent) const {
	return goals.at(Goal(agent, EMPTY_RECIPE, EMPTY_VAL)).probability;
}

bool Sliding_Recogniser::is_probable_normalised(Goal goal, const std::vector<Goal>& available_goals, 
	Agent_Id acting_agent, Agent_Id planning_agent, bool use_non_probability) const {

	float highest_prob = 0.0f;
	auto it = goals.find(goal);
	if (it == goals.end()) {
		return false;
	}

	// Combinations including idle agents are improbable by default
	for (const auto& agent : goal.agents) {
		if (agent != planning_agent && get_non_probability(agent) == 1.0) {
			return false;
		}
	}
	

	for (const auto& goal : available_goals) {
		if (!goal.agents.contains(acting_agent)) {
			continue;
		}
		auto inner_it = goals.find(goal);
		if (inner_it != goals.end() && inner_it->second.probability > highest_prob) {
			highest_prob = inner_it->second.probability;
		}
	}

	// Check none-probability
	if (use_non_probability) {
		auto non_prob = get_non_probability(acting_agent);
		if (non_prob > highest_prob) {
			highest_prob = non_prob;
		}
	}

	if (highest_prob == 0.0f) {
		return false;
	}

	auto normalised_prob = it->second.probability / highest_prob;

	std::stringstream buffer;
	buffer << std::setprecision(3);
	buffer << "Norm Prob: " 
		<< goal.recipe.result_char() 
		<< goal.agents.to_string() 
		<< "/"
		<< goal.handoff_agent.to_string()
		<< ":" 
		<< acting_agent.id 
		<< " = " 
		<< normalised_prob 
		<< "\n";

	PRINT(Print_Category::RECOGNISER, Print_Level::DEBUG, buffer.str());

	return normalised_prob >= charlie;
}

float Sliding_Recogniser::get_probability(const Goal& goal) const {
	auto it = goals.find(goal);
	if (it == goals.end()) {
		return 0.0f;
	} else {
		return it->second.probability;
	}
}

void Sliding_Recogniser::print_probabilities() const {
	for (const auto& [key, val] : goals) {
		if (!val.is_current(time_step)) continue;
		PRINT(Print_Category::RECOGNISER, Print_Level::DEBUG, 
			static_cast<char>(key.recipe.result) + 
			key.agents.to_string() + 
			key.handoff_agent.to_string() + 
			"\t");
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
	buffers.at(2) << "\n";
	for (auto& buffer : buffers) {
		PRINT(Print_Category::RECOGNISER, Print_Level::DEBUG, buffer.str());
	}
}
