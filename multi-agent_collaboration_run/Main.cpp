
#include "Environment.hpp"
#include "Planner.hpp"
#include "Planner_Mac.hpp"
#include "Planner_Still.hpp"
#include "State.hpp"

#include <iostream>
#include <chrono>
#include <ranges>
#include <filesystem>

#define PLAY 0

std::vector<std::string> get_all_files(std::string base_path) {
	std::vector<std::string> paths;
	for (const auto& entry : std::filesystem::directory_iterator(base_path)) {
		auto path = entry.path().string();
		if (path.substr(path.size() - 4) == ".txt") {
			paths.push_back(path);
		}
	}
	return paths;
}

struct Solution {
	long long time;
	size_t actions;
	std::string path;
};



void solve(Environment& environment) {
	//open-divider_salad.txt

	auto paths = get_all_files("../levels/BD/");
	//std::vector<std::string> paths{ "../levels/BD/full-divider_salad.txt" };
	//std::vector<std::string> paths{ "../levels/BD/full-divider_tl.txt" };
	//std::vector<std::string> paths{ "../levels/BD/full-divider_tomato.txt" };
	//std::vector<std::string> paths{ "../levels/BD/open-divider_salad.txt" };
	//std::vector<std::string> paths{ "../levels/BD/open-divider_tl.txt" };
	//std::vector<std::string> paths{ "../levels/BD/open-divider_tomato.txt" };
	//std::vector<std::string> paths{ "../levels/BD/partial-divider_salad.txt" };
	//std::vector<std::string> paths{ "../levels/BD/partial-divider_tl.txt" };
	//std::vector<std::string> paths{ "../levels/BD/partial-divider_tomato.txt" };
	//std::vector<std::string> paths{ "../levels/Test_Scenarios/L1.txt" };
	//std::vector<std::string> paths{ "../levels/Test_Scenarios/L2.txt" };
	
	std::vector<Solution> solutions;

	std::vector<Planner_Types> planner_types{
		//Planner_Types::STILL,
		Planner_Types::MAC,
		Planner_Types::MAC 
	};

	for (const auto& path : paths) {
		auto state = environment.load(path);
		size_t action_count = 0;
		auto time_start = std::chrono::system_clock::now();
		std::vector<Planner> planners;
		for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
			switch (planner_types.at(agent)) {
			case Planner_Types::MAC: {
				planners.emplace_back(std::make_unique<Planner_Mac>(environment, agent, state));
				break;
			}
			case Planner_Types::STILL: {
				planners.emplace_back(std::make_unique<Planner_Still>(environment, agent, state));
				break;
			}
			}
		}
		while (!environment.is_done(state)) {
			std::vector<Action> actions;
			environment.print_state(state);
			//for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
			for (auto& planner : planners) {
				actions.push_back(planner.get_next_action(state));
			}
			environment.act(state, { actions });
			if (action_count == 100) {
				action_count = 0;
				break;
			}
			//environment.act(state, planner.get_next_action(state));
			++action_count;
		}
		environment.print_state(state);
		auto time_end = std::chrono::system_clock::now();

		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
		std::cout << "Total time: " << diff << std::endl;
		std::cout << "Actions: " << action_count << std::endl;
		solutions.push_back({ diff, action_count, path });
	}
	for (const auto& solution : solutions) {
		std::cout << solution.actions << " : " << solution.time << " : " << solution.path << std::endl;
	}
	std::cout << " - " << std::endl;
}

int main(int argc, char* argv[]) {
	auto environment = Environment(2);


	if (PLAY) {
		auto state = environment.load("../levels/BD/partial-divider_salad.txt");
		environment.print_state(state);
		environment.play(state);
	} else {
		solve(environment);
	}

	return 0;
}
