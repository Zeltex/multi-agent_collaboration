
#include "Environment.hpp"
#include "Planner.hpp"
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
	//std::vector<std::string> paths{ "../levels/BD/partial-divider_salad.txt" };
	//std::vector<std::string> paths{ "../levels/BD/full-divider_salad.txt" };
	//std::vector<std::string> paths{ "../levels/BD/open-divider_salad.txt" };
	//std::vector<std::string> paths{ "../levels/BD/open-divider_tl.txt" };
	
	std::vector<Solution> solutions;

	for (const auto& path : paths) {
		auto state = environment.load(path);
		size_t action_count = 0;
		auto time_start = std::chrono::system_clock::now();
		std::vector<Planner> planners;
		for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
			planners.push_back(Planner{ environment, agent, state});

		}
		while (!environment.is_done(state)) {
			std::vector<Action> actions;
			//for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
			for (const auto& planner : planners) {
				actions.push_back(planner.get_next_action(state));
			}
			if (!environment.act(state, { actions })) {
				action_count = 0;
				break;
			}
			//environment.act(state, planner.get_next_action(state));
			environment.print_state(state);
			++action_count;
		}
		auto time_end = std::chrono::system_clock::now();

		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
		std::cout << "Total time: " << diff << std::endl;
		std::cout << "Actions: " << action_count << std::endl;
		solutions.push_back({ diff, action_count, path });
	}
	for (const auto& solution : solutions) {
		std::cout << solution.actions << " : " << solution.time << " : " << solution.path << std::endl;
	}
	size_t something;
	std::cout << " - " << std::endl;
}

int main(int argc, char* argv[]) {
	auto environment = Environment(2);

	auto state = environment.load("../levels/BD/partial-divider_salad.txt");
	environment.print_state(state);

	if (PLAY) {
		environment.play(state);
	} else {
		solve(environment);
	}

	return 0;
}
