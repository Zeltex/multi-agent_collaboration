
#include "Environment.hpp"
#include "Planner.hpp"
#include "Planner_Mac.hpp"
#include "Planner_Mac_One.hpp"
#include "Planner_Still.hpp"
#include "State.hpp"

#include <iostream>
#include <chrono>
#include <ranges>
#include <filesystem>
#include <fstream>

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
	Planner_Types planner1;
	Planner_Types planner2;
	size_t seed;
};


Solution solve_inner(const std::string& path, const std::vector<Planner_Types>& planner_types, size_t seed) {
	constexpr size_t AGENT_COUNT = 2;

	auto environment = Environment(AGENT_COUNT);
	auto state = environment.load(path);
	size_t action_count = 0;
	auto time_start = std::chrono::system_clock::now();
	std::vector<Planner> planners;
	for (size_t agent = 0; agent < environment.get_number_of_agents(); ++agent) {
		switch (planner_types.at(agent)) {
		case Planner_Types::MAC: {
			planners.emplace_back(std::make_unique<Planner_Mac>(environment, agent, state, seed));
			break;
		}
		case Planner_Types::MAC_ONE: {
			planners.emplace_back(std::make_unique<Planner_Mac_One>(environment, agent, state, seed));
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
	return Solution{ diff, action_count, path, planner_types.at(0), planner_types.at(1), seed };
}


std::string trim_path(const std::string& path_in) {
	auto path = path_in;
	path.erase(path.begin(), std::find_if(path.rbegin(), path.rend(), [](unsigned char ch) {
		return ch == '/' || ch == '\\';
		}).base());

	auto dot_it = std::find_if(path.rbegin(), path.rend(), [](unsigned char ch) {
		return std::ispunct(ch);
		}).base();
	--dot_it;
	path.erase(dot_it, path.end());

	return path;
}

std::string planner_to_string(Planner_Types type) {
	switch (type) {
	case Planner_Types::MAC: return "mac";
	case Planner_Types::MAC_ONE: return "mac1";
	case Planner_Types::STILL: return "still";
	}
}

void write_solutions(const std::vector<Solution>& solutions) {
	size_t sum = 0;
	size_t count = 0;
	size_t sum_solved = 0;
	size_t count_solved = 0;
	size_t sum_mac = 0;
	size_t count_mac = 0;
	std::stringstream buffer;
	for (const auto& solution : solutions) {

		size_t length = solution.actions;
		if (length != 100 && length != 0) {
			sum_solved += length;
			++count_solved;
		}
		sum += length;
		++count;
		std::cout
			<< solution.actions
			<< "\t"
			<< solution.time
			<< "\t"
			<< static_cast<char>(solution.planner1)
			<< "\t"
			<< static_cast<char>(solution.planner2)
			<< "\t"
			<< solution.path
			<< std::endl;

		buffer << trim_path(solution.path) << ";"
			<< planner_to_string(solution.planner1) << ';'
			<< planner_to_string(solution.planner2) << ';'
			<< solution.seed << ';'
			<< solution.actions << ';'
			<< solution.time << '\n';

		if (solution.planner1 == Planner_Types::MAC && solution.planner2 == Planner_Types::MAC) {
			sum_mac += solution.actions;
			++count_mac;
		}
	}
	if (count_mac > 0) std::cout << "mac-mac avg = " << ((1.0f * sum_mac) / count_mac) << std::endl;
	if (count > 0) std::cout << count << " total, avg = " << ((1.0f * sum) / count) << std::endl;
	if (count_solved > 0) std::cout << count_solved << " solved, avg = " << ((1.0f * sum_solved) / count_solved) << std::endl;
	std::cout << " - " << std::endl;
	auto file = std::ofstream("../results/result.txt");
	file << buffer.str();
	file.close();
}

void solve() {
	//open-divider_salad.txt


	bool run_single = false;
	//bool run_single = true;


	if (run_single) {
		std::vector<Planner_Types> planner_types{
			Planner_Types::STILL,
			//Planner_Types::MAC_ONE,
			Planner_Types::MAC,
			//Planner_Types::MAC,
			//Planner_Types::STILL,
		};
		//std::vector<std::string> paths{ "../levels/BD/full-divider_salad.txt" };
		//std::vector<std::string> paths{ "../levels/BD/full-divider_tl.txt" };
		//std::vector<std::string> paths{ "../levels/BD/full-divider_tomato.txt" };
		//std::vector<std::string> paths{ "../levels/BD/open-divider_salad.txt" };
		//std::vector<std::string> paths{ "../levels/BD/open-divider_tl.txt" };
		//std::vector<std::string> paths{ "../levels/BD/open-divider_tomato.txt" };
		std::vector<std::string> paths{ "../levels/BD/partial-divider_salad.txt" };
		//std::vector<std::string> paths{ "../levels/BD/partial-divider_tl.txt" };
		//std::vector<std::string> paths{ "../levels/BD/partial-divider_tomato.txt" };
		//std::vector<std::string> paths{ "../levels/Test_Scenarios/L1.txt" };
		//std::vector<std::string> paths{ "../levels/Test_Scenarios/L2.txt" };
		//std::vector<std::string> paths{ "../levels/Test_Scenarios/L3.txt" };
		//std::vector<std::string> paths{ "../levels/Test_Scenarios/L4.txt" };
		size_t seed = 0;			
		bool triple = true;
		Planner_Types triple_type = Planner_Types::MAC;
		if (triple) {
			solve_inner(paths.at(0), { triple_type, triple_type, triple_type }, seed);
		} else {
			solve_inner(paths.at(0), planner_types, seed);
		}
		//std::cin.get();
	} else {
		bool self_play = true;
		auto paths = get_all_files("../levels/BD/");
		std::vector<Solution> solutions;

		std::vector<Planner_Types> planner_types{
			//Planner_Types::STILL,
			//Planner_Types::MAC,
			Planner_Types::MAC,
			Planner_Types::MAC_ONE,
			Planner_Types::STILL,
		};

		bool triple = true;
		Planner_Types triple_type = Planner_Types::MAC;
		for (size_t seed = 0; seed < 1; ++seed) {
			if (triple) {
				for (const auto& path : paths) {
					solutions.push_back(solve_inner(path, { triple_type, triple_type, triple_type }, seed));
					write_solutions(solutions);
				}
			} else {
				for (const auto& planner1 : planner_types) {
					for (const auto& planner2 : planner_types) {
						//if (self_play && (planner1 != Planner_Types::MAC || planner2 != Planner_Types::MAC)) {
						if (self_play && (planner1 != Planner_Types::MAC || planner2 != Planner_Types::MAC_ONE)
							&& (planner1 != Planner_Types::MAC_ONE || planner2 != Planner_Types::MAC)) {
							continue;
						}
						for (const auto& path : paths) {
							solutions.push_back(solve_inner(path, { planner1, planner2 }, seed));
						}
					}
				}
			}
		}
		write_solutions(solutions);
	}
}

int main(int argc, char* argv[]) {
	if (PLAY) {
		auto environment = Environment(2);
		auto state = environment.load("../levels/BD/partial-divider_salad.txt");
		environment.print_state(state);
		environment.play(state);
	} else {
		solve();
	}

	return 0;
}
