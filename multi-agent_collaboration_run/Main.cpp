
#include "Environment.hpp"
#include "Planner.hpp"
#include <iostream>
#include <chrono>

#define PLAY 0

int main(int argc, char* argv[]) {

	auto environment = Environment(2);

	auto state = environment.load("../levels/BD/partial-divider_salad.txt");
	environment.print_state(state);

	if (PLAY) {
		environment.play(state);
	} else {
		Planner planner(environment, { 0 });
		size_t action_count = 0;
		auto time_start = std::chrono::system_clock::now();
		while (!environment.is_done(state)) {
			environment.act(state, planner.get_next_action(state));
			environment.print_state(state);
			++action_count;
		}
		auto time_end = std::chrono::system_clock::now();

		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
		std::cout << "Total time: " << diff << std::endl;
		std::cout << "Actions: " << action_count << std::endl;
	}

	return 0;
}