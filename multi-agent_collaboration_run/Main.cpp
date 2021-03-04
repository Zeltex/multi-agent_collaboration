
#include "Environment.hpp"
#include "Planner.hpp"
#include <iostream>



int main(int argc, char* argv[]) {

	auto environment = Environment(2);

	auto state = environment.load("../levels/BD/partial-divider_salad.txt");
	environment.print_state(state);

	Planner planner(environment, { 0 });
	size_t action_count = 0;
	while (!environment.is_done(state)) {
		environment.act(state, planner.get_next_action(state));
		environment.print_state(state);
		++action_count;
	}

	std::cout << "Actions: " << action_count << std::endl;

	//environment.play(state);

	return 0;
}