
#include "Environment.hpp"
#include <iostream>



int main(int argc, char* argv[]) {

	auto environment = Environment(2);

	auto state = environment.load("../levels/BD/partial-divider_salad.txt");
	environment.print_state(state);


	environment.play(state);

	return 0;
}