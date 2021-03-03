#include "Core.hpp"
#include <iostream>

void print(Print_Level level, const std::string& msg) {
	if (level == PRINT_LEVEL) {
		std::cout << msg << std::endl;
	}
}