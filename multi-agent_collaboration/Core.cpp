#include "Core.hpp"
#include <iostream>

void print(Print_Level level, const std::string& msg) {
	if (level == PRINT_LEVEL) {
		std::cout << msg << std::endl;
	}
}

void print(Print_Category category, const std::string& msg) {
	if (static_cast<size_t>(category) == 1 && PRINT_LEVEL == Print_Level::DEBUG) {
		std::cout << msg;
	}
}

void print(Print_Category category, Print_Level level, const std::string& msg) {
	if (static_cast<size_t>(category) == 1 && PRINT_LEVEL <= level) {
		std::cout << msg;
	}
}