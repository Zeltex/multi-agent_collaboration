#include "Core.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>


static bool save_to_log = false;
static std::stringstream buffer;

void set_logging_enabled() {
	save_to_log = true;
	buffer.str(std::string());
}

void flush_log(const std::string& file_name) {
	std::ofstream file;
	file.open(file_name);
	file << buffer.str();
	file.close();
	buffer.str(std::string());
}

bool is_print_allowed(Print_Level level) {
	return PRINT_LEVEL <= level;
}

void print(Print_Level level, const std::string& msg) {
	if (level == PRINT_LEVEL) {
		if (save_to_log) {
			buffer << msg;
		} else {
			std::cout << msg << std::endl;
		}
	}
}

void print(Print_Category category, const std::string& msg) {
	if (static_cast<size_t>(category) == 1 && PRINT_LEVEL == Print_Level::DEBUG) {
		if (save_to_log) {
			buffer << msg;
		} else {
			std::cout << msg;
		}
	}
}

void print(Print_Category category, Print_Level level, const std::string& msg) {
	if (static_cast<size_t>(category) == 1 && is_print_allowed(level)) {
		if (save_to_log) {
			buffer << msg;
		} else {
			std::cout << msg;
		}
	}
}