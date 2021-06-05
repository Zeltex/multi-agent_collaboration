#pragma once
#include <string>

enum class Print_Level {
	NOPE=0,
	VERBOSE,
	DEBUG,
	INFO
};

enum class Print_Category {
	STATE=1,
	A_STAR=1,
	PLANNER=1,
	ENVIRONMENT=1,
	RECOGNISER=1,
	UTILS=1,
};

#ifndef PRINT_LEVEL
#define PRINT_LEVEL Print_Level::DEBUG
#endif

bool is_print_allowed(Print_Level level);
void print(Print_Level level, const std::string& msg);
void print(Print_Category category, const std::string& msg);
void print(Print_Category category, Print_Level level, const std::string& msg);

void set_logging_enabled();
void flush_log(const std::string& file_name);

#define PRINT(category, level, msg) print(category, level, msg)

#define EMPTY_VAL 9999
#define HIGH_INIT_VAL 99999