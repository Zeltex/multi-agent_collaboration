#pragma once
#include <string>

enum class Print_Level {
	NOPE=0,
	DEBUG=1,
	INFO=2
};

enum class Print_Category {
	STATE=1,
	A_STAR=1,
	PLANNER=1,
	ENVIRONMENT=1
};

#ifndef PRINT_LEVEL
#define PRINT_LEVEL Print_Level::INFO
#endif


void print(Print_Level level, const std::string& msg);
void print(Print_Category category, const std::string& msg);

#define PRINT(level_category, msg) print(level_category, msg)

#define EMPTY_VAL (size_t)-1