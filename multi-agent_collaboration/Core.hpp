#pragma once
#include <string>

enum class Print_Level {
	NOPE=0,
	DEBUG,
	RELEASE
};

#ifndef PRINT_LEVEL
#define PRINT_LEVEL Print_Level::DEBUG
#endif


void print(Print_Level level, const std::string& msg);

#define PRINT(level, msg) print(level, msg)
#define EMPTY_VAL (size_t)-1