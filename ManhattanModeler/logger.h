#pragma once

#include <string>
#include "logger_client.h"

class Logger
{
public: 
	static void initSet(LoggerClient* c);
	static LoggerClient& output(const std::string& msg = "") ;
	static std::string endl();

	static std::string convDouble2Str(double val);
	static std::string convInt2Str (int val);

private:
	static LoggerClient*	client_;
};
