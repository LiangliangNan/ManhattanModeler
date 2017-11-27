#pragma once
 
#include <string>


class LoggerClient 
{
public:
	LoggerClient() {}

	virtual ~LoggerClient() {}

	virtual LoggerClient* output(const std::string& msg) = 0;
	virtual LoggerClient* output(int v) = 0;
	virtual LoggerClient* output(double v) = 0;

	LoggerClient& operator<<(int v) ;
	LoggerClient& operator<<(unsigned int v) ;
	LoggerClient& operator<<(float v) ;
	LoggerClient& operator<<(double v) ;
	LoggerClient& operator<<(const char* x) ;
	LoggerClient& operator<<(const std::string& msg) ;
} ;