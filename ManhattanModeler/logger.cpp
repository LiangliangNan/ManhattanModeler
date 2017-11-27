#include "logger.h"
#include <iostream> 
#include <sstream>
 
LoggerClient* Logger::client_ = NULL;

 
void Logger::initSet(LoggerClient *c) 
{
	client_ = c;
}


LoggerClient& Logger::output(const std::string& msg) {
	if (client_ == NULL) {
		std::cout << "Logger is NOT initialized!" << std::endl;
		return *client_;
	}

	client_->output(msg);
	return *client_;
}

std::string Logger::endl() {
	if (client_ == NULL) {
		std::cout << "Logger is NOT initialized!" << std::endl;
		return "";
	}

	return ("\n");
}

std::string Logger::convDouble2Str(double val)
{
	std::ostringstream sso;
	sso<<val;
	std::string str = sso.str();
	sso.clear();
	return str;
}

std::string Logger::convInt2Str( int val)
{ 
	std::ostringstream sso;
	sso<<val;
	std::string str = sso.str();
	sso.clear();
	return str;
}

