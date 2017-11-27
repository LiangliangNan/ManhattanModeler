#include "logger_client.h"

LoggerClient& LoggerClient::operator<<(int v) 
{
	output(v);
	return *this;
}

LoggerClient& LoggerClient::operator<<(unsigned int v) 
{
	output(int(v));
	return *this;
}

LoggerClient& LoggerClient::operator<<(float v) {
	output(v);
	return *this;
}

LoggerClient& LoggerClient::operator<<(double v) {
	output(v);
	return *this;
}


LoggerClient& LoggerClient::operator<<(const char* x) {
	output(std::string(x));
	return *this;
} 

LoggerClient& LoggerClient::operator<<(const std::string& x) {
	output(x);
	return *this;
} 
