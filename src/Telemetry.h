#pragma once
#include <string>

class Telemetry {
public:
	static void saveCSV(const std::string &filename, const std::string &content);
};
