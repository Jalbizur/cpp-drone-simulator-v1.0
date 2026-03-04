#include "Telemetry.h"
#include <fstream>

void Telemetry::saveCSV(const std::string &filename, const std::string &content) {
	std::ofstream f(filename);
	if (!f.is_open()) return;
	f << content;
}