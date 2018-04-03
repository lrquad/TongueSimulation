#include "converCharToVector.h"
#include <iterator>

std::vector<int> convertCharToInt(const char* text)
{
	std::istringstream iss(text);
	std::istream_iterator<int> it(iss), end;
	std::vector<int> result_(it, end);
	return result_;
}

std::vector<double> convertCharToDouble(const char* text)
{
	std::istringstream iss(text);
	std::istream_iterator<double> it(iss), end;
	std::vector<double> result_(it, end);
	return result_;
}
