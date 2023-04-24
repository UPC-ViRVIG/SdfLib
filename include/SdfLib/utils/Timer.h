#ifndef TIMER_H
#define TIMER_H

#include <chrono>

namespace sdflib
{
class Timer {
public:
	void start();
	float getElapsedSeconds();
	float getElapsedMicroseconds();
	int getElapsedMilliseconds();
private:
	std::chrono::time_point<std::chrono::steady_clock> lastTime;
};
}

#endif