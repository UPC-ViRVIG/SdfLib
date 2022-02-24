#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer {
public:
	void start();
	float getElapsedSeconds();
	int getElapsedMilliseconds();
private:
	std::chrono::time_point<std::chrono::steady_clock> lastTime;
};

#endif