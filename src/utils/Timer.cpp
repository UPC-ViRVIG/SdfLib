#include "SdfLib/utils/Timer.h"

namespace sdflib
{
using namespace std;

void Timer::start() {
	lastTime = chrono::steady_clock::now();
}

float Timer::getElapsedSeconds() {
	return float(chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - lastTime).count())/1000000.0f;
}

float Timer::getElapsedMicroseconds() {
	return float(chrono::duration_cast<chrono::nanoseconds>(chrono::steady_clock::now() - lastTime).count())/1000.0f;
}

int Timer::getElapsedMilliseconds() {
	return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - lastTime).count();
}
}