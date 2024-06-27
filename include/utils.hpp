#ifndef __UTILS_PI__
#define __UTILS_PI__
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

typedef float float32_t;

void delay(int64_t sleep_ms);

void log_msg(const std::string &msg);

uint64_t millis();

template <typename T>
T min(const T& a, const T& b)
{
    if (a <= b)
    {
        return a;
    }
    else {
        return b;
    }
}

template <typename T>
T max(const T& a, const T& b)
{
    if (a <= b)
    {
        return b;
    }
    else {
        return a;
    }
}



#endif