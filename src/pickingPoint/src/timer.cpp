#include <timer.hpp>

#include <cstdio>

Timer::Timer() 
{
    m_Start = std::chrono::high_resolution_clock::now();
}

Timer::~Timer() 
{
    std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = stop - m_Start;
    float ms = duration.count() * 1000.0f;
    printf("\033[1;32mElapsed time: %f ms\033[0m\n", ms);
}