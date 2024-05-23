// Timer.h
#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <chrono>
#include <string>

class Timer
{
public:
    Timer(const std::string &name = "") : m_name(name), m_startTimePoint(std::chrono::high_resolution_clock::now()) {}

    void start()
    {
        m_startTimePoint = std::chrono::high_resolution_clock::now();
    }

    void stop()
    {
        auto endTimePoint = std::chrono::high_resolution_clock::now();
        m_duration = endTimePoint - m_startTimePoint;
    }

    void print() const
    {
        std::cout << m_name << " took " << m_duration.count() << " seconds ("
                  << m_duration.count() * 1000.0 << " milliseconds)\n";
    }

    ~Timer()
    {
        std::cout << m_name << "stop counting\n";
    }

private:
    std::string m_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    std::chrono::duration<double> m_duration{0};
};

#endif // TIMER_H
