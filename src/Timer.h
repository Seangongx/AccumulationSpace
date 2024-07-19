// Timer.h
#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

class Timer {
public:
  Timer(const std::string& name = "") : m_name(name), m_startTimePoint(std::chrono::high_resolution_clock::now()) {}

  void start() { m_startTimePoint = std::chrono::high_resolution_clock::now(); }

  void stop() {
    auto endTimePoint = std::chrono::high_resolution_clock::now();
    m_duration = endTimePoint - m_startTimePoint;
  }

  void print() const {
    std::cout << m_name << " took " << m_duration.count() << " seconds (" << m_duration.count() * 1000.0
              << " milliseconds)\n";
  }

  static std::string now() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
  }

  ~Timer() { std::cout << m_name << "stop counting\n"; }

private:
  std::string m_name;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
  std::chrono::duration<double> m_duration{0};
};

#endif // TIMER_H
