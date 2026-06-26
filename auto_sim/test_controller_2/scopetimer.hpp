#pragma once
#include <chrono>
#include <iostream>

class ScopeTimer {
public:
    // Constructor: Starts the timer
    explicit ScopeTimer(const std::string& name)
        : m_name(name), m_start_time(std::chrono::steady_clock::now()) { }

    // Destructor: Stops the timer and prints elapsed time
    ~ScopeTimer() {
        const auto end_time = std::chrono::steady_clock::now();
        const auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - m_start_time).count();
        std::cout << "Timer [" << m_name << "]: " << duration_us << " us" << std::endl;
    }

private:
    std::string m_name;
    std::chrono::time_point<std::chrono::steady_clock> m_start_time;
};