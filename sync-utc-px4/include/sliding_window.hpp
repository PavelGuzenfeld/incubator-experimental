#pragma once
#include <deque> // std::deque

template<typename T, std::size_t N>
class SlideWindow : public std::deque<T> 
{
public:
    void push(const T& value) {
        if (this->size() == N) {
            this->pop_front();
        }
        this->push_back(value);
    }
};