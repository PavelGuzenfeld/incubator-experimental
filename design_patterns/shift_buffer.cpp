#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>

constexpr size_t calculateWindowSize(size_t frequency, size_t windowSizeMillis)
{
    return (frequency * windowSizeMillis) / 1000;
}

template <typename Entry, size_t BufferSize>
class ShiftBuffer
{
public:
    constexpr ShiftBuffer() : index_(0), is_full_(false) {}

    constexpr void addEntry(Entry const &entry)
    {
        // Shift all entries to the right
        std::move_backward(buffer_.begin(), buffer_.end() - 1, buffer_.end());
        buffer_[0] = entry;

        if (index_ < BufferSize)
        {
            ++index_;
        }
        else
        {
            is_full_ = true;
        }
    }

    constexpr const std::array<Entry, BufferSize> &getBuffer() const
    {
        return buffer_;
    }

    constexpr bool isFull() const
    {
        return is_full_;
    }

private:
    std::array<Entry, BufferSize> buffer_;
    size_t index_;
    bool is_full_;
};

struct DefaultTimeEntry
{
    std::chrono::system_clock::time_point master_sent;
    std::chrono::system_clock::time_point slave_received;
    std::chrono::system_clock::time_point master_received;

    DefaultTimeEntry(std::chrono::system_clock::time_point masterSent = {},
                     std::chrono::system_clock::time_point slaveReceived = {},
                     std::chrono::system_clock::time_point masterReceived = {})
        : master_sent(masterSent), slave_received(slaveReceived), master_received(masterReceived) {}
};

int main()
{
    constexpr size_t frequency = 100;         // Hz
    constexpr size_t windowSizeMillis = 1000; // 1 second
    constexpr size_t bufferSize = calculateWindowSize(frequency, windowSizeMillis);

    ShiftBuffer<DefaultTimeEntry, bufferSize> buffer;

    for (size_t i = 0; i < bufferSize + 5; ++i)
    {
        auto now = std::chrono::system_clock::now();
        buffer.addEntry({now, now + std::chrono::milliseconds(1), now + std::chrono::milliseconds(2)});
    }

    if (buffer.isFull())
    {
        std::cout << "Buffer is full.\n";
    }
    else
    {
        std::cout << "Buffer is not full.\n";
    }

    const auto &history = buffer.getBuffer();
    for (const auto &entry : history)
    {
        std::cout << "Master Sent: " << entry.master_sent.time_since_epoch().count()
                  << " ns, Slave Received: " << entry.slave_received.time_since_epoch().count()
                  << " ns, Master Received: " << entry.master_received.time_since_epoch().count()
                  << " ns\n";
    }

    return 0;
}