#include <atomic>
#include <chrono>
#include <cstddef> // For std::byte and alignas
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <new>       // For placement new
#include <stdexcept> // For std::invalid_argument
#include <string>
#include <thread>
#include <vector>

// To prevent false sharing, we align the head and tail pointers to a cache line boundary.
// 64 bytes is a common cache line size for modern x86 processors.
// False sharing occurs when unrelated data in the same cache line is modified by different
// threads, causing unnecessary cache invalidations and reducing performance.
constexpr size_t CACHE_LINE_SIZE = 64;

/**
 * @brief A Single-Producer, Single-Consumer lock-free queue.
 *
 * This implementation uses a circular buffer and atomic variables with optimized
 * memory ordering (acquire-release semantics) to ensure thread safety and
 * high performance without using locks.
 *
 * It is designed to be used with one dedicated producer thread and one dedicated
 * consumer thread. Using it with multiple producers or consumers will lead to
 * data races and undefined behavior.
 *
 * @tparam T The type of elements to be stored in the queue.
 */
template <typename T>
class SPSCQueue
{
public:
    /**
     * @brief Constructs the queue with a given capacity.
     * @param capacity The maximum number of items the queue can hold.
     */
    explicit SPSCQueue(size_t capacity)
        // We allocate one extra slot to easily distinguish between a full and an empty queue.
        // When head == tail, the queue is empty.
        // When (tail + 1) % capacity == head, the queue is full.
        : capacity_(capacity + 1),
          buffer_(new std::byte[sizeof(T) * (capacity + 1)]),
          head_(0),
          tail_(0)
    {
        if (capacity == 0)
        {
            throw std::invalid_argument("Queue capacity must be positive.");
        }
    }

    /**
     * @brief Destructor. Destroys any remaining elements in the queue.
     */
    ~SPSCQueue()
    {
        // Destruct any elements that were pushed but not popped.
        while (pop([](T && /*item*/) {}))
            ;
        delete[] buffer_;
    }

    // The queue is non-copyable and non-movable to prevent ownership issues.
    SPSCQueue(const SPSCQueue &) = delete;
    SPSCQueue &operator=(const SPSCQueue &) = delete;
    SPSCQueue(SPSCQueue &&) = delete;
    SPSCQueue &operator=(SPSCQueue &&) = delete;

    /**
     * @brief Pushes an item into the queue. To be called by the producer thread only.
     * @param item The item to push.
     * @return true if the item was pushed successfully, false if the queue was full.
     */
    bool push(const T &item)
    {
        // Load tail with relaxed memory order. Only this producer thread modifies tail,
        // so we don't need stronger ordering for this load.
        const auto curr_tail = tail_.load(std::memory_order_relaxed);

        // Calculate the next tail position in the circular buffer.
        const auto next_tail = (curr_tail + 1 == capacity_) ? 0 : curr_tail + 1;

        // Check if the queue is full. We must see the latest value of 'head' from
        // the consumer. An 'acquire' load synchronizes with the consumer's 'release'
        // store on 'head', ensuring we don't overwrite an element that hasn't been read yet.
        if (next_tail == head_.load(std::memory_order_acquire))
        {
            return false; // Queue is full.
        }

        // Construct the item in the buffer at the current tail position using placement new.
        new (reinterpret_cast<T *>(buffer_) + curr_tail) T(item);

        // Store the new tail position. This is a 'release' operation. It ensures that
        // the write to the buffer (the placement new) is visible to the consumer
        // *before* the consumer sees the updated tail. This is the crucial step that
        // prevents the consumer from reading an uninitialized or partially written value.
        tail_.store(next_tail, std::memory_order_release);

        return true;
    }

    /**
     * @brief Pops an item from the queue by passing it to a function.
     * To be called by the consumer thread only.
     * @param func A callable (e.g., a lambda) that will receive the popped item.
     * @return true if an item was popped, false if the queue was empty.
     */
    bool pop(auto &&func)
    {
        // Load head with relaxed memory order. Only this consumer thread modifies head.
        const auto curr_head = head_.load(std::memory_order_relaxed);

        // Check if the queue is empty. We need to see the latest data written by the
        // producer. An 'acquire' load on 'tail' synchronizes with the producer's
        // 'release' store on 'tail'. This guarantees that if we see a new tail value,
        // we also see the data that was written before it.
        if (curr_head == tail_.load(std::memory_order_acquire))
        {
            return false; // Queue is empty.
        }

        // Get a pointer to the element in the buffer.
        T *elem = reinterpret_cast<T *>(buffer_) + curr_head;

        // Pass the element to the user-provided function.
        // We use std::move to efficiently transfer ownership if the function takes by value.
        func(std::move(*elem));

        // Explicitly call the destructor for the object in the buffer.
        elem->~T();

        // Calculate the next head position.
        const auto next_head = (curr_head + 1 == capacity_) ? 0 : curr_head + 1;

        // Store the new head position. This is a 'release' operation. It makes the
        // newly freed slot available to the producer and ensures that the destruction
        // of the object is finished before the producer can try to write to this slot.
        head_.store(next_head, std::memory_order_release);

        return true;
    }

private:
    const size_t capacity_;
    // Use std::byte for raw memory allocation to avoid default construction of T.
    std::byte *const buffer_;

    // Align to prevent false sharing.
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> head_;
    // Add padding to ensure head_ and tail_ are on different cache lines.
    char pad_[CACHE_LINE_SIZE - sizeof(std::atomic<size_t>)];
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> tail_;
};

// A simple struct to test with non-trivial types (e.g., with a std::string).
struct Message
{
    int id;
    std::string data;

    // Default constructor for convenience
    Message() : id(0), data("") {}
    // Parameterized constructor
    Message(int i, std::string d) : id(i), data(std::move(d)) {}
};

// The task for the producer thread.
void producer_task(SPSCQueue<Message> *queue, int message_count)
{
    for (int i = 0; i < message_count; ++i)
    {
        Message msg(i, fmt::format("Message {}", i));
        // Spin-wait until the push is successful.
        while (!queue->push(msg))
        {
            // In a real application, you might yield the thread or use a
            // different synchronization mechanism to avoid burning CPU cycles.
            std::this_thread::yield();
        }
    }
}

// The task for the consumer thread.
void consumer_task(SPSCQueue<Message> *queue, int message_count, std::atomic<int> *consumed_count)
{
    int count = 0;
    while (count < message_count)
    {
        // Spin-wait until a pop is successful.
        bool popped = queue->pop([&](Message &&msg)
                                 {
            // Simple validation to ensure messages are received in order.
            if (msg.id != count) {
                // This should not happen in a correct SPSC implementation.
                fmt::print(stderr, "Error: Consumed message with id {}, expected {}\n", msg.id, count);
            }
            count++; });

        if (!popped)
        {
            std::this_thread::yield();
        }
    }
    // Report the total number of consumed messages back to the main thread.
    consumed_count->store(count);
}

int main()
{
    const int message_count = 500000;
    const int queue_capacity = 1024;

    SPSCQueue<Message> queue(queue_capacity);
    std::atomic<int> consumed_count(0);

    fmt::print("Starting producer and consumer threads to exchange {} messages...\n", message_count);

    auto start_time = std::chrono::high_resolution_clock::now();

    std::thread producer(producer_task, &queue, message_count);
    std::thread consumer(consumer_task, &queue, message_count, &consumed_count);

    producer.join();
    consumer.join();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;

    fmt::print("Finished.\n");
    fmt::print("Total messages consumed: {}\n", consumed_count.load());
    fmt::print("Time taken: {:.2f} ms\n", duration.count());

    return 0;
}
