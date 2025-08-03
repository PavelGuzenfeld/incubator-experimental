High-Performance Lock-Free SPSC Queue
This project provides a C++ implementation of a high-performance, lock-free, single-producer, single-consumer (SPSC) queue. It is based on the optimization techniques presented by Sarthak Sehgal, focusing on correct memory ordering to ensure thread safety and maximize throughput without the use of mutexes.

Overview
The core of this project is the SPSCQueue<T> class, a template-based queue that allows one thread to safely and efficiently push items while another thread safely and efficiently pops them. It uses a circular buffer for storage and atomic operations with acquire-release semantics to synchronize the producer and consumer threads.

This approach avoids the common performance pitfalls of lock-based synchronization and is designed to be highly efficient on modern multi-core processors.

Key Features
Lock-Free: Uses std::atomic instead of mutexes, eliminating contention and the risk of deadlocks.

Single-Producer, Single-Consumer (SPSC): Specialized design for the common SPSC pattern, allowing for simpler and faster logic than a general-purpose MPSC or MPMC queue.

Cache-Friendly: The head and tail indices are aligned to cache line boundaries (alignas(64)) to prevent "false sharing," a subtle but significant performance killer in multi-threaded applications.

Optimized Memory Ordering: Utilizes std::memory_order_acquire and std::memory_order_release to create a minimal, yet sufficient, memory barrier. This prevents memory reordering issues between the threads without incurring the cost of a full, sequentially-consistent fence.

No Dynamic Allocation during Operation: Memory for the queue's buffer is allocated once at construction. The queue then uses placement new and explicit destructor calls to manage the lifecycle of objects within this buffer, avoiding the overhead of new/delete during push/pop operations.

How to Compile and Run
You can run this code using an online compiler like Godbolt or by compiling it locally on your machine.

Option 1: Using Compiler Explorer (Godbolt)
This is the simplest way to test the code.

Open Godbolt: Navigate to https://godbolt.org/.

Select Compiler: Choose a recent C++ compiler, such as x86-64 clang (trunk) or x86-64 gcc (trunk).

Add Library: Click the "Libraries" button (looks like a stack of books) and select the fmt library. This will add the necessary include path.

Paste Code: Copy the entire C++ code from the spsc_queue_godbolt artifact and paste it into the source editor panel.

Set Compiler Flags: In the compiler options field, add the following flags:

-O3 -pthread

-O3: Enables high-level optimizations.

-pthread: Links the POSIX threads library, which is required for std::thread.

Run: The code will compile automatically, and the program's output will appear in the execution window.

Option 2: Compiling Locally (with g++ or Clang)
To compile this on your own machine, you will need a C++ compiler that supports C++17 or newer and the {fmt} library.

1. Install Dependencies:

g++/Clang: Ensure you have a modern C++ compiler installed.

{fmt} Library: You need to install the {fmt} library. On Debian/Ubuntu, you can do this with:

sudo apt-get install libfmt-dev

On other systems, you may need to use a different package manager or build it from source from its GitHub repository.

2. Save the Code:

Save the C++ code from the spsc_queue_godbolt artifact into a file named spsc_queue.cpp.

3. Compile and Run:

Open your terminal, navigate to the directory where you saved the file, and run the following command:

g++ -std=c++17 -O3 -pthread spsc_queue.cpp -o spsc_benchmark -lfmt

-std=c++17: Specifies the C++ standard.

-O3: Enables optimizations.

-pthread: Links the threading library.

-o spsc_benchmark: Names the output executable spsc_benchmark.

-lfmt: Links against the {fmt} library.

After the compilation is successful, run the benchmark:

./spsc_benchmark

You should see output similar to this:

Starting producer and consumer threads to exchange 500000 messages...
Finished.
Total messages consumed: 500000
Time taken: 45.12 ms

(Note: The exact time will vary depending on your system's hardware.)