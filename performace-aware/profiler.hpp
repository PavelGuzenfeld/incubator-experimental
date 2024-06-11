#include <chrono>
#include <cstdint>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#elif defined(__linux__) && !(defined(__arm__) || defined(__aarch64__))
#include <sys/time.h>
#include <x86intrin.h>
#elif defined(__arm__) || defined(__aarch64__)
#include <sys/time.h>
#else
#error "Unsupported platform"
#endif

// get OS timer frequency
constexpr std::uint64_t GetOSTimerFreq()
{
#if defined(_WIN32) || defined(_WIN64)
    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);
    return static_cast<std::uint64_t>(freq.QuadPart);
#else
    return 1'000'000; // microseconds
#endif
}

// read OS timer
std::uint64_t ReadOSTimer()
{
#if defined(_WIN32) || defined(_WIN64)
    LARGE_INTEGER value;
    QueryPerformanceCounter(&value);
    return static_cast<std::uint64_t>(value.QuadPart);
#else
    struct timeval value;
    gettimeofday(&value, nullptr);
    return GetOSTimerFreq() * static_cast<std::uint64_t>(value.tv_sec) + static_cast<std::uint64_t>(value.tv_usec);
#endif
}

// read CPU timer
inline std::uint64_t ReadCPUTimer()
{
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386) || defined(_M_IX86)
    return __rdtsc();
#elif defined(__arm__) || defined(__aarch64__)
    std::uint64_t cntvct;
    asm volatile("mrs %0, cntvct_el0" : "=r"(cntvct));
    return cntvct;
#else
#error "Unsupported platform"
#endif
}

// estimate CPU timer frequency
std::uint64_t EstimateCPUTimerFreq()
{
    constexpr std::uint64_t milliseconds_to_wait = 100;
    const std::uint64_t os_freq = GetOSTimerFreq();

    const std::uint64_t cpu_start = ReadCPUTimer();
    const std::uint64_t os_start = ReadOSTimer();

    std::uint64_t os_end = 0;
    std::uint64_t os_elapsed = 0;
    const std::uint64_t os_wait_time = os_freq * milliseconds_to_wait / 1'000;

    while (os_elapsed < os_wait_time)
    {
        os_end = ReadOSTimer();
        os_elapsed = os_end - os_start;
    }

    const std::uint64_t cpu_end = ReadCPUTimer();
    const std::uint64_t cpu_elapsed = cpu_end - cpu_start;

    return os_elapsed ? (os_freq * cpu_elapsed / os_elapsed) : 0;
}

#ifndef PROFILER
#define PROFILER 0
#endif

#ifndef READ_BLOCK_TIMER
#define READ_BLOCK_TIMER ReadCPUTimer
#endif

#if PROFILER

struct ProfileAnchor
{
    std::uint64_t TSCElapsedExclusive{}; // does NOT include children
    std::uint64_t TSCElapsedInclusive{}; // DOES include children
    std::uint64_t HitCount{};
    std::uint64_t ProcessedByteCount{};
    const char *Label{};
};
constexpr std::size_t MaxProfilerAnchors = 4096;
static ProfileAnchor GlobalProfilerAnchors[MaxProfilerAnchors];
static std::uint32_t GlobalProfilerParent{};

struct ProfileBlock
{
    ProfileBlock(const char *label, std::uint32_t anchorIndex, std::uint64_t byteCount)
        : Label{label}, AnchorIndex{anchorIndex}
    {
        ParentIndex = GlobalProfilerParent;

        ProfileAnchor *anchor = &GlobalProfilerAnchors[AnchorIndex];
        OldTSCElapsedInclusive = anchor->TSCElapsedInclusive;
        anchor->ProcessedByteCount += byteCount;

        GlobalProfilerParent = AnchorIndex;
        StartTSC = READ_BLOCK_TIMER();
    }

    ~ProfileBlock()
    {
        std::uint64_t elapsed = READ_BLOCK_TIMER() - StartTSC;
        GlobalProfilerParent = ParentIndex;

        ProfileAnchor *parent = &GlobalProfilerAnchors[ParentIndex];
        ProfileAnchor *anchor = &GlobalProfilerAnchors[AnchorIndex];

        parent->TSCElapsedExclusive -= elapsed;
        anchor->TSCElapsedExclusive += elapsed;
        anchor->TSCElapsedInclusive = OldTSCElapsedInclusive + elapsed;
        ++anchor->HitCount;

        // This write happens every time solely because there is no straightforward way in C++
        // to have the same ease-of-use. In a better programming language, it would be simple
        // to have the anchor points gathered and labeled at compile time, and this repetitive write would be eliminated.
        anchor->Label = Label;
    }

    const char *Label;
    std::uint64_t OldTSCElapsedInclusive{};
    std::uint64_t StartTSC{};
    std::uint32_t ParentIndex{};
    std::uint32_t AnchorIndex{};
};

#define NameConcat2(A, B) A##B
#define NameConcat(A, B) NameConcat2(A, B)
#define TimeBandwidth(Name, ByteCount) ProfileBlock NameConcat(Block, __LINE__)(Name, __COUNTER__ + 1, ByteCount)
#define ProfilerEndOfCompilationUnit static_assert(__COUNTER__ < MaxProfilerAnchors, "Number of profile points exceeds size of profiler::Anchors array")

static void PrintTimeElapsed(std::uint64_t totalTSCElapsed, std::uint64_t timerFreq, const ProfileAnchor *anchor)
{
    double percent = 100.0 * (static_cast<double>(anchor->TSCElapsedExclusive) / static_cast<double>(totalTSCElapsed));
    std::printf("  %s[%llu]: %llu (%.2f%%", anchor->Label, anchor->HitCount, anchor->TSCElapsedExclusive, percent);
    if (anchor->TSCElapsedInclusive != anchor->TSCElapsedExclusive)
    {
        double percentWithChildren = 100.0 * (static_cast<double>(anchor->TSCElapsedInclusive) / static_cast<double>(totalTSCElapsed));
        std::printf(", %.2f%% w/children", percentWithChildren);
    }
    std::printf(")");

    if (anchor->ProcessedByteCount)
    {
        constexpr double Megabyte = 1024.0 * 1024.0;
        constexpr double Gigabyte = Megabyte * 1024.0;

        double seconds = static_cast<double>(anchor->TSCElapsedInclusive) / static_cast<double>(timerFreq);
        double bytesPerSecond = static_cast<double>(anchor->ProcessedByteCount) / seconds;
        double megabytes = static_cast<double>(anchor->ProcessedByteCount) / Megabyte;
        double gigabytesPerSecond = bytesPerSecond / Gigabyte;

        std::printf("  %.3fmb at %.2fgb/s", megabytes, gigabytesPerSecond);
    }

    std::printf("\n");
}

static void PrintAnchorData(std::uint64_t totalCPUElapsed, std::uint64_t timerFreq)
{
    for (std::size_t anchorIndex = 0; anchorIndex < MaxProfilerAnchors; ++anchorIndex)
    {
        ProfileAnchor *anchor = &GlobalProfilerAnchors[anchorIndex];
        if (anchor->TSCElapsedInclusive)
        {
            PrintTimeElapsed(totalCPUElapsed, timerFreq, anchor);
        }
    }
}

#else

#define TimeBandwidth(...)
#define PrintAnchorData(...)
#define ProfilerEndOfCompilationUnit

#endif

struct Profiler
{
    std::uint64_t StartTSC{};
    std::uint64_t EndTSC{};
};
static Profiler GlobalProfiler;

#define TimeBlock(Name) TimeBandwidth(Name, 0)
#define TimeFunction TimeBlock(__func__)

static std::uint64_t EstimateBlockTimerFreq()
{
    (void)&EstimateCPUTimerFreq; // prevent compilers from warning us that it is not used

    constexpr std::uint64_t millisecondsToWait = 100;
    const std::uint64_t osFreq = GetOSTimerFreq();

    const std::uint64_t blockStart = READ_BLOCK_TIMER();
    const std::uint64_t osStart = ReadOSTimer();
    std::uint64_t osEnd = 0;
    std::uint64_t osElapsed = 0;
    const std::uint64_t osWaitTime = osFreq * millisecondsToWait / 1000;
    while (osElapsed < osWaitTime)
    {
        osEnd = ReadOSTimer();
        osElapsed = osEnd - osStart;
    }

    const std::uint64_t blockEnd = READ_BLOCK_TIMER();
    const std::uint64_t blockElapsed = blockEnd - blockStart;

    return osElapsed ? (osFreq * blockElapsed / osElapsed) : 0;
}

static void BeginProfile()
{
    GlobalProfiler.StartTSC = READ_BLOCK_TIMER();
}

static void EndAndPrintProfile()
{
    GlobalProfiler.EndTSC = READ_BLOCK_TIMER();
    const std::uint64_t timerFreq = EstimateBlockTimerFreq();

    const std::uint64_t totalTSCElapsed = GlobalProfiler.EndTSC - GlobalProfiler.StartTSC;

    if (timerFreq)
    {
        std::printf("\nTotal time: %0.4fms (timer freq %llu)\n", 1000.0 * static_cast<double>(totalTSCElapsed) / static_cast<double>(timerFreq), timerFreq);
    }

    PrintAnchorData(totalTSCElapsed, timerFreq);
}
