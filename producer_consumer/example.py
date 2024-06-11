import cProfile
import pstats
from memory_profiler import profile
from time import sleep
from producer_consumer import ProducerConsumer
from performance_measure import PerformanceMeasure
from typing import Generator
import threading


@profile
def example_producer(stop_event: threading.Event) -> Generator[int, None, None]:
    """
    An example producer function that yields integers from 0 to 499 with a 0.01 second delay between each item.
    """
    for i in range(250):
        if stop_event.is_set():
            return
        yield i
        sleep(0.01)
    stop_event.set()  # Signal to stop
    for i in range(250, 500):
        if stop_event.is_set():
            return
        yield i
        sleep(0.01)


def example_consumer(item: int) -> None:
    """
    An example consumer function that waits for 0.05 seconds for each item.
    """
    sleep(0.05)


def main() -> None:
    perf_measure = PerformanceMeasure()
    pc = ProducerConsumer(
        producer_func=perf_measure.measure_produce(lambda: example_producer(pc.stop_event)),
        consumer_func=perf_measure.measure_consume(example_consumer),
        maxsize=10,
        max_workers=2,
    )

    # Start the pipeline
    pc.start()

    # Continue running other tasks
    for i in range(3):
        print(f"Main thread working... {i}")
        sleep(1)

    perf_measure.print_averages()

    print("Stopped the pipeline.")

    # Restart the pipeline
    pc.restart()
    print("Restarted the pipeline.")
    perf_measure.print_averages()


if __name__ == "__main__":
    profiler = cProfile.Profile()
    profiler.enable()

    main()

    profiler.disable()
    stats = pstats.Stats(profiler)
    stats.sort_stats(pstats.SortKey.TIME)
    stats.print_stats()
