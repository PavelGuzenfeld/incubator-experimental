"""
This module contains the PerformanceMeasure class that measures the performance of the producer and consumer functions.

The PerformanceMeasure class has the following methods:
    __init__: Initialize the PerformanceMeasure object.
    measure_produce: Measure the time taken to produce each item.
    measure_consume: Measure the time taken to consume each item.
    print_averages: Print the average producing time, consuming time, round trip time, total round trip time, producer throughput, consumer throughput, and round trip throughput.
    
    
The PerformanceMeasure class is used in the example.py script to measure the performance of the producer and consumer functions in the ProducerConsumer pipeline.
"""

import time
from typing import Callable, Any, Iterable, List, Tuple

class PerformanceMeasure:
    """
    The PerformanceMeasure class measures the performance of the producer and consumer functions.

    Attributes:
        produce_times (List[float]): The list of producing times for each item.
        consume_times (List[float]): The list of consuming times for each item.
        round_trip_times (List[float]): The list of round trip times for each item.
        produce_count (int): The total number of items produced.
        consume_count (int): The total number of items consumed.
        produce_start_time (float): The start time of producing items.
        produce_end_time (float): The end time of producing items.
        consume_start_time (float): The start time of consuming items.
        consume_end_time (float): The end time of consuming items.
    """

    def __init__(self) -> None:
        """
        Initialize the PerformanceMeasure object.
        """
        self.produce_times: List[float] = []
        self.consume_times: List[float] = []
        self.round_trip_times: List[float] = []
        self.produce_count: int = 0
        self.consume_count: int = 0
        self.produce_start_time: float = 0
        self.produce_end_time: float = 0
        self.consume_start_time: float = 0
        self.consume_end_time: float = 0

    def measure_produce(self, func: Callable[[], Iterable]) -> Callable[[], Iterable]:
        """
        Measure the time taken to produce each item.

        Args:
            func (Callable[[], Iterable]): The producer function.

        Returns:
            Callable[[], Iterable]: The wrapped producer function.

        """

        def wrapper() -> Any:
            """
            Wrapper function to measure the time taken to produce each item.

            Returns:
                Any: The item produced.
            """
            self.produce_start_time = time.time()
            for item in func():
                item_start_time = time.time()
                yield item, item_start_time
                item_end_time = time.time()
                produce_time = item_end_time - item_start_time
                self.produce_times.append(produce_time)
                self.produce_count += 1
            self.produce_end_time = time.time()

        return wrapper

    def measure_consume(
        self, func: Callable[[Any], None]
    ) -> Callable[[Tuple[Any, float]], None]:
        """
        Measure the time taken to consume each item.

        Args:
            func (Callable[[Any], None]): The consumer function.

        Returns:
            Callable[[Tuple[Any, float]], None]: The wrapped consumer function.
        """

        def wrapper(item_tuple: Tuple[Any, float]) -> None:
            item, item_start_time = item_tuple
            if item is None:
                return
            if self.consume_start_time == 0:
                self.consume_start_time = time.time()
            consume_start_time = time.time()
            func(item)
            consume_end_time = time.time()
            consume_time = consume_end_time - consume_start_time
            round_trip_time = consume_end_time - item_start_time
            self.consume_times.append(consume_time)
            self.round_trip_times.append(round_trip_time)
            self.consume_count += 1
            self.consume_end_time = consume_end_time

        return wrapper

    def print_averages(self) -> None:
        """
        Print the average producing time, consuming time, round trip time, total round trip time,
        producer throughput, consumer throughput, and round trip throughput.
        """
        avg_produce_time = (
            sum(self.produce_times) / len(self.produce_times)
            if self.produce_times
            else 0
        )
        avg_consume_time = (
            sum(self.consume_times) / len(self.consume_times)
            if self.consume_times
            else 0
        )
        avg_round_trip_time = (
            sum(self.round_trip_times) / len(self.round_trip_times)
            if self.round_trip_times
            else 0
        )

        total_produce_time = (
            self.produce_end_time - self.produce_start_time
            if self.produce_end_time > self.produce_start_time
            else 0
        )
        total_consume_time = (
            self.consume_end_time - self.consume_start_time
            if self.consume_end_time > self.consume_start_time
            else 0
        )

        round_trip_time = (
            self.consume_end_time - self.produce_start_time
            if self.consume_end_time > self.produce_start_time
            else 0
        )

        produce_throughput = (
            self.produce_count / total_produce_time if total_produce_time > 0 else 0
        )
        consume_throughput = (
            self.consume_count / total_consume_time if total_consume_time > 0 else 0
        )
        round_trip_throughput = (
            self.consume_count / round_trip_time if round_trip_time > 0 else 0
        )

        print(f"Total number of items: {self.produce_count}")
        print(f"Average producing time: {avg_produce_time:.6f} sec")
        print(f"Average consuming time: {avg_consume_time:.6f} sec")
        print(f"Average round trip time: {avg_round_trip_time:.6f} sec")
        print(f"Total round trip time: {round_trip_time:.6f} sec")
        print(f"Producer throughput: {produce_throughput:.2f} items/sec")
        print(f"Consumer throughput: {consume_throughput:.2f} items/sec")
        print(f"Round trip throughput: {round_trip_throughput:.2f} items/sec")
