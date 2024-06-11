import concurrent.futures
import queue
from typing import Callable, Any, Iterable
import threading


class ProducerConsumer:
    def __init__(
        self,
        producer_func: Callable[[], Iterable],
        consumer_func: Callable[[Any], None],
        maxsize: int = 10,
        max_workers: int = 2,
    ) -> None:
        self.queue: queue.Queue[Any] = queue.Queue(maxsize=maxsize)
        self.producer_func: Callable[[], Iterable] = producer_func
        self.consumer_func: Callable[[Any], None] = consumer_func
        self.executor: concurrent.futures.ThreadPoolExecutor = (
            concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)
        )
        self.producer_future = None
        self.consumer_future = None
        self.stop_event = threading.Event()

    def start(self) -> None:
        self.stop_event.clear()
        self.producer_future = self.executor.submit(self.__produce)
        self.consumer_future = self.executor.submit(self.__consume)
        self.producer_future.result()
        self.consumer_future.result()

    def stop(self) -> None:
        self.stop_event.set()
        if self.producer_future:
            self.producer_future.result()
        if self.consumer_future:
            self.consumer_future.result()

    def restart(self) -> None:
        self.start()

    def __produce(self) -> None:
        for item in self.producer_func():
            self.queue.put(item)
            if self.stop_event.is_set():
                break
        self.queue.put(None)

    def __consume(self) -> None:
        while True:
            item = self.queue.get()
            if item is None:
                break
            self.consumer_func(item)

    # def __del__(self) -> None:
    #     self.executor.shutdown(wait=False)
    #     self.stop_event.set()
    #     self.stop()
