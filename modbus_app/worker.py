"""Background serial worker thread."""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable

import serial


class SerialWorker:
    def __init__(self) -> None:
        self.tasks: "queue.Queue[tuple]" = queue.Queue()
        self.results: "queue.Queue[tuple]" = queue.Queue()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.ser: serial.Serial | None = None
        self._running = True
        self.thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                func, args, kwargs, cb = self.tasks.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                res = func(self, *args, **kwargs)
                self.results.put((cb, True, res))
            except Exception as exc:
                self.results.put((cb, False, exc))

    def submit(self, func: Callable, *args, callback=None, **kwargs) -> None:
        self.tasks.put((func, args, kwargs, callback))

    def stop(self) -> None:
        self._running = False
        self.thread.join()
