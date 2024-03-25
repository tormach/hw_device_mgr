import threading
import queue
from functools import cached_property, lru_cache


class AsyncTaskQueue:
    """
    Generic class to process commands in an asynchronous queue.

    Multiple instances with separate queues `enqueue` commands.  A single worker
    thread rotates among queues, pulling off the next command and executing it.
    """

    # Idle wait time
    idle_wait = 0.1 # s

    # Class-level singleton queue data
    _queue_names = list()
    _cmd_queues = dict()
    _errors = dict()
    _queue_stop = dict()

    _queues = list()

    #
    # Class methods:  worker thread is managed by the class itself
    #

    @classmethod
    @lru_cache
    def lock(cls):
        return threading.Lock()

    @classmethod
    @lru_cache  # Only needs to run once per class
    def start(cls):
        """
        Create new worker instance and start worker thread.

        This may be called from multiple instances enqueuing commands,
        but will only ever create a single worker instance running a
        single thread.
        """
        assert not hasattr(cls, "_thread")  # Sanity
        cls._thread = threading.Thread(target=cls._work, daemon=True)
        cls._stop = threading.Event()
        cls._have_data = threading.Event()
        cls._have_error = threading.Event()
        cls._thread.start()

    @classmethod
    def _work(cls):
        """
        Worker thread loop callback.

        Pop commands off command queue and executes method with args.
        Repeat untill `join` method is called.
        """
        while not cls._stop.is_set():
            cls._have_data.wait()
            for cmd_queue in cls._queues:
                cmd_queue.churn()
            with cls.lock():  # Synchronize queue check & have_data clear
                if all(i.empty for i in cls._queues):
                    cls._have_data.clear()  # All cmd queues empty

    @classmethod
    def join(cls):
        """Block until queue processed and join worker thread."""
        assert hasattr(cls, "_thread")
        # Unblock worker thread & signal to stop
        with cls.lock():  # Synchronize stop & have_data set
            cls._stop.set()
            cls._have_data.set()
            for stop in cls._queue_stop.values():
                stop.set()
        # Join queue
        for cmd_queue in cls._queues:
            cmd_queue.join_queue()
        # Join worker thread
        cls._thread.join()

    #
    # Instance methods:  Named instances
    #

    def __init__(self, name):
        self.name = name
        self.queue = queue.Queue()
        self.error = None
        self.stop = threading.Event()
        self._queues.append(self)

    def churn(self):
        """Pop command off queue and execute it."""
        if self.queue.empty():
            return
        if self.stop.is_set():
            return
        method, args, kwargs = self.queue.get()
        try:
            method(*args, **kwargs)
        except Exception as e:
            # Put exception in errors & stop queue
            with self.lock():
                self.stop.set()
                self.error = (e, method, args, kwargs)
                self._have_error.set()
        self.queue.task_done()

    @property
    def empty(self):
        return self.queue.empty()

    def join_queue(self):
        """Flush and stop queue"""
        self.flush_queue_and_clear_error()
        self.queue.join()

    def enqueue(self, method, *args, **kwargs):
        """Enqueue one command."""
        self.start()  # Automatically start class thread worker
        with self.lock():  # Synchronize queue put and have_data set
            assert not self.stop.is_set()
            self.queue.put((method, args, kwargs))
            self._have_data.set()

    @property
    def started(self):
        return hasattr(self, "_thread")

    def flush_queue_and_clear_error(self):
        with self.lock():
            self.stop.set()  # No queue processing while flushing
            while not self.queue.empty():
                self.queue.get()
                self.queue.task_done()
            self.error = None
            if self.started and not any(i.error for i in self._queues):
                self._have_error.clear()
            self.stop.clear()
