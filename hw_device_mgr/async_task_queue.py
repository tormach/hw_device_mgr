import threading
import queue
from functools import cached_property, lru_cache

class AsyncTaskQueue:
    """
    Generic class to process commands in an asynchronous queue.

    Multiple instances may `enqueue` commands for a singleton worker instance
    processing those in a thread with the `process_queue` command (implemented
    in subclasses).
    """

    # Class-level dict of singleton queues
    queues = dict()

    @classmethod
    @lru_cache
    def _get_queue(cls, name):
        return cls.queues.setdefault(name, queue.Queue())

    @cached_property
    def cmd_queue(self):
        """Property returning the command queue singleton instance."""
        return self._get_queue("cmd")

    @cached_property
    def progress_queue(self):
        """Property returning the progress queue singleton instance."""
        return self._get_queue("progress")

    def __init__(self):
        self.cmd_version = 0

    #
    # Worker-related methods
    #
    # There should only ever be a single worker thread running, even if many
    # AsyncTaskQueue instances are enqueuing commands.
    #

    def process_cmd(self, cmd):
        """
        Process one command from command queue.

        Main worker function to be implemented in subclasses.
        """
        pass

    def _work(self):
        """
        Worker thread loop callback.

        Pop commands off command queue and pass to `process_cmd` method.  Repeat
        untill `join` method is called.
        """
        while True:
            cmd_version, cmd = self.cmd_queue.get()
            self.process_cmd(cmd)
            self.progress_queue.put(cmd_version)
            self.cmd_queue.task_done()

    @classmethod
    @lru_cache  # Only needs to run once per class
    def start(cls):
        """
        Create new worker instance and start worker thread.

        This may be called from multiple instances enqueuing commands, but will
        only ever create a single worker instance running a single thread.
        """
        if hasattr(cls, "_worker_instance"):
            assert type(cls._worker_instance) is cls  # Subclass sanity
            return  # Already started
        cls._worker_instance = cls()
        cls._worker_instance._start()

    @lru_cache  # Only needs to run once per worker instance
    def _start(self):
        # Start the new thread from the worker instance
        threading.Thread(target=self._work, daemon=True).start()

    #
    # Command-side-related methods
    #
    # Multiple instances may exist, but must all be running in the same
    # (probably main) thread.

    @classmethod
    def _bump_cmd_version(cls):
        # Bump the global (class-level) command version and return it
        if not hasattr(cls, "_cmd_version"):
            cls._cmd_version = 0
        cls._cmd_version += 1
        return cls._cmd_version

    def enqueue(self, cmd):
        """Enqueue one command."""
        # Automatically start thread
        self.start()
        # Set local command version to incremented global version
        ver = self.cmd_version = self._bump_cmd_version()
        self.cmd_queue.put((ver, cmd))

    @classmethod
    def _get_progress_version(cls):
        if not hasattr(cls, "_progress_version"):
            cls._progress_version = 0
        # Process the progress queue
        progress = cls._get_queue("progress")
        try:
            while True:
                cls._progress_version = progress.get(block=False)
                progress.task_done()
        except queue.Empty:
            pass
        return cls._progress_version

    @property
    def progress_version(self):
        """Return version of most recent processed command."""
        return self._get_progress_version()

    def all_cmds_complete(self):
        """
        Return `True` if all commands enqueued by this instance were processed.

        This may return `True` for one instance while returning `False` for
        another instance with commands still waiting to be processed.
        """
        return self.progress_version >= self.cmd_version

    def join(self):
        """Block until all queued items are processed and join worker thread."""
        self.cmd_queue.join()  # Wait for worker to drain cmd queue & join
        self.all_cmds_complete()  # Drain progress queue
        self.progress_queue.join()  # & join
