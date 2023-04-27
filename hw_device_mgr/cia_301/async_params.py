from ..async_task_queue import AsyncTaskQueue


class AsyncParamsQueue(AsyncTaskQueue):
    """Process device configuration commands in an asynchronous queue."""

    def process_cmd(self, cmd_raw):
        """Execute one config command in worker."""
        config, cmd, args, kwargs = cmd_raw
        return getattr(config, cmd)(*args, **kwargs)

    def download(self, config, values, **cmd_kwargs):
        """Enqueue one param config 'download' command."""
        for sdo, val in values.items():
            cmd = (config, "download", (sdo, val), cmd_kwargs)
            super().enqueue(cmd)
