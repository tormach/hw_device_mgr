import logging
import asyncio
import time
import sys
from functools import cached_property

from hw_device_mgr.mgr.async_task import AsyncBase

class AsyncParams(AsyncBase):

    name = "param loop"

    def __init__(self):
        self.queue = dict()

    def queue_download(self, dev, sdo, val):
        self.queue.setdefault(dev, dict())[sdo] = val

    def is_done(self, dev):
        return len(self.queue.get(dev, dict())) == 0

    async def do_download(self):
        if not self.queue:
            # No device queues; nothing to do
            await asyncio.sleep(0.1)  # Avoid a tight loop
            return
        for dev, sdos in self.queue.items():
            break  # Grab the first device queue
        for sdo, val in sdos.items():
            sdos.pop(sdo)  # Remove item from device queue
            if not sdos:  # Device queue is empty...
                self.queue.pop(dev)  # ...remove the queue
            break  # Grab the first SDO and value

        self.logger.info(f"Device {dev}:  Setting {sdo} to {val}")

        # cmd = ["echo", "ethercat", "download", "-p", str(dev), sdo, str(val)]
        cmd = f"echo ethercat download -p {str(dev)} {sdo} {str(val)}"
        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )

        stdout, stderr = await proc.communicate()

        self.logger.info(f'[{cmd!r} exited with {proc.returncode}]')

        if stdout:
            self.logger.info(f'[stdout] {stdout.decode()}')
        if stderr:
            self.logger.info(f'[stderr] {stderr.decode()}')

        await asyncio.sleep(0.05)  # Fake writing SDO
        self.logger.info(f"Device {dev}:  Done setting {sdo} to {val}")

    async def run_once(self):
        res = 0
        await self.do_download()
        return res

class AsyncRunner(AsyncBase):

    name = "Main loop"

    commands = (
        # (dev, sdo, val)
        (1, "2002-01h", 2),
        (2, "2002-01h", 2),
        (3, "2002-01h", 4),
        (1, "2002-02h", 7),
        (1, "2002-03h", 42),
        (2, "2002-02h", 7),
        (2, "2002-03h", 42),
        (3, "2002-02h", 8),
        (3, "2002-03h", 44),
        (1, "2002-09h", 10),
        (2, "2002-09h", 11),
        (1, "2002-0Ah", -1),
        (2, "2002-0Ah", -1),
        (1, "2002-09h", 20),
        (2, "2002-09h", 21),
    )

    @cached_property
    def params(self):
        # Async params downloader
        if self.task.cancelled():
            # Don't start params task when main loop is canceled
            return None
        self._params = AsyncParams()
        self._params.create_task()
        return self._params

    def enqueue(self, arg=None):
        self.logger.info(f"Queuing commands")
        for dev, sdo, val in self.commands:
            self.params.queue_download(dev, sdo, val)

    async def cleanup(self):
        self.logger.info(" Cleaning up before exit")
        if self.params is not None:
            self.logger.info(" Stopping & waiting on params task")
            self.params.cancel()
            await self.params.task
        self.logger.info(" Cleaning up complete")

    @cached_property
    def devices(self):
        res = set()
        for dev, sdo, val in self.commands:
            res.add(dev)
        return res

    async def run_once(self):
        i = self.counter = getattr(self, "counter", -1) + 1

        self.logger.info(f"run_once {i}")
        await asyncio.sleep(0.1)
        if i == 5:
            self.enqueue()
        elif i > 5:
            for dev in list(self.devices):
                if self.params.is_done(dev):
                    self.logger.info(f"run_once:  Device {dev} finished updating")
                    self.devices.remove(dev)
        if i > 20 and not self.devices:
            self.logger.info("run_once:  All devices updated; canceling")
            self.cancel()


if __name__ == "__main__":
    debug = True
    logging.basicConfig(
        format="%(asctime)s [%(levelname)s]%(name)s: %(message)s",
        level=logging.DEBUG if debug else logging.INFO
    )
    ar = AsyncRunner()
    sys.exit(ar.run(debug=debug))
