#!/usr/bin/env python3.8

import asyncio
import traceback
from functools import cached_property
from ..logging import LoggingMixin


class AsyncBase(LoggingMixin):

    name = None

    def logging_name(self):
        return self.name

    async def cleanup(self):
        self.logger.debug("cleanup()")

    async def run_once(self):
        res = 0
        self.logger.debug("run_once()")
        return res

    async def run_loop(self):
        res = 0
        try:
            self.logger.info("Starting run loop")
            while True:
                await self.run_once()
        except asyncio.CancelledError:
            self.logger.info("Run loop canceled")
        except Exception:
            # Exceptions are sometimes silenced in the IO loop; print it out
            e_str = traceback.format_exc()
            self.logger.error(f"Unhandled exception\n{e_str}")
            raise
        finally:
            self.logger.info("Cleaning up before run loop exit")
            await self.cleanup()
            self.logger.info(f"Exiting run loop, res={res}")
            return res

    def cancel(self):
        if not self.task.cancelled():
            self.logger.info("Canceling task upon request")
            self.task.cancel()
        else:
            self.logger.info("Cancel task request but task already canceled")

    def create_task(self):
        self.logger.info("Creating task")
        loop = asyncio.get_event_loop()
        self.task = loop.create_task(self.run_loop(), name=self.name)
        return self.task

    def run(self, debug=False):
        """Program main."""
        loop = asyncio.get_event_loop()
        loop.set_debug(debug)
        res = 500
        try:
            if getattr(self, "task", None) is None:
                self.create_task()
            res = loop.run_until_complete(self.task)
            self.logger.debug(f"First run_until_complete() complete")
            exc = self.task.exception()
            if exc is not None:
                raise exc
            self.logger.info(f"Run complete (res={res})")
        except KeyboardInterrupt:
            self.logger.info("Caught keyboard interrupt, canceling tasks")
            try:
                self.cancel()
                # Task will get asyncio.CancelledError once it continues
                res = loop.run_until_complete(self.task)
                self.logger.debug(f"Second run_until_complete() complete")
            except Exception:
                e_str = traceback.format_exc()
                self.logger.info(f"Exception after cancel\n{e_str}")
                raise
        except Exception as e:
            e_str = traceback.format_exc()
            self.logger.info(f"Unhandled exception in task:\n{e_str}")
            raise
        finally:
            self.logger.debug(f"Task cancelled() = {self.task.cancelled()}")
            self.logger.debug(f"Task done() = {self.task.done()}")
            # self.logger.debug(f"Task result() = {self.task.result()}")
            self.logger.debug(f"Task exiting, status={res}")
            return res
