"""Record the body protocol between a robot (or simulator) and a body driver.

The proxy listens where the robot expects the driver, forwards every message to
the driver unchanged and every reply back, and appends both directions to one
record file.  It never decodes or alters a message, so a recording is the exact
conversation and does not depend on which driver produced the replies.

Record file layout (integers little-endian):
    header   b"BDWIRE1\\n"
    record   kind:u8     b'R' robot->driver binary, b'D' driver->robot binary,
                         b'r' / b'd' the same directions for text frames,
                         b'C' a new robot connection (empty payload)
             t_ns:u64    monotonic nanoseconds since the proxy started
             length:u32
             payload     the WebSocket message bytes

Usage: python wire_proxy.py LISTEN_PORT DRIVER_URL OUT_FILE
"""

import asyncio
import signal
import struct
import sys
import time

import websockets

HEADER = b"BDWIRE1\n"


class Recorder:
    def __init__(self, path):
        self.t0 = time.monotonic_ns()
        self.out = open(path, "wb")
        self.out.write(HEADER)

    def write(self, kind, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
            kind = kind.lower()
        self.out.write(struct.pack("<BQI", ord(kind), time.monotonic_ns() - self.t0, len(data)))
        self.out.write(data)

    def close(self):
        self.out.flush()
        self.out.close()


async def main(listen_port, driver_url, out_path):
    rec = Recorder(out_path)
    stop = asyncio.get_running_loop().create_future()
    for sig in (signal.SIGTERM, signal.SIGINT):
        asyncio.get_running_loop().add_signal_handler(sig, lambda: stop.done() or stop.set_result(None))

    async def handler(robot):
        rec.write("C", b"")
        async with websockets.connect(driver_url, max_size=None, ping_interval=None, close_timeout=1) as driver:
            async def pump(src, dst, kind):
                async for message in src:
                    rec.write(kind, message)
                    await dst.send(message)

            tasks = [asyncio.create_task(pump(robot, driver, "R")),
                     asyncio.create_task(pump(driver, robot, "D"))]
            # Either side closing ends this connection; the driver keeps listening for the next one.
            _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()

    async with websockets.serve(handler, "127.0.0.1", listen_port, max_size=None, ping_interval=None):
        await stop
    rec.close()


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    asyncio.run(main(int(sys.argv[1]), sys.argv[2], sys.argv[3]))
