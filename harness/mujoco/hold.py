# hold.py PORT BEATS: simulator-side rig code, a stand-in at the driver's end of the body protocol for testing the hall
# alone (no driver): it answers robot.py's hello, reset and every beat's update_obs and get_action, asks for nothing
# (an empty action: every group holds), and reports what the driver would be given: the observation's numeric groups
# and, by the driver's own rule (driver-observations.adb Recognize: a last key found twice, the same size, is a group's
# reading and its echoed command; found once, a group that cannot be commanded), which of them it could command.
import asyncio
import sys

import msgpack
import numpy as np
import websockets

port, beats = int(sys.argv[1]), int(sys.argv[2])


def arrays(obj, path=()):
    """Every numeric array of the observation with its path (pictures, given as 2-D or 3-D arrays, apart)."""
    if isinstance(obj, dict) and obj.get("nd"):
        yield path, tuple(obj["shape"])
    elif isinstance(obj, dict):
        for k, v in obj.items():
            yield from arrays(v, path + (k,))


done = None


async def serve(ws):
    seen = 0
    async for raw in ws:
        msg = msgpack.unpackb(raw, raw=False, strict_map_key=False)
        reply = {"message_type": "ok", "payload": {}}
        if msg.get("message_type") == "call":
            p = msg["payload"]
            if p["func_name"] == "update_obs":
                seen += 1
                if seen == 1:
                    groups = [(path, shape) for path, shape in arrays(p["obs"]) if len(shape) == 1]
                    pictures = [(path, shape) for path, shape in arrays(p["obs"]) if len(shape) >= 2]
                    last = {}
                    for path, shape in groups:
                        last.setdefault((path[-1], shape), []).append(path)
                    pairs = sum(1 for v in last.values() if len(v) == 2)
                    lone = [k for k, v in last.items() if len(v) == 1]
                    print("hold: %d pictures %s; %d numeric groups: %d read and commandable (reading and echo), %d "
                          "read only, sizes of the read-only: %s" % (len(pictures), sorted(set(s for _, s in pictures)),
                          len(groups), pairs, len(lone), sorted(s[0] for _, s in lone)), flush=True)
                    print("hold: commandable sizes: %s" % sorted(k[1][0] for k, v in last.items() if len(v) == 2), flush=True)
            elif p["func_name"] == "get_action":
                reply = {"message_type": "ok", "payload": {"result": []}}
        await ws.send(msgpack.packb(reply, use_bin_type=True))
        if seen >= beats and msg.get("payload", {}).get("func_name") == "get_action":
            break   # the robot's last beat answered
    done.set()


async def main():
    global done
    done = asyncio.Event()
    async with websockets.serve(serve, "127.0.0.1", port, max_size=None):
        await done.wait()   # one robot, then gone (a stand-in left listening answered the next test's robot)

asyncio.run(main())
