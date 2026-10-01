"""Play the robot side of a recording against a running driver, and check its replies.

Every robot-to-driver message of the recording is sent unchanged, one at a time,
and the driver's reply is checked against the protocol (docs/body-protocol.md 4):
the reply type matches the request, message_id and step are echoed, and every
get_action reply carries one action map whose keys are the command keys of the
recorded driver's actions with the same number of values.

Usage: python feed.py RECORDING ws://127.0.0.1:PORT [MAX_MESSAGES]
"""

import asyncio
import struct
import sys

import msgpack
import websockets

EXPECT = {"hello": "hello_ack", "prepare_case": "prepare_case_ack", "reset": "reset_result",
          "call": "call_result", "infer": "infer_result", "trial_end": "trial_end_ack",
          "heartbeat": "heartbeat_ack"}


def records(path):
    with open(path, "rb") as f:
        assert f.read(8) == b"BDWIRE1\n", "not a BDWIRE1 recording"
        while True:
            head = f.read(13)
            if len(head) < 13:
                return
            kind, _, length = struct.unpack("<BQI", head)
            yield chr(kind), f.read(length)


async def main(path, url, limit):
    recorded_actions = []
    for kind, payload in records(path):
        if kind == "D":
            m = msgpack.unpackb(payload, raw=False, strict_map_key=False)
            result = (m.get("payload") or {}).get("result")
            if result:
                recorded_actions.append({k: len(v) for k, v in result[0].items()})
        if len(recorded_actions) > 3:
            break
    reference = recorded_actions[0] if recorded_actions else {}
    problems, sent, actions = [], 0, 0
    async with websockets.connect(url, max_size=None, ping_interval=None) as ws:
        for kind, payload in records(path):
            if kind != "R":
                continue
            request = msgpack.unpackb(payload, raw=False, strict_map_key=False)
            await ws.send(payload)
            reply = msgpack.unpackb(await ws.recv(), raw=False, strict_map_key=False)
            sent += 1
            want = EXPECT.get(request.get("message_type"))
            if reply.get("message_type") != want:
                problems.append(f"{request.get('message_type')} answered with {reply.get('message_type')}")
            for key in ("message_id", "step"):
                if key in request and reply.get(key) != request.get(key):
                    problems.append(f"{key} not echoed")
            func = (request.get("payload") or {}).get("func_name")
            if func == "get_action":
                result = reply["payload"].get("result")
                if not result:
                    problems.append(f"empty action at message {sent}")
                else:
                    actions += 1
                    got = {k: len(v) for k, v in result[0].items()}
                    if got != reference:
                        problems.append(f"action keys {got} differ from the recorded driver's {reference}")
            if limit and sent >= limit:
                break
    print(f"sent {sent} messages, {actions} actions, {len(problems)} problems")
    for p in problems[:20]:
        print("  " + p)
    return 1 if problems else 0


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    sys.exit(asyncio.run(main(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 0)))
