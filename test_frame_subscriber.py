"""test_frame_subscriber.py — Milestone 2 verification stand-in for perception_worker.

Subscribes to the io_bridge frame bus (downscaled frames) and state bus (status /
detect requests) and prints what it receives. Use this to confirm, while flying
manually, that a *second process* gets live downscaled frames with no control lag.

Run order:
  1. Start Xlab.exe (Unity).
  2. venv\\Scripts\\python.exe io_bridge.py
  3. venv\\Scripts\\python.exe test_frame_subscriber.py   (this script)

Press Ctrl+C to stop. It is purely diagnostic — no GPU, no model loads.
"""

import argparse
import os
import time

import numpy as np

import frame_bus
from io_bridge import load_config

REPO = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(description="Frame/state bus verification subscriber")
    parser.add_argument("--config", default=None)
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="auto-exit after N seconds (0 = run until Ctrl+C)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    frame_port = cfg["network"]["frame_bus_port"]
    state_port = cfg["network"]["state_bus_port"]

    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_sub = frame_bus.StateSubscriber(state_port)
    print(f"[subscriber] listening: frames :{frame_port}  state :{state_port}")
    print("[subscriber] waiting for frames from io_bridge ... (Ctrl+C to quit)\n")

    n_frames = 0
    last_frame_id = None
    gaps = 0                       # count of dropped/skipped frame_ids (expected: conflate drops)
    last_report = time.monotonic()
    frames_this_window = 0
    last_latency_ms = 0.0
    start = time.monotonic()

    try:
        while True:
            # frames (conflated -> always newest)
            got = frame_sub.recv(timeout_ms=200)
            if got is not None:
                frame, meta = got
                now = time.monotonic()
                n_frames += 1
                frames_this_window += 1
                last_latency_ms = (now - meta["mono_ts"]) * 1000.0
                fid = meta["frame_id"]
                if last_frame_id is not None and fid > last_frame_id + 1:
                    gaps += fid - last_frame_id - 1
                last_frame_id = fid

            # drain any state-bus messages (non-blocking-ish)
            msg = state_sub.recv(timeout_ms=0)
            while msg is not None:
                topic, payload = msg
                if topic == "detection":
                    print(f"[subscriber] >>> DETECT REQUEST {payload}")
                msg = state_sub.recv(timeout_ms=0)

            now = time.monotonic()
            if now - last_report >= 1.0:
                fps = frames_this_window / (now - last_report)
                if got is not None:
                    c = meta["controls"]
                    print(f"[subscriber] {fps:4.1f} fps | frame {frame.shape} {frame.dtype} "
                          f"| id={last_frame_id} skipped={gaps} | latency={last_latency_ms:5.1f}ms "
                          f"| sim_t={meta['sim_time']:.1f} | trigger={c['trigger']} yaw={c['yaw']}")
                else:
                    print(f"[subscriber] {fps:4.1f} fps | no frame this window "
                          f"(is io_bridge running and Unity streaming?)")
                frames_this_window = 0
                last_report = now

            if args.seconds and now - start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n[subscriber] total frames received: {n_frames}, frame_ids skipped (conflated): {gaps}")
        frame_sub.close()
        state_sub.close()


if __name__ == "__main__":
    main()
