"""frame_bus.py — localhost message bus + in-process drop-old ring for Cartographer.

Two transports live here:

1. In-process `DropOldRing` — a thread-safe single-slot buffer. The NDI capture
   thread writes every frame; the display loop reads the newest. Old frames are
   silently overwritten (that is the intended "drop-old" semantics for a live UI,
   NOT a hidden failure path).

2. Cross-process ZeroMQ PUB/SUB — `FramePublisher`/`FrameSubscriber` carry
   downscaled frames from io_bridge (P1) to the perception worker (P2) on
   `frame_bus_port`; `StatePublisher`/`StateSubscriber` carry poses / depth /
   detections back on `state_bus_port`.

   Frames use ZMQ_CONFLATE so a subscriber always receives the *latest* frame and
   never a stale backlog — exactly what dense SLAM / depth want. CONFLATE forbids
   multipart messages, so each frame is a single length-prefixed blob:
       [ 4-byte big-endian header length ][ header JSON utf-8 ][ raw frame bytes ]

NO SILENT FALLBACKS: bind/connect failures raise. Callers fail-fast.
"""

import json
import struct
import threading
import time

import numpy as np
import zmq

# Topic strings for the (non-conflated) state bus.
TOPIC_POSE = b"pose"
TOPIC_DEPTH = b"depth"
TOPIC_MAP = b"map"          # perception_worker -> visualizer: compact top-down occupancy summary
TOPIC_DETECTION = b"detection"   # object_worker -> perception/UI: target bbox/center in a frame
TOPIC_TARGET = b"target"    # perception_worker -> UI/report: lifted 3D target position + uncertainty
TOPIC_STATUS = b"status"


def _addr(port: int) -> str:
    return f"tcp://127.0.0.1:{port}"


# ---------------------------------------------------------------------------
# In-process drop-old ring
# ---------------------------------------------------------------------------
class DropOldRing:
    """Thread-safe single-slot latest-value buffer.

    The writer never blocks; a new frame overwrites the unread previous one.
    Readers get the newest frame available (or None if nothing seen yet).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._item = None  # (frame, meta) tuple

    def put(self, frame, meta):
        with self._lock:
            self._item = (frame, meta)

    def get(self):
        """Return the newest (frame, meta), or None. Does not block."""
        with self._lock:
            return self._item


# ---------------------------------------------------------------------------
# Frame serialization (single-part, CONFLATE-compatible)
# ---------------------------------------------------------------------------
def encode_frame(frame: np.ndarray, meta: dict) -> bytes:
    """Pack a contiguous numpy frame + metadata into one blob.

    `meta` is augmented with the array's dtype/shape so the receiver can rebuild
    it with zero ambiguity.
    """
    if not frame.flags["C_CONTIGUOUS"]:
        frame = np.ascontiguousarray(frame)
    header = dict(meta)
    header["dtype"] = str(frame.dtype)
    header["shape"] = list(frame.shape)
    header_bytes = json.dumps(header).encode("utf-8")
    return struct.pack(">I", len(header_bytes)) + header_bytes + frame.tobytes()


def decode_frame(blob: bytes):
    """Inverse of `encode_frame`. Returns (frame, meta)."""
    (header_len,) = struct.unpack(">I", blob[:4])
    header = json.loads(blob[4 : 4 + header_len].decode("utf-8"))
    raw = blob[4 + header_len :]
    frame = np.frombuffer(raw, dtype=np.dtype(header["dtype"])).reshape(header["shape"])
    return frame, header


# ---------------------------------------------------------------------------
# Frame bus (conflated: always newest)
# ---------------------------------------------------------------------------
class FramePublisher:
    """PUB side of the downscaled-frame stream. io_bridge owns one of these."""

    def __init__(self, port: int, ctx: zmq.Context | None = None):
        self._ctx = ctx or zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        # Keep only the latest frame queued in each direction.
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        self._sock.bind(_addr(port))  # raises if the port is taken — fail-fast
        self.port = port

    def publish(self, frame: np.ndarray, meta: dict):
        self._sock.send(encode_frame(frame, meta), flags=zmq.NOBLOCK)

    def close(self):
        self._sock.close(linger=0)


class FrameSubscriber:
    """SUB side of the downscaled-frame stream. Perception worker owns one."""

    def __init__(self, port: int, ctx: zmq.Context | None = None):
        self._ctx = ctx or zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.RCVHWM, 1)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")  # CONFLATE requires empty subscription
        self._sock.connect(_addr(port))
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)
        self.port = port

    def recv(self, timeout_ms: int = 1000):
        """Return the newest (frame, meta), or None if nothing arrived in time."""
        events = dict(self._poller.poll(timeout_ms))
        if self._sock in events:
            return decode_frame(self._sock.recv())
        return None

    def close(self):
        self._sock.close(linger=0)


# ---------------------------------------------------------------------------
# State bus (NOT conflated: every message matters — poses, depth, detections)
# ---------------------------------------------------------------------------
class StatePublisher:
    """PUB side of the state bus. Workers publish JSON-serializable payloads."""

    def __init__(self, port: int, ctx: zmq.Context | None = None):
        self._ctx = ctx or zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 100)
        self._sock.bind(_addr(port))
        self.port = port

    def publish(self, topic: bytes, payload: dict):
        body = json.dumps(payload).encode("utf-8")
        self._sock.send_multipart([topic, body])

    def close(self):
        self._sock.close(linger=0)


class StateSubscriber:
    """SUB side of the state bus. `topics=None` subscribes to everything."""

    def __init__(self, port: int, topics=None, ctx: zmq.Context | None = None):
        self._ctx = ctx or zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.RCVHWM, 100)
        if topics is None:
            self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        else:
            for t in topics:
                self._sock.setsockopt(zmq.SUBSCRIBE, t)
        self._sock.connect(_addr(port))
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)
        self.port = port

    def recv(self, timeout_ms: int = 1000):
        """Return (topic_str, payload_dict) or None on timeout."""
        events = dict(self._poller.poll(timeout_ms))
        if self._sock in events:
            topic, body = self._sock.recv_multipart()
            return topic.decode("utf-8"), json.loads(body.decode("utf-8"))
        return None

    def close(self):
        self._sock.close(linger=0)


# ---------------------------------------------------------------------------
# Self-test: round-trip a frame through the conflated bus in-process.
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("[frame_bus] self-test: publishing 512x288 frames, subscriber reads newest")
    PORT = 5601
    pub = FramePublisher(PORT)
    sub = FrameSubscriber(PORT)
    time.sleep(0.3)  # let the SUB connection settle before the first send

    sent = 0
    for i in range(30):
        f = np.full((288, 512, 3), i % 256, dtype=np.uint8)
        pub.publish(f, {"frame_id": i, "mono_ts": time.monotonic()})
        sent += 1
        time.sleep(0.005)

    got = sub.recv(timeout_ms=500)
    assert got is not None, "subscriber received nothing"
    frame, meta = got
    print(f"[frame_bus] sent {sent} frames; subscriber holds frame_id={meta['frame_id']} "
          f"shape={frame.shape} dtype={frame.dtype} (newest-wins confirmed)")
    pub.close()
    sub.close()
    print("[frame_bus] OK")
