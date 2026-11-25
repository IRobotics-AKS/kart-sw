"""Freshness/session rules for simulation-only joint efforts; no ROS imports."""
import math
import time
import uuid


class CommandGate:
    def __init__(self, simulation, command_timeout_s, wall_timeout_s, *, monotonic=time.monotonic):
        for value in (command_timeout_s, wall_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("command and wall timeouts must be finite and positive")
        self.sim = simulation
        self.max_validity_ns = round(command_timeout_s * 1e9)
        if self.max_validity_ns < 1:
            raise ValueError("command timeout must be at least one nanosecond")
        self.wall_timeout_s = wall_timeout_s
        self.monotonic = monotonic
        self.invalidate()

    def invalidate(self):
        self.sim.clear_efforts()
        self.session = str(uuid.uuid4())
        self.last_sequence = -1
        self._expires_ns = None
        self._wall_deadline = None
        self._names = ()
        self._efforts = ()

    def accept(self, *, session, sequence, stamp_ns, validity_ns, names, efforts, paused):
        """Return an error string on rejection, None on atomic acceptance."""
        now_ns = round(self.sim.snapshot.time_s * 1e9)
        if paused:
            return "simulation is paused"
        if session != self.session:
            return "wrong simulation session"
        if not isinstance(sequence, int) or isinstance(sequence, bool) or not 0 <= sequence < 2**64:
            return "invalid sequence"
        if sequence <= self.last_sequence:
            return "duplicate or out-of-order sequence"
        if not isinstance(stamp_ns, int) or not isinstance(validity_ns, int):
            return "timestamps must be integer nanoseconds"
        if stamp_ns < 0 or stamp_ns > now_ns:
            return "negative or future command timestamp"
        if not 0 < validity_ns <= self.max_validity_ns or now_ns >= stamp_ns + validity_ns:
            return "invalid validity duration or expired command"
        try:
            self.sim.set_efforts(names, efforts)
        except ValueError as error:
            return str(error)
        self.last_sequence = sequence
        self._names, self._efforts = tuple(names), tuple(efforts)
        self._expires_ns = stamp_ns + validity_ns
        self._wall_deadline = self.monotonic() + self.wall_timeout_s
        return None

    def expire(self):
        """Zero effort before the next physics step if either deadline elapsed."""
        if self._expires_ns is None:
            self.sim.clear_efforts()
            return
        if (round(self.sim.snapshot.time_s * 1e9) >= self._expires_ns
                or self.monotonic() >= self._wall_deadline):
            self.sim.clear_efforts()
            self._expires_ns = None
            self._wall_deadline = None
        else:
            # The passive viewer can change data.ctrl during sync. Only the
            # validated command is allowed to reach the next physics step.
            self.sim.set_efforts(self._names, self._efforts)
