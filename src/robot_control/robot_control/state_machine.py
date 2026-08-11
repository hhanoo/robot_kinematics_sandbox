"""
Motion State Machine
====================
Pure transition logic for the motion server (no ROS imports):

    IDLE -> MOVING  (goal accepted)   MOVING -> IDLE (trajectory done / stop)
    IDLE -> JOG     (twist received)  JOG    -> IDLE (deadman timeout / stop)

Time is injected as float seconds so the logic is unit-testable.
"""


# =========================================================
# State machine
# =========================================================
class MotionStateMachine:
    """Tracks idle / moving / jog and validates transitions."""

    IDLE = "idle"
    MOVING = "moving"
    JOG = "jog"

    def __init__(self, jog_deadman_timeout=0.3):
        self.jog_deadman_timeout = jog_deadman_timeout
        self._state = self.IDLE
        self._last_jog = None

    @property
    def state(self):
        return self._state

    def can_accept_move(self):
        """(ok, reason) — a move goal may be accepted only when IDLE."""
        if self._state != self.IDLE:
            return False, f"busy: {self._state}"
        return True, ""

    def enter_moving(self):
        """IDLE -> MOVING. Call only after can_accept_move() returned True."""
        self._state = self.MOVING

    def finish_move(self):
        """MOVING -> IDLE when the trajectory is exhausted."""
        self._state = self.IDLE

    def on_jog(self, now):
        """Twist received at time `now`; True if accepted (IDLE or JOG)."""
        if self._state == self.MOVING:
            return False
        self._state = self.JOG
        self._last_jog = now
        return True

    def tick(self, now):
        """Periodic check: JOG falls back to IDLE after the deadman timeout."""
        if self._state == self.JOG and now - self._last_jog > self.jog_deadman_timeout:
            self._state = self.IDLE

    def stop(self):
        """Any state -> IDLE, holding the current position."""
        self._state = self.IDLE
