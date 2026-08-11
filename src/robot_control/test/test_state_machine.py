"""
MotionStateMachine Unit Tests
=============================
Pure transition logic: accept/reject, jog deadman, stop. Time is injected.
"""

from robot_control.state_machine import MotionStateMachine


class TestMoveTransitions:
    def test_initial_state_is_idle(self):
        sm = MotionStateMachine()
        assert sm.state == "idle"

    def test_accepts_move_from_idle(self):
        sm = MotionStateMachine()
        ok, msg = sm.can_accept_move()
        assert ok and msg == ""
        sm.enter_moving()
        assert sm.state == "moving"

    def test_rejects_move_while_moving(self):
        sm = MotionStateMachine()
        sm.enter_moving()
        ok, msg = sm.can_accept_move()
        assert not ok
        assert msg == "busy: moving"

    def test_finish_move_returns_to_idle(self):
        sm = MotionStateMachine()
        sm.enter_moving()
        sm.finish_move()
        assert sm.state == "idle"

    def test_stop_from_moving_returns_to_idle(self):
        sm = MotionStateMachine()
        sm.enter_moving()
        sm.stop()
        assert sm.state == "idle"


class TestJog:
    def test_jog_enters_from_idle(self):
        sm = MotionStateMachine()
        assert sm.on_jog(now=10.0) is True
        assert sm.state == "jog"

    def test_jog_rejected_while_moving(self):
        sm = MotionStateMachine()
        sm.enter_moving()
        assert sm.on_jog(now=10.0) is False
        assert sm.state == "moving"

    def test_move_rejected_while_jogging(self):
        sm = MotionStateMachine()
        sm.on_jog(now=10.0)
        ok, msg = sm.can_accept_move()
        assert not ok
        assert msg == "busy: jog"

    def test_deadman_timeout_returns_to_idle(self):
        sm = MotionStateMachine(jog_deadman_timeout=0.3)
        sm.on_jog(now=10.0)
        sm.tick(now=10.2)
        assert sm.state == "jog"
        sm.tick(now=10.4)
        assert sm.state == "idle"

    def test_jog_refresh_extends_deadline(self):
        sm = MotionStateMachine(jog_deadman_timeout=0.3)
        sm.on_jog(now=10.0)
        sm.on_jog(now=10.25)
        sm.tick(now=10.4)
        assert sm.state == "jog"

    def test_stop_from_jog_returns_to_idle(self):
        sm = MotionStateMachine()
        sm.on_jog(now=10.0)
        sm.stop()
        assert sm.state == "idle"
