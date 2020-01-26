import rospy
from ds4_driver.msg import Trackpad


class TrackpadHandler(object):
    STATE_INACTIVE = 0
    STATE_TAPPED = 1
    STATE_ONE_FINGER_DRAG = 2
    STATE_TWO_FINGER_DRAG = 3

    def __init__(self, tap_timeout=0.3):
        self._state = TrackpadHandler.STATE_INACTIVE
        self._prev_coord = None
        self._prev_touch0_active = False
        self._tap_timeout = rospy.Duration.from_sec(tap_timeout)
        self._tap_timer = None

    def process(self, touch0, touch1):
        """
        :type touch0: Trackpad
        :type touch1: Trackpad
        :return: state and the offset from the coordinate where dragging began
        """
        offset = (0, 0)

        # Trackpad not touched
        if self._state is TrackpadHandler.STATE_INACTIVE:
            next_state = self._state_inactive_(touch0, touch1)
            self._prev_coord = None
        # Tapped once
        elif self._state is TrackpadHandler.STATE_TAPPED:
            next_state = self._state_tapped_(touch0, touch1)
        # During one-finger drag
        elif self._state is TrackpadHandler.STATE_ONE_FINGER_DRAG:
            if self._tap_timer.is_alive():
                self._tap_timer.shutdown()

            next_state = self._state_one_finger_drag_(touch0, touch1)

            # Just started one-finger drag
            if self._prev_coord is None:
                self._prev_coord = (touch0.x, touch0.y)

            offset = (
                touch0.x - self._prev_coord[0],
                touch0.y - self._prev_coord[1],
            )
            self._prev_coord = (touch0.x, touch0.y)
        # During two-finger drag
        elif self._state is TrackpadHandler.STATE_TWO_FINGER_DRAG:
            next_state = self._state_two_finger_drag_(touch0, touch1)

            # Just started two-finger drag
            if self._prev_coord is None:
                self._prev_coord = (touch0.x, touch0.y)

            self._state = TrackpadHandler.STATE_TWO_FINGER_DRAG
            offset = (
                touch0.x - self._prev_coord[0],
                touch0.y - self._prev_coord[1],
            )
            self._prev_coord = (touch0.x, touch0.y)
        else:
            raise ValueError('Invalid state: {}'.format(self._state))

        self._state = next_state
        self._prev_touch0_active = touch0.active
        return self._state, offset

    def _state_inactive_(self, touch0, touch1):
        if touch0.active and touch1.active:
            next_state = TrackpadHandler.STATE_TWO_FINGER_DRAG
        elif not self._prev_touch0_active and touch0.active:
            next_state = TrackpadHandler.STATE_TAPPED
            def reset_state(_): self._state = TrackpadHandler.STATE_INACTIVE
            self._tap_timer = rospy.Timer(self._tap_timeout, reset_state, oneshot=True)
        else:
            next_state = TrackpadHandler.STATE_INACTIVE
        return next_state

    def _state_tapped_(self, touch0, touch1):
        if not self._prev_touch0_active and touch0.active:
            next_state = TrackpadHandler.STATE_ONE_FINGER_DRAG
        elif touch0.active and touch1.active:
            next_state = TrackpadHandler.STATE_TWO_FINGER_DRAG
        else:
            next_state = TrackpadHandler.STATE_TAPPED
        return next_state

    @staticmethod
    def _state_one_finger_drag_(touch0, touch1):
        if not touch0.active and not touch1.active:
            next_state = TrackpadHandler.STATE_INACTIVE
        elif touch0.active and touch1.active:
            next_state = TrackpadHandler.STATE_TWO_FINGER_DRAG
        else:
            next_state = TrackpadHandler.STATE_ONE_FINGER_DRAG
        return next_state

    @staticmethod
    def _state_two_finger_drag_(touch0, touch1):
        if not touch0.active or not touch1.active:
            next_state = TrackpadHandler.STATE_INACTIVE
        else:
            next_state = TrackpadHandler.STATE_TWO_FINGER_DRAG
        return next_state
