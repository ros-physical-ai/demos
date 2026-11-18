"""Terminal keyboard teleop for the AIC arm.

Reads its own terminal in raw mode, so only keystrokes typed into this
terminal move the arm. Publishes a velocity-mode MotionUpdate in the tool frame
at the contract rate through the same encoder the deployed policy uses.

Key layout (AIC's Cartesian teleop):
  a/d  -/+ linear X     w/s  -/+ linear Y     r/f  -/+ linear Z
  S/W  -/+ angular X    A/D  -/+ angular Y    q/e  -/+ angular Z
  k slow    l fast    Esc quit
"""

import os
import select
import sys
import termios
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from aic_control_interfaces.msg import MotionUpdate, TargetMode
from aic_control_interfaces.srv import ChangeTargetMode
from rclpy.node import Node

from pai_aic.codecs import encode_motion_update

COMMAND_TOPIC = "/aic_controller/pose_commands"
TARGET_MODE_SERVICE = "/aic_controller/change_target_mode"
ESC = "\x1b"


@dataclass(frozen=True)
class Speed:
    """Axis speeds in m/s and rad/s."""

    linear: float
    angular: float


SLOW = Speed(linear=0.02, angular=0.02)
FAST = Speed(linear=0.1, angular=0.1)

# key -> unit twist (linear x, y, z, angular x, y, z)
KEY_AXES = {
    "d": (1, 0, 0, 0, 0, 0),
    "a": (-1, 0, 0, 0, 0, 0),
    "w": (0, -1, 0, 0, 0, 0),
    "s": (0, 1, 0, 0, 0, 0),
    "r": (0, 0, -1, 0, 0, 0),
    "f": (0, 0, 1, 0, 0, 0),
    "W": (0, 0, 0, 1, 0, 0),
    "S": (0, 0, 0, -1, 0, 0),
    "A": (0, 0, 0, 0, -1, 0),
    "D": (0, 0, 0, 0, 1, 0),
    "e": (0, 0, 0, 0, 0, 1),
    "q": (0, 0, 0, 0, 0, -1),
}


class TwistCommander:
    """Turns key presses into a twist. A key drives its axis for `hold_s` after each press."""

    def __init__(self, hold_s: float = 0.25):
        """`hold_s` is how long one press keeps its axis moving, in seconds."""
        self.hold_s = hold_s
        self.speed = FAST
        self.quit_requested = False
        self._last_press: dict[str, float] = {}

    def press(self, key: str, now: float) -> None:
        """Register one key press at wall time `now` (seconds)."""
        if key == ESC:
            self.quit_requested = True
        elif key == "k":
            self.speed = SLOW
        elif key == "l":
            self.speed = FAST
        elif key in KEY_AXES:
            self._last_press[key] = now

    def twist(self, now: float) -> np.ndarray:
        """The twist to command at wall time `now`: every key pressed within the hold window contributes."""
        out = np.zeros(6)
        for key, t in list(self._last_press.items()):
            if now - t > self.hold_s:
                del self._last_press[key]
                continue
            axes = np.array(KEY_AXES[key], dtype=float)
            out[0:3] += axes[0:3] * self.speed.linear
            out[3:6] += axes[3:6] * self.speed.angular
        return out


class RawTerminal:
    """Raw, non-blocking stdin for the lifetime of the `with` block."""

    def __init__(self):
        """Wrap stdin; the terminal changes only inside the `with` block."""
        self._fd = sys.stdin.fileno()
        self._saved = None

    def __enter__(self):
        """Switch stdin to raw, non-blocking mode."""
        if not os.isatty(self._fd):
            raise RuntimeError("teleop needs a terminal on stdin")
        self._saved = termios.tcgetattr(self._fd)
        raw = termios.tcgetattr(self._fd)
        raw[3] &= ~(termios.ICANON | termios.ECHO | termios.IEXTEN | termios.ISIG)
        raw[0] &= ~(termios.IXON | termios.ICRNL)
        raw[6][termios.VMIN] = 0
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, raw)
        return self

    def __exit__(self, *_):
        """Restore the terminal."""
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSANOW, self._saved)

    def read_keys(self) -> list[str]:
        """Every key available right now. A lone escape byte is Esc; escape sequences are dropped."""
        ready, _, _ = select.select([self._fd], [], [], 0.0)
        if not ready:
            return []
        data = os.read(self._fd, 64).decode(errors="ignore")
        if data == ESC:
            return [ESC]
        if data.startswith(ESC):
            return []
        return list(data)


class TeleopNode(Node):
    """Publishes the commander's twist at a fixed rate."""

    def __init__(self, commander: TwistCommander, terminal: RawTerminal):
        """Publish `commander`'s twist, fed by keys read from `terminal`."""
        super().__init__("pai_aic_teleop")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("hold_s", commander.hold_s)
        commander.hold_s = self.get_parameter("hold_s").value
        self._commander = commander
        self._terminal = terminal
        self._pub = self.create_publisher(MotionUpdate, COMMAND_TOPIC, 10)
        self._mode_client = self.create_client(ChangeTargetMode, TARGET_MODE_SERVICE)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)

    def select_cartesian_mode(self) -> None:
        """Ask the controller for Cartesian targets; skip when the service is absent."""
        if not self._mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(f"{TARGET_MODE_SERVICE} unavailable; assuming Cartesian mode")
            return
        req = ChangeTargetMode.Request()
        req.target_mode.mode = TargetMode.MODE_CARTESIAN
        future = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().success:
            self.get_logger().warning("change_target_mode did not confirm Cartesian mode")

    def publish_twist(self, twist: np.ndarray) -> None:
        """Publish one command for `twist`, stamped with the node clock."""
        self._pub.publish(encode_motion_update(twist, None, stamp_ns=self.get_clock().now().nanoseconds))

    def _tick(self) -> None:
        now = time.monotonic()
        for key in self._terminal.read_keys():
            self._commander.press(key, now)
        if self._commander.quit_requested:
            raise KeyboardInterrupt
        self.publish_twist(self._commander.twist(now))


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    commander = TwistCommander()
    with RawTerminal() as terminal:
        node = TeleopNode(commander, terminal)
        node.select_cartesian_mode()
        node.get_logger().info("Teleop ready. a/d w/s r/f linear, S/W A/D q/e angular, k slow, l fast, Esc quit.")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.publish_twist(np.zeros(6))
            node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
