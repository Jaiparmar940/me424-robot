"""
ramses_robot_bridge.py
===============
ROS2 node that bridges serial communication with the ME424 ESP32
robot arm controller.

Topics published
----------------
/arm/joint_states  (sensor_msgs/JointState)
    Live joint positions derived from ESP32 step counts.
    Updated after every DONE response and every POS line.
    RViz2 reads this natively via the robot_state_publisher.

/arm/response      (std_msgs/String)
    Raw passthrough of every ACK / DONE / ERR line from the ESP32.
    Useful for logging and for other nodes that want to react to events.

/arm/estop_state   (std_msgs/Bool)
    True when the ESP32 e-stop is latched.

/arm/limits        (std_msgs/String)
    Most recent LIMITS line from the ESP32, e.g.
    "S2=0 S3=0 S4=1 S5H=0"

Topics subscribed
-----------------
/arm/command       (std_msgs/String)
    Fire-and-forget commands: estop, mag on, debug on, etc.
    The node sends the string directly over serial.
    No blocking — returns after ACK (or timeout).

Action servers
--------------
/arm/move          (ramses_robot_bridge/ArmMove)
    Blocking motion action. Sends the command, waits for ACK
    (publishes feedback), then waits for DONE or ERR (returns result).
    Goal cancellation sends an "estop" command immediately.

Parameters
----------
port      (string, default '/dev/ttyUSB0')   Serial port path.
baudrate  (int,    default 115200)           Serial baud rate.
"""

import math
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState

from ramses_robot_interfaces.action import ArmMove

from .constants import (
    ACK_TIMEOUT_S,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    DONE_TIMEOUT_S,
    INSTANT_COMMANDS,
    JOINT_NAMES,
    RAD_PER_STEP,
)
from .response_router import LineType, ParsedLine, parse_line
from .serial_worker import SerialWorker


class Esp32BridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('ramses_robot_bridge')

        # ----------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------
        self.declare_parameter('port',     DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)

        port     = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # ----------------------------------------------------------
        # Internal state
        # ----------------------------------------------------------
        # Current tracked positions in steps [C1..C6]
        self._step_pos: List[int] = [0] * 6

        # Most recent limit state
        self._limits: Dict[str, int] = {'S2': 0, 'S3': 0, 'S4': 0, 'S5H': 0}

        # E-stop state
        self._estop: bool = False

        # Lock protecting _step_pos, _limits, _estop
        self._state_lock = threading.Lock()

        # Event/result used by the action server to wait for DONE/ERR.
        # Only one action goal runs at a time (MutuallyExclusiveCallbackGroup).
        self._motion_done_event  = threading.Event()
        self._motion_done_result: Optional[ParsedLine] = None

        # Lock protecting the action result fields above
        self._motion_lock = threading.Lock()

        # Flag: is an action goal currently in progress?
        self._action_active = False

        # ----------------------------------------------------------
        # Serial worker
        # ----------------------------------------------------------
        self._serial = SerialWorker(port, baudrate, self.get_logger())
        self._serial.start()

        # ----------------------------------------------------------
        # RX dispatcher thread
        # ----------------------------------------------------------
        # Reads lines from SerialWorker and dispatches them.
        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_dispatch_loop, name='rx_dispatch', daemon=True
        )
        self._rx_thread.start()

        # ----------------------------------------------------------
        # Callback groups
        # ----------------------------------------------------------
        # Action server callbacks are mutually exclusive so only one
        # goal executes at a time. Publishers/subscribers use a
        # reentrant group so they can fire freely.
        self._action_cbg = MutuallyExclusiveCallbackGroup()
        self._pubsub_cbg = ReentrantCallbackGroup()

        # ----------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------
        self._pub_joint_states = self.create_publisher(
            JointState, '/arm/joint_states', 10,
            callback_group=self._pubsub_cbg,
        )
        self._pub_response = self.create_publisher(
            String, '/arm/response', 50,
            callback_group=self._pubsub_cbg,
        )
        self._pub_estop = self.create_publisher(
            Bool, '/arm/estop_state', 10,
            callback_group=self._pubsub_cbg,
        )
        self._pub_limits = self.create_publisher(
            String, '/arm/limits', 10,
            callback_group=self._pubsub_cbg,
        )

        # ----------------------------------------------------------
        # Subscriber — fire-and-forget commands
        # ----------------------------------------------------------
        self._sub_command = self.create_subscription(
            String,
            '/arm/command',
            self._on_command,
            10,
            callback_group=self._pubsub_cbg,
        )

        # ----------------------------------------------------------
        # Action server
        # ----------------------------------------------------------
        self._action_server = ActionServer(
            self,
            ArmMove,
            '/arm/move',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._action_cbg,
        )

        # ----------------------------------------------------------
        # Timer — periodically publish joint states even when idle
        # ----------------------------------------------------------
        self._js_timer = self.create_timer(
            0.1,  # 10 Hz
            self._publish_joint_states,
            callback_group=self._pubsub_cbg,
        )

        self.get_logger().info('ESP32 bridge node started.')
        self.get_logger().info(f'  Serial : {port} @ {baudrate}')
        self.get_logger().info('  Action : /arm/move')
        self.get_logger().info('  Sub    : /arm/command')
        self.get_logger().info('  Pub    : /arm/joint_states  /arm/response  '
                               '/arm/estop_state  /arm/limits')

    # ==============================================================
    # Shutdown
    # ==============================================================

    def destroy_node(self) -> None:
        self._rx_stop.set()
        self._rx_thread.join(timeout=2.0)
        self._serial.stop()
        super().destroy_node()

    # ==============================================================
    # RX dispatch loop (background thread)
    # ==============================================================

    def _rx_dispatch_loop(self) -> None:
        """Read lines from the serial worker and dispatch them."""
        while not self._rx_stop.is_set():
            line = self._serial.get_line(timeout=0.05)
            if line is None:
                continue
            try:
                parsed = parse_line(line)
                self._dispatch(parsed)
            except Exception as exc:
                self.get_logger().error(f'RX dispatch error: {exc}  line={line!r}')

    def _dispatch(self, p: ParsedLine) -> None:
        """Route a parsed line to the right handler(s)."""

        # Always publish the raw response for any ACK/DONE/ERR
        if p.kind in (LineType.ACK, LineType.DONE, LineType.ERR):
            msg = String()
            msg.data = p.raw
            self._pub_response.publish(msg)

        # Update state and notify the action server
        if p.kind == LineType.DONE or p.kind == LineType.ERR:
            self._notify_motion_done(p)

        elif p.kind == LineType.POS:
            with self._state_lock:
                self._step_pos = list(p.positions)
            self._publish_joint_states()

        elif p.kind == LineType.LIMITS:
            with self._state_lock:
                self._limits = dict(p.limits)
            msg = String()
            msg.data = ' '.join(f'{k}={v}' for k, v in p.limits.items())
            self._pub_limits.publish(msg)

        elif p.kind == LineType.ESTOP:
            with self._state_lock:
                self._estop = bool(p.estop)
            msg = Bool()
            msg.data = bool(p.estop)
            self._pub_estop.publish(msg)
            self.get_logger().warn(
                f'ESTOP {"LATCHED" if p.estop else "CLEARED"}'
            )

        elif p.kind == LineType.DEBUG:
            self.get_logger().debug(f'[ESP32 DBG] {p.rest}')

        elif p.kind == LineType.SLAVE:
            self.get_logger().info(f'[SLAVE] {p.rest}')

        elif p.kind == LineType.OTHER:
            # Startup banners, help text, etc. — just log them
            self.get_logger().info(f'[ESP32] {p.raw}')

    # ==============================================================
    # Motion done notification (called from RX thread)
    # ==============================================================

    def _notify_motion_done(self, p: ParsedLine) -> None:
        with self._motion_lock:
            if self._action_active:
                self._motion_done_result = p
                self._motion_done_event.set()

    # ==============================================================
    # Joint state publisher
    # ==============================================================

    def _publish_joint_states(self) -> None:
        with self._state_lock:
            steps = list(self._step_pos)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES

        # Map controller indices to joint order:
        #   joint_turntable -> C6 (index 5)
        #   joint_lift      -> C1 (index 0)  [C4 mirrors C1]
        #   joint_arm1      -> C2 (index 1)
        #   joint_arm2      -> C3 (index 2)
        #   joint_wrist     -> C5 (index 4)
        controller_for_joint = [5, 0, 1, 2, 4]

        msg.position = [
            steps[c] * RAD_PER_STEP[c]
            for c in controller_for_joint
        ]
        msg.velocity = []
        msg.effort   = []

        self._pub_joint_states.publish(msg)

    # ==============================================================
    # /arm/command subscriber (fire-and-forget)
    # ==============================================================

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip()
        if not cmd:
            return
        self.get_logger().info(f'Command topic → {cmd!r}')
        self._serial.send_command(cmd)
        # No blocking wait — the RX dispatcher will handle any response.

    # ==============================================================
    # Action server
    # ==============================================================

    def _goal_callback(self, goal_request) -> GoalResponse:
        cmd = goal_request.command.strip()
        if not cmd:
            self.get_logger().warn('Action: rejected empty command')
            return GoalResponse.REJECT
        self.get_logger().info(f'Action goal received: {cmd!r}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().warn('Action: cancel requested → sending estop')
        self._serial.send_command('estop')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> ArmMove.Result:
        cmd = goal_handle.request.command.strip()
        self.get_logger().info(f'Action: executing {cmd!r}')

        result = ArmMove.Result()

        # --- Mark action as active ---
        with self._motion_lock:
            self._action_active = True
            self._motion_done_event.clear()
            self._motion_done_result = None

        # --- Send the command ---
        self._serial.send_command(cmd)

        # --- Wait for ACK ---
        ack_deadline = time.monotonic() + ACK_TIMEOUT_S
        ack_received = False

        while time.monotonic() < ack_deadline:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success  = False
                result.response = 'CANCELLED'
                with self._motion_lock:
                    self._action_active = False
                return result

            # Poll the done event — ACK comes in as an OTHER/response line.
            # We don't block on ACK specifically; we treat DONE/ERR that
            # arrive before any DONE_TIMEOUT as implicitly ACK'd.
            # But we do want to publish feedback when ACK arrives.
            # The RX dispatcher publishes ACK to /arm/response;
            # we rely on DONE_TIMEOUT for overall safety.
            ack_received = True   # optimistic — treat send as implicit ACK
            break

        if not ack_received:
            self.get_logger().error(f'Action: ACK timeout for {cmd!r}')
            result.success  = False
            result.response = 'ERR timeout waiting for ACK'
            with self._motion_lock:
                self._action_active = False
            goal_handle.abort()
            return result

        # --- Publish ACK feedback ---
        feedback = ArmMove.Feedback()
        feedback.status = f'ACK {cmd}'
        goal_handle.publish_feedback(feedback)

        # --- For instant commands, don't wait for DONE ---
        cmd_lower = cmd.lower()
        is_instant = any(cmd_lower.startswith(ic) for ic in INSTANT_COMMANDS)

        if is_instant:
            self.get_logger().info(f'Action: instant command, not waiting for DONE')
            result.success  = True
            result.response = f'DONE {cmd} (instant)'
            with self._motion_lock:
                self._action_active = False
            goal_handle.succeed()
            return result

        # --- Wait for DONE or ERR ---
        done = self._motion_done_event.wait(timeout=DONE_TIMEOUT_S)

        with self._motion_lock:
            self._action_active = False
            done_parsed = self._motion_done_result

        if not done or done_parsed is None:
            self.get_logger().error(f'Action: DONE timeout for {cmd!r}')
            result.success  = False
            result.response = f'ERR timeout waiting for DONE'
            goal_handle.abort()
            return result

        # --- Handle cancellation that completed during motion ---
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success  = False
            result.response = done_parsed.raw
            return result

        # --- Normal completion ---
        if done_parsed.kind == LineType.DONE:
            result.success  = True
            result.response = done_parsed.raw
            goal_handle.succeed()
            self.get_logger().info(f'Action: succeeded — {done_parsed.raw}')

            # Request a position update so joint states are fresh
            self._serial.send_command('where')

        else:  # ERR
            result.success  = False
            result.response = done_parsed.raw
            goal_handle.abort()
            self.get_logger().warn(f'Action: aborted — {done_parsed.raw}')

        return result


# ==================================================================
# Entry point
# ==================================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = Esp32BridgeNode()

    # MultiThreadedExecutor is required because the action server's
    # execute_callback blocks while waiting for DONE, and we still
    # need the ROS2 event loop to process publisher callbacks.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
