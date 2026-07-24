"""Control a PX4 position setpoint through MAVROS with a Mode 2 DS4."""

import math
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply a rescaled symmetric deadzone while preserving full range."""
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return ENU yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def body_velocity_to_enu(
    forward: float, right: float, yaw: float
) -> tuple:
    """Rotate simultaneous body-forward/right commands into ENU."""
    east = math.cos(yaw) * forward + math.sin(yaw) * right
    north = math.sin(yaw) * forward - math.cos(yaw) * right
    return east, north


class Ds4PositionControl(Node):
    """Integrate Mode 2 DS4 sticks into a MAVROS ENU pose setpoint."""

    def __init__(self) -> None:
        super().__init__('ds4_position_control')

        self._declare_parameters()
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.max_xy_speed = float(
            self.get_parameter('max_xy_speed').value)
        self.max_z_speed = float(self.get_parameter('max_z_speed').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate').value)
        self.max_horizontal_offset = float(
            self.get_parameter('max_horizontal_offset').value)
        self.max_vertical_offset = float(
            self.get_parameter('max_vertical_offset').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.prestream_duration = float(
            self.get_parameter('prestream_duration').value)
        self.body_frame_xy = bool(
            self.get_parameter('body_frame_xy').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # Mode 2 ("American hand"): left X yaw, left Y throttle/altitude,
        # right X roll/lateral, and right Y pitch/forward.
        self.axis_yaw = int(self.get_parameter('axis_yaw').value)
        self.axis_up = int(self.get_parameter('axis_up').value)
        self.axis_right = int(self.get_parameter('axis_right').value)
        self.axis_forward = int(self.get_parameter('axis_forward').value)
        self.sign_yaw = float(self.get_parameter('sign_yaw').value)
        self.sign_up = float(self.get_parameter('sign_up').value)
        self.sign_right = float(self.get_parameter('sign_right').value)
        self.sign_forward = float(
            self.get_parameter('sign_forward').value)
        self.button_activate = int(
            self.get_parameter('button_activate').value)
        self.button_land = int(self.get_parameter('button_land').value)
        self.button_deadman = int(
            self.get_parameter('button_deadman').value)
        self.button_hold_here = int(
            self.get_parameter('button_hold_here').value)

        mavros_ns = str(
            self.get_parameter('mavros_namespace').value).rstrip('/')
        if not mavros_ns.startswith('/'):
            mavros_ns = '/' + mavros_ns

        self.setpoint_pub = self.create_publisher(
            PoseStamped, f'{mavros_ns}/setpoint_position/local', 10)
        self.create_subscription(
            PoseStamped,
            f'{mavros_ns}/local_position/pose',
            self._position_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            State,
            f'{mavros_ns}/state',
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Joy, '/joy', self._joy_callback, qos_profile_sensor_data)

        self.mode_client = self.create_client(
            SetMode, f'{mavros_ns}/set_mode')
        self.arm_client = self.create_client(
            CommandBool, f'{mavros_ns}/cmd/arming')

        self.axes: List[float] = []
        self.buttons: List[int] = []
        self.previous_buttons: List[int] = []
        self.last_joy_time: Optional[float] = None
        self.current_pose: Optional[PoseStamped] = None
        self.mavros_state: Optional[State] = None
        self.last_confirmed_active = False

        self.target = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        self.anchor = [0.0, 0.0, 0.0]
        self.streaming = False
        self.activation_requested = False
        self.activation_started = 0.0
        self.commands_sent = False
        self.land_requested = False
        self.last_tick = time.monotonic()
        self.input_warning_printed = False

        self.create_timer(1.0 / self.rate_hz, self._timer_callback)
        self.get_logger().info(
            'Mode 2 ready through MAVROS. CROSS: Offboard+arm, '
            'L1: move, TRIANGLE: hold here, CIRCLE: land.')

    def _declare_parameters(self) -> None:
        defaults = {
            'rate_hz': 20.0,
            'deadzone': 0.08,
            'max_xy_speed': 2.0,
            'max_z_speed': 1.0,
            'max_yaw_rate': 1.0,
            'max_horizontal_offset': 20.0,
            'max_vertical_offset': 10.0,
            'joy_timeout': 0.5,
            'prestream_duration': 1.0,
            'body_frame_xy': True,
            'frame_id': 'map',
            'mavros_namespace': '/mavros',
            'axis_yaw': 0,
            'axis_up': 1,
            'axis_right': 3,
            'axis_forward': 4,
            'sign_yaw': 1.0,
            'sign_up': 1.0,
            'sign_right': -1.0,
            'sign_forward': 1.0,
            'button_activate': 1,
            'button_land': 2,
            'button_deadman': 4,
            'button_hold_here': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _position_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _state_callback(self, msg: State) -> None:
        self.mavros_state = msg
        active = self._is_px4_active()
        if active != self.last_confirmed_active:
            state = 'active' if active else 'inactive'
            self.get_logger().info(f'MAVROS reports Offboard control {state}.')
            self.last_confirmed_active = active

    def _joy_callback(self, msg: Joy) -> None:
        self.axes = list(msg.axes)
        self.previous_buttons = self.buttons
        self.buttons = list(msg.buttons)
        self.last_joy_time = time.monotonic()
        self.input_warning_printed = False

        if self._rising_edge(self.button_activate):
            self._request_activation()
        if self._rising_edge(self.button_land):
            self._request_land()
        if self._rising_edge(self.button_hold_here):
            self._hold_current_position()

    def _timer_callback(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.2)
        self.last_tick = now

        if not self.streaming:
            return

        if self.activation_requested and not self.commands_sent:
            if now - self.activation_started >= self.prestream_duration:
                if self._joy_is_fresh():
                    self._send_offboard_request()
                else:
                    self._cancel_activation('controller data timed out')
                    return

        if self._is_px4_active() and self._deadman_pressed():
            if self._joy_is_fresh() and self._axes_available():
                self._integrate_sticks(dt)
        elif not self._joy_is_fresh() and not self.input_warning_printed:
            self.get_logger().warning(
                'Controller data timed out; holding the last setpoint.')
            self.input_warning_printed = True

        self._publish_setpoint()

    def _request_activation(self) -> None:
        if self.current_pose is None:
            self.get_logger().warning(
                'Cannot activate: no MAVROS local position yet.')
            return
        if self.mavros_state is None or not self.mavros_state.connected:
            self.get_logger().warning(
                'Cannot activate: MAVROS is not connected to the FCU.')
            return
        if not self._axes_available():
            self.get_logger().warning(
                'Cannot activate: Joy axis mapping exceeds the axes array.')
            return

        self._copy_current_to_target()
        self.anchor = list(self.target)
        self.streaming = True
        self.activation_requested = True
        self.commands_sent = False
        self.land_requested = False
        self.activation_started = time.monotonic()
        self.get_logger().info(
            f'Pre-streaming MAVROS pose setpoints for '
            f'{self.prestream_duration:.1f} s.')

    def _cancel_activation(self, reason: str) -> None:
        self.streaming = False
        self.activation_requested = False
        self.commands_sent = False
        self.get_logger().warning(f'Activation cancelled: {reason}.')

    def _send_offboard_request(self) -> None:
        if not self.mode_client.service_is_ready():
            self._cancel_activation('MAVROS set_mode service is unavailable')
            return
        self.commands_sent = True
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'OFFBOARD'
        future = self.mode_client.call_async(request)
        future.add_done_callback(self._offboard_response)
        self.get_logger().info('MAVROS OFFBOARD mode request sent.')

    def _offboard_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: B902
            self._cancel_activation(f'set_mode call failed: {error}')
            return
        if not response.mode_sent:
            self._cancel_activation('PX4 rejected the OFFBOARD mode request')
            return
        if not self.arm_client.service_is_ready():
            self._cancel_activation('MAVROS arming service is unavailable')
            return

        request = CommandBool.Request()
        request.value = True
        arm_future = self.arm_client.call_async(request)
        arm_future.add_done_callback(self._arm_response)
        self.get_logger().info('MAVROS arm request sent.')

    def _arm_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: B902
            self._cancel_activation(f'arming call failed: {error}')
            return
        if response.success:
            self.activation_requested = False
            self.get_logger().info(
                'Arm request accepted; waiting for MAVROS state confirmation.')
        else:
            self._cancel_activation(
                f'PX4 rejected arming (result={response.result})')

    def _request_land(self) -> None:
        if self.land_requested:
            return
        if self.mavros_state is None or not self.mavros_state.connected:
            self.get_logger().warning(
                'Cannot land: MAVROS is not connected to the FCU.')
            return
        if not self.mode_client.service_is_ready():
            self.get_logger().warning(
                'Cannot land: MAVROS set_mode service is unavailable.')
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'AUTO.LAND'
        future = self.mode_client.call_async(request)
        future.add_done_callback(self._land_response)
        self.land_requested = True
        self.get_logger().warning('MAVROS AUTO.LAND mode request sent.')

    def _land_response(self, future) -> None:
        self.land_requested = False
        try:
            response = future.result()
        except Exception as error:  # noqa: B902
            self.get_logger().error(f'Landing service call failed: {error}')
            return
        if response.mode_sent:
            self.streaming = False
            self.activation_requested = False
            self.commands_sent = False
            self.get_logger().warning(
                'AUTO.LAND accepted; position setpoint streaming stopped.')
        else:
            self.get_logger().error('PX4 rejected AUTO.LAND; holding position.')

    def _hold_current_position(self) -> None:
        if self.current_pose is None:
            return
        self._copy_current_to_target()
        self.anchor = list(self.target)
        self.get_logger().info('Setpoint reset to the current MAVROS pose.')

    def _copy_current_to_target(self) -> None:
        position = self.current_pose.pose.position
        orientation = self.current_pose.pose.orientation
        self.target = [
            float(position.x),
            float(position.y),
            float(position.z),
        ]
        self.target_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def _integrate_sticks(self, dt: float) -> None:
        # All four Mode 2 channels are read from the same Joy sample so they
        # can affect the pose target simultaneously.
        yaw_input = self._axis(self.axis_yaw, self.sign_yaw)
        up = self._axis(self.axis_up, self.sign_up)
        right = self._axis(self.axis_right, self.sign_right)
        forward = self._axis(self.axis_forward, self.sign_forward)

        forward *= self.max_xy_speed
        right *= self.max_xy_speed
        if self.body_frame_xy:
            # MAVROS local_position and setpoint_position use ROS ENU.
            east, north = body_velocity_to_enu(
                forward, right, self.target_yaw)
        else:
            east = right
            north = forward

        self.target[0] += east * dt
        self.target[1] += north * dt
        self.target[2] += up * self.max_z_speed * dt
        self.target_yaw = wrap_pi(
            self.target_yaw + yaw_input * self.max_yaw_rate * dt)
        self._limit_target()

    def _limit_target(self) -> None:
        dx = self.target[0] - self.anchor[0]
        dy = self.target[1] - self.anchor[1]
        radius = math.hypot(dx, dy)
        if radius > self.max_horizontal_offset:
            scale = self.max_horizontal_offset / radius
            self.target[0] = self.anchor[0] + dx * scale
            self.target[1] = self.anchor[1] + dy * scale
        self.target[2] = max(
            self.anchor[2] - self.max_vertical_offset,
            min(
                self.anchor[2] + self.max_vertical_offset,
                self.target[2],
            ),
        )

    def _publish_setpoint(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(self.target[0])
        msg.pose.position.y = float(self.target[1])
        msg.pose.position.z = float(self.target[2])
        half_yaw = 0.5 * self.target_yaw
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.setpoint_pub.publish(msg)

    def _axis(self, index: int, sign: float) -> float:
        return apply_deadzone(float(self.axes[index]) * sign, self.deadzone)

    def _axes_available(self) -> bool:
        indices = [
            self.axis_yaw,
            self.axis_up,
            self.axis_right,
            self.axis_forward,
        ]
        return bool(self.axes) and min(indices) >= 0 and max(indices) < len(
            self.axes)

    def _deadman_pressed(self) -> bool:
        return self._button(self.buttons, self.button_deadman)

    def _rising_edge(self, index: int) -> bool:
        return (
            self._button(self.buttons, index)
            and not self._button(self.previous_buttons, index)
        )

    @staticmethod
    def _button(buttons: List[int], index: int) -> bool:
        return 0 <= index < len(buttons) and buttons[index] != 0

    def _joy_is_fresh(self) -> bool:
        return (
            self.last_joy_time is not None
            and time.monotonic() - self.last_joy_time <= self.joy_timeout
        )

    def _is_px4_active(self) -> bool:
        return (
            self.mavros_state is not None
            and self.mavros_state.connected
            and self.mavros_state.armed
            and self.mavros_state.mode.upper() == 'OFFBOARD'
        )


def main(args=None) -> None:
    """Run the Mode 2 MAVROS position control node."""
    rclpy.init(args=args)
    node = Ds4PositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
