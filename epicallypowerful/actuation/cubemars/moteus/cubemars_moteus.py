"""CubeMars actuators driven by an mjbots moteus (e.g. n1) over CAN-FD."""

from __future__ import annotations

import math
import time

import can

from epicallypowerful.actuation.actuator_abc import Actuator
from epicallypowerful.actuation.cubemars.moteus import moteus_driver as md
from epicallypowerful.actuation.motor_data import MotorData
from epicallypowerful.actuation.torque_monitor import RMSTorqueMonitor

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
REV2RAD = 2.0 * math.pi
RAD2REV = 1.0 / REV2RAD


class CubeMarsMoteus(can.Listener, Actuator):
    """Control a CubeMars motor attached to a moteus CAN-FD controller (n1/r4/c1/x1).

    This class must be initialized as part of an :py:class:`ActuatorGroup` so the
    CAN-FD bus is shared and brought up with moteus timings (1M/5M, ``fd on``).

    Unlike MIT-mode CubeMars drivers, communication uses the moteus register
    protocol. The public API still uses **radians / rad/s / N·m** at the output
    shaft; conversions to moteus revolutions are handled internally.

    ``kp`` / ``kd`` arguments to :meth:`set_position` / :meth:`set_control` are
    interpreted as moteus **kp_scale / kd_scale** (unitless multipliers of the
    onboard ``servo.pid_position`` gains), not MIT-mode gains. Tune the onboard
    PID with ``moteus_tool`` / ``tview``, then scale from software if needed.

    Before first use, calibrate the controller for the attached motor::

        python3 -m moteus.moteus_tool --can-iface socketcan --can-chan can0 \\
            --target <ID> --calibrate

    For an 8:1 gearbox (AKE80-8), also configure
    ``motor_position.rotor_to_output_ratio = 0.125``.

    Available ``motor_type`` strings:
        * ``'AKE80-8-Moteus'``

    Example:
        .. code-block:: python

            from epicallypowerful.actuation import ActuatorGroup, CubeMarsMoteus

            motor = CubeMarsMoteus(1, 'AKE80-8-Moteus')
            group = ActuatorGroup([motor])
            group.set_torque(1, 0.5)

    Args:
        can_id: moteus servo CAN ID (destination), typically 1-127.
        motor_type: Key into :data:`MOTOR_PARAMS` for limits / gearing metadata.
        invert: Reverse commanded and reported direction when True.
    """

    requires_can_fd = True

    def __init__(self, can_id: int, motor_type: str, invert: bool = False):
        self.can_id = can_id
        self.motor_type = motor_type
        self.invert = -1 if invert else 1
        self._bus = None
        self.data = MotorData(
            motor_id=self.can_id,
            motor_type=self.motor_type,
            current_position=0,
            current_velocity=0,
            current_torque=0,
            commanded_position=0,
            commanded_velocity=0,
            commanded_torque=0,
            kp=0,
            kd=0,
            timestamp=-1,
            running_torque=(),
            rms_torque=0,
            rms_time_prev=0,
        )
        self.torque_monitor = RMSTorqueMonitor(
            limit=abs(self.data.rated_torque_limits[1]),
            window=20.0,
        )
        self._over_limit = False
        self.voltage = 0.0
        self.fault_code = 0
        self.mode = md.MODE_STOPPED

    def on_message_received(self, msg: can.Message) -> None:
        if not getattr(msg, "is_fd", False):
            return
        if md.reply_source_id(msg.arbitration_id) != self.can_id:
            return

        values = md.parse_reply(bytes(msg.data))
        if not values:
            return

        if md.REG_MODE in values:
            self.mode = int(values[md.REG_MODE])
            self.data.motor_mode = self.mode
        if md.REG_POSITION in values:
            self.data.current_position = values[md.REG_POSITION] * REV2RAD * self.invert
        if md.REG_VELOCITY in values:
            self.data.current_velocity = values[md.REG_VELOCITY] * REV2RAD * self.invert
        if md.REG_TORQUE in values:
            self.data.current_torque = values[md.REG_TORQUE] * self.invert
        if md.REG_TEMPERATURE in values:
            self.data.current_temperature = values[md.REG_TEMPERATURE]
            # Match CubeMarsV3 attribute used by some callers
            self.data.temperature = values[md.REG_TEMPERATURE]
        if md.REG_VOLTAGE in values:
            self.voltage = values[md.REG_VOLTAGE]
        if md.REG_FAULT in values:
            self.fault_code = int(values[md.REG_FAULT])
            self.data.error_code = self.fault_code

        self.data.timestamp = time.perf_counter()
        self.data.responding = True
        rms_torque, _ = self.torque_monitor.update(self.data.current_torque)
        self.data.rms_torque = rms_torque
        self._over_limit = self.torque_monitor.over_limit()

    def call_response_latency(self) -> float:
        return self.data.last_command_time - self.data.timestamp

    def _send(self, msg: can.Message) -> None:
        self.data.last_command_time = time.perf_counter()
        self._bus.send(msg, timeout=0)

    def set_control(
        self,
        pos: float,
        vel: float,
        torque: float,
        kp: float,
        kd: float,
        degrees: bool = False,
    ) -> None:
        if degrees:
            pos *= DEG2RAD
            vel *= DEG2RAD
        pos *= self.invert
        vel *= self.invert
        torque *= self.invert

        self.data.commanded_position = pos
        self.data.commanded_velocity = vel
        self.data.commanded_torque = torque
        self.data.kp = kp
        self.data.kd = kd

        self._send(
            md.create_position_message(
                self.can_id,
                position=pos * RAD2REV,
                velocity=vel * RAD2REV,
                feedforward_torque=torque,
                kp_scale=kp,
                kd_scale=kd,
                query=True,
            )
        )

    def set_torque(self, torque: float) -> None:
        torque = self.invert * torque
        self.data.commanded_torque = torque
        self.data.commanded_position = 0.0
        self.data.commanded_velocity = 0.0
        self.data.kp = 0.0
        self.data.kd = 0.0
        self._send(md.create_torque_message(self.can_id, torque, query=True))

    def set_position(self, position: float, kp: float, kd: float, degrees: bool = False) -> None:
        if degrees:
            position *= DEG2RAD
        position = self.invert * position
        self.data.commanded_position = position
        self.data.commanded_velocity = 0.0
        self.data.commanded_torque = 0.0
        self.data.kp = kp
        self.data.kd = kd
        self._send(
            md.create_position_message(
                self.can_id,
                position=position * RAD2REV,
                velocity=0.0,
                feedforward_torque=0.0,
                kp_scale=kp,
                kd_scale=kd,
                query=True,
            )
        )

    def set_velocity(self, velocity: float, kd: float, degrees: bool = False) -> None:
        if degrees:
            velocity *= DEG2RAD
        velocity = self.invert * velocity
        self.data.commanded_velocity = velocity
        self.data.commanded_position = 0.0
        self.data.commanded_torque = 0.0
        self.data.kp = 0.0
        self.data.kd = kd
        self._send(
            md.create_position_message(
                self.can_id,
                position=math.nan,
                velocity=velocity * RAD2REV,
                feedforward_torque=0.0,
                kp_scale=0.0,
                kd_scale=kd,
                query=True,
            )
        )

    def get_data(self) -> MotorData:
        return self.data

    def get_torque(self) -> float:
        return self.data.current_torque

    def get_position(self, degrees: bool = False) -> float:
        if degrees:
            return self.data.current_position * RAD2DEG
        return self.data.current_position

    def get_velocity(self, degrees: bool = False) -> float:
        if degrees:
            return self.data.current_velocity * RAD2DEG
        return self.data.current_velocity

    def get_temperature(self) -> float:
        return getattr(self.data, "temperature", self.data.current_temperature)

    def zero_encoder(self) -> None:
        self._send(md.create_set_output_exact_message(self.can_id, position=0.0, query=True))

    def _enable(self) -> None:
        # Clear faults / leave stopped, then stream zero torque so the watchdog stays happy.
        self._send(md.create_stop_message(self.can_id, query=True))
        time.sleep(0.01)
        self._send(md.create_torque_message(self.can_id, 0.0, query=True))

    def _disable(self) -> None:
        self._send(md.create_stop_message(self.can_id, query=True))

    def _set_zero_torque(self) -> None:
        self.set_torque(0.0)
