#!/usr/bin/env python3

"""
Stream position / velocity / torque from CubeMars motors driven by an mjbots
moteus controller over CAN-FD (same path as ``ep-stream-actuator-canfd``).

ActuatorGroup auto-brings can0 up at moteus timings (1 Mbps arb / 5 Mbps data)
when any actuator sets ``requires_can_fd = True``.
"""

from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

print("epicallypowerful: CAN-FD / moteus Motor Connection Demo")
print(
"""
Streams raw state from a moteus-driven CubeMars actuator (e.g. AKE80-8).
Angles in radians, torques in Nm, CCW positive by default.
"""
)

OPERATING_FREQ = 200  # [Hz]
clocking_loop = TimedLoop(rate=OPERATING_FREQ)

actuator_type = input(
    "\nSpecify moteus actuator type [AKE80-8-Moteus]: "
).strip() or "AKE80-8-Moteus"

actuator_ids = input("Specify actuator ids (separate multiple with commas): ")
actuator_ids = [int(s) for s in actuator_ids.replace(" ", "").split(",")]
initialization_dict = {actuator_id: actuator_type for actuator_id in actuator_ids}

actuators = ActuatorGroup.from_dict(
    initialization_dict,
    can_args={"bustype": "socketcan", "channel": "can0", "fd": True},
)

while clocking_loop():
    print("\033[A\033[A\033[A")
    print(
        f"| Actuator | Position [rad] | Velocity [rad/s] | Torque [Nm] | "
        f"Temp [C] | Voltage [V] | Fault |"
    )

    for can_id in actuator_ids:
        actuators.set_torque(can_id, 0)
        act = actuators.actuators[can_id]
        act_data = act.get_data()
        voltage = getattr(act, "voltage", 0.0)
        fault = getattr(act, "fault_code", 0)
        temp = act.get_temperature()
        print(
            f"| {int(can_id):^8} | {act_data.current_position:^14.2f} | "
            f"{act_data.current_velocity:^16.2f} | {act_data.current_torque:^11.2f} | "
            f"{temp:^8.1f} | {voltage:^11.1f} | {int(fault):^5} |"
        )
