#!/usr/bin/env python3
"""Opt-in integration test; requires vcan0 and a sourced, built workspace."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController,
)
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from processes import MotorPeer, Processes


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def decoded_command(snapshot):
    """Independent wire decoding; do not derive expectations with driver code."""
    data = snapshot["data"]
    if len(data) != 8:
        return None

    def field(offset, low, high):
        raw = (data[offset] << 8) | data[offset + 1]
        return low + raw * (high - low) / 65535

    torque_raw = (snapshot["id"] >> 8) & 0xFFFF
    return (
        field(0, -4 * math.pi, 4 * math.pi),
        field(2, -50, 50),
        -6 + torque_raw * 12 / 65535,
        field(4, 0, 500),
        field(6, 0, 5),
    )


class Scenario:
    def __init__(self, processes, peer):
        self.processes = processes
        self.peer = peer
        self.node = rclpy.create_node("controller_manager_vcan_test")
        self.states = {}
        self.state_count = 0
        self.subscription = self.node.create_subscription(
            JointState, "/joint_states", self.receive_state, 10)
        self.publishers = {
            mode: self.node.create_publisher(
                Float64MultiArray, f"/{mode}_controller/commands", 10)
            for mode in ("position", "velocity", "effort")
        }
        self.description = self.node.create_publisher(
            String, "/robot_description",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.clients = {}
        for name, service in (
            ("list_controllers", ListControllers),
            ("list_hardware_components", ListHardwareComponents),
            ("switch_controller", SwitchController),
            ("set_hardware_component_state", SetHardwareComponentState),
        ):
            self.clients[name] = self.node.create_client(service, f"/controller_manager/{name}")

    def receive_state(self, message):
        self.state_count += 1
        for index, name in enumerate(message.name):
            self.states[name] = (
                message.position[index], message.velocity[index], message.effort[index])

    def wait(self, predicate, message, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.processes.check()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        raise TimeoutError(message)

    def call(self, name, request):
        client = self.clients[name]
        self.wait(client.service_is_ready, f"Service unavailable: {name}")
        future = client.call_async(request)
        self.wait(future.done, f"Service timed out: {name}")
        return future.result()

    def switch(self, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        require(self.call("switch_controller", request).ok, "Controller switch rejected")
        states = {c.name: c.state for c in
                  self.call("list_controllers", ListControllers.Request()).controller}
        for name in activate:
            require(states[name] == "active", f"{name} is not active")
        for name in deactivate:
            require(states[name] == "inactive", f"{name} is not inactive")

    def hardware(self, state):
        request = SetHardwareComponentState.Request()
        request.name = "test_system"
        request.target_state.id = state
        response = self.call("set_hardware_component_state", request)
        require(response.ok and response.state.id == state, "Hardware transition failed")

    def expect_command(self, expected, mode=None, target=None):
        before = self.peer.request()["motions"]

        def received():
            if mode:
                self.publishers[mode].publish(Float64MultiArray(data=[target]))
            snapshot = self.peer.request()
            command = decoded_command(snapshot)
            return (snapshot["motions"] > before and command is not None
                    and snapshot["id"] >> 24 == 1 and snapshot["id"] & 0xFF == 1
                    and all(abs(a - b) < 0.02 for a, b in zip(command, expected)))

        self.wait(received, f"CAN command not observed: {expected}")

    def expect_position(self, expected):
        before = self.state_count
        self.wait(lambda: self.state_count > before and "test_joint" in self.states
                  and abs(self.states["test_joint"][0] - expected) < 0.002,
                  f"Measured joint position did not reach {expected}")

    def run(self, assets):
        robot = String(data=(assets / "robot.urdf").read_text())

        def initialized():
            self.description.publish(robot)
            return self.clients["list_hardware_components"].service_is_ready()

        self.wait(initialized, "controller_manager did not initialize", timeout=30)
        def hardware_active():
            components = self.call(
                "list_hardware_components", ListHardwareComponents.Request()).component
            return (len(components) == 1 and components[0].name == "test_system"
                    and components[0].state.id == State.PRIMARY_STATE_ACTIVE)

        self.wait(hardware_active, "Hardware plugin did not activate")
        snapshot = self.peer.request()
        require(snapshot["mode"] == 2 and snapshot["watchdog"] == 20000
                and snapshot["run_mode"] == 0, "Motor startup parameters were not applied")
        self.expect_command((0, 0, 0, 0, 0))

        for name in ("joint_state_broadcaster", "position_controller",
                     "velocity_controller", "effort_controller"):
            self.processes.run(f"spawner_{name}", [
                "ros2", "run", "controller_manager", "spawner", name,
                "--inactive", "--param-file", str(assets / "controllers.yaml"),
                "--controller-manager", "/controller_manager",
            ])
        self.switch(activate=["joint_state_broadcaster"])
        self.expect_position(-0.2)  # motor 0.6, direction -1, ratio 2, offset 0.1

        self.switch(activate=["position_controller"])
        self.expect_command((0.6, 0, 0, 20, 0.5))
        self.expect_command((-0.8, 0, 0, 20, 0.5), "position", 0.5)
        self.expect_position(-0.2)  # A command must not masquerade as measured feedback.
        self.peer.request("position 1.0")
        self.expect_position(-0.4)

        self.switch(activate=["velocity_controller"], deactivate=["position_controller"])
        self.expect_command((0, 0, 0, 0, 0.5))
        self.expect_command((0, -1.4, 0, 0, 0.5), "velocity", 0.7)
        self.switch(activate=["effort_controller"], deactivate=["velocity_controller"])
        self.expect_command((0, 0, 0, 0, 0))
        self.expect_command((0, 0, -0.4, 0, 0), "effort", 0.8)

        self.switch(deactivate=["effort_controller", "joint_state_broadcaster"])
        before = self.peer.request()
        self.hardware(State.PRIMARY_STATE_INACTIVE)
        stopped = self.peer.request()
        require(stopped["mode"] == 0 and stopped["stops"] > before["stops"],
                "Hardware deactivation did not stop the motor on CAN")

        # Change the measurement while inactive. Reactivation must use this position,
        # not the previous controller target (0.5) or previous measurement (-0.4).
        self.peer.request("position -0.6")
        self.hardware(State.PRIMARY_STATE_ACTIVE)
        require(self.peer.request()["enables"] > before["enables"], "Motor was not re-enabled")
        self.expect_command((0, 0, 0, 0, 0))
        self.switch(activate=["joint_state_broadcaster"])
        self.expect_position(0.4)
        self.switch(activate=["position_controller"])
        self.expect_command((-0.6, 0, 0, 20, 0.5))
        self.expect_command((-0.2, 0, 0, 20, 0.5), "position", 0.2)
        self.switch(deactivate=["position_controller", "joint_state_broadcaster"])
        self.hardware(State.PRIMARY_STATE_INACTIVE)
        require(self.peer.request()["mode"] == 0, "Final motor stop was not confirmed")
        print("PASS: plugin loading, controller commands, feedback, switching and reactivation")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake-motor", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    link = json.loads(subprocess.check_output(
        ["ip", "-details", "-json", "link", "show", "dev", "vcan0"], timeout=5))
    require(len(link) == 1 and link[0].get("linkinfo", {}).get("info_kind") == "vcan",
            "This test only runs on a virtual CAN interface, never physical actuators")
    assets = Path(__file__).resolve().parent
    processes = Processes(args.output_dir)
    peer = None
    scenario = None
    rclpy.init()
    try:
        peer = MotorPeer(processes, args.fake_motor.resolve())
        peer.request("position 0.6")
        processes.start("socketcan", [
            "ros2", "launch", "ros2_socketcan", "socket_can_bridge.launch.xml",
            "interface:=vcan0", "enable_can_fd:=false", "enable_frame_loopback:=true",
            "from_can_bus_topic:=/integration/from_bus",
            "to_can_bus_topic:=/integration/to_bus",
        ])
        scenario = Scenario(processes, peer)
        processes.start("controller_manager", [
            "ros2", "run", "controller_manager", "ros2_control_node", "--ros-args",
            "--params-file", str(assets / "controllers.yaml"),
            "-r", "~/robot_description:=/robot_description",
        ])
        scenario.run(assets)
    finally:
        processes.close()
        if peer:
            peer.close()
        if scenario:
            scenario.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
