"""Bounded subprocess and fake-motor control helpers for opt-in Linux tests."""

import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time


class Processes:
    def __init__(self, output_dir):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.children = []
        self.logs = []

    def start(self, name, args, pipe=False):
        log = (self.output_dir / f"{name}.log").open("w")
        self.logs.append(log)
        child = subprocess.Popen(
            args, stdin=subprocess.PIPE if pipe else subprocess.DEVNULL,
            stdout=subprocess.PIPE if pipe else log, stderr=log,
            start_new_session=True,
        )
        self.children.append((name, child))
        return child

    def check(self):
        for name, child in self.children:
            if child.poll() is not None:
                raise RuntimeError(f"{name} exited unexpectedly: {child.returncode}")

    def run(self, name, args, timeout=25):
        child = self.start(name, args)
        # On timeout the registered process remains available to close(), which
        # kills its process group rather than only the `ros2 run` wrapper.
        code = child.wait(timeout=timeout)
        if code != 0:
            raise RuntimeError(f"{name} failed: {code}")
        self.stop_group(child)
        self.children.remove((name, child))

    @staticmethod
    def stop_group(child):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                child.poll()  # Reap the leader, but also wait for its descendants.
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                continue
            break
        child.wait(timeout=3)

    def close(self):
        # Stop controller_manager before the bridge and fake motor, so lifecycle
        # stop frames still have a live transport during graceful shutdown.
        errors = []
        for name, child in reversed(self.children):
            try:
                self.stop_group(child)
            except (OSError, subprocess.TimeoutExpired) as error:
                errors.append(f"{name}: {error}")
            finally:
                if child.stdin:
                    child.stdin.close()
                if child.stdout:
                    child.stdout.close()
        for log in self.logs:
            log.close()
        if errors:
            raise RuntimeError("Process cleanup failed: " + "; ".join(errors))


class MotorPeer:
    def __init__(self, processes, executable):
        self.child = processes.start("fake_motor", [str(executable)], pipe=True)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.child.stdout, selectors.EVENT_READ)
        self.transcript = (processes.output_dir / "motor_observations.jsonl").open("w")

    def request(self, command="snapshot"):
        self.child.stdin.write((command + "\n").encode())
        self.child.stdin.flush()
        if not self.selector.select(timeout=2):
            raise TimeoutError(f"Fake motor did not answer {command}")
        line = self.child.stdout.readline()
        if not line:
            raise RuntimeError("Fake motor control channel closed")
        result = json.loads(line)
        self.transcript.write(json.dumps({"request": command, "response": result}) + "\n")
        self.transcript.flush()
        return result

    def close(self):
        self.selector.close()
        self.transcript.close()
