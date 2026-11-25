#!/usr/bin/env python3
"""
masterCtrl.py: one web page for the robot, served from the Jetson. It moves the two moteus
motors (CAN IDs 1 and 2) by angle, and runs the Waveshare/Feetech ST gripper servo the same
way gripCtrl.py does.

Setup: put this file on the Jetson next to the scservo_sdk folder (where gripCtrl.py is).
Close tview and gripCtrl.py first: only one program can use each adapter at a time.

On the Jetson:
    chmod +x masterCtrl.py
    ./masterCtrl.py                  # ./masterCtrl.py --help lists every option
On your laptop:
    ssh -L 8080:localhost:8080 jetson
then open http://localhost:8080

Motors (shown as ID 1 and ID 2):
  * Angles are at each motor's output shaft, in degrees from the zero you set.
  * Go to an angle, step a few degrees CW or CCW, or press and hold CW or CCW and let go to stop.
  * Each motor has a "CW is" setting (+ angle or - angle). Check it once by holding CW and
    looking at the shaft end; it's saved in masterCtrl_settings.json next to this file.
  * "Both motors" moves ID 1 and ID 2 at once, in any of the four CW/CCW combinations, while you
    hold a button or by a set step. They start and stop together: if either can't make the move,
    neither moves, and if either stops (stuck, fault, end of its range), both stop.
  * Moves stay inside each controller's saved limits (servopos.position_min / max) unless
    "Allow moving past the limits" is on for that motor. Even then the page won't send a motor
    more than half a turn from zero (--travel-cap): past that, the encoder reads a full turn
    off after a power-up.
  * "Set lower/upper limit here" saves the current angle as that limit on the controller.

Safety behavior:
  * Stop all, Space or Esc stops everything: motors stop and hold where they are, and the
    gripper stops turning.
  * A motor that gets stuck short of its target (a hard stop or a jam) stops pushing and holds
    there: after 1 second at its torque cap, or 3 seconds without any progress.
  * If the page stops checking in for --watchdog seconds, moving motors stop and hold and the
    gripper stops. Motors that are holding keep holding while this program runs.
  * A motor that faults stays limp until you move it again.
  * When this program exits, both motors are released (limp) and the gripper stops.
  * The page only answers on the Jetson itself unless you pass --host.
"""

import argparse
import asyncio
import glob
import ipaddress
import json
import math
import os
import re
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import urlsplit

# Make scservo_sdk/ (next to this file) importable from any working directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import moteus
    MOTEUS_ERROR = None
except Exception as exc:                # not installed for this Python (or run with sudo)
    moteus = None
    MOTEUS_ERROR = exc

try:
    from scservo_sdk import PortHandler, COMM_SUCCESS
    from scservo_sdk.sms_sts import (
        sms_sts,
        SMS_STS_MODE,
        SMS_STS_TORQUE_ENABLE,
        SMS_STS_GOAL_SPEED_L,
        SMS_STS_PRESENT_POSITION_L,
    )
    SCSERVO_ERROR = None
except Exception as exc:
    SCSERVO_ERROR = exc


def log(message):
    print(time.strftime("%H:%M:%S"), message, flush=True)


def clamp(value, low, high):
    return max(low, min(high, value))


def finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def num(x, digits):
    """A JSON-safe number: None for missing or NaN values."""
    return round(x, digits) if finite(x) else None


def deg(turns):
    return turns * 360.0


def fmt_deg(turns):
    return f"{deg(turns):+.1f}°"


# ---------------------------------------------------------------------------
# Gripper: Waveshare / Feetech ST-series bus servo in constant-speed mode.
# This is gripCtrl.py's controller, unchanged apart from how it finds its port.
# ---------------------------------------------------------------------------

BAUDRATE = 1000000          # ST-series factory default
RPM_PER_UNIT = 0.732        # speed register unit
STEPS_PER_REV = 4096
ADDR_TORQUE_LIMIT = 48      # 0-1000 in 0.1% steps; resets to the maximum at power-on
FEEDBACK_LEN = 15           # addresses 56-70: position, speed, load, voltage, temperature, status, current
MODE_CONSTANT_SPEED = 1
STATUS_OVERLOAD = 1 << 5
GRIP_POLL_SECONDS = 0.25


def can_adapter_paths(extra=()):
    """Device paths of mjbots CAN adapters, so the gripper search never opens one."""
    paths = set()
    for link in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(link).lower()
        if "fdcanusb" in name or "mjbots" in name or "mjcanfd" in name:
            paths.add(os.path.realpath(link))
    if moteus is not None:
        try:
            for dev in moteus.FdcanusbDevice.pyserial_detect_fdcanusbs():
                paths.add(os.path.realpath(dev))
        except Exception:
            pass
    for dev in extra or ():
        paths.add(os.path.realpath(dev))
    return paths


def gripper_port_candidates(exclude):
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    return [p for p in ports if os.path.realpath(p) not in exclude]


class GripperController:
    """Owns the gripper's serial port. Every method that talks to the servo holds self.lock."""

    def __init__(self, args, can_ports=()):
        self.args = args
        self.can_ports = list(can_ports)
        self.sid = args.id
        self.lock = threading.RLock()
        self.port = None
        self.servo = None
        self.port_name = None
        self.port_open = False
        self.ready = False              # speed mode set, stopped, torque limit applied
        self.model = None
        self.direction = 0              # -1 backward, 0 stopped, 1 forward
        self.rpm = float(args.default_rpm)
        self.strength = args.default_strength
        self.high_grip = args.default_strength > args.normal_strength
        self.torque_on = False
        self.overheated = False
        self.telemetry = {}
        self.problem = "Connecting to the gripper."
        self.last_error = ""
        self.notice_id = 0
        self.notice = ""
        self.notice_time = ""
        self.failures = 0
        self.polls = 0
        self.next_attempt = 0.0
        self.last_contact = time.time()

    # ---------- low-level helpers ----------

    def _notify(self, text):
        self.notice_id += 1
        self.notice = text
        self.notice_time = time.strftime("%H:%M:%S")
        log(f"Gripper: {text}")

    def _xfer(self, what, fn, *args):
        """Run one library call. Returns its result tuple, or None if it failed."""
        try:
            out = fn(*args)
        except Exception as exc:        # USB unplugged or another port error
            self._port_lost(f"{what} failed: {exc}")
            return None
        if out[-2] != COMM_SUCCESS:
            self.failures += 1
            self.last_error = f"{what} failed: {self.servo.getTxRxResult(out[-2])}"
            return None
        self.failures = 0
        return out

    def _port_lost(self, reason):
        log(f"Gripper: {reason}")
        self.last_error = reason
        if self.port is not None:
            self.port.is_using = False
            try:
                self.port.closePort()
            except Exception:
                pass
        self.port_open = self.ready = False
        self.direction = 0
        self.problem = "Lost the connection to the gripper adapter. Check its USB cable."

    def _open_port(self):
        if self.args.serial:
            names = [self.args.serial]
        else:
            names = gripper_port_candidates(can_adapter_paths(self.can_ports))
        if not names:
            self.problem = "No gripper adapter found. Check its USB cable and that its jumper is on B."
            return False
        errors = []
        for name in names:
            try:
                port = PortHandler(name)
                if not (port.openPort() and port.setBaudRate(BAUDRATE)):
                    raise OSError(f"could not set {BAUDRATE} baud")
            except Exception as exc:
                errors.append(f"{name}: {exc}")
                continue
            servo = sms_sts(port)
            if len(names) > 1:
                # Several serial ports: keep the one where the servo answers.
                try:
                    answered = servo.ping(self.sid)[-2] == COMM_SUCCESS
                except Exception:
                    answered = False
                if not answered:
                    try:
                        port.closePort()
                    except Exception:
                        pass
                    errors.append(f"{name}: servo {self.sid} didn't answer")
                    continue
            self.port, self.servo, self.port_name, self.port_open = port, servo, name, True
            log(f"Gripper: opened {name}")
            return True
        hint = (" Run: sudo usermod -aG dialout $USER, then log in again."
                if any("ermission" in e for e in errors) else "")
        self.problem = "Couldn't open the gripper adapter (" + "; ".join(errors) + ")." + hint
        return False

    def _units(self, rpm):
        return max(1, round(rpm / RPM_PER_UNIT))

    def _signed(self, direction):
        return direction * (-1 if self.args.invert else 1)

    # ---------- setup ----------

    def _setup(self):
        """Put the servo in constant-speed mode, stopped, with the chosen torque limit."""
        out = self._xfer("Ping", self.servo.ping, self.sid)
        if out is None:
            self.problem = (f"Gripper servo {self.sid} is not answering. Check its power supply, that "
                            "it is plugged into the adapter, and that the jumper is on B.")
            return False
        self.model = out[0]
        mode = self._xfer("Reading mode", self.servo.read1ByteTxRx, self.sid, SMS_STS_MODE)
        if mode is None:
            self.problem = self.last_error
            return False

        s, sid = self.servo, self.sid
        if mode[0] == MODE_CONSTANT_SPEED:
            steps = [("Stopping", s.WriteSpec, sid, 0, self.args.stop_accel)]
        else:
            # Torque off before changing modes, so the switch can't make the servo jump.
            steps = [("Torque off", s.write1ByteTxRx, sid, SMS_STS_TORQUE_ENABLE, 0),
                     ("Zero speed", s.write2ByteTxRx, sid, SMS_STS_GOAL_SPEED_L, 0)]
        steps.append(("Setting grip strength", s.write2ByteTxRx, sid, ADDR_TORQUE_LIMIT, self.strength * 10))
        if mode[0] != MODE_CONSTANT_SPEED:
            steps.append(("Setting speed mode", s.write1ByteTxRx, sid, SMS_STS_MODE, MODE_CONSTANT_SPEED))
        steps.append(("Torque on", s.write1ByteTxRx, sid, SMS_STS_TORQUE_ENABLE, 1))

        for what, fn, *fn_args in steps:
            if self._xfer(what, fn, *fn_args) is None:
                self.problem = f"Setup stopped at '{what}'. {self.last_error}"
                return False
        self.direction, self.torque_on, self.ready, self.problem = 0, True, True, ""
        log(f"Gripper: servo {sid} ready (model {self.model}): speed mode, stopped, grip {self.strength}%")
        return True

    # ---------- commands from the page ----------

    def command(self, action, data):
        with self.lock:
            self.last_contact = time.time()
            if action == "drive":
                return self._drive(int(data.get("direction", 0)), float(data.get("rpm", self.rpm)))
            if action == "stop":
                return self._stop("Stopped.")
            if action == "speed":
                return self._set_speed(float(data["rpm"]))
            if action == "strength":
                return self._set_strength(int(data["percent"]), bool(data.get("high", self.high_grip)))
            if action == "release":
                return self._set_torque(False)
            if action == "hold":
                return self._set_torque(True)
            return False, f"Unknown action: {action}"

    def _drive(self, direction, rpm):
        if direction == 0:
            return self._stop("Stopped.")
        if direction not in (-1, 1):
            return False, "Direction must be forward or backward."
        if not self.ready:
            return False, self.problem or "The gripper isn't ready yet."
        if self.overheated:
            return False, f"The gripper is too hot. It can run again once it cools to {self.args.cool_temp} °C."
        self.rpm = clamp(rpm, self.args.min_rpm, self.args.max_rpm)
        if not self.torque_on:
            ok, message = self._set_torque(True)
            if not ok:
                return ok, message
        units = self._units(self.rpm) * self._signed(direction)
        if self._xfer("Run", self.servo.WriteSpec, self.sid, units, self.args.accel) is None:
            return False, f"The servo didn't accept the command. {self.last_error}"
        self.direction = direction
        word = "forward" if direction > 0 else "backward"
        log(f"Gripper: run {word} at {self.rpm:.0f} RPM")
        return True, f"Running {word} at {self.rpm:.0f} RPM."

    def _stop(self, message):
        was_running = self.direction != 0
        self.direction = 0
        if self.port_open:
            if self._xfer("Stop", self.servo.WriteSpec, self.sid, 0, self.args.stop_accel) is None:
                return False, f"The stop command failed. {self.last_error}"
        if was_running:
            log("Gripper: stop")
        return True, message

    def _set_speed(self, rpm):
        self.rpm = clamp(rpm, self.args.min_rpm, self.args.max_rpm)
        if self.direction != 0 and self.ready:
            units = self._units(self.rpm) * self._signed(self.direction)
            if self._xfer("Speed change", self.servo.WriteSpec, self.sid, units, self.args.accel) is None:
                return False, f"The speed change failed. {self.last_error}"
            return True, f"Now running at {self.rpm:.0f} RPM."
        return True, f"Speed set to {self.rpm:.0f} RPM."

    def _set_strength(self, percent, high):
        cap = self.args.high_strength if high else self.args.normal_strength
        value = clamp(percent, self.args.min_strength, cap)
        if self.ready and self._xfer("Grip strength", self.servo.write2ByteTxRx,
                                     self.sid, ADDR_TORQUE_LIMIT, value * 10) is None:
            return False, f"The grip strength change failed. {self.last_error}"
        self.strength, self.high_grip = value, high
        log(f"Gripper: grip strength {value}%{' (high grip allowed)' if high else ''}")
        if percent > cap:
            return True, (f"Grip strength is capped at {cap}%. Turn on Allow high grip to go higher."
                          if not high else f"Grip strength is capped at {cap}%.")
        return True, f"Grip strength set to {value}%."

    def _set_torque(self, enable):
        if not self.ready:
            return False, self.problem or "The gripper isn't ready yet."
        # Zero the speed first in both cases, so torque never comes on at an old speed.
        if self._xfer("Stop", self.servo.WriteSpec, self.sid, 0, self.args.stop_accel) is None:
            return False, f"The command failed. {self.last_error}"
        self.direction = 0
        if self._xfer("Torque", self.servo.write1ByteTxRx, self.sid, SMS_STS_TORQUE_ENABLE,
                      1 if enable else 0) is None:
            return False, f"The command failed. {self.last_error}"
        self.torque_on = enable
        log("Gripper: torque on" if enable else "Gripper: torque released")
        return True, ("Torque on. The gripper is holding still." if enable
                      else "Torque released. The gripper is limp.")

    # ---------- background monitoring ----------

    def poll(self):
        with self.lock:
            now = time.time()
            if not self.port_open or not self.ready:
                if now >= self.next_attempt:
                    self.next_attempt = now + 2.0
                    if (self.port_open or self._open_port()) and self._setup():
                        self._notify("Connected. It is stopped and ready.")
                return

            out = self._xfer("Reading feedback", self.servo.readTxRx, self.sid,
                             SMS_STS_PRESENT_POSITION_L, FEEDBACK_LEN)
            if out is None:
                if self.port_open and self.failures >= 4:
                    self.ready = False
                    self.direction = 0
                    self.problem = ("The gripper servo stopped answering. Check its power supply and "
                                    "that it is plugged into the adapter.")
                    log(f"Gripper: {self.problem}")
                return
            self._parse(out[0])

            self.polls += 1
            if self.polls % 4 == 0:
                self._check_settings()
            if self.direction != 0 and now - self.last_contact > self.args.watchdog:
                self._stop("")
                self._notify("Stopped because the control page lost contact.")
            temp = self.telemetry.get("temp_c", 0)
            if temp >= self.args.hot_temp and not self.overheated:
                self.overheated = True
                self._stop("")
                self._notify(f"Stopped: the servo reached {temp} °C. "
                             f"It can run again once it cools to {self.args.cool_temp} °C.")
            elif self.overheated and temp <= self.args.cool_temp:
                self.overheated = False
                self._notify("The servo has cooled down and can run again.")

    def _parse(self, d):
        word = lambda i: d[i] | (d[i + 1] << 8)
        sign = -1 if self.args.invert else 1
        position = self.servo.scs_tohost(word(0), 15)
        speed = self.servo.scs_tohost(word(2), 15)
        load_raw = word(4)
        load = (load_raw & 0x3FF) / 10.0 * (-1 if load_raw & 0x400 else 1)
        self.telemetry = {
            "position_deg": round((position % STEPS_PER_REV) * 360 / STEPS_PER_REV, 1),
            "speed_rpm": round(speed * RPM_PER_UNIT * sign, 1),
            "load_pct": round(load * sign, 1),
            "voltage_v": d[6] / 10,
            "temp_c": d[7],
            "overload": bool(d[9] & STATUS_OVERLOAD),
            "status": d[9],
            "current_a": round((word(13) & 0x7FFF) * 6.5 / 1000, 2),
        }

    def _check_settings(self):
        """Catch a servo that restarted (for example after a power dip). It comes back in
        position mode or with torque off, and always with its torque limit back at 100%."""
        mode = self._xfer("Reading mode", self.servo.read1ByteTxRx, self.sid, SMS_STS_MODE)
        torque = self._xfer("Reading torque", self.servo.read1ByteTxRx, self.sid, SMS_STS_TORQUE_ENABLE)
        limit = self._xfer("Reading torque limit", self.servo.read2ByteTxRx, self.sid, ADDR_TORQUE_LIMIT)
        if mode is None or torque is None or limit is None:
            return
        expected_torque = 1 if self.torque_on else 0
        if mode[0] != MODE_CONSTANT_SPEED or torque[0] != expected_torque:
            log(f"Gripper: servo settings changed (mode {mode[0]}, torque {torque[0]}). Restoring.")
            self.ready = False
            self.direction = 0
            if self._setup():
                self._notify("The servo restarted, possibly from a power dip. "
                             "It was stopped and its settings restored.")
        elif limit[0] != self.strength * 10:
            self._stop("")
            if self._xfer("Grip strength", self.servo.write2ByteTxRx, self.sid,
                          ADDR_TORQUE_LIMIT, self.strength * 10) is not None:
                self._notify(f"The servo changed its own torque limit to {limit[0] / 10:.0f}%, likely "
                             f"overload protection. It was stopped and grip strength set back to "
                             f"{self.strength}%.")

    # ---------- page data and shutdown ----------

    def status(self):
        with self.lock:
            self.last_contact = time.time()
            a = self.args
            return {
                "connected": self.ready, "problem": self.problem,
                "notice": self.notice, "notice_id": self.notice_id, "notice_time": self.notice_time,
                "port": self.port_name, "id": self.sid, "model": self.model,
                "direction": self.direction, "rpm": self.rpm,
                "strength": self.strength, "high_grip": self.high_grip,
                "torque_on": self.torque_on, "overheated": self.overheated,
                "telemetry": self.telemetry if self.ready else {},
                "limits": {"min_rpm": a.min_rpm, "max_rpm": a.max_rpm,
                           "min_strength": a.min_strength, "normal_strength": a.normal_strength,
                           "high_strength": a.high_strength},
            }

    def shutdown(self):
        with self.lock:
            if self.port_open:
                if self.ready:
                    self._stop("")
                    log("Gripper: stopped and holding. It stays in speed mode until its power is cycled.")
                try:
                    self.port.closePort()
                except Exception:
                    pass
                self.port_open = False


# ---------------------------------------------------------------------------
# Motors: two moteus controllers on one CAN bus, in position mode.
# All CAN traffic happens on one asyncio loop in its own thread. The web server
# threads only change what each motor should be doing, under MoteusManager.lock.
# ---------------------------------------------------------------------------

CONTROL_HZ = 50
CAN_TIMEOUT_S = 0.1             # a controller that hasn't answered by then counts as a miss
OFFLINE_AFTER_MISSES = 5
OFFLINE_RETRY_S = 1.0
COMMAND_WATCHDOG_S = 0.5        # the controller pauses itself if commands stop for this long
STALL_SECONDS = 1.0
STALL_NO_PROGRESS_S = 3.0
STALL_PROGRESS = 0.0015         # turns (0.5°): moving at least this much counts as progress
STALL_MIN_ERROR = 0.004         # turns (1.4°) short of the target
STALL_MAX_SPEED = 0.01          # turns/s (3.6°/s)
STALL_TORQUE_FRACTION = 0.85
AT_TARGET = 0.003               # turns (about 1°)
LAGGING = 0.02                  # turns (7°): further than this behind its setpoint, it's stuck
NUDGE_LEASE_S = 0.6             # hold-to-turn stops unless the page renews it this often
LIMIT_TOLERANCE = 0.002         # turns (0.7°)
SETTLE_REPLIES = 2              # replies right after a stop can still show the old mode
DIAG_TIMEOUT_S = 3.0

FAULT_TEXT = {
    32: "calibration fault", 33: "motor driver fault", 34: "supply voltage too high",
    35: "encoder fault", 36: "motor not configured", 37: "PWM cycle overrun",
    38: "controller too hot", 39: "it started outside its limits", 40: "supply voltage too low",
    41: "configuration changed", 42: "hall rotor angle invalid", 43: "position invalid",
    44: "motor driver enable fault", 45: "stop position no longer supported", 46: "timing violation",
    47: "back-EMF feedforward needs an accel limit", 48: "invalid limits",
    49: "position error too large", 50: "velocity error too large",
    51: "power stage too hot", 52: "motor too hot",
}
LIMIT_TEXT = {
    96: "at its speed limit", 97: "at its power limit", 98: "at the supply voltage limit",
    99: "at its current limit", 100: "slowed by controller temperature",
    101: "slowed by motor temperature", 102: "pushing at its torque cap",
    103: "held at a position limit", 104: "braking", 105: "field weakening",
    106: "slowed by power stage temperature",
}
DRIVEN_MODES = (10, 11, 12, 13)     # position, timed out, zero velocity, stay within


class AdapterLost(Exception):
    pass


class SerializedTransport:
    """Shares one CAN adapter between the control loop and settings reads: one exchange at a
    time, and none can hang. (With an fdcanusb, a controller that never answers would otherwise
    stall the exchange forever.)"""

    def __init__(self, transport, timeout):
        self._transport = transport
        self._timeout = timeout
        self._lock = asyncio.Lock()

    async def cycle(self, commands, **kwargs):
        # Not asyncio.wait_for: on Python 3.10 it can swallow a cancellation when it's nested in
        # another timeout, which leaves the moteus library's flush loop spinning forever.
        async with self._lock:
            task = asyncio.ensure_future(self._transport.cycle(commands, **kwargs))
            try:
                done, _ = await asyncio.wait({task}, timeout=self._timeout)
            except asyncio.CancelledError:
                task.cancel()
                raise
            if not done:
                task.cancel()
                raise asyncio.TimeoutError()
            return task.result()

    async def read(self, *args, **kwargs):
        async with self._lock:
            return await self._transport.read(*args, **kwargs)

    def close(self):
        try:
            self._transport.close()
        except Exception:
            pass

    def __getattr__(self, name):
        return getattr(self._transport, name)


def default_transport(args):
    """The CAN adapter, found the same way tview finds it (or set with --fdcanusb etc.)."""
    import moteus.transport_factory as factory
    if factory.GLOBAL_TRANSPORT is not None:         # reconnecting: start fresh
        try:
            factory.GLOBAL_TRANSPORT.close()
        except Exception:
            pass
        factory.GLOBAL_TRANSPORT = None
    return moteus.get_singleton_transport(args)


def make_query_resolution():
    qr = moteus.QueryResolution()
    qr.q_current = moteus.F32
    qr.trajectory_complete = moteus.INT8
    qr.voltage = moteus.INT16
    qr.temperature = moteus.INT16
    qr._extra = {int(moteus.Register.CONTROL_POSITION): moteus.F32,
                 int(moteus.Register.CONTROL_VELOCITY): moteus.F32}
    return qr


class Joint:
    def __init__(self, can_id, name, args):
        self.id = can_id
        self.name = name
        # What the page asked for. Guarded by MoteusManager.lock.
        self.want = "released"          # "released" or "position"
        self.target = None              # turns at the output shaft
        self.speed_rpm = float(args.joint_default_rpm)
        self.max_torque = float(args.joint_default_torque)
        self.accel = args.joint_accel
        self.beyond = False             # ignore the controller's saved limits
        self.cw_sign = 1                # +1: clockwise raises the angle; -1: it lowers it
        self.group = None               # set while moving together with the other motor
        self.move_rpm = None            # speed for the current move, if not speed_rpm
        self.nudge_dir = 0
        self.nudge_until = 0.0
        self.need_stop = False          # send one stop: clears a fault or timeout, resets the integrator
        self.settle = 0
        self.stall_since = None
        self.stall_pos = 0.0
        self.fault_reported = False
        self.sent = None
        # What the controller reports.
        self.controller = None
        self.stream = None
        self.stream_ready = False
        self.online = False
        self.adopted = False
        self.misses = 0
        self.next_try = 0.0
        self.t = {}
        self.t_time = 0.0
        self.limits = None              # (min, max) in turns; NaN where none is saved
        self.limits_read = False
        self.limits_busy = False
        self.limits_failures = 0
        self.limits_next_try = 0.0
        self.problem = "Looking for the controller."
        self.notice = ""
        self.notice_id = 0
        self.notice_time = ""


class MoteusManager:
    def __init__(self, args, transport_factory=None):
        self.args = args
        self.transport_factory = transport_factory or default_transport
        self.lock = threading.RLock()
        self.joints = [Joint(i, args.names.get(i, f"ID {i}"), args) for i in args.moteus_ids]
        self.by_id = {j.id: j for j in self.joints}
        self.pair = self.joints[:2] if len(self.joints) >= 2 else []
        self.pair_rpm = float(args.joint_default_rpm)
        self._next_group = 1
        self.settings_path = args.settings
        self.settings = load_settings(self.settings_path)
        cw = self.settings.get("cw_sign", {})
        for j in self.joints:
            try:
                j.cw_sign = -1 if float(cw.get(str(j.id), 1)) < 0 else 1
            except (TypeError, ValueError):
                j.cw_sign = 1
        self.transport = None
        self.adapter_problem = "Connecting to the CAN adapter."
        self.last_contact = time.time()
        self.contact_lost = False
        self.loop = None
        self._stopping = False
        self._reconnect_at = 0.0
        self._tasks = set()
        self._last_error_log = 0.0
        self._thread = threading.Thread(target=self._thread_main, name="moteus", daemon=True)
        self._started = threading.Event()
        self._finished = threading.Event()

    # ---------- thread and main loop ----------

    def start(self):
        self._thread.start()
        self._started.wait(5)

    def shutdown(self):
        self._stopping = True
        if not self._finished.wait(4):
            log("The motor loop didn't finish in time.")

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self._started.set()
        try:
            self.loop.run_until_complete(self._run())
        except Exception as exc:
            log(f"Motor loop stopped: {exc!r}")
        finally:
            self._finished.set()

    async def _run(self):
        period = 1.0 / CONTROL_HZ
        while not self._stopping:
            began = time.monotonic()
            if self.transport is None and (began < self._reconnect_at or not await self._connect()):
                if began >= self._reconnect_at:
                    self._reconnect_at = began + 2.0
                await asyncio.sleep(0.1)
                continue
            try:
                for j in self.joints:
                    await self._service(j)
                self._check_page_contact()
            except AdapterLost as exc:
                self._adapter_lost(exc)
                continue
            except Exception as exc:            # keep commanding the motors no matter what
                if time.monotonic() - self._last_error_log > 2.0:
                    self._last_error_log = time.monotonic()
                    log(f"Motor loop error (continuing): {exc!r}")
            await asyncio.sleep(max(0.0, period - (time.monotonic() - began)))
        for task in list(self._tasks):          # settings reads still in progress
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        await self._release_all()
        if self.transport is not None:
            self.transport.close()

    async def _connect(self):
        try:
            raw = self.transport_factory(self.args)
        except Exception as exc:
            reasons = re.findall(r"'([^']+)'", str(exc)) or [str(exc)]
            problem = (f"CAN adapter not found ({'; '.join(reasons)}). Check its USB cable, and "
                       "close tview if it's open.")
            with self.lock:
                if problem != self.adapter_problem:
                    log(problem)
                self.adapter_problem = problem
                for j in self.joints:
                    j.online = False
                    j.problem = problem
            return False
        self.transport = SerializedTransport(raw, CAN_TIMEOUT_S)
        qr = make_query_resolution()
        with self.lock:
            self.adapter_problem = ""
            for j in self.joints:
                j.controller = moteus.Controller(id=j.id, transport=self.transport, query_resolution=qr)
                j.stream = moteus.Stream(j.controller)
                j.stream_ready = False
                j.online = j.adopted = False
                j.misses = 0
                j.next_try = 0.0
        log("CAN adapter ready.")
        return True

    def _adapter_lost(self, exc):
        log(f"Lost the CAN adapter: {exc!r}")
        with self.lock:
            self.adapter_problem = "Lost the CAN adapter. Check its USB cable."
            for j in self.joints:
                if j.online:
                    self._notify(j, "Lost the CAN adapter. The controller pauses on its own when "
                                    "commands stop arriving.")
                j.online = j.adopted = False
                j.problem = self.adapter_problem
        if self.transport is not None:
            self.transport.close()
        self.transport = None
        self._reconnect_at = time.monotonic() + 1.0

    async def _release_all(self):
        for j in self.joints:
            if self.transport is None or j.controller is None or not j.online:
                continue
            try:
                await j.controller.set_stop()
                log(f"{j.name} released (limp).")
            except Exception as exc:
                log(f"Couldn't release {j.name}: {exc!r}")

    # ---------- one exchange with one controller ----------

    async def _service(self, j):
        now = time.monotonic()
        if not j.online and now < j.next_try:
            return
        with self.lock:
            command = self._next_command(j, now)
        try:
            result = await j.controller.execute(command)
        except asyncio.TimeoutError:
            result = None
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            raise AdapterLost(exc) from exc
        if result is None:
            self._missed(j, now)
            return
        self._received(j, result)
        with self.lock:
            read_limits = (j.online and not j.limits_read and not j.limits_busy
                           and time.monotonic() >= j.limits_next_try)
            if read_limits:
                j.limits_busy = True
        if read_limits:
            task = asyncio.ensure_future(self._read_limits(j))
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)

    def _next_command(self, j, now):
        """Decide what to send this cycle. Called with self.lock held."""
        c = j.controller
        if j.need_stop:
            j.need_stop = False
            j.settle = SETTLE_REPLIES
            j.sent = "stop"
            return c.make_stop(query=True)
        if j.adopted and j.want == "position" and j.target is not None:
            if j.nudge_until and now > j.nudge_until:
                self._hold(j, quick=True)       # the page stopped renewing hold-to-turn
                self._group_stopped(j, None)
            kwargs = dict(position=j.target, velocity=0.0, maximum_torque=j.max_torque,
                          velocity_limit=(j.move_rpm or j.speed_rpm) / 60.0, accel_limit=j.accel,
                          watchdog_timeout=COMMAND_WATCHDOG_S, query=True)
            if j.beyond:
                kwargs["ignore_position_bounds"] = True
            j.sent = "position"
            return c.make_position(**kwargs)
        j.sent = "query"
        return c.make_query()

    def _missed(self, j, now):
        with self.lock:
            j.misses += 1
            if j.online and j.misses >= OFFLINE_AFTER_MISSES:
                j.online = j.adopted = False
                self._notify(j, "Stopped answering. Check its power and CAN cable.")
                self._group_stopped(j, f"Stopped because {j.name} stopped answering.")
            if not j.online:
                j.next_try = now + OFFLINE_RETRY_S
                j.problem = (self.adapter_problem or
                             f"{j.name} (ID {j.id}) isn't answering. Check its power and CAN cable.")

    def _received(self, j, result):
        v = result.values
        R = moteus.Register

        def get(reg):
            x = v.get(reg)
            return float(x) if x is not None else math.nan

        fault = v.get(R.FAULT)
        t = {"mode": int(v.get(R.MODE, -1)), "pos": get(R.POSITION), "vel": get(R.VELOCITY),
             "torque": get(R.TORQUE), "q": get(R.Q_CURRENT), "volt": get(R.VOLTAGE),
             "temp": get(R.TEMPERATURE), "fault": int(fault) if finite(fault) else 0,
             "cpos": get(R.CONTROL_POSITION), "cvel": get(R.CONTROL_VELOCITY)}
        with self.lock:
            j.t = t
            j.t_time = time.monotonic()
            j.misses = 0
            j.online = True
            j.problem = ""
            if j.settle > 0:
                j.settle -= 1
                return
            if not j.adopted:
                if j.limits_read:
                    self._adopt(j)
                return
            mode = t["mode"]
            if mode != 1:
                j.fault_reported = False
            if j.need_stop:
                return      # a reply that left before the latest request; the stop comes next
            if mode == 1:
                # The controller has already cut the motor. It stays limp until the next move.
                if not j.fault_reported:
                    j.fault_reported = True
                    self._notify(j, f"Fault {t['fault']}: {FAULT_TEXT.get(t['fault'], 'unknown fault')}. "
                                    "It's limp. Moving it again clears the fault.")
                    self._group_stopped(j, f"Stopped because {j.name} faulted.")
                j.want, j.target, j.nudge_until = "released", None, 0.0
            elif mode == 11 and j.want == "position":
                # Our commands arrived late, so the controller paused the move.
                self._hold(j, quick=False, at=t["pos"])
                j.need_stop = True
                self._notify(j, "Commands arrived late, so the controller paused the move. "
                                "It's holding where it is now.")
                self._group_stopped(j, f"Stopped because {j.name} paused.")
            elif j.want == "position" and j.sent == "position" and mode == 10:
                if (j.group is not None and j.nudge_until and finite(t["cvel"])
                        and abs(t["cvel"]) < 1e-3 and abs(j.target - t["pos"]) < AT_TARGET):
                    # Turning together, and this one reached the end of its range: stop both.
                    j.nudge_dir, j.nudge_until = 0, 0.0
                    self._notify(j, "Reached the end of its range, so both motors stopped.")
                    self._group_stopped(j, f"Stopped because {j.name} reached the end of its range.")
                self._check_stall(j)
                self._group_done(j)

    def _adopt(self, j):
        """First reply after connecting: take over without moving anything."""
        t = j.t
        j.adopted = True
        j.nudge_until = 0.0
        j.stall_since = None
        if t["mode"] in DRIVEN_MODES and finite(t["pos"]):
            note = ""
            if self._outside_limits(j):
                j.beyond = True
                note = " It's outside its limits, so moving past the limits is on for it."
            j.want, j.target, j.accel = "position", t["pos"], self.args.joint_accel
            if t["mode"] != 10:
                j.need_stop = True
            self._notify(j, f"Connected. It was already powered, so it's holding where it is "
                            f"({fmt_deg(t['pos'])}).{note}")
        elif t["mode"] == 1:
            j.want, j.target = "released", None
            j.fault_reported = True
            self._notify(j, f"Connected, but it has fault {t['fault']}: "
                            f"{FAULT_TEXT.get(t['fault'], 'unknown fault')}. It's limp. "
                            "Moving it clears the fault.")
        else:
            j.want, j.target = "released", None
            self._notify(j, "Connected. It's released (limp) until you move it.")

    def _check_stall(self, j):
        """Stuck short of the target: pushing at the torque cap without moving for STALL_SECONDS,
        or the move finished on the controller's side but the motor made no progress for
        STALL_NO_PROGRESS_S. Slow creeping (the integrator finishing a move) doesn't count."""
        t = j.t
        if not (finite(t["pos"]) and finite(t["vel"]) and finite(t["torque"])):
            return
        at_cap = t["fault"] == 102 or abs(t["torque"]) >= STALL_TORQUE_FRACTION * j.max_torque
        setpoint_done = (finite(t["cpos"]) and finite(t["cvel"]) and abs(t["cvel"]) < 1e-3
                         and abs(t["cpos"] - j.target) < 1e-3)
        stuck = (abs(j.target - t["pos"]) > STALL_MIN_ERROR and abs(t["vel"]) < STALL_MAX_SPEED
                 and (at_cap or setpoint_done))
        now = time.monotonic()
        if not stuck:
            j.stall_since = None
            return
        if j.stall_since is None or abs(t["pos"] - j.stall_pos) > STALL_PROGRESS:
            j.stall_since, j.stall_pos = now, t["pos"]      # (still) making progress
            return
        if now - j.stall_since >= (STALL_SECONDS if at_cap else STALL_NO_PROGRESS_S):
            target = j.target
            self._hold(j, quick=False, at=t["pos"])
            j.need_stop = True              # also clears what the integrator built up
            self._notify(j, f"Stopped pushing: it was stuck at {fmt_deg(t['pos'])} on the way to "
                            f"{fmt_deg(target)} with {abs(t['torque']):.1f} Nm, so something is in "
                            "the way or it reached a hard stop. It's holding there.")
            self._group_stopped(j, f"Stopped because {j.name} got stuck.")

    # ---------- settings on the controller (diagnostic channel) ----------

    async def _diag(self, j, text, value=False):
        if not j.stream_ready:
            await asyncio.wait_for(j.stream.write_message(b"tel stop"), DIAG_TIMEOUT_S)
            await j.stream.flush_read()
            j.stream_ready = True
        reply = await asyncio.wait_for(j.stream.command(text.encode(), allow_any_response=value),
                                       DIAG_TIMEOUT_S)
        if value:
            if reply.startswith(b"ERR"):
                raise RuntimeError(reply.decode("latin1", "replace"))
            return float(reply)
        return reply

    async def _read_limit_values(self, j):
        lo = await self._diag(j, "conf get servopos.position_min", value=True)
        hi = await self._diag(j, "conf get servopos.position_max", value=True)
        return lo, hi

    async def _read_limits(self, j):
        try:
            lo, hi = await self._read_limit_values(j)
            with self.lock:
                j.limits, j.limits_read, j.limits_failures = (lo, hi), True, 0
            log(f"{j.name}: limits {self._limits_text(j)}")
        except Exception as exc:
            j.stream_ready = False
            with self.lock:
                j.limits_failures += 1
                j.limits_next_try = time.monotonic() + 2.0
                if j.limits_failures >= 3:
                    j.limits, j.limits_read = (math.nan, math.nan), True
                    self._notify(j, f"Couldn't read its saved limits ({exc!r}), so only the "
                                    f"±{deg(self.args.travel_cap):.0f}° travel cap applies.")
        finally:
            j.limits_busy = False

    async def _save_limit(self, j, which, value):
        name = "servopos.position_min" if which == "lower" else "servopos.position_max"
        try:
            await self._diag(j, f"conf set {name} {value:.6f}")
            await self._diag(j, "conf write")
            lo, hi = await self._read_limit_values(j)
            with self.lock:
                j.limits, j.limits_read = (lo, hi), True
            return lo, hi
        except Exception:
            j.stream_ready = False
            raise
        finally:
            j.limits_busy = False

    # ---------- helpers (call with self.lock held) ----------

    def _notify(self, j, text):
        j.notice_id += 1
        j.notice = text
        j.notice_time = time.strftime("%H:%M:%S")
        log(f"{j.name}: {text}")

    def _limits_text(self, j):
        if not j.limits or not (finite(j.limits[0]) or finite(j.limits[1])):
            return "none saved"
        lo, hi = j.limits
        return (f"{fmt_deg(lo) if finite(lo) else 'no lower limit'} to "
                f"{fmt_deg(hi) if finite(hi) else 'no upper limit'}")

    def _range(self, j):
        """Where moves may go, in turns: the saved limits (unless past-limits is on), always
        inside the travel cap."""
        cap = self.args.travel_cap
        lo, hi = -cap, cap
        if not j.beyond and j.limits:
            if finite(j.limits[0]):
                lo = max(lo, j.limits[0])
            if finite(j.limits[1]):
                hi = min(hi, j.limits[1])
        return lo, hi

    def _clamp(self, j, turns):
        lo, hi = self._range(j)
        if lo <= turns <= hi:
            return turns, ""
        edge = lo if turns < lo else hi
        cap_edge = abs(abs(edge) - self.args.travel_cap) < 1e-9
        why = (f"the most this page sends it ({deg(self.args.travel_cap):.0f}° from zero)" if cap_edge
               else f"its {'lower' if edge == lo else 'upper'} limit")
        return edge, f"Limited to {fmt_deg(edge)}, {why}."

    def _outside_limits(self, j):
        if not j.limits:
            return False
        lo, hi = j.limits
        p = j.t.get("pos")
        if not finite(p):
            return False
        return ((finite(lo) and p < lo - LIMIT_TOLERANCE) or
                (finite(hi) and p > hi + LIMIT_TOLERANCE))

    def _is_moving(self, j):
        t = j.t
        if j.want != "position" or j.target is None or not finite(t.get("pos")):
            return False
        return (abs(j.target - t["pos"]) > AT_TARGET or
                (finite(t.get("vel")) and abs(t["vel"]) > STALL_MAX_SPEED) or
                (finite(t.get("cvel")) and abs(t["cvel"]) > 1e-3))

    def _stop_point(self, j):
        """Where it can stop smoothly from its current motion, without backing up."""
        t = j.t
        p, cp, cv = t.get("pos"), t.get("cpos"), t.get("cvel")
        if (j.want == "position" and finite(cp) and finite(cv) and finite(p)
                and abs(cp - p) < LAGGING):
            a = self.args.joint_stop_accel
            # The reading is a little old and the next command leaves within one cycle.
            ahead = cv * (time.monotonic() - j.t_time + 1.5 / CONTROL_HZ)
            return cp + ahead + math.copysign(cv * cv / (2 * a), cv)
        return p

    def _hold(self, j, quick, at=None):
        """Stop and hold. quick: decelerate with the stop accel from the current motion."""
        target = at if at is not None else (self._stop_point(j) if quick else j.t.get("pos"))
        if not finite(target):
            return
        j.want = "position"
        j.target = self._clamp(j, target)[0]
        j.accel = self.args.joint_stop_accel if quick else self.args.joint_accel
        j.nudge_dir, j.nudge_until = 0, 0.0
        j.stall_since = None

    def _begin_move(self, j, target):
        if j.t.get("mode") not in (0, 10):
            j.need_stop = True          # clears a fault or a pause first
        j.want, j.target = "position", target
        j.accel = self.args.joint_accel
        j.move_rpm = None
        j.stall_since = None

    def _base(self, j):
        """Where a step starts from: the current target, unless it's turning or released."""
        if j.want == "position" and j.target is not None and j.nudge_until <= time.monotonic():
            return j.target
        return j.t["pos"]

    def _sign(self, j, rot):
        """+1 or -1: which way the angle goes for "cw" or "ccw" on this motor."""
        return j.cw_sign if rot == "cw" else -j.cw_sign

    def _rot_of(self, j, d):
        return "CW" if d == j.cw_sign else "CCW"

    # ---------- moving both motors together ----------

    def _group_stopped(self, j, reason):
        """j stopped on its own (stuck, fault, limit...): stop the motor it was moving with."""
        gid = j.group
        if gid is None:
            return
        for k in self.joints:
            if k.group != gid:
                continue
            k.group = None
            if k is not j and k.online and self._is_moving(k):
                self._hold(k, quick=True)
                if reason:
                    self._notify(k, reason)

    def _group_done(self, j):
        """A step made together is over once neither motor is moving any more."""
        gid = j.group
        if gid is None or j.nudge_until or self._is_moving(j):
            return
        members = [k for k in self.joints if k.group == gid]
        if all(not k.nudge_until and not self._is_moving(k) for k in members):
            for k in members:
                k.group = None

    def _leave_group(self, j):
        """A separate command for j ends a move together; the other motor stops."""
        self._group_stopped(j, f"Stopped because {j.name} got its own command.")

    def _move_problem(self, j):
        if not j.online:
            return j.problem or f"{j.name} isn't answering."
        if not j.adopted:
            return "Still reading its settings. Try again in a moment."
        if not finite(j.t.get("pos")):
            return "Its position isn't valid yet."
        if not j.beyond and self._outside_limits(j):
            return (f"It's at {fmt_deg(j.t['pos'])}, outside its limits ({self._limits_text(j)}). "
                    "Turn on Allow moving past the limits to bring it back.")
        return ""

    def _check_page_contact(self):
        with self.lock:
            if time.time() - self.last_contact <= self.args.watchdog:
                self.contact_lost = False
                return
            if self.contact_lost:
                return
            self.contact_lost = True
            for j in self.joints:
                j.group = None
                if j.online and self._is_moving(j):
                    self._hold(j, quick=True)
                    self._notify(j, "Stopped because the control page lost contact. "
                                    "It's holding where it is.")

    # ---------- commands from the page (web server threads) ----------

    def command(self, joint_id, action, data):
        if action == "set_limit":
            return self._set_limit(joint_id, str(data.get("which", "")), float(data["deg"]))
        with self.lock:
            self.last_contact = time.time()
            j = self.by_id.get(joint_id)
            if j is None:
                return False, f"No motor with ID {joint_id} is set up here."
            if action == "speed":
                j.speed_rpm = clamp(float(data["rpm"]), 1.0, self.args.joint_max_rpm)
                return True, f"Speed set to {j.speed_rpm:.0f} RPM."
            if action == "torque":
                j.max_torque = clamp(float(data["nm"]), self.args.joint_min_torque,
                                     self.args.joint_max_torque)
                return True, f"Max torque set to {j.max_torque:.1f} Nm."
            if action == "cw_sign":
                sign = -1 if float(data["sign"]) < 0 else 1
                j.cw_sign = sign
                self.settings.setdefault("cw_sign", {})[str(j.id)] = sign
                where = "the angle goes up" if sign > 0 else "the angle goes down"
                try:
                    save_settings(self.settings_path, self.settings)
                except OSError as exc:
                    return True, (f"Set: turning CW, {where}. Couldn't save it for next time "
                                  f"({exc}), so it lasts until this program stops.")
                return True, f"Set: turning {j.name} CW, {where}. Saved for next time."
            if action == "release":
                self._leave_group(j)
                j.want, j.target = "released", None
                j.nudge_dir, j.nudge_until = 0, 0.0
                if j.online:
                    j.need_stop = True
                return True, "Released. The motor is limp."
            if action == "beyond":
                on = bool(data.get("on"))
                if not on and j.beyond and j.online and self._outside_limits(j):
                    return False, (f"It's at {fmt_deg(j.t['pos'])}, outside its limits "
                                   f"({self._limits_text(j)}). Bring it back inside first.")
                j.beyond = on
                if not on and j.want == "position" and j.target is not None:
                    j.target = self._clamp(j, j.target)[0]
                return True, ("Moving past the limits is on. Watch the joint: it can reach its hard "
                              "stops. It stops pushing on its own if it gets stuck." if on
                              else "Moves stay inside the limits again.")

            # Everything below moves the motor. Directions come as "rot": "cw"/"ccw" (or as a
            # signed "direction"/"deg").
            rot = str(data.get("rot", "")).lower()
            if action == "nudge" and data.get("renew"):
                # Only keeps a turn going. If it was stopped (stuck, Stop all), it stays stopped.
                d = (self._sign(j, rot) if rot in ("cw", "ccw")
                     else (1 if float(data.get("direction", 0)) > 0 else -1))
                now = time.monotonic()
                if j.group is not None or not (j.nudge_dir == d and j.nudge_until > now):
                    return False, ""
                j.nudge_until = now + NUDGE_LEASE_S
                return True, ""
            problem = self._move_problem(j)
            if problem:
                return False, problem
            if action in ("goto", "jog", "nudge", "hold"):
                self._leave_group(j)
            pos = j.t["pos"]
            if action == "goto":
                wanted = float(data["deg"]) / 360.0
                if not math.isfinite(wanted):
                    return False, "Enter an angle in degrees."
                target, note = self._clamp(j, wanted)
                self._begin_move(j, target)
                j.nudge_dir, j.nudge_until = 0, 0.0
                return True, note or f"Going to {fmt_deg(target)}."
            if action == "jog":
                size = float(data["deg"]) / 360.0
                step = abs(size) * self._sign(j, rot) if rot in ("cw", "ccw") else size
                if not math.isfinite(step) or step == 0:
                    return False, "Pick a step."
                base = self._base(j)
                target, note = self._clamp(j, base + step)
                if abs(target - base) < 1e-6:
                    return False, note or "It's already at the edge of its range."
                self._begin_move(j, target)
                j.nudge_dir, j.nudge_until = 0, 0.0
                return True, note or (f"Stepping {abs(deg(step)):.0f}° {self._rot_of(j, 1 if step > 0 else -1)} "
                                      f"to {fmt_deg(target)}.")
            if action == "nudge":
                d = (self._sign(j, rot) if rot in ("cw", "ccw")
                     else (1 if float(data.get("direction", 0)) > 0 else -1))
                lo, hi = self._range(j)
                edge = hi if d > 0 else lo
                if (d > 0 and pos >= hi - AT_TARGET) or (d < 0 and pos <= lo + AT_TARGET):
                    return False, self._clamp(j, edge + d)[1] or "It's at the edge of its range."
                self._begin_move(j, edge)
                j.nudge_dir, j.nudge_until = d, time.monotonic() + NUDGE_LEASE_S
                return True, f"Turning {self._rot_of(j, d)}. Let go to stop."
            if action == "hold":
                if j.want == "position":
                    self._hold(j, quick=True)
                else:
                    self._begin_move(j, pos)
                return True, f"Holding at about {fmt_deg(j.target)}."
            return False, f"Unknown action: {action}"

    def both_command(self, action, data):
        """Move the first two motors together. Both start and stop together; if either can't
        make the move, neither moves."""
        with self.lock:
            self.last_contact = time.time()
            if len(self.pair) < 2:
                return False, "Moving both needs two motors."
            if data.get("rpm") is not None:
                self.pair_rpm = clamp(float(data["rpm"]), 1.0, self.args.joint_max_rpm)
            if action == "speed":
                return True, f"Speed for moving both set to {self.pair_rpm:.0f} RPM."
            if action == "hold":
                for j in self.pair:
                    j.group = None
                    if j.online and j.want == "position":
                        self._hold(j, quick=True)
                return True, "Both holding."
            rots = data.get("rots") or {}
            moves = []
            for j in self.pair:
                rot = str(rots.get(str(j.id), "")).lower()
                if rot not in ("cw", "ccw"):
                    return False, f"Pick CW or CCW for {j.name}."
                moves.append((j, rot, self._sign(j, rot)))
            now = time.monotonic()
            if action == "nudge" and data.get("renew"):
                gid = self.pair[0].group
                if gid is None or not all(j.group == gid and j.nudge_dir == d and j.nudge_until > now
                                          for j, _, d in moves):
                    return False, ""
                for j, _, _ in moves:
                    j.nudge_until = now + NUDGE_LEASE_S
                return True, ""
            for j, _, _ in moves:
                problem = self._move_problem(j)
                if problem:
                    return False, f"{j.name}: {problem} Neither motor moved."
            what = " and ".join(f"{j.name} {rot.upper()}" for j, rot, _ in moves)
            if action == "nudge":
                for j, rot, d in moves:
                    lo, hi = self._range(j)
                    pos = j.t["pos"]
                    if (d > 0 and pos >= hi - AT_TARGET) or (d < 0 and pos <= lo + AT_TARGET):
                        return False, f"{j.name} can't go any further {rot.upper()}, so neither motor moved."
                plans = [(j, d, (self._range(j)[1] if d > 0 else self._range(j)[0])) for j, _, d in moves]
                message = f"Turning {what}. Let go to stop."
            elif action == "step":
                size = abs(float(data["deg"])) / 360.0
                if not 0 < size <= 0.25:
                    return False, "Pick a step from 0.1° to 90°."
                plans = []
                for j, rot, d in moves:
                    base = self._base(j)
                    wanted = base + d * size
                    target, note = self._clamp(j, wanted)
                    if abs(target - wanted) > 1e-6:
                        room = abs(deg(target - base))
                        return False, (f"{j.name} can only go {room:.1f}° further {rot.upper()}, "
                                       "so neither motor moved.")
                    plans.append((j, 0, target))
                message = f"Stepping {deg(size):g}° each: {what}."
            else:
                return False, f"Unknown action: {action}"
            gid = self._next_group
            self._next_group += 1
            for j, d, target in plans:
                self._begin_move(j, target)
                j.group, j.move_rpm = gid, self.pair_rpm
                j.nudge_dir, j.nudge_until = (d, now + NUDGE_LEASE_S) if d else (0, 0.0)
            return True, message

    def pair_status(self):
        if len(self.pair) < 2:
            return None
        moving = any(j.group is not None and self._is_moving(j) for j in self.pair)
        return {"ids": [j.id for j in self.pair], "names": [j.name for j in self.pair],
                "rpm": self.pair_rpm, "moving": moving,
                "ready": all(j.online and j.adopted for j in self.pair)}

    def _set_limit(self, joint_id, which, wanted_deg):
        with self.lock:
            self.last_contact = time.time()
            j = self.by_id.get(joint_id)
            if j is None:
                return False, f"No motor with ID {joint_id} is set up here."
            if which not in ("lower", "upper"):
                return False, "Pick the lower or the upper limit."
            if not j.online or not j.adopted or not finite(j.t.get("pos")):
                return False, j.problem or "It isn't ready yet."
            value = wanted_deg / 360.0
            pos = j.t["pos"]
            if not math.isfinite(value) or abs(value - pos) > 0.003:
                return False, (f"It's at {fmt_deg(pos)} now, not {wanted_deg:+.1f}°. "
                               "It moved since you clicked. Try again.")
            if abs(value) > 0.49:
                return False, ("Limits must be within ±176° of zero: past half a turn, the encoder "
                               "reads a full turn off after a power-up.")
            lo, hi = j.limits if j.limits else (math.nan, math.nan)
            if which == "lower" and finite(hi) and value > hi - 0.01:
                return False, f"The lower limit has to be at least 3.6° below the upper limit ({fmt_deg(hi)})."
            if which == "upper" and finite(lo) and value < lo + 0.01:
                return False, f"The upper limit has to be at least 3.6° above the lower limit ({fmt_deg(lo)})."
            if j.limits_busy:
                return False, "It's busy reading its settings. Try again in a moment."
            j.limits_busy = True
        future = asyncio.run_coroutine_threadsafe(self._save_limit(j, which, value), self.loop)
        try:
            lo, hi = future.result(timeout=12)
        except Exception as exc:
            with self.lock:
                j.limits_busy = False
            log(f"{j.name}: saving the {which} limit failed: {exc!r}")
            return False, f"The controller didn't save it ({exc}). The old limits are still in place."
        with self.lock:
            self._notify(j, f"Saved {fmt_deg(value)} as its {which} limit. Limits are now "
                            f"{self._limits_text(j)}.")
        return True, f"Saved. Limits are now {self._limits_text(j)}."

    def stop_all(self):
        with self.lock:
            self.last_contact = time.time()
            for j in self.joints:
                j.group = None
                if j.online and j.want == "position":
                    self._hold(j, quick=True)
                j.nudge_dir, j.nudge_until = 0, 0.0

    # ---------- page data ----------

    def status(self):
        with self.lock:
            self.last_contact = time.time()
            return [self._joint_status(j) for j in self.joints], self.pair_status()

    def _joint_status(self, j):
        t = j.t if j.online else {}
        mode = t.get("mode")
        if not j.online:
            state = "offline"
        elif not j.adopted:
            state = "connecting"
        elif mode == 1:
            state = "fault"
        elif j.want == "position":
            state = "moving" if self._is_moving(j) else "holding"
        else:
            state = "released"
        fault = t.get("fault") or 0
        lo, hi = j.limits if j.limits else (math.nan, math.nan)
        pos = t.get("pos")
        return {
            "id": j.id, "name": j.name, "online": j.online, "ready": j.online and j.adopted,
            "problem": j.problem, "state": state, "mode": mode,
            "fault": fault, "fault_text": FAULT_TEXT.get(fault) or LIMIT_TEXT.get(fault, ""),
            "position_deg": num(deg(pos), 2) if finite(pos) else None,
            "target_deg": num(deg(j.target), 2) if (j.want == "position" and finite(j.target)) else None,
            "speed_rpm_now": num(t.get("vel", math.nan) * 60.0, 1) if t else None,
            "torque_nm": num(t.get("torque"), 2), "current_a": num(t.get("q"), 2),
            "voltage_v": num(t.get("volt"), 1), "temp_c": num(t.get("temp"), 0),
            "limits_read": j.limits_read,
            "limits_deg": [num(deg(lo), 2), num(deg(hi), 2)] if (finite(lo) or finite(hi)) else None,
            "beyond": j.beyond, "speed_rpm": j.speed_rpm, "max_torque": j.max_torque,
            "cw_sign": j.cw_sign, "together": j.group is not None,
            "outside_limits": j.online and self._outside_limits(j),
            "past_half_turn": finite(pos) and abs(pos) > 0.5,
            "nudging": j.nudge_until > time.monotonic(),
            "notice": j.notice, "notice_id": j.notice_id, "notice_time": j.notice_time,
        }


# ---------------------------------------------------------------------------
# Web server
# ---------------------------------------------------------------------------

class Robot:
    """The page's view of everything: the motors, the gripper, and why either is missing."""

    def __init__(self, args, manager, gripper, gripper_problem, moteus_problem):
        self.args = args
        self.manager = manager
        self.gripper = gripper
        self.gripper_problem = gripper_problem
        self.moteus_problem = moteus_problem

    def status(self):
        a = self.args
        joints, pair = self.manager.status() if self.manager else ([], None)
        return {
            "joints": joints,
            "pair": pair,
            "joints_problem": self.moteus_problem,
            "joint_limits": {"min_rpm": 1, "max_rpm": a.joint_max_rpm,
                             "min_torque": a.joint_min_torque, "max_torque": a.joint_max_torque,
                             "travel_cap_deg": round(deg(a.travel_cap), 1)},
            "gripper": self.gripper.status() if self.gripper else None,
            "gripper_problem": self.gripper_problem,
            "watchdog_s": a.watchdog,
        }

    def command(self, data):
        device = str(data.get("device", ""))
        action = str(data.get("action", ""))
        if device == "all" and action == "stop":
            if self.manager:
                self.manager.stop_all()
            if self.gripper:
                self.gripper.command("stop", {})
            log("Stop all")
            return True, "Stopped everything. The motors are holding where they are."
        if device == "joint":
            if not self.manager:
                return False, self.moteus_problem or "Motor control is off."
            return self.manager.command(int(data["id"]), action, data)
        if device == "both":
            if not self.manager:
                return False, self.moteus_problem or "Motor control is off."
            return self.manager.both_command(action, data)
        if device == "gripper":
            if not self.gripper:
                return False, self.gripper_problem or "The gripper is off."
            return self.gripper.command(action, data)
        return False, f"Unknown device: {device}"


def make_handler(robot, page, allowed_hosts):
    class Handler(BaseHTTPRequestHandler):
        server_version = "MasterCtrl/1.0"

        def log_message(self, fmt, *args):
            pass                            # commands are logged by the controllers instead

        def _allowed(self):
            # Refuse requests from other websites open in the same browser.
            host = self.headers.get("Host", "")
            name = urlsplit("//" + host).hostname or ""
            try:
                ipaddress.ip_address(name)
                is_ip = True
            except ValueError:
                is_ip = False
            if not (is_ip or name in allowed_hosts):
                return False
            origin = self.headers.get("Origin")
            return origin is None or urlsplit(origin).netloc == host

        def _send(self, code, body, content_type="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not self._allowed():
                return self._send(403, {"ok": False, "message": "Blocked: unexpected address or origin."})
            path = urlsplit(self.path).path
            if path in ("/", "/index.html"):
                return self._send(200, page, "text/html; charset=utf-8")
            if path == "/api/status":
                return self._send(200, robot.status())
            self._send(404, {"ok": False, "message": "Not found."})

        def do_POST(self):
            if not self._allowed():
                return self._send(403, {"ok": False, "message": "Blocked: unexpected address or origin."})
            if urlsplit(self.path).path != "/api/command":
                return self._send(404, {"ok": False, "message": "Not found."})
            if not self.headers.get("Content-Type", "").startswith("application/json"):
                return self._send(415, {"ok": False, "message": "Send JSON."})
            # 200 for any command that was understood (the reply's "ok" says whether it was
            # carried out); 400 only for malformed requests.
            code = 200
            try:
                length = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(data, dict):
                    raise TypeError("expected a JSON object")
                ok, message = robot.command(data)
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                code, ok, message = 400, False, f"Bad request: {exc}"
            except Exception as exc:
                log(f"Error handling a command: {exc!r}")
                code, ok, message = 500, False, f"Error: {exc}"
            self._send(code, {"ok": ok, "message": message, "status": robot.status()})

    return Handler


def parse_names(values):
    names = {}
    for item in values:
        key, sep, name = item.partition("=")
        if not sep or not key.strip().isdigit() or not name.strip():
            raise argparse.ArgumentTypeError(f"--name wants ID=NAME, like 2=Wrist (got {item!r})")
        names[int(key)] = name.strip()
    return names


def load_settings(path):
    """Settings changed from the page (which way is CW for each motor)."""
    try:
        with open(path) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except FileNotFoundError:
        return {}
    except Exception as exc:
        log(f"Couldn't read {path} ({exc}). Starting with default settings.")
        return {}


def save_settings(path, data):
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Web control page for the two moteus motors and the ST-series gripper servo.")
    p.add_argument("--host", default="127.0.0.1",
                   help="address to serve on (default 127.0.0.1: reachable only through an SSH tunnel)")
    p.add_argument("--http-port", type=int, default=8080, help="web page port (default 8080)")
    p.add_argument("--allow-host", action="append", default=[],
                   help="extra host name allowed in the browser's address bar (IP addresses are always allowed)")
    p.add_argument("--watchdog", type=float, default=2.0,
                   help="stop moving things if the page hasn't checked in for this many seconds (default 2)")

    m = p.add_argument_group("motors (moteus)")
    m.add_argument("--moteus-ids", default="1,2", help="CAN IDs to control, comma separated (default 1,2)")
    m.add_argument("--no-moteus", action="store_true", help="run without the motors")
    m.add_argument("--name", action="append", default=[], metavar="ID=NAME",
                   help="name shown for a motor instead of 'ID n', e.g. 2=Wrist")
    m.add_argument("--settings", default=str(Path(__file__).resolve().with_name("masterCtrl_settings.json")),
                   help="file that remembers which way is CW for each motor "
                        "(default: masterCtrl_settings.json next to this file)")
    m.add_argument("--joint-default-rpm", type=float, default=10,
                   help="move speed at startup, output-shaft RPM (default 10)")
    m.add_argument("--joint-max-rpm", type=float, default=60, help="fastest move speed, RPM (default 60)")
    m.add_argument("--joint-accel", type=float, default=1.0,
                   help="acceleration for moves, turns/s^2 (default 1.0)")
    m.add_argument("--joint-stop-accel", type=float, default=3.0,
                   help="deceleration when stopping, turns/s^2 (default 3.0)")
    m.add_argument("--joint-default-torque", type=float, default=4.0,
                   help="max torque at startup, Nm at the output (default 4.0)")
    m.add_argument("--joint-min-torque", type=float, default=0.5, help="lowest max-torque setting, Nm")
    m.add_argument("--joint-max-torque", type=float, default=4.5,
                   help="highest max-torque setting, Nm (default 4.5; 1.66 A is about 4.9 Nm)")
    m.add_argument("--travel-cap", type=float, default=0.5,
                   help="furthest from zero the page sends a motor, even past its limits, in turns "
                        "(default 0.5 = 180°)")
    if moteus is not None:
        moteus.make_transport_args(p.add_argument_group("CAN adapter (default: found like tview finds it)"))

    g = p.add_argument_group("gripper (Waveshare / Feetech ST servo)")
    g.add_argument("--no-gripper", action="store_true", help="run without the gripper")
    g.add_argument("--serial", help="gripper adapter port, e.g. /dev/ttyACM1 (found automatically if omitted)")
    g.add_argument("--servo-id", type=int, default=1, help="gripper servo ID (factory default 1)")
    g.add_argument("--grip-min-rpm", type=float, default=10)
    g.add_argument("--grip-max-rpm", type=float, default=60)
    g.add_argument("--grip-default-rpm", type=float, default=20)
    g.add_argument("--grip-accel", type=int, default=20,
                   help="ramp for starting and speed changes, units of 8.7 deg/s^2 (1-254, default 20)")
    g.add_argument("--grip-stop-accel", type=int, default=50,
                   help="ramp for stopping, units of 8.7 deg/s^2 (1-254, default 50)")
    g.add_argument("--grip-min-strength", type=int, default=5, help="lowest grip strength, percent")
    g.add_argument("--grip-normal-strength", type=int, default=50, help="grip cap without high grip, percent")
    g.add_argument("--grip-high-strength", type=int, default=100, help="grip cap with high grip, percent")
    g.add_argument("--grip-default-strength", type=int, default=30, help="grip strength at startup, percent")
    g.add_argument("--grip-hot-temp", type=int, default=65, help="stop at this temperature in C (default 65)")
    g.add_argument("--grip-cool-temp", type=int, default=58, help="allow running again at this temperature in C")
    g.add_argument("--grip-invert", action="store_true", help="swap which way counts as forward")
    a = p.parse_args(argv)

    try:
        a.moteus_ids = [int(x) for x in a.moteus_ids.split(",") if x.strip()]
    except ValueError:
        p.error("--moteus-ids wants numbers like 1,2")
    if any(not 1 <= i <= 126 for i in a.moteus_ids):
        p.error("moteus CAN IDs are 1-126")
    try:
        a.names = parse_names(a.name)
    except argparse.ArgumentTypeError as exc:
        p.error(str(exc))
    if not 1 <= a.joint_default_rpm <= a.joint_max_rpm:
        p.error("speeds must satisfy 1 <= --joint-default-rpm <= --joint-max-rpm")
    if a.joint_accel <= 0 or a.joint_stop_accel <= 0:
        p.error("--joint-accel and --joint-stop-accel must be above 0")
    if not 0 < a.joint_min_torque <= a.joint_default_torque <= a.joint_max_torque:
        p.error("torques must satisfy 0 < --joint-min-torque <= --joint-default-torque <= --joint-max-torque")
    if not 0 < a.travel_cap <= 2:
        p.error("--travel-cap must be above 0 and at most 2 turns")
    if not 1 <= a.grip_accel <= 254 or not 1 <= a.grip_stop_accel <= 254:
        p.error("--grip-accel and --grip-stop-accel must be 1-254 (0 would mean maximum acceleration)")
    if not 0 < a.grip_min_rpm <= a.grip_default_rpm <= a.grip_max_rpm:
        p.error("speeds must satisfy 0 < --grip-min-rpm <= --grip-default-rpm <= --grip-max-rpm")
    if not 1 <= a.grip_min_strength <= a.grip_normal_strength <= a.grip_high_strength <= 100:
        p.error("strengths must satisfy 1 <= --grip-min-strength <= --grip-normal-strength "
                "<= --grip-high-strength <= 100")
    if not a.grip_min_strength <= a.grip_default_strength <= a.grip_high_strength:
        p.error("--grip-default-strength must be between --grip-min-strength and --grip-high-strength")
    if a.grip_cool_temp >= a.grip_hot_temp:
        p.error("--grip-cool-temp must be below --grip-hot-temp")
    if a.watchdog <= 0.5:
        p.error("--watchdog must be more than 0.5 seconds")
    return a


def gripper_args(a):
    """gripCtrl.py's option names, from this program's --grip-* options."""
    return SimpleNamespace(
        serial=a.serial, id=a.servo_id, min_rpm=a.grip_min_rpm, max_rpm=a.grip_max_rpm,
        default_rpm=a.grip_default_rpm, accel=a.grip_accel, stop_accel=a.grip_stop_accel,
        min_strength=a.grip_min_strength, normal_strength=a.grip_normal_strength,
        high_strength=a.grip_high_strength, default_strength=a.grip_default_strength,
        watchdog=a.watchdog, hot_temp=a.grip_hot_temp, cool_temp=a.grip_cool_temp,
        invert=a.grip_invert)


def main(argv=None, transport_factory=None):
    args = parse_args(argv)
    if os.geteuid() == 0:
        log("Note: running as root. Run it as your normal user if moteus or the serial ports "
            "aren't found.")

    manager, moteus_problem = None, None
    if args.no_moteus or not args.moteus_ids:
        moteus_problem = "Motor control is off (--no-moteus)."
    elif moteus is None:
        moteus_problem = (f"The moteus Python library isn't installed for this Python ({MOTEUS_ERROR}). "
                          "Install it with: pip3 install moteus")
        log(moteus_problem)
    else:
        manager = MoteusManager(args, transport_factory)

    gripper, gripper_problem = None, None
    if args.no_gripper:
        gripper_problem = "The gripper is off (--no-gripper)."
    elif SCSERVO_ERROR is not None:
        gripper_problem = (f"The scservo_sdk folder isn't next to this file ({SCSERVO_ERROR}). Copy it with: "
                           f"cp -r ~/FTServo_Python/scservo_sdk {Path(__file__).resolve().parent}/")
        log(gripper_problem)
    else:
        gripper = GripperController(gripper_args(args), can_ports=getattr(args, "fdcanusb", None) or ())

    robot = Robot(args, manager, gripper, gripper_problem, moteus_problem)
    allowed = {"localhost", "127.0.0.1", "::1"} | set(args.allow_host)
    try:
        server = ThreadingHTTPServer((args.host, args.http_port),
                                     make_handler(robot, PAGE.encode(), allowed))
    except OSError as exc:
        sys.exit(f"Could not serve on {args.host}:{args.http_port}: {exc}. "
                 "Is masterCtrl.py or gripCtrl.py already running?")
    server.daemon_threads = True

    stop_event = threading.Event()

    def gripper_monitor():
        # This loop runs the gripper's watchdog and temperature checks, so it must never die.
        while not stop_event.is_set():
            try:
                gripper.poll()
            except Exception as exc:
                log(f"Gripper monitor error (continuing): {exc!r}")
            stop_event.wait(GRIP_POLL_SECONDS)

    def on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
    if manager:
        manager.start()
    if gripper:
        threading.Thread(target=gripper_monitor, name="gripper", daemon=True).start()

    log(f"Serving the control page on {args.host}:{args.http_port}. Press Ctrl+C to stop.")
    if args.host in ("127.0.0.1", "localhost", "::1"):
        log(f"On your laptop run: ssh -L {args.http_port}:localhost:{args.http_port} jetson")
        log(f"Then open http://localhost:{args.http_port}")
    else:
        log("Warning: anyone who can reach this address can move the robot.")
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print()
    finally:
        stop_event.set()
        if gripper:
            gripper.shutdown()
        if manager:
            manager.shutdown()
        server.server_close()
        log("Control page closed.")


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot control</title>
<link rel="icon" href="data:,">
<style>
:root {
  --panel: #d9dee0; --raised: #eef1f2; --ink: #1d2427; --ink-soft: #55616a; --line: #b6bfc3;
  --window: #162024; --glow: #bfe9da; --glow-dim: #6f8f87; --glow-faint: #2c3d3a;
  --run: #2e5e86; --run-ink: #f2f6f8; --stop: #c3362b; --stop-ink: #ffffff;
  --caution: #b97c06; --caution-soft: #f3e3bd; --good: #2f8a57; --bad: #c3362b;
  color-scheme: light;
}
@media (prefers-color-scheme: dark) {
  :root {
    --panel: #1b2225; --raised: #242d31; --ink: #e3e8ea; --ink-soft: #9aa7ae; --line: #3a464b;
    --window: #0e1517; --glow: #a9e6d2; --glow-dim: #5f7d76; --glow-faint: #22302d;
    --run: #4a83b3; --run-ink: #0e1517; --stop: #d4473b; --caution: #e0a13a; --caution-soft: #3b3220;
    --good: #4cbf7f; --bad: #e0574b;
    color-scheme: dark;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--panel); color: var(--ink);
  font: 16px/1.45 system-ui, -apple-system, "Segoe UI", "Noto Sans", Roboto, sans-serif; }
main { max-width: 1340px; margin: 0 auto; padding: 0 16px 40px; }
button, input { font: inherit; color: inherit; }
button { cursor: pointer; }
button:disabled { opacity: .45; cursor: not-allowed; }

.top { position: sticky; top: 0; z-index: 10; background: var(--panel); display: flex;
  justify-content: space-between; align-items: center; gap: 16px; padding: 14px 0 12px;
  border-bottom: 1px solid var(--line); margin-bottom: 18px; }
h1 { font-size: 24px; line-height: 1.2; margin: 0; font-weight: 650; letter-spacing: -0.01em; }
.sub { margin: 3px 0 0; color: var(--ink-soft); font-size: 14px; }
.topright { display: flex; align-items: center; gap: 18px; }
.link { margin: 0; display: flex; align-items: center; gap: 8px; font-size: 14px; white-space: nowrap; }
.led { width: 10px; height: 10px; border-radius: 50%; background: var(--ink-soft); flex: none; }
.link[data-state="ok"] .led { background: var(--good); box-shadow: 0 0 0 3px color-mix(in srgb, var(--good) 25%, transparent); }
.link[data-state="warn"] .led { background: var(--caution); }
.link[data-state="bad"] .led { background: var(--bad); }
.stopall { height: 52px; padding: 0 28px; border-radius: 26px; border: none; background: var(--stop);
  color: var(--stop-ink); font-size: 18px; font-weight: 700; white-space: nowrap;
  box-shadow: 0 0 0 4px var(--raised), 0 0 0 6px var(--line), 0 4px 10px rgba(0,0,0,.2); }
.stopall:active { transform: translateY(2px); }
#gmsg { margin: -6px 2px 14px; min-height: 1.45em; font-size: 15px; }

.banner { margin: 0; padding: 9px 12px; border-radius: 10px; border-left: 4px solid var(--caution);
  background: var(--caution-soft); font-size: 14.5px; }
.banner[data-tone="bad"] { border-left-color: var(--bad); }
#banner { margin-bottom: 16px; }

.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); gap: 20px; align-items: start; }
.panel { background: var(--raised); border-radius: 16px; padding: 16px; display: grid; gap: 14px;
  border: 2px solid transparent; min-width: 0; }
.panel.caution { border-color: var(--caution); }
.phead { display: flex; justify-content: space-between; align-items: flex-start; gap: 12px; }
h2 { font-size: 19px; line-height: 1.25; margin: 0; font-weight: 650; overflow-wrap: anywhere; }
.pill { font-size: 13px; font-weight: 650; padding: 3px 10px; border-radius: 999px; white-space: nowrap;
  background: color-mix(in srgb, var(--ink-soft) 18%, var(--raised)); color: var(--ink); }
.pill[data-state="holding"] { background: color-mix(in srgb, var(--good) 22%, var(--raised)); }
.pill[data-state="moving"], .pill[data-state="running"] { background: var(--run); color: var(--run-ink); }
.pill[data-state="fault"], .pill[data-state="offline"] { background: color-mix(in srgb, var(--bad) 22%, var(--raised)); }

.window { background: var(--window); color: var(--glow); border-radius: 12px; padding: 14px;
  display: grid; grid-template-columns: 100px 1fr; gap: 16px; align-items: center;
  box-shadow: inset 0 2px 8px rgba(0,0,0,.55); }
.dial { width: 100px; height: 100px; display: block; }
.dial .ring, .dial .tick { stroke: var(--glow-dim); fill: none; }
.dial .range { stroke: var(--glow); stroke-width: 5; fill: none; stroke-linecap: round; opacity: .55; }
.panel.caution .dial .range { stroke: var(--caution); stroke-dasharray: 4 5; opacity: .9; }
.dial .tgt { fill: var(--caution); }
.dial .needle { stroke: var(--glow); stroke-width: 3; stroke-linecap: round; }
.dial .hub { fill: var(--glow); }
dl { margin: 0; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px 14px; }
dt { font-size: 12.5px; color: var(--glow-dim); }
dd { margin: 0; font-size: 21px; line-height: 1.15; font-variant-numeric: tabular-nums; white-space: nowrap; }
dd small { font-size: 13px; color: var(--glow-dim); margin-left: 2px; }
.window.idle dd { color: var(--glow-dim); }

.block { display: grid; gap: 10px; padding-top: 12px; border-top: 1px solid var(--line); }
.block.flush { border-top: none; padding: 6px 0 4px; }
#gBody { display: grid; gap: 14px; }
.label { font-weight: 600; font-size: 15px; }
.hint { margin: 0; font-size: 13.5px; color: var(--ink-soft); }
.nudgerow { display: grid; grid-template-columns: 1.35fr repeat(4, 1fr) 1.35fr; gap: 6px; }
.rot { font-size: 1.15em; line-height: 1; }
.nudge { display: flex; align-items: center; justify-content: center; gap: 5px; font-size: 15px; }
.nudge .rot { font-size: 20px; }

.panel.both { grid-column: 1 / -1; }
.combos { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
.combo { min-height: 74px; border-radius: 14px; border: 2px solid var(--run); background: transparent;
  display: grid; gap: 6px; align-content: center; justify-items: start; padding: 10px 16px;
  touch-action: none; user-select: none; -webkit-user-select: none; }
.combo:hover { background: color-mix(in srgb, var(--run) 10%, transparent); }
.combo .cl { display: flex; align-items: center; gap: 8px; font-size: 16px; }
.combo .who { color: var(--ink-soft); font-size: 14px; font-weight: 600; min-width: 36px; text-align: left; }
.combo .rot { color: var(--run); font-size: 19px; }
.combo b { color: var(--run); font-weight: 700; }
.combo[aria-pressed="true"] { background: var(--run); }
.combo[aria-pressed="true"] .who, .combo[aria-pressed="true"] .rot, .combo[aria-pressed="true"] b { color: var(--run-ink); }
.bopts { display: flex; flex-wrap: wrap; align-items: center; gap: 12px 22px; }
.bopts .field { height: 38px; }
.bopts .field input { width: 56px; }
.bopts .field span { white-space: nowrap; }
.bspeed { flex: 1 1 320px; grid-template-columns: auto 1fr 70px; max-width: 520px; }
.seg { display: inline-flex; border: 1.5px solid var(--line); border-radius: 10px; overflow: hidden; background: var(--panel); }
.seg label { display: flex; position: relative; }
.seg input { position: absolute; opacity: 0; width: 1px; height: 1px; margin: 0; }
.seg span { padding: 7px 13px; font-size: 14px; font-weight: 600; cursor: pointer; white-space: nowrap; }
.seg label + label span { border-left: 1.5px solid var(--line); }
.seg input:checked + span { background: var(--run); color: var(--run-ink); }
.seg input:focus-visible + span { outline: 3px solid var(--run); outline-offset: -3px; }
.cwrow { display: flex; align-items: center; gap: 12px; }
.field.dim { opacity: .5; }
.cwrow .what { font-size: 15px; }
.nudge { height: 54px; border-radius: 12px; border: 2px solid var(--run); background: transparent;
  color: var(--run); font-size: 19px; font-weight: 700; touch-action: none; user-select: none;
  -webkit-user-select: none; }
.nudge:hover { background: color-mix(in srgb, var(--run) 10%, transparent); }
.nudge[aria-pressed="true"] { background: var(--run); color: var(--run-ink); }
.step { height: 54px; border-radius: 10px; border: 1.5px solid var(--line); background: var(--panel);
  font-weight: 650; font-size: 15px; font-variant-numeric: tabular-nums; padding: 0; }
.step:hover { border-color: var(--ink-soft); }
.row { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; }
.goto { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.goto label { font-weight: 600; }
.field { display: flex; align-items: center; border: 1.5px solid var(--line); border-radius: 10px;
  background: var(--panel); padding: 0 10px; height: 42px; }
.field input { width: 84px; border: none; background: transparent; font-size: 17px;
  font-variant-numeric: tabular-nums; outline: none; text-align: right; padding: 0 4px 0 0; }
.field:focus-within { outline: 3px solid var(--run); outline-offset: 2px; }
.go { height: 42px; padding: 0 18px; border-radius: 10px; border: none; background: var(--run);
  color: var(--run-ink); font-weight: 650; }
.quiet { height: 40px; padding: 0 14px; border-radius: 10px; border: 1.5px solid var(--ink-soft);
  background: transparent; font-size: 15px; font-weight: 600; }
.quiet:hover { border-color: var(--ink); }
.quiet.small { height: 34px; padding: 0 11px; font-size: 14px; }

.setting { display: grid; grid-template-columns: 98px 1fr 70px; gap: 4px 12px; align-items: center; }
.setting > label { font-weight: 600; font-size: 15px; }
output { text-align: right; font-variant-numeric: tabular-nums; font-weight: 650; }
.setting .hint, .setting .check { grid-column: 2 / 4; }
input[type="range"] { width: 100%; accent-color: var(--run); margin: 0; }
input[type="range"].hot { accent-color: var(--caution); }
.check { display: flex; align-items: center; gap: 8px; font-size: 15px; }
.check input { width: 18px; height: 18px; margin: 0; accent-color: var(--run); flex: none; }
.panel.caution .beyondCheck { font-weight: 650; }
.limits { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 10px; }
.limits .what { flex-basis: 100%; font-size: 15px; }
.limits b { font-variant-numeric: tabular-nums; }
.confirm { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 10px; padding: 10px 12px;
  border-radius: 10px; background: var(--caution-soft); border-left: 4px solid var(--caution); font-size: 14.5px; }
.confirm span { margin-right: auto; }
.save { height: 34px; padding: 0 14px; border-radius: 10px; border: none; background: var(--caution);
  color: #1d1406; font-weight: 700; font-size: 14px; }
.msg { min-height: 1.45em; margin: 0; font-size: 14.5px; }
.msg[data-tone="bad"] { color: var(--bad); }

.drive { display: grid; grid-template-columns: 1fr auto 1fr; gap: 12px; align-items: center; }
.dir { height: 66px; border-radius: 14px; border: 2px solid var(--run); background: transparent; color: var(--run);
  font-size: 17px; font-weight: 650; touch-action: none; user-select: none; -webkit-user-select: none;
  display: flex; align-items: center; justify-content: center; gap: 8px; }
.dir:hover { background: color-mix(in srgb, var(--run) 10%, transparent); }
.dir[aria-pressed="true"] { background: var(--run); color: var(--run-ink); }
.glyph { font-size: 14px; }
.stop { width: 92px; height: 92px; border-radius: 50%; border: none; background: var(--stop);
  color: var(--stop-ink); font-size: 19px; font-weight: 700;
  box-shadow: 0 0 0 6px var(--raised), 0 0 0 8px var(--line), 0 5px 12px rgba(0,0,0,.25); }
.stop:active { transform: translateY(2px); }
.off { color: var(--ink-soft); font-size: 15px; margin: 0; }

.foot { margin: 22px 2px 0; font-size: 14px; color: var(--ink-soft); }
:focus-visible { outline: 3px solid var(--run); outline-offset: 3px; }
.stop:focus-visible, .stopall:focus-visible { outline-color: var(--ink); }
[hidden] { display: none !important; }

@media (max-width: 900px) {
  .combos { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}
@media (max-width: 560px) {
  .top { flex-wrap: wrap; }
  .topright { width: 100%; justify-content: space-between; }
  .stopall { height: 48px; padding: 0 22px; }
  .window { grid-template-columns: 84px 1fr; gap: 12px; padding: 12px; }
  .dial { width: 84px; height: 84px; }
  dd { font-size: 18px; }
  .setting { grid-template-columns: 1fr 64px; }
  .setting > label { grid-column: 1 / 3; }
  .setting .hint, .setting .check { grid-column: 1 / 3; }
  .nudgerow { grid-template-columns: repeat(6, minmax(0, 1fr)); gap: 5px; }
  .nudge { flex-direction: column; gap: 0; font-size: 12.5px; }
  .nudge .rot { font-size: 17px; }
  .step { font-size: 13px; }
  .combo { padding: 10px 12px; }
  .combo .cl { font-size: 15px; gap: 6px; }
  .bspeed { grid-template-columns: 1fr 64px; flex-basis: 100%; }
  .dir { height: 58px; font-size: 15px; }
  .stop { width: 78px; height: 78px; font-size: 17px; }
}
@media (prefers-reduced-motion: reduce) { * { transition: none !important; } }
</style>
</head>
<body>
<main>
  <header class="top">
    <div>
      <h1>Robot control</h1>
      <p class="sub" id="sub">Connecting to the Jetson.</p>
    </div>
    <div class="topright">
      <p class="link" id="link" data-state="warn"><span class="led"></span><span id="linkText">Connecting</span></p>
      <button class="stopall" id="stopAll">Stop all</button>
    </div>
  </header>
  <p id="gmsg" aria-live="polite"></p>
  <p class="banner" id="banner" data-tone="bad" role="alert" hidden></p>

  <div class="grid" id="grid">
    <section class="panel both" id="both" aria-labelledby="bTitle" hidden>
      <div class="phead">
        <div>
          <h2 id="bTitle">Both motors</h2>
          <p class="sub" id="bSub">They start and stop together.</p>
        </div>
        <span class="pill" id="bPill" data-state="offline">Not ready</span>
      </div>
      <div class="combos" role="group" aria-label="Directions for both motors">
        <button class="combo" data-r1="ccw" data-r2="ccw" aria-pressed="false">
          <span class="cl"><span class="who">ID 1</span><span class="rot" aria-hidden="true">↺</span><b>CCW</b></span>
          <span class="cl"><span class="who">ID 2</span><span class="rot" aria-hidden="true">↺</span><b>CCW</b></span>
        </button>
        <button class="combo" data-r1="ccw" data-r2="cw" aria-pressed="false">
          <span class="cl"><span class="who">ID 1</span><span class="rot" aria-hidden="true">↺</span><b>CCW</b></span>
          <span class="cl"><span class="who">ID 2</span><span class="rot" aria-hidden="true">↻</span><b>CW</b></span>
        </button>
        <button class="combo" data-r1="cw" data-r2="ccw" aria-pressed="false">
          <span class="cl"><span class="who">ID 1</span><span class="rot" aria-hidden="true">↻</span><b>CW</b></span>
          <span class="cl"><span class="who">ID 2</span><span class="rot" aria-hidden="true">↺</span><b>CCW</b></span>
        </button>
        <button class="combo" data-r1="cw" data-r2="cw" aria-pressed="false">
          <span class="cl"><span class="who">ID 1</span><span class="rot" aria-hidden="true">↻</span><b>CW</b></span>
          <span class="cl"><span class="who">ID 2</span><span class="rot" aria-hidden="true">↻</span><b>CW</b></span>
        </button>
      </div>
      <div class="bopts">
        <div class="seg" role="radiogroup" aria-label="How the buttons move">
          <label><input type="radio" name="bmode" value="hold" checked><span>Hold to move</span></label>
          <label><input type="radio" name="bmode" value="step"><span>Step</span></label>
        </div>
        <span class="field" id="bStepField"><input id="bStep" type="number" min="0.5" max="90" step="0.5" value="5" aria-label="Step size in degrees"><span aria-hidden="true">° each</span></span>
        <div class="setting bspeed">
          <label for="bSpeed">Speed</label>
          <input type="range" id="bSpeed" min="1" max="60" step="1" value="10">
          <output id="bSpeedOut" for="bSpeed">10 RPM</output>
        </div>
      </div>
      <p class="hint" id="bHint">Hold a button to turn both motors, and let go to stop. If either one can't move, neither does; if either stops, both stop.</p>
      <p class="msg" id="bMsg" aria-live="polite"></p>
    </section>

    <section class="panel" id="jointsOff" hidden>
      <div class="phead"><h2>Motors</h2></div>
      <p class="off" id="jointsOffText"></p>
    </section>

    <section class="panel" id="gripper" aria-labelledby="gTitle">
      <div class="phead">
        <div>
          <h2 id="gTitle">Gripper</h2>
          <p class="sub" id="gSub">Looking for the gripper servo.</p>
        </div>
        <span class="pill" id="gPill" data-state="offline">Connecting</span>
      </div>
      <p class="off" id="gOff" hidden></p>
      <div id="gBody">
        <div class="window idle" id="gWin" aria-label="Gripper readings">
          <svg class="dial" viewBox="0 0 120 120" aria-hidden="true">
            <circle class="ring" cx="60" cy="60" r="52" stroke-width="2"></circle>
            <g class="ticks"></g>
            <line class="needle" id="gNeedle" x1="60" y1="60" x2="60" y2="16"></line>
            <circle class="hub" cx="60" cy="60" r="4"></circle>
          </svg>
          <dl>
            <div><dt>Angle</dt><dd><span id="gPos">--</span><small>°</small></dd></div>
            <div><dt>Speed</dt><dd><span id="gSpd">--</span><small>RPM</small></dd></div>
            <div><dt>Load</dt><dd><span id="gLoad">--</span><small>%</small></dd></div>
            <div><dt>Current</dt><dd><span id="gCur">--</span><small>A</small></dd></div>
            <div><dt>Supply</dt><dd><span id="gVolt">--</span><small>V</small></dd></div>
            <div><dt>Temperature</dt><dd><span id="gTemp">--</span><small>°C</small></dd></div>
          </dl>
        </div>
        <p class="banner" id="gBanner" role="alert" hidden></p>
        <div class="block flush">
          <div class="drive" aria-label="Gripper direction">
            <button class="dir" id="gBack" data-dir="-1" aria-pressed="false"><span class="glyph" aria-hidden="true">◀</span>Backward</button>
            <button class="stop" id="gStop">Stop</button>
            <button class="dir" id="gFwd" data-dir="1" aria-pressed="false">Forward<span class="glyph" aria-hidden="true">▶</span></button>
          </div>
        </div>
        <div class="block">
          <div class="setting">
            <label for="gSpeed">Speed</label>
            <input type="range" id="gSpeed" min="10" max="60" step="1" value="20">
            <output id="gSpeedOut" for="gSpeed">20 RPM</output>
          </div>
          <div class="setting">
            <label for="gStrength">Grip strength</label>
            <input type="range" id="gStrength" min="5" max="50" step="1" value="30">
            <output id="gStrengthOut" for="gStrength">30%</output>
            <p class="hint" id="gStrengthHint">How hard the gripper can push when something stops it.</p>
            <label class="check"><input type="checkbox" id="gHigh"><span>Allow high grip, up to <span id="gHighMax">100</span>%</span></label>
          </div>
        </div>
        <div class="block">
          <label class="check"><input type="checkbox" id="gHoldMode"><span>Hold to move</span></label>
          <p class="hint">When on, the gripper runs only while you hold Backward or Forward down.</p>
          <div class="row"><button class="quiet" id="gTorque">Release torque</button></div>
          <p class="hint" id="gTorqueHint">Lets the gripper go limp. Anything it's gripping will be let go.</p>
        </div>
        <p class="msg" id="gMsg" aria-live="polite"></p>
      </div>
    </section>
  </div>

  <p class="foot">Space or Esc stops everything, like Stop all. If this page loses contact with the Jetson for
    <span id="wd">2</span> seconds, moving motors stop and hold and the gripper stops.</p>
</main>

<template id="jointTpl">
  <section class="panel joint">
    <div class="phead">
      <div>
        <h2 class="jname">Motor</h2>
        <p class="sub jsub">moteus</p>
      </div>
      <span class="pill jpill" data-state="offline">Connecting</span>
    </div>
    <div class="window idle jwin">
      <svg class="dial" viewBox="0 0 120 120" aria-hidden="true">
        <circle class="ring" cx="60" cy="60" r="52" stroke-width="2"></circle>
        <path class="range" d=""></path>
        <g class="ticks"></g>
        <path class="tgt" d="M60 13 L55.5 3.5 L64.5 3.5 Z" style="display: none"></path>
        <line class="needle" x1="60" y1="60" x2="60" y2="14"></line>
        <circle class="hub" cx="60" cy="60" r="4"></circle>
      </svg>
      <dl>
        <div><dt>Angle</dt><dd><span class="v-pos">--</span><small>°</small></dd></div>
        <div><dt>Target</dt><dd><span class="v-tgt">--</span><small>°</small></dd></div>
        <div><dt>Speed</dt><dd><span class="v-spd">--</span><small>RPM</small></dd></div>
        <div><dt>Torque</dt><dd><span class="v-trq">--</span><small>Nm</small></dd></div>
        <div><dt>Current</dt><dd><span class="v-cur">--</span><small>A</small></dd></div>
        <div><dt>Temperature</dt><dd><span class="v-tmp">--</span><small>°C</small></dd></div>
      </dl>
    </div>
    <p class="banner jbanner" role="alert" hidden></p>

    <div class="block">
      <div class="nudgerow" role="group" aria-label="Turn">
        <button class="nudge mv" data-rot="ccw" aria-pressed="false" aria-label="Hold to turn CCW"><span class="rot" aria-hidden="true">↺</span><span>CCW</span></button>
        <button class="step mv" data-rot="ccw" data-deg="5" aria-label="Step 5° CCW">↺ 5°</button>
        <button class="step mv" data-rot="ccw" data-deg="1" aria-label="Step 1° CCW">↺ 1°</button>
        <button class="step mv" data-rot="cw" data-deg="1" aria-label="Step 1° CW">↻ 1°</button>
        <button class="step mv" data-rot="cw" data-deg="5" aria-label="Step 5° CW">↻ 5°</button>
        <button class="nudge mv" data-rot="cw" aria-pressed="false" aria-label="Hold to turn CW"><span>CW</span><span class="rot" aria-hidden="true">↻</span></button>
      </div>
      <p class="hint">Hold CCW or CW to turn, and let go to stop. The degree buttons move one step.</p>
      <form class="goto">
        <label class="gotoLabel">Go to</label>
        <span class="field"><input class="gotoIn" type="number" step="0.5" inputmode="decimal" placeholder="0.0"><span aria-hidden="true">°</span></span>
        <button type="submit" class="go mv">Go</button>
        <button type="button" class="quiet zero mv">0°</button>
      </form>
      <div class="row">
        <button class="quiet hold mv">Hold here</button>
        <button class="quiet release">Release</button>
      </div>
    </div>

    <div class="block">
      <div class="setting">
        <label class="spdLabel">Speed</label>
        <input type="range" class="spd" min="1" max="60" step="1" value="10">
        <output class="spdOut">10 RPM</output>
      </div>
      <div class="setting">
        <label class="trqLabel">Max torque</label>
        <input type="range" class="trq" min="0.5" max="4.5" step="0.1" value="4">
        <output class="trqOut">4.0 Nm</output>
        <p class="hint">How hard it can push. If it gets stuck (a hard stop or a jam), it stops pushing on its own.</p>
      </div>
    </div>

    <div class="block">
      <label class="check beyondCheck"><input type="checkbox" class="beyond"><span>Allow moving past the limits</span></label>
      <div class="limits">
        <span class="what">Limits <b class="limText">reading</b></span>
        <button class="quiet small setLo">Set lower limit here</button>
        <button class="quiet small setHi">Set upper limit here</button>
      </div>
      <div class="confirm" hidden>
        <span class="confirmText"></span>
        <button class="save">Save</button>
        <button class="quiet small cancel">Cancel</button>
      </div>
      <div class="cwrow">
        <span class="what">CW is</span>
        <div class="seg" role="radiogroup">
          <label><input type="radio" class="cwOpt" value="1"><span>+ angle</span></label>
          <label><input type="radio" class="cwOpt" value="-1"><span>− angle</span></label>
        </div>
      </div>
      <p class="hint">Check this once: hold CW and look at the shaft end. If the shaft turns counterclockwise, switch it.</p>
    </div>
    <p class="msg" aria-live="polite"></p>
  </section>
</template>

<script>
const $ = (id) => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";
let last = null, contact = true;
const joints = new Map();
let gLastNotice = 0, gHolding = false, gDragSpeed = false, gDragStrength = false, gSpeedTimer = null;

function say(el, text, bad) { el.textContent = text || ""; el.dataset.tone = bad ? "bad" : ""; }
function fmt(v, digits, signed) {
  if (v === undefined || v === null) return "--";
  let s = Number(v).toFixed(digits);
  if (Number(s) === 0) s = (0).toFixed(digits);            // no "-0.0"
  if (s.startsWith("-")) return "−" + s.slice(1);    // a real minus sign
  return signed && Number(s) > 0 ? "+" + s : s;
}
function fmtDeg(v) { return v === null || v === undefined ? "--" : fmt(v, 1, true) + "°"; }

function addTicks(g) {
  for (let a = 0; a < 360; a += 30) {
    const r = (a - 90) * Math.PI / 180, inner = a % 90 === 0 ? 40 : 45;
    const l = document.createElementNS(NS, "line");
    l.setAttribute("class", "tick");
    l.setAttribute("x1", 60 + inner * Math.cos(r)); l.setAttribute("y1", 60 + inner * Math.sin(r));
    l.setAttribute("x2", 60 + 50 * Math.cos(r)); l.setAttribute("y2", 60 + 50 * Math.sin(r));
    l.setAttribute("stroke-width", a % 90 === 0 ? 2.5 : 1.5);
    g.appendChild(l);
  }
}
function polar(a, r) { const t = (a - 90) * Math.PI / 180; return [60 + r * Math.cos(t), 60 + r * Math.sin(t)]; }
function arc(a0, a1, r) {
  const sweep = a1 - a0;
  if (sweep >= 359.5) return `M 60 ${60 - r} A ${r} ${r} 0 1 1 60 ${60 + r} A ${r} ${r} 0 1 1 60 ${60 - r}`;
  if (sweep <= 0.2) return "";
  const [x0, y0] = polar(a0, r), [x1, y1] = polar(a1, r);
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 ${sweep > 180 ? 1 : 0} 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`;
}

async function post(body) {
  try {
    const r = await fetch("/api/command", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
    });
    const j = await r.json();
    if (j.status) render(j.status);
    return j;
  } catch (e) {
    return { ok: false, message: "Couldn't reach the Jetson. Moving motors stop on their own when the page loses contact." };
  }
}
async function jcmd(id, action, extra, quiet) {
  const j = await post(Object.assign({ device: "joint", id, action }, extra || {}));
  const p = joints.get(id);
  if (p && j.message && !(quiet && j.ok)) say(p.msg, j.message, !j.ok);
  return j;
}
async function gcmd(action, extra) {
  const j = await post(Object.assign({ device: "gripper", action }, extra || {}));
  if (j.message) say($("gMsg"), j.message, !j.ok);
  return j;
}
async function stopAll() {
  for (const p of joints.values()) p.endNudge(false);
  bEnd(false);
  const j = await post({ device: "all", action: "stop" });
  say($("gmsg"), j.message, !j.ok);
}

// ---------- motor panels ----------

function makeJoint(js) {
  const root = $("jointTpl").content.firstElementChild.cloneNode(true);
  const q = (sel) => root.querySelector(sel);
  const id = js.id;
  const p = {
    id, root, msg: q(".msg"), lastNotice: 0, dragSpd: false, dragTrq: false, spdTimer: null,
    nudgeTimer: null, nudgeDir: 0, pendingLimit: null,
  };
  root.setAttribute("aria-label", js.name);
  addTicks(q(".ticks"));
  for (const [cls, name] of [["spd", "Speed"], ["trq", "Max torque"]]) {
    const input = q("." + cls), label = q("." + cls + "Label");
    input.id = `${cls}-${id}`; label.htmlFor = input.id;
  }
  q(".gotoIn").id = `goto-${id}`; q(".gotoLabel").htmlFor = `goto-${id}`;

  for (const b of root.querySelectorAll(".step")) {
    b.addEventListener("click", () => jcmd(id, "jog", { rot: b.dataset.rot, deg: Number(b.dataset.deg) }));
  }

  // Hold-to-turn: renew every 200 ms while held; the Jetson stops it if renewals stop.
  p.startNudge = (rot, btn) => {
    if (btn.disabled || p.nudgeTimer) return;
    btn.setAttribute("aria-pressed", "true");
    jcmd(id, "nudge", { rot }).then((r) => { if (!r.ok) p.endNudge(false); });
    p.nudgeTimer = setInterval(async () => {
      const r = await jcmd(id, "nudge", { rot, renew: true }, true);
      if (!r.ok) p.endNudge(false);       // stopped on the Jetson's side (stuck, Stop all)
    }, 200);
  };
  p.endNudge = (sendHold) => {
    if (!p.nudgeTimer) return;
    clearInterval(p.nudgeTimer); p.nudgeTimer = null;
    for (const b of root.querySelectorAll(".nudge")) b.setAttribute("aria-pressed", "false");
    if (sendHold) jcmd(id, "hold");
  };
  for (const b of root.querySelectorAll(".nudge")) {
    const dir = b.dataset.rot;
    b.addEventListener("pointerdown", (e) => {
      if (e.button !== 0) return;
      b.setPointerCapture(e.pointerId);
      p.startNudge(dir, b);
    });
    const end = () => p.endNudge(true);
    b.addEventListener("pointerup", end);
    b.addEventListener("pointercancel", end);
    b.addEventListener("lostpointercapture", end);
    b.addEventListener("blur", end);
    b.addEventListener("contextmenu", (e) => e.preventDefault());
    b.addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); if (!e.repeat) p.startNudge(dir, b); } });
    b.addEventListener("keyup", (e) => { if (e.key === "Enter") end(); });
  }

  q(".goto").addEventListener("submit", (e) => {
    e.preventDefault();
    const raw = q(".gotoIn").value.trim();
    const v = Number(raw);
    if (raw === "" || !Number.isFinite(v)) { say(p.msg, "Enter an angle in degrees.", true); return; }
    jcmd(id, "goto", { deg: v });
  });
  q(".zero").addEventListener("click", () => jcmd(id, "goto", { deg: 0 }));
  q(".hold").addEventListener("click", () => jcmd(id, "hold"));
  q(".release").addEventListener("click", () => { p.endNudge(false); jcmd(id, "release"); });

  const spd = q(".spd"), trq = q(".trq");
  spd.addEventListener("pointerdown", () => { p.dragSpd = true; });
  trq.addEventListener("pointerdown", () => { p.dragTrq = true; });
  document.addEventListener("pointerup", () => { p.dragSpd = false; p.dragTrq = false; });
  spd.addEventListener("input", () => {
    q(".spdOut").textContent = spd.value + " RPM";
    if (p.spdTimer !== null) return;
    p.spdTimer = setTimeout(() => { p.spdTimer = null; jcmd(id, "speed", { rpm: Number(spd.value) }, true); }, 200);
  });
  trq.addEventListener("input", () => { q(".trqOut").textContent = Number(trq.value).toFixed(1) + " Nm"; });
  trq.addEventListener("change", () => { p.dragTrq = false; jcmd(id, "torque", { nm: Number(trq.value) }, true); });

  q(".beyond").addEventListener("change", (e) => jcmd(id, "beyond", { on: e.target.checked }));
  for (const r of root.querySelectorAll(".cwOpt")) {
    r.name = `cw-${id}`;
    r.addEventListener("change", () => { if (r.checked) jcmd(id, "cw_sign", { sign: Number(r.value) }); });
  }
  root.querySelector(".cwrow .seg").setAttribute("aria-label", `CW is, for ${js.name}`);

  const askLimit = (which) => {
    const s = p.last;
    if (!s || s.position_deg === null) { say(p.msg, "Its position isn't known yet.", true); return; }
    p.pendingLimit = { which, deg: Math.round(s.position_deg * 10) / 10 };
    q(".confirmText").textContent = `Save ${fmtDeg(p.pendingLimit.deg)} as ${s.name}'s ${which} limit? The controller won't go past it after this.`;
    q(".confirm").hidden = false;
    q(".save").focus();
  };
  q(".setLo").addEventListener("click", () => askLimit("lower"));
  q(".setHi").addEventListener("click", () => askLimit("upper"));
  q(".cancel").addEventListener("click", () => { p.pendingLimit = null; q(".confirm").hidden = true; });
  q(".save").addEventListener("click", async () => {
    const pl = p.pendingLimit;
    q(".confirm").hidden = true; p.pendingLimit = null;
    if (!pl) return;
    say(p.msg, "Saving the limit on the controller.", false);
    await jcmd(id, "set_limit", { which: pl.which, deg: pl.deg });
  });

  p.q = q;
  $("grid").insertBefore(root, $("gripper"));
  joints.set(id, p);
  return p;
}

// ---------- both motors ----------

let bMode = "hold", bTimer = null, bActive = null, bDragSpeed = false, bSpeedTimer = null;

async function bcmd(action, extra, quiet) {
  const j = await post(Object.assign({ device: "both", action, rpm: Number($("bSpeed").value) }, extra || {}));
  if (j.message && !(quiet && j.ok)) say($("bMsg"), j.message, !j.ok);
  return j;
}
function bRots(btn) {
  const pr = last && last.pair;
  if (!pr) return {};
  return { [pr.ids[0]]: btn.dataset.r1, [pr.ids[1]]: btn.dataset.r2 };
}
function bStart(btn) {
  if (btn.disabled || bTimer) return;
  bActive = btn;
  btn.setAttribute("aria-pressed", "true");
  const rots = bRots(btn);
  bcmd("nudge", { rots }).then((r) => { if (!r.ok) bEnd(false); });
  bTimer = setInterval(async () => {
    const r = await bcmd("nudge", { rots, renew: true }, true);
    if (!r.ok) bEnd(false);               // stopped on the Jetson's side
  }, 200);
}
function bEnd(sendHold) {
  if (!bTimer) return;
  clearInterval(bTimer); bTimer = null;
  if (bActive) bActive.setAttribute("aria-pressed", "false");
  bActive = null;
  if (sendHold) bcmd("hold", {}, true);
}
for (const btn of document.querySelectorAll(".combo")) {
  btn.addEventListener("pointerdown", (e) => {
    if (e.button !== 0 || bMode !== "hold") return;
    btn.setPointerCapture(e.pointerId);
    bStart(btn);
  });
  const end = () => bEnd(true);
  btn.addEventListener("pointerup", end);
  btn.addEventListener("pointercancel", end);
  btn.addEventListener("lostpointercapture", end);
  btn.addEventListener("blur", end);
  btn.addEventListener("contextmenu", (e) => e.preventDefault());
  btn.addEventListener("click", () => {
    if (bMode !== "step") return;
    const size = Number($("bStep").value);
    if (!Number.isFinite(size) || size <= 0) { say($("bMsg"), "Enter a step in degrees.", true); return; }
    bcmd("step", { rots: bRots(btn), deg: size });
  });
  btn.addEventListener("keydown", (e) => {
    if (e.key === "Enter" && bMode === "hold") { e.preventDefault(); if (!e.repeat) bStart(btn); }
  });
  btn.addEventListener("keyup", (e) => { if (e.key === "Enter" && bMode === "hold") end(); });
}
for (const r of document.querySelectorAll("input[name='bmode']")) {
  r.addEventListener("change", () => {
    if (!r.checked) return;
    bEnd(true);
    bMode = r.value;
    $("bStepField").classList.toggle("dim", bMode !== "step");
    $("bStep").disabled = bMode !== "step";
    $("bHint").textContent = bMode === "step"
      ? "Each click moves both motors by the step. If either can't move that far, neither moves."
      : "Hold a button to turn both motors, and let go to stop. If either one can't move, neither does; if either stops, both stop.";
  });
}
$("bStep").disabled = true;
$("bSpeed").addEventListener("pointerdown", () => { bDragSpeed = true; });
document.addEventListener("pointerup", () => { bDragSpeed = false; });
$("bSpeed").addEventListener("input", () => {
  $("bSpeedOut").textContent = $("bSpeed").value + " RPM";
  if (bSpeedTimer !== null) return;
  bSpeedTimer = setTimeout(() => { bSpeedTimer = null; bcmd("speed", {}, true); }, 200);
});

function renderBoth(pr, L) {
  $("both").hidden = !pr;
  if (!pr) { bEnd(false); return; }
  for (const btn of document.querySelectorAll(".combo")) {
    const who = btn.querySelectorAll(".who");
    who[0].textContent = pr.names[0]; who[1].textContent = pr.names[1];
    const r1 = btn.dataset.r1.toUpperCase(), r2 = btn.dataset.r2.toUpperCase();
    btn.setAttribute("aria-label", `${bMode === "hold" ? "Hold to turn" : "Step"} ${pr.names[0]} ${r1} and ${pr.names[1]} ${r2}`);
    btn.disabled = !contact || !pr.ready;
  }
  if (!contact || !pr.ready) bEnd(false);
  $("bSub").textContent = `${pr.names[0]} and ${pr.names[1]} start and stop together. Each uses its own max torque.`;
  const state = !contact || !pr.ready ? "offline" : pr.moving ? "moving" : "holding";
  $("bPill").dataset.state = state;
  $("bPill").textContent = !contact ? "No contact" : !pr.ready ? "Not ready" : pr.moving ? "Moving" : "Ready";
  const sp = $("bSpeed");
  sp.max = L.max_rpm;
  if (!bDragSpeed && bSpeedTimer === null) sp.value = pr.rpm;
  $("bSpeedOut").textContent = sp.value + " RPM";
}

const STATE_TEXT = { released: "Released", holding: "Holding", moving: "Moving", fault: "Fault",
                     offline: "Not answering", connecting: "Connecting" };

function renderJoint(js, L) {
  const p = joints.get(js.id) || makeJoint(js);
  const q = p.q;
  p.last = js;
  q(".jname").textContent = js.name;
  const sub = [js.name === `ID ${js.id}` ? "moteus controller" : `moteus ID ${js.id}`];
  if (js.voltage_v !== null && js.online) sub.push(`${fmt(js.voltage_v, 1)} V supply`);
  if (js.together) sub.push(`moving together`);
  q(".jsub").textContent = sub.join(", ");
  const state = contact ? js.state : "offline";
  q(".jpill").dataset.state = state;
  q(".jpill").textContent = contact ? STATE_TEXT[state] || state : "No contact";
  q(".jwin").classList.toggle("idle", !js.online || !contact);
  p.root.classList.toggle("caution", js.beyond);

  q(".v-pos").textContent = fmt(js.position_deg, 1, true);
  q(".v-tgt").textContent = fmt(js.target_deg, 1, true);
  q(".v-spd").textContent = fmt(js.speed_rpm_now, 1);
  q(".v-trq").textContent = fmt(js.torque_nm, 2);
  q(".v-cur").textContent = fmt(js.current_a, 2);
  q(".v-tmp").textContent = fmt(js.temp_c, 0);
  // The dial turns the way the shaft does: clockwise on screen is CW on the motor.
  const k = js.cw_sign < 0 ? -1 : 1;
  if (js.position_deg !== null) q(".needle").setAttribute("transform", `rotate(${k * js.position_deg} 60 60)`);
  const tgt = q(".tgt");
  tgt.style.display = js.target_deg === null ? "none" : "";
  if (js.target_deg !== null) tgt.setAttribute("transform", `rotate(${k * js.target_deg} 60 60)`);
  if (js.limits_deg) {
    const a = k * (js.limits_deg[0] === null ? -180 : js.limits_deg[0]);
    const b2 = k * (js.limits_deg[1] === null ? 180 : js.limits_deg[1]);
    q(".range").setAttribute("d", arc(Math.min(a, b2), Math.max(a, b2), 52));
    q(".limText").textContent = `${js.limits_deg[0] === null ? "none" : fmtDeg(js.limits_deg[0])} to ${js.limits_deg[1] === null ? "none" : fmtDeg(js.limits_deg[1])}`;
  } else {
    q(".range").setAttribute("d", "");
    q(".limText").textContent = js.limits_read ? "none saved" : "reading";
  }

  let banner = "", tone = "";
  if (!contact) { banner = ""; }
  else if (!js.online) { banner = js.problem; tone = "bad"; }
  else if (js.state === "fault") { banner = `Fault ${js.fault}: ${js.fault_text}. It's limp. Moving it again clears the fault.`; tone = "bad"; }
  else if (js.outside_limits && !js.beyond) { banner = `It's at ${fmtDeg(js.position_deg)}, outside its limits. Turn on Allow moving past the limits to bring it back.`; }
  else if (js.past_half_turn) { banner = "It's more than half a turn from zero. If its controller restarts here, it will read a full turn off."; }
  else if (js.beyond) { banner = `Limits are off for this motor. Watch it: it can reach its hard stops. Moves still stop at ${L.travel_cap_deg}° from zero.`; }
  else if (js.fault === 102 && js.state !== "released") { banner = "It's pushing at its torque cap."; }
  const b = q(".jbanner");
  b.hidden = !banner; b.textContent = banner; b.dataset.tone = tone;

  const canMove = contact && js.ready;
  for (const el of p.root.querySelectorAll(".mv")) el.disabled = !canMove;
  q(".release").disabled = !contact || !js.online;
  q(".setLo").disabled = q(".setHi").disabled = !canMove;
  if (!canMove) p.endNudge(false);

  const beyond = q(".beyond");
  if (document.activeElement !== beyond) beyond.checked = js.beyond;
  for (const r of p.root.querySelectorAll(".cwOpt")) r.checked = Number(r.value) === k;
  const spd = q(".spd"), trq = q(".trq");
  spd.max = L.max_rpm; trq.min = L.min_torque; trq.max = L.max_torque;
  if (!p.dragSpd && p.spdTimer === null) spd.value = js.speed_rpm;
  q(".spdOut").textContent = spd.value + " RPM";
  if (!p.dragTrq) trq.value = js.max_torque;
  q(".trqOut").textContent = Number(trq.value).toFixed(1) + " Nm";

  if (js.notice_id !== p.lastNotice) {
    if (js.notice_id > 0) say(p.msg, `${js.notice} (${js.notice_time})`, /Fault|Stopped|Lost|isn't|Couldn't/.test(js.notice));
    p.lastNotice = js.notice_id;
  }
}

// ---------- gripper panel ----------

function renderGripper(s, problem) {
  const off = !s;
  $("gOff").hidden = !off; $("gBody").hidden = off;
  if (off) {
    $("gOff").textContent = problem || "The gripper is off.";
    $("gSub").textContent = "Not in use";
    $("gPill").dataset.state = "released"; $("gPill").textContent = "Off";
    return;
  }
  const L = s.limits, t = s.telemetry || {};
  $("gSub").textContent = s.port
    ? `Servo ${s.id}${s.model !== null ? ", model " + s.model : ""}, on ${s.port}`
    : `Servo ${s.id}, adapter not found yet`;
  let state = "offline", text = "Not ready";
  if (!contact) { text = "No contact"; }
  else if (s.connected && s.direction !== 0) { state = "running"; text = s.direction > 0 ? "Forward" : "Backward"; }
  else if (s.connected && !s.torque_on) { state = "released"; text = "Limp"; }
  else if (s.connected) { state = "holding"; text = "Stopped"; }
  $("gPill").dataset.state = state; $("gPill").textContent = text;

  $("gWin").classList.toggle("idle", !s.connected || !contact);
  $("gPos").textContent = fmt(t.position_deg, 1);
  $("gSpd").textContent = fmt(t.speed_rpm, 1);
  $("gLoad").textContent = fmt(t.load_pct, 0);
  $("gCur").textContent = fmt(t.current_a, 2);
  $("gVolt").textContent = fmt(t.voltage_v, 1);
  $("gTemp").textContent = fmt(t.temp_c, 0);
  if (t.position_deg !== undefined) $("gNeedle").setAttribute("transform", `rotate(${t.position_deg} 60 60)`);

  let banner = "", tone = "";
  if (!contact) banner = "";
  else if (!s.connected) { banner = s.problem; tone = "bad"; }
  else if (s.overheated) banner = "The gripper is too hot and is stopped until it cools down.";
  else if (t.overload) banner = "The gripper reports an overload. Lower the grip strength or check for a jam.";
  $("gBanner").hidden = !banner; $("gBanner").textContent = banner; $("gBanner").dataset.tone = tone;

  for (const b of [$("gFwd"), $("gBack")]) {
    b.setAttribute("aria-pressed", String(s.direction === Number(b.dataset.dir)));
    b.disabled = !contact || !s.connected || s.overheated;
  }
  $("gTorque").textContent = s.torque_on ? "Release torque" : "Turn torque on";
  $("gTorqueHint").textContent = s.torque_on
    ? "Lets the gripper go limp. Anything it's gripping will be let go."
    : "The gripper is limp. Turning torque on makes it hold still.";

  const sp = $("gSpeed");
  sp.min = L.min_rpm; sp.max = L.max_rpm;
  if (!gDragSpeed && gSpeedTimer === null) sp.value = s.rpm;
  $("gSpeedOut").textContent = sp.value + " RPM";
  const st = $("gStrength");
  $("gHigh").checked = s.high_grip;
  $("gHighMax").textContent = L.high_strength;
  st.min = L.min_strength; st.max = s.high_grip ? L.high_strength : L.normal_strength;
  if (!gDragStrength) st.value = s.strength;
  showStrength();

  if (s.notice_id !== gLastNotice) {
    if (s.notice_id > 0) say($("gMsg"), `${s.notice} (${s.notice_time})`, false);
    gLastNotice = s.notice_id;
  }
}

function showStrength() {
  const v = Number($("gStrength").value), normal = last && last.gripper ? last.gripper.limits.normal_strength : 50;
  $("gStrengthOut").textContent = v + "%";
  $("gStrength").classList.toggle("hot", v > normal);
  $("gStrengthHint").textContent = v > normal
    ? "High grip. The gripper can squeeze hard enough to damage delicate parts."
    : "How hard the gripper can push when something stops it.";
}

const gDrive = (dir) => gcmd("drive", { direction: dir, rpm: Number($("gSpeed").value) });
const gStop = () => gcmd("stop");
$("gSpeed").addEventListener("pointerdown", () => { gDragSpeed = true; });
$("gStrength").addEventListener("pointerdown", () => { gDragStrength = true; });
document.addEventListener("pointerup", () => { gDragSpeed = false; gDragStrength = false; });
$("gSpeed").addEventListener("input", () => {
  $("gSpeedOut").textContent = $("gSpeed").value + " RPM";
  if (gSpeedTimer !== null) return;
  gSpeedTimer = setTimeout(() => { gSpeedTimer = null; gcmd("speed", { rpm: Number($("gSpeed").value) }); }, 200);
});
$("gStrength").addEventListener("input", showStrength);
$("gStrength").addEventListener("change", () => {
  gDragStrength = false;
  gcmd("strength", { percent: Number($("gStrength").value), high: $("gHigh").checked });
});
$("gHigh").addEventListener("change", () => {
  const st = $("gStrength"), L = last && last.gripper ? last.gripper.limits : { normal_strength: 50, high_strength: 100 };
  st.max = $("gHigh").checked ? L.high_strength : L.normal_strength;
  if (Number(st.value) > Number(st.max)) st.value = st.max;
  showStrength();
  gcmd("strength", { percent: Number(st.value), high: $("gHigh").checked });
});
$("gHoldMode").addEventListener("change", () => { gHolding = $("gHoldMode").checked; if (gHolding) gStop(); });
$("gTorque").addEventListener("click", () => gcmd(last && last.gripper && last.gripper.torque_on ? "release" : "hold"));
$("gStop").addEventListener("click", gStop);
for (const b of [$("gFwd"), $("gBack")]) {
  const dir = Number(b.dataset.dir);
  b.addEventListener("click", () => { if (!gHolding) gDrive(dir); });
  b.addEventListener("pointerdown", (e) => {
    if (!gHolding || b.disabled) return;
    b.setPointerCapture(e.pointerId);
    gDrive(dir);
  });
  const release = () => { if (gHolding) gStop(); };
  b.addEventListener("pointerup", release);
  b.addEventListener("pointercancel", release);
  b.addEventListener("lostpointercapture", release);
  b.addEventListener("contextmenu", (e) => e.preventDefault());
}
addTicks(document.querySelector("#gripper .ticks"));

// ---------- whole page ----------

function render(s) {
  last = s;
  $("wd").textContent = s.watchdog_s;
  renderBoth(s.pair, s.joint_limits);
  for (const js of s.joints) renderJoint(js, s.joint_limits);
  $("jointsOff").hidden = s.joints.length > 0;
  if (!s.joints.length) $("jointsOffText").textContent = s.joints_problem || "No motors are set up.";
  renderGripper(s.gripper, s.gripper_problem);

  const names = s.joints.map((j) => j.name);
  const parts = [];
  if (names.length) parts.push(names.join(" and ") + " on CAN");
  if (s.gripper) parts.push("gripper on " + (s.gripper.port || "a serial port"));
  $("sub").textContent = parts.length ? parts.join(", ") : "Nothing connected";

  const notReady = s.joints.filter((j) => !j.ready).length + (s.gripper && !s.gripper.connected ? 1 : 0);
  if (!contact) setLink("bad", "No contact with the Jetson");
  else if (notReady) setLink("warn", notReady === 1 ? "1 device not ready" : `${notReady} devices not ready`);
  else setLink("ok", "Connected");
  $("banner").hidden = contact;
  $("banner").textContent = `No contact with the Jetson. Moving motors stop and hold, and the gripper stops, within ${s.watchdog_s} seconds.`;
}
function setLink(state, text) { $("link").dataset.state = state; $("linkText").textContent = text; }

async function poll() {
  try {
    const r = await fetch("/api/status", { cache: "no-store" });
    contact = r.ok;
    render(await r.json());
  } catch (e) {
    contact = false;
    if (last) render(last); else setLink("bad", "No contact with the Jetson");
  } finally {
    setTimeout(poll, 250);
  }
}

$("stopAll").addEventListener("click", stopAll);
// Space and Esc always stop everything, wherever the focus is.
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" || e.key === " ") { e.preventDefault(); stopAll(); }
}, true);
document.addEventListener("keyup", (e) => { if (e.key === " ") e.preventDefault(); }, true);

poll();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
