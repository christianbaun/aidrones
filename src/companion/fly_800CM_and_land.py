#!/usr/bin/env python3
"""
This module is supposed to run from companion computer (ie. Raspberry pi)

Guided-mode hold for ArduPilot + optical flow + rangefinder (no GPS).

Before flying, confirm these params match what made Loiter work: 
EK3_SRC1_POSXY=3, EK3_SRC1_VELXY=5, EK3_SRC1_POSZ=2, and FLOW_TYPE/RNGFND1_TYPE set for the optical flow + range finder.
"""

from pymavlink import mavutil
import time
import signal
import sys


# ================= CONFIG =================

PORT = "/dev/serial0"
BAUD = 921600

TAKEOFF_HEIGHT = 0.8      # meters - keep above ~0.5 m; Optical flow typically is poor lower
SETTLE_TIME = 6           # seconds to let the EKF settle after reaching height
HOLD_TIME = 30            # seconds

SEND_RATE_HZ = 10         # setpoint send rate
TELEM_RATE_HZ = 20        # requested LOCAL_POSITION_NED / DISTANCE_SENSOR rate

HOLD_MODE = "velocity"    # "velocity" (robust on flow-only) or "position"
HOLD_BLEND = 0.03         # position mode only: 0 = rigid, 0.02-0.05 = follow drift

RF_ABORT_M = 0.15         # land if rangefinder drops below this during hold
RF_ABORT_ENABLE = True

HEARTBEAT_WAIT = 30       # s to wait for a valid autopilot heartbeat
MODE_TIMEOUT = 5          # s to wait for mode change confirmation
ARM_TIMEOUT = 10          # s to wait for armed confirmation
CLIMB_TIMEOUT = 20        # s to wait for takeoff height
MIN_CLIMB_M = 0.15        # must gain at least this much altitude to count as airborne

# Reference ArduCopter custom_mode numbers - used to validate the dynamic
# mode_mapping() and as a fallback.
COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11,
    "SPORT": 13, "FLIP": 14, "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17,
    "THROW": 18, "AVOID_ADSB": 19, "GUIDED_NOGPS": 20, "SMART_RTL": 21,
    "FLOWHOLD": 22, "FOLLOW": 23, "ZIGZAG": 24, "SYSTEMID": 25,
    "AUTOROTATE": 26, "AUTO_RTL": 27,
}

COPTER_TYPES = {
    mavutil.mavlink.MAV_TYPE_QUADROTOR,
    mavutil.mavlink.MAV_TYPE_HEXAROTOR,
    mavutil.mavlink.MAV_TYPE_OCTOROTOR,
    mavutil.mavlink.MAV_TYPE_TRICOPTER,
    mavutil.mavlink.MAV_TYPE_HELICOPTER,
    mavutil.mavlink.MAV_TYPE_COAXIAL,
    mavutil.mavlink.MAV_TYPE_DODECAROTOR,
}

# type_mask bits (set = ignore):
#   b0 px b1 py b2 pz | b3 vx b4 vy b5 vz | b6 ax b7 ay b8 az
#   b9 force | b10 yaw b11 yaw_rate
MASK_POSITION_ONLY = 0b110111111000
MASK_VELOCITY_ONLY = 0b110111000111

# =========================================


master = None
mode_map = dict(COPTER_MODES)
last_pos = None       # (x, y, z) from LOCAL_POSITION_NED
last_rf = None        # rangefinder distance, meters
last_hb = None        # most recent HEARTBEAT from the autopilot
boot_t = time.time()
landed = False


class FlightError(Exception):
    pass


def ms_since_boot():
    return int((time.time() - boot_t) * 1000) & 0xFFFFFFFF


# ---------------- connection ----------------

def connect():
    """Connect and wait for a REAL autopilot heartbeat with a non-zero sysid."""
    print("Connecting...")
    m = mavutil.mavlink_connection(PORT, baud=BAUD)

    print("Waiting for a valid autopilot heartbeat...")
    deadline = time.time() + HEARTBEAT_WAIT
    hb = None
    while time.time() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is None:
            continue
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        # Reject sysid 0 (invalid / broadcast) and non-autopilot components
        # such as GCS software or our own MAVLink echo.
        if sysid == 0:
            print("  ignoring heartbeat with sysid 0")
            continue
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            print(f"  ignoring non-autopilot heartbeat from {sysid}/{compid} "
                  f"(type={msg.type})")
            continue
        if compid not in (mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, 0, 1):
            print(f"  ignoring heartbeat from component {compid}")
            continue

        m.target_system = sysid
        m.target_component = compid
        hb = msg
        break

    if hb is None:
        raise FlightError("No valid autopilot heartbeat received - check wiring, "
                          "baud rate, and that SERIALx_PROTOCOL=2 (MAVLink2).")

    print(f"Connected. SYS: {m.target_system} COMP: {m.target_component} "
          f"type={hb.type} autopilot={hb.autopilot}")

    if hb.type not in COPTER_TYPES:
        raise FlightError(f"Vehicle type {hb.type} is not a multicopter/heli. "
                          "Refusing to run a copter flight script.")
    return m


def resolve_mode_map():
    """
    Prefer pymavlink's dynamic mapping, but only trust it if it agrees with the
    known copter numbers on a few unambiguous modes. Otherwise fall back.
    """
    try:
        dyn = master.mode_mapping()
    except Exception:
        dyn = None

    if not dyn:
        print("mode_mapping() unavailable - using built-in copter table.")
        return dict(COPTER_MODES)

    dyn = {k.upper(): v for k, v in dyn.items()}
    for key in ("GUIDED", "LAND", "LOITER", "ALT_HOLD", "RTL"):
        if dyn.get(key) != COPTER_MODES[key]:
            print(f"mode_mapping() disagrees on {key} "
                  f"({dyn.get(key)} vs {COPTER_MODES[key]}) - "
                  "using built-in copter table.")
            return dict(COPTER_MODES)

    print("Using dynamic mode_mapping() (validated against copter table).")
    merged = dict(COPTER_MODES)
    merged.update(dyn)
    return merged


# ---------------- telemetry ----------------

def request_message_interval(msg_id, hz):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        int(1e6 / hz),
        0, 0, 0, 0, 0
    )


def drain_telemetry():
    """Non-blocking: consume all pending messages and update cached state."""
    global last_pos, last_rf, last_hb
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_srcSystem() != master.target_system:
            continue
        t = msg.get_type()
        if t == "LOCAL_POSITION_NED":
            last_pos = (msg.x, msg.y, msg.z)
        elif t == "DISTANCE_SENSOR":
            last_rf = msg.current_distance / 100.0
        elif t == "HEARTBEAT":
            last_hb = msg
        elif t == "STATUSTEXT":
            txt = msg.text.strip() if isinstance(msg.text, str) else str(msg.text)
            if txt:
                print("  [FC]", txt)


def current_altitude():
    """Prefer the rangefinder; fall back to EKF z."""
    if last_rf is not None:
        return last_rf
    if last_pos is not None:
        return -last_pos[2]
    return None


def wait_for_position(timeout=5.0):
    deadline = time.time() + timeout
    while last_pos is None and time.time() < deadline:
        drain_telemetry()
        time.sleep(0.02)
    if last_pos is None:
        print("WARNING: no LOCAL_POSITION_NED received yet.")
    return last_pos


def wait_command_ack(command, timeout=3.0):
    """Return the MAV_RESULT for a command, or None on timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.3)
        if msg is None:
            continue
        if msg.command == command:
            return msg.result
        # not ours - keep the cache warm
        drain_telemetry()
    return None


# ---------------- commands, all verified ----------------

def set_mode(mode, timeout=MODE_TIMEOUT, required=True):
    """
    Change mode using MAV_CMD_DO_SET_MODE (which ACKs), then confirm via
    HEARTBEAT.custom_mode. Raises FlightError if required and unconfirmed.
    """
    if mode not in mode_map:
        raise FlightError("Mode not available: " + mode)
    target = mode_map[mode]
    print(f"Changing mode: {mode} (custom_mode={target})")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        target,
        0, 0, 0, 0, 0
    )

    result = wait_command_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=2.0)
    if result is not None and result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        msg = f"Mode change to {mode} REJECTED by autopilot (result={result})."
        if required:
            raise FlightError(msg)
        print("WARNING:", msg)
        return False

    # Confirm from the heartbeat regardless of the ACK.
    deadline = time.time() + timeout
    while time.time() < deadline:
        drain_telemetry()
        if last_hb is not None and last_hb.custom_mode == target:
            print(f"Mode confirmed: {mode}")
            return True
        time.sleep(0.05)

    got = last_hb.custom_mode if last_hb is not None else "unknown"
    msg = (f"Mode change to {mode} NOT confirmed (heartbeat still reports "
           f"custom_mode={got}). Common causes: pre-arm/EKF failure, "
           f"no position estimate, or RC override.")
    if required:
        raise FlightError(msg)
    print("WARNING:", msg)
    return False


def is_armed():
    if last_hb is None:
        return False
    return bool(last_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def arm(timeout=ARM_TIMEOUT):
    print("Arming")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    result = wait_command_ack(
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=3.0)
    if result is not None and result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise FlightError(f"Arm command REJECTED (result={result}). "
                          "Check pre-arm messages above.")

    deadline = time.time() + timeout
    while time.time() < deadline:
        drain_telemetry()
        if is_armed():
            print("Armed (confirmed via heartbeat)")
            return True
        time.sleep(0.05)
    raise FlightError("Arming NOT confirmed within timeout.")


def disarm():
    print("Disarming")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
    except Exception as e:
        print("Disarm failed:", e)


def takeoff():
    print("Takeoff to", TAKEOFF_HEIGHT, "m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        TAKEOFF_HEIGHT
    )
    result = wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=3.0)
    if result is None:
        print("WARNING: no ACK for takeoff - will verify by altitude.")
    elif result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise FlightError(f"Takeoff command REJECTED (result={result}).")
    else:
        print("Takeoff accepted")


def wait_for_takeoff_height(timeout=CLIMB_TIMEOUT):
    """
    Confirm the vehicle actually left the ground. Raises if it never climbed,
    so we don't run a hold loop on a grounded aircraft.
    """
    drain_telemetry()
    start_alt = current_altitude() or 0.0
    goal = TAKEOFF_HEIGHT * 0.85
    deadline = time.time() + timeout
    best = start_alt

    while time.time() < deadline:
        drain_telemetry()
        if not is_armed():
            raise FlightError("Vehicle disarmed during takeoff.")
        alt = current_altitude()
        if alt is not None:
            best = max(best, alt)
            if alt >= goal:
                print(f"Reached {alt:.2f} m")
                return True
        time.sleep(0.05)

    if best - start_alt < MIN_CLIMB_M:
        raise FlightError(
            f"Vehicle never left the ground (max alt {best:.2f} m, "
            f"started at {start_alt:.2f} m). Aborting.")
    print(f"Takeoff height not fully reached (max {best:.2f} m) but the "
          f"vehicle is airborne - continuing.")
    return False


# ---------------- hold ----------------

def capture_hold_target():
    print("\nSettling before capturing hold target...")
    t_end = time.time() + SETTLE_TIME
    while time.time() < t_end:
        drain_telemetry()
        if not is_armed():
            raise FlightError("Vehicle disarmed while settling.")
        time.sleep(0.02)

    print("Sampling position...")
    samples = []
    t_end = time.time() + 2.0
    while time.time() < t_end:
        drain_telemetry()
        if last_pos:
            samples.append(last_pos)
        time.sleep(0.02)

    if not samples:
        raise FlightError("No LOCAL_POSITION_NED received - check message "
                          "interval / link.")

    x = sum(p[0] for p in samples) / len(samples)
    y = sum(p[1] for p in samples) / len(samples)
    z = sum(p[2] for p in samples) / len(samples)
    print(f"Hold target: x={x:.3f} y={y:.3f} z={z:.3f} ({len(samples)} samples)")

    if last_rf is not None:
        if last_rf < RF_ABORT_M:
            raise FlightError(
                f"Rangefinder reads {last_rf:.2f} m - vehicle appears to be on "
                f"the ground. Refusing to start hold.")
        print(f"  (rangefinder {last_rf:.2f} m AGL vs EKF-implied {-z:.2f} m; "
              f"a large mismatch indicates EKF origin drift)")
    return x, y, z


def send_position_target(x, y, z):
    master.mav.set_position_target_local_ned_send(
        ms_since_boot(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        MASK_POSITION_ONLY,
        x, y, z,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0
    )


def send_velocity_target(vx=0.0, vy=0.0, vz=0.0):
    master.mav.set_position_target_local_ned_send(
        ms_since_boot(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        MASK_VELOCITY_ONLY,
        0.0, 0.0, 0.0,
        vx, vy, vz,
        0.0, 0.0, 0.0,
        0.0, 0.0
    )


def hold_loop(hold_x, hold_y, hold_z):
    print(f"\nStarting hold (mode={HOLD_MODE})\n")
    period = 1.0 / SEND_RATE_HZ
    start = time.time()
    next_send = start

    while time.time() - start < HOLD_TIME:
        now = time.time()
        drain_telemetry()

        if not is_armed():
            print("Vehicle disarmed - ending hold.")
            return
        if RF_ABORT_ENABLE and last_rf is not None and last_rf < RF_ABORT_M:
            print(f"Rangefinder {last_rf:.2f} m below abort threshold "
                  f"{RF_ABORT_M} m - aborting hold.")
            return

        if now >= next_send:
            next_send += period
            if next_send < now:
                next_send = now + period

            rf_str = f"  RF: {last_rf:.2f}m" if last_rf is not None else ""
            if HOLD_MODE == "velocity":
                send_velocity_target(0.0, 0.0, 0.0)
                if last_pos:
                    print(f"Vel target: 0 0 0   Current: "
                          f"{last_pos[0]:.2f} {last_pos[1]:.2f} "
                          f"{last_pos[2]:.2f}{rf_str}")
            else:
                send_position_target(hold_x, hold_y, hold_z)
                if HOLD_BLEND > 0 and last_pos:
                    hold_x += (last_pos[0] - hold_x) * HOLD_BLEND
                    hold_y += (last_pos[1] - hold_y) * HOLD_BLEND
                if last_pos:
                    print(f"Target: {hold_x:.2f} {hold_y:.2f} {hold_z:.2f}  "
                          f"Current: {last_pos[0]:.2f} {last_pos[1]:.2f} "
                          f"{last_pos[2]:.2f}{rf_str}")

        time.sleep(0.005)


def land():
    global landed
    if landed:
        return
    landed = True
    print("\nLanding")
    try:
        # Not "required" - if LAND is refused we still want to try RTL/disarm.
        if not set_mode("LAND", required=False):
            print("LAND not confirmed - retrying once.")
            set_mode("LAND", required=False)
    except Exception as e:
        print("Land command failed:", e)


def shutdown(sig, frame):
    print("\nInterrupted - landing")
    land()
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


# ================= MAIN =================

def main():
    global master, mode_map
    master = connect()
    mode_map = resolve_mode_map()

    request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, TELEM_RATE_HZ)
    request_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, TELEM_RATE_HZ)

    # Warm the caches so is_armed() / altitude checks have real data.
    t_end = time.time() + 2.0
    while time.time() < t_end:
        drain_telemetry()
        time.sleep(0.02)

    if is_armed():
        raise FlightError("Vehicle is already armed - refusing to start.")

    set_mode("GUIDED")          # raises on rejection - no silent continue
    arm()                       # raises unless armed is confirmed

    try:
        takeoff()
        print("\nWaiting after takeoff...")
        wait_for_position()
        wait_for_takeoff_height()
        hold_x, hold_y, hold_z = capture_hold_target()
        hold_loop(hold_x, hold_y, hold_z)
        land()
    except FlightError as e:
        print("FLIGHT ABORT:", e)
        if current_altitude() is not None and current_altitude() > MIN_CLIMB_M:
            land()
        else:
            disarm()
        raise

    print("Finished")


if __name__ == "__main__":
    try:
        main()
    except FlightError as e:
        print("ERROR:", e)
        sys.exit(1)
    except Exception as e:
        print("UNEXPECTED ERROR:", e)
        try:
            if current_altitude() is not None and current_altitude() > MIN_CLIMB_M:
                land()
            else:
                disarm()
        except Exception:
            pass
        raise
