#!/usr/bin/env python3
"""Move the Gen3 with Kinova's OWN high-level joint moves (what the Web App does).

The arm stays under Kinova's controller (single-level servoing) the whole time, so
nothing here depends on our gravity model. Refuses to run if anything holds the arm
in low-level servoing. Ctrl-C stops the arm (Base.Stop).

  move.py --angles=0,0,0,0,0,0,0 --speed 15          # one move (deg), e.g. candle
                                                      # (use --angles=..., a leading "-" is read as a flag)
  move.py --poses poses.csv --speed 15 --hold 6       # the whole grid, holding each pose
  move.py --poses poses.csv --start P10 --plan        # print what would run, move nothing

Run with PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python (kortex_api vs protobuf).
"""
import argparse
import csv
import sys
import threading
import time

from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2

MAX_SPEED = 20.0  # deg/s, hard cap
# URDF limits of the revolute joints (deg); continuous joints are None.
LIMITS = [None, 128.9, None, 147.8, None, 120.3, None]


def check(angles):
    for i, (a, lim) in enumerate(zip(angles, LIMITS)):
        if lim is not None and abs(a) > lim - 2.0:
            sys.exit(f"j{i + 1} target {a:.1f} deg is within 2 deg of its {lim} limit; refusing")


class Arm:
    def __init__(self, ip):
        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, lambda e: print("router error:", e))
        self.transport.connect(ip, 10000)
        info = Session_pb2.CreateSessionInfo(username="admin", password="admin",
                                             session_inactivity_timeout=60000,
                                             connection_inactivity_timeout=2000)
        self.session = SessionManager(self.router)
        self.session.CreateSession(info)
        self.base = BaseClient(self.router)
        mode = self.base.GetServoingMode().servoing_mode
        if mode == Base_pb2.LOW_LEVEL_SERVOING:
            self.close()
            sys.exit("arm is in LOW_LEVEL_SERVOING: something else is driving it; refusing")
        if mode != Base_pb2.SINGLE_LEVEL_SERVOING:
            self.base.SetServoingMode(Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING))

    def angles(self):
        a = [j.value for j in self.base.GetMeasuredJointAngles().joint_angles]
        return [x - 360.0 if x > 180.0 else x for x in a]

    def move(self, target, speed, timeout_s, travel=None):
        done, result = threading.Event(), {}

        def on_event(n):
            if n.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
                result["event"] = n.action_event
                if n.action_event == Base_pb2.ACTION_ABORT:
                    from kortex_api.autogen.messages import Errors_pb2
                    try:
                        why = Errors_pb2.SubErrorCodes.Name(n.abort_details)
                    except ValueError:
                        why = str(n.abort_details)
                    print("           ABORT reason:", why, flush=True)
                done.set()

        handle = self.base.OnNotificationActionTopic(on_event, Base_pb2.NotificationOptions())
        action = Base_pb2.Action(name="gravity_id move")
        rja = action.reach_joint_angles
        for i, a in enumerate(target):
            ja = rja.joint_angles.joint_angles.add()
            ja.joint_identifier, ja.value = i, a % 360.0
        # A DURATION constraint (1.5x the time at `speed`, so the peak stays near `speed`):
        # on dojo a SPEED constraint was rejected (METHOD_FAILED) for a 104 deg move that
        # Kinova's own validator accepts.
        rja.constraint.type = Base_pb2.JOINT_CONSTRAINT_DURATION
        rja.constraint.value = max(1.5 * travel / speed, 2.0)
        try:
            self.base.ExecuteAction(action)
            if not done.wait(timeout_s):
                self.base.Stop()
                sys.exit(f"move did not finish in {timeout_s:.0f} s; arm stopped")
        except KeyboardInterrupt:
            self.base.Stop()
            sys.exit("interrupted; arm stopped")
        finally:
            self.base.Unsubscribe(handle)
        if result["event"] != Base_pb2.ACTION_END:
            sys.exit("move ABORTED by the arm (limit, collision or fault); stopping the sequence")

    def close(self):
        self.session.CloseSession()
        self.transport.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.10")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--angles", help="7 comma-separated degrees")
    g.add_argument("--poses", help="gravity_id poses.csv")
    g.add_argument("--where", action="store_true", help="connect, print joint angles, move nothing")
    ap.add_argument("--start", help="first pose id of the sequence")
    ap.add_argument("--stop", help="last pose id of the sequence (inclusive)")
    ap.add_argument("--speed", type=float, default=15.0, help=f"max joint speed deg/s (cap {MAX_SPEED})")
    ap.add_argument("--hold", type=float, default=6.0, help="seconds to hold still after each move")
    ap.add_argument("--plan", action="store_true", help="print the moves, move nothing")
    ap.add_argument("--bidirectional", type=float, default=0.0, metavar="DEG",
                    help="approach every pose twice, from +DEG and from -DEG on every joint, and "
                         "hold after each arrival (the sensors see joint friction, so one-sided "
                         "approaches bias the reading)")
    a = ap.parse_args()
    if not 0 < a.speed <= MAX_SPEED:
        sys.exit(f"--speed must be in (0, {MAX_SPEED}]")

    if a.where:
        arm = Arm(a.ip)
        print("servoing mode:", Base_pb2.ServoingMode.Name(arm.base.GetServoingMode().servoing_mode))
        print("joint angles [deg]:", [round(x, 2) for x in arm.angles()])
        arm.close()
        return
    if a.angles:
        targets = [("angles", [float(x) for x in a.angles.split(",")])]
    else:
        with open(a.poses) as f:
            rows = list(csv.DictReader(f))
        ids = [r["id"] for r in rows]
        lo = ids.index(a.start) if a.start else 0
        hi = ids.index(a.stop) + 1 if a.stop else len(rows)
        targets = [(r["id"], [float(r[f"j{i}_deg"]) for i in range(1, 8)]) for r in rows[lo:hi]]
    for name, t in targets:
        if len(t) != 7:
            sys.exit(f"{name}: need 7 angles")
        check(t)
    if a.bidirectional:
        def nudged(t, sign):
            out = []
            for x, lim in zip(t, LIMITS):
                y = x + sign * a.bidirectional
                out.append(x if lim is not None and abs(y) > lim - 10.0 else y)  # no room: skip
            return out
        targets = [step for name, t in targets for step in (
            (f"{name}+", nudged(t, +1)), (name, t), (f"{name}-", nudged(t, -1)), (name, t))]
    if a.plan:
        for name, t in targets:
            print(name, " ".join(f"{x:8.2f}" for x in t))
        return

    arm = Arm(a.ip)
    try:
        for name, t in targets:
            here = arm.angles()
            delta = max(abs((x - y + 180) % 360 - 180) for x, y in zip(t, here))
            print(f"[{time.strftime('%H:%M:%S')}] {name}: max joint travel {delta:.1f} deg", flush=True)
            arm.move(t, a.speed, timeout_s=2.5 * delta / a.speed + 15.0, travel=delta)
            if name[-1] in "+-" and a.bidirectional:
                continue  # an approach waypoint, not a pose: no hold
            time.sleep(a.hold)
            print(f"           reached {[round(x, 2) for x in arm.angles()]}", flush=True)
    finally:
        arm.close()


if __name__ == "__main__":
    main()
