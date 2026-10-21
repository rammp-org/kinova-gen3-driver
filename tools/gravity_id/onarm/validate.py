#!/usr/bin/env python3
"""Validate joint targets with Kinova's planner WITHOUT moving: ValidateWaypointList."""
import sys
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2 as B

def validate(base, targets, speed):
    wl = B.WaypointList(use_optimal_blending=False)
    prev = [v - 360.0 if v > 180.0 else v for v in (j.value for j in base.GetMeasuredJointAngles().joint_angles)]
    for name, t in targets:
        w = wl.waypoints.add(name=name)
        w.angular_waypoint.angles.extend([a % 360.0 for a in t])
        travel = max(abs((x - y + 180) % 360 - 180) if lim is None else abs(x - y)  # see move.travel
                     for x, y, lim in zip(t, prev, [None, 1, None, 1, None, 1, None]))
        w.angular_waypoint.duration = max(travel / speed * 1.5, 1.0)
        prev = t
    rep = base.ValidateWaypointList(wl)
    return rep.trajectory_error_report.trajectory_error_elements

if __name__ == "__main__":
    t_ = TCPTransport(); r = RouterClient(t_, lambda e: print(e)); t_.connect("192.168.1.10", 10000)
    s = SessionManager(r)
    s.CreateSession(Session_pb2.CreateSessionInfo(username="admin", password="admin", session_inactivity_timeout=60000, connection_inactivity_timeout=2000))
    base = BaseClient(r)
    targets = [(l.split()[0], [float(x) for x in l.split()[1:8]]) for l in sys.stdin if l.strip()]  # move.py --plan output
    for e in validate(base, targets, 15.0):
        print("ERR", B.TrajectoryErrorType.Name(e.error_type), "id", e.error_identifier, "val", round(e.error_value, 3),
              "range", round(e.min_value, 3), round(e.max_value, 3), "idx", e.index, "wp", e.waypoint_index, repr(e.message))
    print("validated", len(targets))
    s.CloseSession(); t_.disconnect()
