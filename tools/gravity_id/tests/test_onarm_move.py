"""onarm/move.py's safety paths, against a fake kortex_api (no arm, no kortex install)."""

import importlib.util
import sys
import types
from pathlib import Path

import pytest

MOVE_PY = Path(__file__).resolve().parents[1] / "onarm" / "move.py"
END, ABORT, PREPROCESS_ABORT = 1, 2, 7


class FakeBase:
    """Records what move.py asks of Kinova; `outcome` is the event each action ends in."""

    def __init__(self, angles, outcome=END):
        self.angles, self.outcome = list(angles), outcome
        self.actions, self.stops = [], 0

    def GetServoingMode(self):
        return types.SimpleNamespace(servoing_mode="SINGLE")

    def SetServoingMode(self, _):
        pass

    def GetMeasuredJointAngles(self):
        return types.SimpleNamespace(joint_angles=[types.SimpleNamespace(value=a % 360) for a in self.angles])

    def OnNotificationActionTopic(self, cb, _opts):
        self.cb = cb
        return "handle"

    def Unsubscribe(self, _):
        pass

    def ExecuteAction(self, action):
        self.actions.append(action)
        if self.outcome == END:
            self.angles = [((j.value + 180) % 360) - 180 for j in action.joint_angles]
        self.cb(types.SimpleNamespace(action_event=self.outcome, abort_details=0))

    def Stop(self):
        self.stops += 1


def load_move(monkeypatch, base):
    """Import move.py with every kortex_api module it touches replaced by fakes."""
    class Msg(types.SimpleNamespace):
        pass

    class Action:
        def __init__(self, name=""):
            self.joint_angles = []
            outer = self

            class JA:
                def add(self_inner):
                    j = types.SimpleNamespace(joint_identifier=0, value=0.0)
                    outer.joint_angles.append(j)
                    return j

            self.reach_joint_angles = types.SimpleNamespace(
                joint_angles=types.SimpleNamespace(joint_angles=JA()),
                constraint=types.SimpleNamespace(type=None, value=None))

    base_pb2 = types.SimpleNamespace(
        ACTION_END=END, ACTION_ABORT=ABORT, ACTION_PREPROCESS_ABORT=PREPROCESS_ABORT,
        LOW_LEVEL_SERVOING="LOW", SINGLE_LEVEL_SERVOING="SINGLE", JOINT_CONSTRAINT_DURATION="DUR",
        Action=Action, NotificationOptions=Msg, ServoingModeInformation=Msg,
        ServoingMode=types.SimpleNamespace(Name=lambda m: m))
    errors_pb2 = types.SimpleNamespace(SubErrorCodes=types.SimpleNamespace(Name=lambda d: "X"))
    session_pb2 = types.SimpleNamespace(CreateSessionInfo=Msg)
    fakes = {
        "kortex_api": types.ModuleType("kortex_api"),
        "kortex_api.RouterClient": types.SimpleNamespace(RouterClient=lambda *a: None),
        "kortex_api.SessionManager": types.SimpleNamespace(SessionManager=lambda r: types.SimpleNamespace(
            CreateSession=lambda i: None, CloseSession=lambda: None)),
        "kortex_api.TCPTransport": types.SimpleNamespace(TCPTransport=lambda: types.SimpleNamespace(
            connect=lambda *a: None, disconnect=lambda: None)),
        "kortex_api.autogen": types.ModuleType("kortex_api.autogen"),
        "kortex_api.autogen.client_stubs": types.ModuleType("kortex_api.autogen.client_stubs"),
        "kortex_api.autogen.client_stubs.BaseClientRpc": types.SimpleNamespace(BaseClient=lambda r: base),
        "kortex_api.autogen.messages": types.SimpleNamespace(Base_pb2=base_pb2, Session_pb2=session_pb2,
                                                             Errors_pb2=errors_pb2),
        "kortex_api.autogen.messages.Errors_pb2": errors_pb2,
    }
    for k, v in fakes.items():
        monkeypatch.setitem(sys.modules, k, v)
    spec = importlib.util.spec_from_file_location("onarm_move", MOVE_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run(monkeypatch, mod, *args):
    monkeypatch.setattr(sys, "argv", ["move.py", *args])
    monkeypatch.setattr(mod.time, "sleep", lambda s: None)
    mod.main()


def test_travel_counts_limited_joints_the_long_way(monkeypatch):
    mod = load_move(monkeypatch, FakeBase([0] * 7))
    assert mod.travel([0, 130, 0, 0, 0, 0, 0], [0, -130, 0, 0, 0, 0, 0]) == 260   # j2 limited
    assert mod.travel([170, 0, 0, 0, 0, 0, 0], [-170, 0, 0, 0, 0, 0, 0]) == pytest.approx(20)  # j1 wraps


def test_duration_follows_real_travel(monkeypatch):
    base = FakeBase([0, -130, 0, 0, 0, 0, 0])
    mod = load_move(monkeypatch, base)
    run(monkeypatch, mod, "--angles=0,100,0,0,0,0,0", "--speed", "10", "--hold", "0", "--from-anywhere")
    assert base.actions[0].reach_joint_angles.constraint.value == pytest.approx(1.5 * 230 / 10)


def test_long_first_move_refused_without_flag(monkeypatch):
    base = FakeBase([0, 15, 180, -130, 0, 55, 90])  # Kinova Home
    mod = load_move(monkeypatch, base)
    with pytest.raises(SystemExit, match="from-anywhere"):
        run(monkeypatch, mod, "--angles=0,0,0,0,0,0,0")
    assert base.actions == []  # refused BEFORE anything was sent


@pytest.mark.parametrize("outcome", [ABORT, PREPROCESS_ABORT])
def test_abort_stops_the_arm(monkeypatch, outcome):
    base = FakeBase([0] * 7, outcome=outcome)
    mod = load_move(monkeypatch, base)
    with pytest.raises(SystemExit, match="ABORTED"):
        run(monkeypatch, mod, "--angles=5,0,0,0,0,0,0")
    assert base.stops == 1


def test_ctrl_c_during_hold_stops_the_arm(monkeypatch):
    base = FakeBase([0] * 7)
    mod = load_move(monkeypatch, base)
    monkeypatch.setattr(sys, "argv", ["move.py", "--angles=5,0,0,0,0,0,0", "--hold", "6"])

    def interrupt(_):
        raise KeyboardInterrupt

    monkeypatch.setattr(mod.time, "sleep", interrupt)
    with pytest.raises(SystemExit, match="interrupted"):
        mod.main()
    assert base.stops == 1
