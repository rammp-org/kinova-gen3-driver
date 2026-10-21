"""onarm/wrist_sweep.sh's motion plan, run against a fake move.py and docker (no arm)."""

import os
import shutil
import subprocess
from collections import defaultdict
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "onarm" / "wrist_sweep.sh"
FAKE_MOVE = """import sys
a = next(x for x in sys.argv if x.startswith("--angles="))[9:]
open(__file__.replace("move.py", "moves.log"), "a").write(a + " " + sys.argv[sys.argv.index("--hold") + 1] + "\\n")
"""
# `docker ps -a` lists nothing (no stale logger); `docker ps` lists the running logger.
FAKE_DOCKER = '#!/bin/sh\n[ "$1" = ps ] && [ "$2" = -a ] && exit 0\n[ "$1" = ps ] && echo gid-dryrun\nexit 0\n'


def run(tmp_path, *args):
    shutil.copy(SCRIPT, tmp_path)
    (tmp_path / "move.py").write_text(FAKE_MOVE)
    (tmp_path / "dryrun.sh").write_text("#!/bin/sh\nexit 0\n")
    (tmp_path / "bin").mkdir()
    (tmp_path / "bin" / "docker").write_text(FAKE_DOCKER)
    for f in ("dryrun.sh", "bin/docker"):
        (tmp_path / f).chmod(0o755)
    script = tmp_path / "wrist_sweep.sh"
    script.write_text(script.read_text().replace("\nsleep 5\n", "\nsleep 0\n"))
    env = {**os.environ, "PATH": f"{tmp_path / 'bin'}:{os.environ['PATH']}"}
    r = subprocess.run(["bash", str(script), *args], capture_output=True, text=True, env=env)
    log = tmp_path / "moves.log"
    moves = [(list(map(float, a.split(","))), float(h)) for a, h in
             (line.split() for line in log.read_text().splitlines())] if log.exists() else []
    return r, moves


def test_every_hold_two_sided_and_no_net_winding(tmp_path):
    r, moves = run(tmp_path, "0,30,180,-90,0", "30", "90", "-40")
    assert r.returncode == 0 and "DONE 0" in r.stdout
    q = [m[0] for m in moves]
    assert all(p[:5] == [0, 30, 180, -90, 0] for p in q)
    j7 = [p[6] for p in q]
    steps = [b - a for a, b in zip(j7, j7[1:])]
    assert -175 <= min(j7) and max(j7) <= 175  # never across +-180: no short way round
    assert max(map(abs, steps)) <= 90 and sum(steps) == 0
    sides = defaultdict(set)
    for (prev, _), (p, hold) in zip(moves, moves[1:]):
        if hold > 0:
            sides[(p[5], p[6])].add((prev[5] > p[5], prev[6] > p[6]))
    assert len(sides) == 3 * 12
    assert all(s == {(True, True), (False, False)} for s in sides.values())
    assert all(hold == 0 or (p[5], p[6]) in sides for p, hold in moves)  # no held waypoint


@pytest.mark.parametrize("j6s", [("30.5", "90"), ("30", "30"), ("30", "030")])
def test_bad_j6_angles_refused_before_anything_moves(tmp_path, j6s):
    r, moves = run(tmp_path, "0,30,180,-90,0", *j6s)
    assert r.returncode == 2 and moves == []
