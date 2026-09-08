# Contributing

This repo follows the RAMMP module workflow, with the additions a real-time
control core needs.

Read `CLAUDE.md` first — it states the architectural invariants (the seven units
and the boundaries between them) and the RT contract. This file is about
*process*: branches, gates, and how to verify a change.

## Branches

- `main` — stable, deployable. Updated periodically from `dev`. This is what
  downstream repos pin.
- `dev` — staging ground for tested new code. PRs land here.
- `feature/<issue-number>-<brief-description>` — forked from the latest `dev`.
  Use `bug/<issue-number>-<brief-description>` for fixes.

Every branch name carries its issue number, so the branch list reads as a work
list.

## Pre-commit hooks

Hygiene is enforced before each commit — file hygiene, YAML/XML validity,
`actionlint` on the workflows, and `shellcheck` on `scripts/` (those scripts
grant RT privileges, so a quoting bug there is not a style problem).

Run this once after cloning:

```bash
uv tool install pre-commit
pre-commit install
```

Without `uv`: `pip install pre-commit && pre-commit install`.

Hook revisions are pinned. Updating them is a deliberate PR
(`pre-commit autoupdate`), never silent drift.

C++ is formatted by `clang-format` (stock Google at 100 columns, see
`.clang-format`). The hook is pinned to the exact version the tree was formatted
with — clang-format output differs between major versions, so bumping it means
re-running the sweep, not just editing the rev.

One bulk reformat is recorded in `.git-blame-ignore-revs`. Enable it so
`git blame` reaches the author rather than the reformat:

```bash
git config blame.ignoreRevsFile .git-blame-ignore-revs
```

## Building and testing

The default build is **sim-only** and needs no KORTEX SDK — the whole test suite
runs with no robot attached. It needs Pinocchio via a cmeel prefix:

```bash
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  && cmake --build build -j && ctest --test-dir build --output-on-failure
```

`CMAKE_PREFIX_PATH` must point at the prefix holding `pinocchioConfig.cmake`
(`<prefix>/lib/cmake/pinocchio/`); adjust for the Python version.

All tests are one gtest binary (`unit_tests`), registered as a single ctest test.
Run a subset with a filter: `./build/unit_tests --gtest_filter='CartesianImpedance*'`.

The real-robot build is opt-in and needs an aarch64 SDK:
`-DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=<dir>`.

## The gates

CI runs all of these on every PR into `main` or `dev`, on **both amd64 and
arm64** — arm64 is the only architecture this driver actually deploys to, so a
green amd64 build alone would be testing a machine nobody runs.

| Gate | What it proves |
|---|---|
| `ctest` | The suite passes. |
| `unit_tests --gtest_filter='RtSafety*'` | **Allocation-freedom** — zero major page faults and zero dropped telemetry samples in steady state. |
| Install + consume | The exported package resolves at the current major.minor, refuses the next major, and `package.xml` agrees with `project(VERSION)`. |
| Sim smoke | The executor, transport, dynamics and telemetry run a 1 kHz loop together. |
| **KORTEX build** | The real-arm transport still compiles and links, and the symbol is actually in the binary. The SDK is fetched from Kinova's public artifactory, per architecture. |
| KORTEX guard | With the option on and no SDK, configure fails loudly and for the documented reason. |

The KORTEX job matters more than it looks: `src/kortex_transport.cpp` compiles
only under `-DKINOVA_ENABLE_KORTEX=ON`, and it is the only real-path consumer of
`Transport`, `joint_types` and `units`. Without that job, a change to a shared
type breaks it silently and you find out on the arm.

**No CI job ever touches a robot.** The KORTEX job builds and runs the
transport-agnostic suite; nothing passes an `--ip`.

**A change to the RT path is not done until that gate has been run and read.**
`compute()` and the executor cycle may not allocate, lock, or block; the gate is
what catches a regression, and CI is what makes it unskippable.

### What CI cannot tell you

The RT contract has two halves, and CI only gates one of them.

CI runs on hosted runners. They are not `PREEMPT_RT`, have no isolated core,
and cannot get `SCHED_FIFO` — `rt_system` degrades to `SCHED_OTHER` by design.
So the half CI gates is the one that is a property of the *code*: the allocation
cases assert `majflt_delta == 0` and `ring.dropped() == 0` after a warm-up run,
which is broken by an allocation, a lock, or a blocking call on the RT path
regardless of which kernel it runs on.

`RtSafety.NanosleepPacingProducesSamples` is the exception and is excluded from
the gate: it asserts a throughput floor rather than either of those, which is a
timing claim this runner cannot honestly make. It still runs under `ctest`.

The other half — latency, jitter, deadline misses — is a property of the *kernel
and the machine*, and CI says nothing about it. A green build is not evidence
that a change is fast enough.

Any change that plausibly affects per-cycle cost or jitter has to be measured on
the Jetson, on real numbers:

```bash
./build/benchmark_grav_comp --sim --urdf models/gen3_7dof.urdf --rate 1000 --duration 5 --csv before.csv
```

Compare p50 / p99 / p99.9 / max, overruns and faults before and after. Note that
`NanoHistogram::percentile` returns the lower bound of a log2 bucket, so console
percentiles are low-biased — tail conclusions come from the CSV.

"Looks fine" is not evidence for a driver this much depends on.

## Documentation

Docs are a first-class deliverable, not a follow-up. **When you change behaviour,
update the docs in the same change**, and add new pages to the `nav:` in
`mkdocs.yml` — an orphaned page will not appear on the site.

The one deliberate exception is `docs/integration/`, which stays out of the nav:
those pages are bring-up and hardware-procedure records for whoever is at the
arm, and they render on GitHub, which is where that reader already is.

```bash
pip install mkdocs mkdocs-material
mkdocs serve      # live preview
mkdocs build      # static site into ./site
```

## On-robot work

**Attended only.** Follow `docs/integration-runbook.md`: e-stop in hand.

Do not enable `KINOVA_ENABLE_KORTEX` and connect to hardware unattended. Claim
the cell before driving the arm, and never lift the hardware guard yourself — it
is a human's statement that the cell is attended right now.

## Versioning

Semantic versioning, from **1.0.0** on. **Downstream repos pin an exact tag,
never a branch.**

- **MAJOR** — a breaking change to the public surface.
- **MINOR** — additive: new modes, new methods with a default body, new fields.
- **PATCH** — fixes that change no declared signature or contract.

### The public surface

Everything under `include/kinova_lowlevel/`, plus the behavioural guarantees the
rest of this file states: SI/radians internally, and the RT contract (no
allocation, lock, or blocking call in `compute()` or the executor cycle).

Not covered: `src/`, tests, docs, and anything a new `ControlMode`
*implementation* adds — a new mode is additive by construction.

### The rule that is easy to get wrong

**Adding a pure virtual to an interface someone else implements is a MAJOR
change, even though adding things is usually minor.** Give the new method a
default implementation and it stays minor.

This binds hardest on `interface/ports.h`. Those ports are implemented
downstream — `CommandSink`, `ActionServerPort`, `ArbitrationSink` and friends
appear in the ROS2 front-end's production code *and* across several of its test
fakes, so a new pure virtual breaks the build in more places than the one that
is obvious from here. Both breaking changes in the gripper round were of exactly
this shape.

`ControlMode` is the milder case: it is implemented only inside this repo, so a
new pure virtual there is a same-commit fix across the five modes. Still declare
it, but it is not the trap.

### Cutting a release

1. Bump `project(... VERSION x.y.z)` in `CMakeLists.txt` **and** `<version>` in
   `package.xml` — the generated `kinova_lowlevelConfigVersion.cmake` follows the
   first, and ROS tooling reads the second.
2. Merge `dev` into `main`.
3. Tag `main` as `vx.y.z` and push the tag.
4. Move the consuming repo's `.repos` pin to the new tag.

Compatibility is `SameMajorVersion`, so a consumer asking
`find_package(kinova_lowlevel 1.0 CONFIG REQUIRED)` accepts any 1.x and refuses
2.0.

Note that the abra bare-metal loop rsyncs a local working tree and bypasses
`.repos` entirely, so a tag pin alone protects only the container build. The
`find_package` version check is what protects both, because it fires at
configure time however the source arrived.
