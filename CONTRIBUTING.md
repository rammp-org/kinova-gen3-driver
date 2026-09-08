# Contributing

This repo follows the RAMMP module workflow, matching
[`kinova-gen3-ros2`](https://github.com/rammp-org/kinova-gen3-ros2), with the
additions a real-time control core needs.

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

`clang-format` is deliberately **not** in the hook set yet. The repo has no
`.clang-format` and ~40 source files written to a consistent house style by hand;
adopting it means one bulk reformat plus a `.git-blame-ignore-revs` entry, which
is its own reviewable change rather than a rider on something else.

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

CI runs both on every PR into `main` or `dev`:

| Gate | What it proves |
|---|---|
| `ctest` | The suite passes. |
| `unit_tests --gtest_filter='RtSafety*'` | **The RT contract holds** — zero major page faults and zero dropped telemetry samples in steady state. |

**A change to the RT path is not done until the RT-safety gate has been run and
read.** `compute()` and the executor cycle may not allocate, lock, or block; the
gate is what catches a regression, and CI is what makes it unskippable.

### What CI cannot tell you

CI runs on a hosted x86-64 runner. It is not `PREEMPT_RT`, has no isolated core,
and cannot get `SCHED_FIFO` — `rt_system` degrades to `SCHED_OTHER` by design.
So CI proves *allocation-freedom*, never *timing*.

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

The driver ships **0.x tags** while the API is still moving; a minor bump may
break the public surface (`include/kinova_lowlevel/`, and specifically the
interface tier's ports that front-ends implement against). Downstream repos pin
an exact tag, never a branch.

The bar for 1.0 is one release cycle in which nothing downstream needs an
adoption commit. See #37 and the [v1.0.0
milestone](https://github.com/rammp-org/kinova-gen3-driver/milestone/1).
