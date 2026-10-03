# Nightly Integration Channel — Phase 0 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up `rammp-org/rammp-nightly` as a self-contained nightly channel: build the whole arm stack from same-instant dev SHAs, publish dated image tags + `:nightly` aliases + a `.repos` manifest, **touching no other repo**.

**Architecture:** One repo. A scheduled workflow resolves the five member SHAs, then builds each image *from that member's own unmodified build recipe* in its own job (base ∥ planner → node), pushing dated tags with a `write:packages` PAT. Cross-repo pins the members can't yet accept as inputs are applied as build-time patches inside the nightly's checkout (the node's `FROM` line and its CuRobo `.repos` pin). A final job re-points `:nightly`, records digests, and commits the manifest. This is spec **Phase 0**; the per-repo dispatch contracts are the later migration.

**Tech Stack:** GitHub Actions on `ubuntu-*-arm` runners, docker + buildx/imagetools, ghcr, vcstool `.repos` manifests, Python 3 + PyYAML + pytest (via `uv`).

**Spec:** `docs/superpowers/specs/2026-10-02-nightly-integration-channel-design.md` (kinova-gen3-driver, same branch) — see its "Phase 0" section for what is deferred and why.

## Global Constraints

- **No commits to any repo but rammp-nightly** (and this docs branch). Member repos are checked out read-only at SHAs; patches exist only in the runner's workspace.
- Nightly tags match `^nightly-[0-9]{8}(-rc[0-9]+)?$`; the resolve job enforces it and refuses a set id that already has a manifest. `:nightly` moves only in the bless job, only on full green.
- Release behavior of member repos is untouched by construction — nothing in them changes.
- All images arm64, hosted arm runners, matching each member's own CI.
- Secrets: `NIGHTLY_GHCR_TOKEN` (classic PAT, `write:packages` + `read:packages`) is the only credential; SHA resolution of public repos uses the default token.

## Review Focus

1. **Patched builds drift from the repo's declared sources** (the whole point, but silently wrong if a patch stops matching — e.g. the node repo changes its base pin format and the `sed` matches nothing) → every patch asserts it changed the file (`grep` after `sed`, fail loud). Pinned in Task 2's node job.
2. **A nightly tag lands in release space** → resolve-job regex + manifest tooling never accept it. Pinned by Task 1's tests and Task 2's resolve guard; rehearsed in Task 3.
3. **Re-running a blessed date rebuilds an "immutable" set** → `write_manifest.py` refuses existing files (Task 1 test) and resolve refuses a set id already in `manifests/` (Task 2).
4. **Partial night promotes** → bless `needs:` every build job; a red leg means `:nightly` and the manifest never move; dated tags already pushed that night are unblessed debris, documented as such. Structural in Task 2; exercised in Task 3 (deliberate red).
5. **Disk exhaustion on shared runners** (base+cuda ≈ several GB, planner ≈ 25 GB) → one image family per job, each job reclaims preinstalled toolchains first, mirroring the members' own CI. Structural in Task 2; watched during Task 3's first full run.

---

### Task 1: Scaffold + manifest tooling (TDD)

**Files:**
- Create: `pyproject.toml`, `stacks/kinova-arm.yml`, `scripts/stack_members.py`, `scripts/write_manifest.py`, `tests/test_stack_members.py`, `tests/test_write_manifest.py`, `README.md`, `.gitignore`

**Interfaces (produced for Task 2):**
- `python3 scripts/stack_members.py stacks/kinova-arm.yml` → JSON `{name: {url, branch}}`.
- `python3 scripts/write_manifest.py --stack <yml> --set-id <id> --shas <json file> --digests <json file> --out-dir <dir>` → writes `<id>.repos` + `<id>.images.json`; exit 1 on existing set or missing member SHA.
- `stacks/kinova-arm.yml`: `stack` (str), `members` (ordered map name → `{url, branch}`), `images` (list).

- [ ] **Step 1: Scaffold locally** — `~/atdev/rammp-nightly`, `git init`. `pyproject.toml` with `pyyaml>=6`, dev group `pytest>=8`. Stack file members: rammp-interfaces-ros2@dev, RAMMP-docker@main, kinova-gen3-driver@dev, RAMMP-CuRobo@dev, kinova-gen3-ros2@dev; images: rammp-base, rammp-cuda, rammp-curobo, kinova-gen3-ros2.
- [ ] **Step 2: Write the four failing tests** — members round-trip; manifest content (`.repos` is vcstool-loadable with exact SHAs, `images.json` carries set/stack/digests); refusal of an existing set id (stderr says "already exists"); refusal of a missing member SHA (stderr names the member). Run `uv run pytest -q`, expect failures (scripts absent).
- [ ] **Step 3: Implement the two scripts** (argparse, PyYAML; `write_manifest.py` writes `{"repositories": {name: {type: git, url, version: <sha>}}}` with `sort_keys=False` and a timestamped images.json).
- [ ] **Step 4: `uv run pytest -q`** → 4 passed.
- [ ] **Step 5: README** — what a set is and its CI-grade guarantee (no RT gate, no arm); consuming via `:nightly` / dated tags / `vcs import` of a manifest; immutability rules; red = tracking issue, channel degrades to stale never broken; phase-0 note that builds run here until member repos can host dispatch contracts.
- [ ] **Step 6: Commit.**

### Task 2: The kinova-arm nightly workflow

**Files:**
- Create: `.github/workflows/kinova-arm.yml`

**Interfaces:** consumes Task 1's scripts/stack file and secret `NIGHTLY_GHCR_TOKEN`.

Jobs (`concurrency: kinova-arm-nightly`, cron `0 9 * * *` + `workflow_dispatch` with per-member ref-override inputs and an `rc` suffix input):

- [ ] **Step 1: `resolve`** — checkout self; `gh api repos/rammp-org/<member>/commits/<ref>` per member (override input wins over stack branch); `set_id=nightly-$(date -u +%Y%m%d)[-rc]`; fail if `manifests/kinova-arm/<set_id>.repos` is tracked; outputs `set_id`, `shas` JSON, `refs` JSON.
- [ ] **Step 2: `base`** (needs resolve; ubuntu-24.04-arm) — checkout `RAMMP-docker` at SHA (`actions/checkout` with `repository:`/`ref:`), free disk, `make base cuda VERSION=<set_id> INTERFACES_REF=<interfaces sha>`, `make test` (their smoke gate), ghcr login with `NIGHTLY_GHCR_TOKEN`, push **dated tags only** for rammp-base + rammp-cuda (explicit `docker tag`/`push`, never `make push` — that would move the `:humble` release alias).
- [ ] **Step 3: `planner`** (needs resolve; ubuntu-22.04-arm, timeout 180) — checkout `RAMMP-CuRobo` at SHA, free disk, `docker build -f docker/Dockerfile -t ghcr.io/rammp-org/rammp-curobo:<set_id> .` (unmodified, still l4t-jetpack per the spec's phase-0 deferral), login, push.
- [ ] **Step 4: `node`** (needs resolve + base; ubuntu-22.04-arm) — checkout `kinova-gen3-ros2` at SHA; **two asserted patches**: `sed` the Dockerfile `FROM ghcr.io/rammp-org/rammp-base:...` → `:<set_id>` then `grep -q "rammp-base:<set_id>" Dockerfile || exit 1`; python-patch `kinova_gen3.repos` RAMMP-CuRobo `version:` → CuRobo SHA, assert changed; build with `--build-arg KINOVA_ENABLE_KORTEX=ON --build-arg CORE_REF=<driver sha>` (the repo's own escape hatch), login, push dated tag.
- [ ] **Step 5: `bless`** (needs resolve, base, planner, node) — login; `docker buildx imagetools create -t ghcr.io/rammp-org/<img>:nightly <img>:<set_id>` for all four; collect digests via `imagetools inspect --format '{{json .Manifest}}'`; `write_manifest.py`; commit `manifests/` to main (`bless <set_id>`); close any open `nightly-red` issue.
- [ ] **Step 6: `report-failure`** (`if: failure()`, needs all) — file-or-comment the single `nightly-red` issue (CuRobo's dedup pattern) with the run URL and "`:nightly` still serves the last green set".
- [ ] **Step 7: actionlint** (`docker run --rm -v "$PWD":/repo -w /repo rhysd/actionlint:latest -color`) → clean; commit. Ship with `schedule:` commented out until Task 3 passes.

### Task 3: Publish, rehearse, turn on

- [ ] **Step 1 [HUMAN unless `gh` is authorized]:** `gh repo create rammp-org/rammp-nightly --public`; push. Create `nightly-red` label. **[HUMAN]** classic PAT (`write:packages`, `read:packages`) → repo secret `NIGHTLY_GHCR_TOKEN`.
- [ ] **Step 2: Rehearsal set** — `gh workflow run kinova-arm.yml -f rc=rc1`; watch. Expect: four dated `-rc1` tags on the members' own ghcr packages, `:nightly` re-pointed, one manifest commit whose digests match `imagetools inspect`.
- [ ] **Step 3: Rehearse the guards** — re-run with the same `rc=rc1` (resolve must refuse the existing set); run with `-f node_ref=<garbage>` (resolve 404s → red issue filed, nothing promoted, `:nightly` digest unchanged).
- [ ] **Step 4: Clean up** — delete `-rc1` ghcr versions; re-point `:nightly` at the last real set if a rehearsal moved it (or leave for the first true nightly); close the rehearsal red issue.
- [ ] **Step 5: Enable cron** — uncomment `schedule:`, push. After the first scheduled bless, point collaborators at the README.
