# Nightly Integration Channel — Design Spec

## Goal

Let collaborators build on dev-branch code without anyone cutting a release.
Today every repo in the arm stack consumes the others only through released,
pinned artifacts, so shipping one new feature to a collaborator forces the full
five-release train in a fixed order: rammp-interfaces-ros2 → rammp-base →
kinova-gen3-driver + RAMMP-CuRobo → move the pins → kinova-gen3-ros2. The
2026-10-02 lock/via/speed train was exactly this, and it is too slow and too
manual to repeat for every collaborator-visible change.

The fix is a second, cheaper channel: every night, build the whole stack from
each repo's dev HEAD, test it **as a set**, and publish the set only if green.
Releases keep their current mechanics and their current guarantee (on-arm
validation, RT gate); they just become rarer and carry larger, well-tested
deltas, because nobody is blocked waiting for one.

The deeper effect is that integration moves from release time to every day.
The train was painful because weeks of divergence got reconciled in one go;
if the stack must assemble every night, integration debt can never pile up
past ~24 hours.

## What the channel guarantees — and what it does not

A published set guarantees exactly: **these five SHAs compile together and
pass the stack's CI test suites together** (sim builds, unit tests, the node's
gtest suites against the set's driver and CuRobo sources). Nothing more.

It does **not** run the RT gate on the Jetson, does not touch the arm, and
carries no hardware validation of any kind. That remains the release channel's
claim, and the gap is the reason releases continue to exist.

Consumers are always on the same interfaces as the images they run — the
channel targets **code compatibility** (the set builds and tests together),
not wire compatibility across mixed sets. Anyone mixing images from different
sets is off the map, same as mixing release versions today. The interface
versioning rules (which field changes are MAJOR, and the Cyclone DDS condition
they depend on) live in rammp-interfaces-ros2's README and are unchanged by
this spec.

## Out of scope

- **Automating releases.** The release train, its pins, and who cuts it are
  untouched. (One release-side convenience falls out for free — see "What
  releases become".)
- **A DAG engine or general build system.** Ordered waves cover this stack and
  anything shaped like it. No cross-stack dependencies, no conditional graphs.
- **A buildfarm.** ROS buildfarm, Zuul, Jenkins and friends solve this problem
  class at a scale that requires operating them as a service. Five repos do
  not justify a second robot. The orchestrator is composed from standard
  GitHub-native parts instead.
- **Wire-compatibility gating.** No IDL diff classification in the pipeline.
  If it ever becomes wanted, tools shaped like `roschema` exist; it bolts on
  without changing this design.

## Phase 0 (2026-10-02): everything lives in rammp-nightly

Standing constraint at rollout: **no changes to the member repos.** The target
architecture below (per-repo dispatch contracts, builds and logs in the owning
repos) needs each image repo to accept nightly inputs, so it waits. Phase 0
delivers the same consumer-facing channel — same dated tags on each repo's own
ghcr package, same `:nightly` aliases, same manifests, same guarantee — with
the builds running inside rammp-nightly's own workflow instead:

- Each leg checks out the member repo at the resolved SHA and runs its **own,
  unmodified** build (RAMMP-docker's Makefile already takes `INTERFACES_REF`;
  the node Dockerfile already takes `CORE_REF` for the driver).
- Cross-repo pins the repos cannot yet accept as inputs are applied as
  **build-time patches in the nightly's checkout**: the node's `FROM` line is
  pointed at tonight's base tag, and the CuRobo entry in its
  `kinova_gen3.repos` copy at the CuRobo SHA. The manifest, not the patched
  checkout, is the set's source of truth.
- Pushes use one classic PAT with `write:packages` (ghcr), stored only in
  rammp-nightly; dated tags and the `:nightly` promotion both happen there.
- Deferred with the repo freeze: the planner-image rebase onto rammp-cuda
  (phase 0 builds it from l4t-jetpack exactly as its repo does today), the
  `/etc/rammp-set.json` stamp (needs a Dockerfile ARG), and `expected_sha`
  (moot — phase 0 checks out SHAs directly, so nothing can move mid-run).

When the freeze lifts, the migration is per-repo and incremental: add a repo's
dispatch contract, switch its leg from "build here" to "dispatch and await",
delete the patch. Tags, manifests and consumers never notice.

## The set

The unit of publication is a **set**: the five repos' dev HEADs resolved at
the same instant, identified by a date — `nightly-20261002`.

A green set publishes:

- **Four images**, each pushed by its own repo's workflow to its own ghcr
  package, right next to that repo's release tags:
  `rammp-base:nightly-<date>` and `rammp-cuda:nightly-<date>` (one RAMMP-docker
  invocation builds both; GPU modules ride rammp-cuda),
  `rammp-curobo:nightly-<date>`, `kinova-gen3-ros2:nightly-<date>`. Dated tags
  are **immutable** — never rebuilt, never re-pointed.
- **A moving alias** `:nightly` on each image, re-pointed to the dated tag
  only after the entire set is green. Collaborators who want "latest known
  good" pull `:nightly`; collaborators who want reproducibility pin a date.
- **A manifest** committed to rammp-nightly under `manifests/kinova-arm/`:
  - `nightly-<date>.repos` — a vcstool file pinning all five repos at exact
    SHAs. This is deliberately the org's existing pin format: a source-build
    collaborator can `vcs import` it directly, and the release-time pin-move
    can be generated from it mechanically.
  - `nightly-<date>.images.json` — the four image digests.
- **A stamp inside each image** at `/etc/rammp-set.json` (set id, the five
  SHAs, build time), so a running container can always answer "which set am
  I from". Fail loud beats silent mis-mapping, here as everywhere.

The manifest history in rammp-nightly is the authority on which dates are
blessed sets. Dated image tags from a night whose later legs failed may exist
on ghcr (an earlier wave already pushed before the failure) — they are inert
debris, not sets: `:nightly` never moved and no manifest names them.

## rammp-nightly: a thin orchestrator

A new repo. It owns **no build logic and writes to no ghcr package** — it
schedules, dispatches, awaits, and records. Its contents:

- `stacks/<name>.yml` — one definition per stack; `stacks/kinova-arm.yml` is
  the first. A stack file declares the member repos, their branches, and the
  images a set publishes — the data both the workflow and the manifest
  tooling read.
- One workflow per stack (cron + manual dispatch with ref overrides). Build
  **order** lives here, not in the stack file: GitHub's `needs:` graph is the
  native way to express waves (each wave a set of parallel dispatches gated
  on the previous wave's success), and a per-stack workflow keeps it legible.
  Per run: resolve every member's branch HEAD to a SHA (one instant) →
  dispatch wave by wave, awaiting each leg → on full green, dispatch the
  promote step into each image repo → commit the manifest. Adding a stack is
  a stack file plus a workflow following this pattern; the shared logic
  (member resolution, manifest writing) lives in `scripts/`, so the workflow
  is wiring, not code.
- `manifests/<stack>/` — the blessed-set history.
- One tracking issue per stack for red nights (updated, not duplicated).

The dispatch-and-wait plumbing is a marketplace action
(`aurelien-baudet/workflow-dispatch` or equivalent), not hand-rolled polling.
The set id is passed as a dispatch input to every leg, which makes runs
correlatable from either side.

Cross-repo dispatch cannot use the default `GITHUB_TOKEN`. rammp-nightly holds
one org credential — a fine-grained PAT or GitHub App token with
`actions:write` on the member repos and nothing else — as a repo secret. That
credential triggers workflows; it never pushes images and never writes to
packages, because every write happens inside the owning repo under that
repo's own default token.

### The kinova-arm stack

Five repos, three dispatched builds — interfaces and the driver have no image
of their own and enter as source pins:

| wave | repo | build | set SHAs injected as |
| --- | --- | --- | --- |
| 1 | RAMMP-docker | rammp-base + rammp-cuda images | `INTERFACES_REF` = interfaces SHA |
| 2 | RAMMP-CuRobo | planner image | its own dev SHA; base = `rammp-cuda:nightly-<date>` |
| 2 | kinova-gen3-ros2 | node image | driver SHA + CuRobo SHA via the `CORE_REF`-style overrides; base = `rammp-base:nightly-<date>` |
| 3 | all three image repos | promote | re-point `:nightly` to `nightly-<date>` |

Wave 2 may start only after wave 1 reports success, which also guarantees the
base tags it builds `FROM` exist. kinova-gen3-driver's own dev CI continues to
run per push as today; the set-level proof of the driver is the node leg
compiling and testing against its SHA.

**Enabling work: the planner image moves onto rammp-cuda.** Today
RAMMP-CuRobo's image builds `FROM nvcr.io/nvidia/l4t-jetpack` and reinstalls
what the base family already carries — ROS 2 Humble, torch 2.10.0 from the
jp6/cu126 index, the openblas/cuDSS runtime fixes, the Cyclone RMW. Those
layers are deleted; the image becomes `FROM rammp-cuda` (parameterized, so the
nightly can inject the dated tag) and keeps only the cuRobo v0.7.8 kernel
compile — which must now apt-install the CUDA toolchain itself, because
rammp-cuda deliberately ships runtime libraries only, no `nvcc` — plus the
repo's own packages. The repo's existing ~1 h image gate (build-and-link, no
GPU) proves the rebase; the functional gate remains the Jetson, as today.

## The dispatch contract (the interface this design creates)

A repo is **nightly-able** when its image build workflow accepts
`workflow_dispatch` with:

- the ref of itself to build,
- the refs/pins of its upstream inputs (base image tag, source SHAs for
  `.repos` overrides),
- the tag to publish,
- the set id (for correlation and the `/etc/rammp-set.json` stamp),
- the SHA the orchestrator resolved for the dispatched repo itself
  (`expected_sha`): `workflow_dispatch` targets a branch, not a SHA, so the
  dispatched run fails loudly if HEAD moved between resolve and dispatch
  rather than silently building a commit the manifest will not name,

and exposes a **promote** entry (re-tag a given dated tag as `:nightly` using
its own token). Any org repo implementing this contract can join a stack.

The release path is untouched by the contract: the existing tag triggers call
the same build with a semver tag and the pins read from the committed `.repos`
file exactly as today. The node repo's "pinned to a TAG, never a branch"
stance survives intact — nightly legs inject SHAs through the existing
`CORE_REF`-style escape hatch (generalized to cover CuRobo); no pin file is
ever touched by this channel, and every nightly artifact is reproducible from
its manifest even though no pins moved.

## When a night goes red

A failing leg stops the chain: later waves never fire, nothing is promoted,
no manifest is written. The channel **degrades to stale, never to broken** —
`:nightly` keeps serving the last green set, dated pins are unaffected. The
orchestrator updates the stack's tracking issue with the failing leg and a
link to its run (whose logs live in the owning repo, where the people who can
fix it work).

A red night is not an incident; it is the system declining to bless a set
that does not exist yet. The pressure it creates — dev branches must stay
mutually buildable — is the point.

## Breaking interface changes

Additive interface changes (new message/service/action; field appended with a
default — minor bumps per the interfaces README) need no coordination: the
set simply builds and flows.

A true break (insert/remove/rename/retype, append without default) is a rare,
deliberate event with a fixed choreography:

1. The interfaces change and its companion PRs in dependent repos are
   prepared together.
2. Optionally **pre-flighted as a set**: the orchestrator's workflow also runs
   on manual `workflow_dispatch` with per-repo ref overrides — "build the
   stack with interfaces `feature/x` + driver `feature/y`, dev elsewhere" —
   so the combination is proven before anything merges. (RAMMP-CuRobo's
   required-review ruleset makes same-day companion merges the slowest leg;
   pre-flighting is how a train avoids camping on a red channel while reviews
   land.)
3. Companions merge to dev the same day. Merging one side alone just means
   tonight's set doesn't publish and the tracking issue names the missing
   piece.

## What releases become

Mechanically identical, but easier and rarer:

- Rarer, because the standing reason to cut one — a collaborator needs the
  code — is served by the channel.
- Easier, because a release now starts from "dev already integrates" (proven
  nightly, debt ≤ a day) instead of "now reconcile everything".
- A release candidate is just a **promoted set**: pick a green manifest, and
  the pin-move commits are generated from it (`nightly-<date>.repos` already
  holds the SHAs; tagging turns them into the release pins). Cutting releases
  remains a human decision, per standing policy.

Future work, explicitly not now: Renovate (or similar) to automate
release-time pin-bump PRs when upstream tags land.

## Build & rollout order

1. **Dispatch contract in each image repo** (RAMMP-docker, RAMMP-CuRobo,
   kinova-gen3-ros2): `workflow_dispatch` inputs + promote entry on the
   existing build workflows. Each repo is independently testable at this
   point by hand-dispatching a nightly-tagged build.
2. **rammp-nightly**: stack file, orchestration workflow, credential, manifest
   commit, tracking issue. First manual end-to-end run.
3. **Cron on**, collaborators pointed at `:nightly` / dated tags, channel
   freshness visible from the manifest history.

Step 1 carries no risk to existing CI (new trigger, same workflow); step 2
touches nothing outside the new repo; the first blessed set exists before any
collaborator depends on the channel.
