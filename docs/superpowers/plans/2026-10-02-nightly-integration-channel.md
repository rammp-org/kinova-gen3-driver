# Nightly Integration Channel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a nightly, integration-tested set of the arm stack (four images + a SHA manifest) from dev branches, so collaborators build on new code without a release train.

**Architecture:** A new thin orchestrator repo (`rammp-nightly`) resolves the five member repos' branch HEADs at one instant, then `workflow_dispatch`-triggers each image repo's own build workflow in dependency waves, awaits each leg, and — only on full green — dispatches a promote step in each repo and commits a vcstool manifest. Every build, log, and ghcr write stays in the repo that owns the image. Releases are untouched.

**Tech Stack:** GitHub Actions (`workflow_dispatch`), `convictional/trigger-workflow-and-wait` marketplace action, Docker/buildx + ghcr, vcstool `.repos` manifests, Python 3 + PyYAML + pytest (manifest tooling, run with `uv`).

**Spec:** `docs/superpowers/specs/2026-10-02-nightly-integration-channel-design.md` (same branch as this plan). Read it first; this plan argues from it.

## Global Constraints

- Nightly tags match `^nightly-[0-9]{8}(-rc[0-9]+)?$` and nothing else — every repo's publish path validates this so a nightly dispatch can never write into the semver release space. `-rcN` is for rehearsals only.
- Dated set tags are immutable once a manifest names them: `write_manifest.py` and the record job refuse an existing set id. `:nightly` is the only moving tag, and only the promote step moves it.
- No release mechanism changes: no `.repos` pin file is modified, no semver tag logic is touched, and every new behavior is gated behind "a nightly input was provided" so push/PR/tag events behave byte-identically to today.
- All images remain arm64-only, built on `ubuntu-*-arm` hosted runners, exactly as today.
- `workflow_dispatch` targets a branch, never a SHA. Every dispatched contract takes `expected_sha` and fails if `GITHUB_SHA` differs — a branch that moves mid-run fails loudly instead of building a commit the manifest won't name.
- `set_info` travels as **base64-encoded JSON** (shell→make→docker quoting is otherwise fragile) and lands decoded at `/etc/rammp-set.json`.
- The three contract repos' changes must reach their **default branch (main)** before the orchestrator can dispatch them by filename (the Actions API resolves workflow files against the default branch). These are CI-only changes: PR to `dev`, then a workflows-only PR to `main` — no release implied. RAMMP-CuRobo's ruleset requires a review on every PR; Swapnil admin-merges there.
- Repo creation, the fine-grained PAT, and the org secret are human (Swapnil) steps — marked **[HUMAN]** inline. Everything else is executable by an agent with `gh` auth on rammp-org.

## Review Focus

Failure modes the spec implies that need pinned tests, most likely to bite first:

1. **A dev branch moves between resolve and dispatch** → the dispatched leg builds a SHA the manifest doesn't name. Pinned by the `expected_sha` guard in every contract (Tasks 1, 3, 4) and rehearsed with a deliberately stale SHA in Task 7.
2. **A nightly tag lands in release space** (someone dispatches `publish_tag: 1.3.0`) → it would shadow a semver release. Pinned by the regex validation step in each contract (Tasks 1, 3, 4) and rehearsed with a bad tag in Task 7.
3. **Re-running a date that already has a blessed manifest** → would rebuild an "immutable" set. Pinned by `write_manifest.py`'s refusal test (Task 5) and the record job's `git ls-files` guard (Task 6).
4. **A partial night promotes anyway** (base green, node red) → `:nightly` would advance to a broken set. Pinned structurally: promote and record `needs:` every leg and `propagate_failure: true` makes an inner failure fail the outer job (Task 6); Task 7's stale-SHA rehearsal doubles as proof that a red leg stops the chain before promote.
5. **The nightly plumbing changes today's CI behavior** (empty `inputs` context on push/PR/tag events accidentally enables publishing) → pinned per repo: after each contract merges, watch one ordinary push run and confirm it publishes nothing / behaves as before (Tasks 1, 3, 4 each carry this verify step).

---

### Task 1: RAMMP-docker — nightly dispatch contract + promote

**Files:**
- Modify: `RAMMP-docker/.github/workflows/build-push.yml` (dispatch inputs, guards, nightly publish path)
- Modify: `RAMMP-docker/Makefile` (pass `SET_INFO`, add `push-version` target)
- Modify: `RAMMP-docker/docker/base/Dockerfile` (SET_INFO stamp)
- Create: `RAMMP-docker/.github/workflows/promote.yml`

**Interfaces:**
- Produces (consumed by Task 6's orchestrator): `build-push.yml` dispatchable with inputs `publish_tag` (string), `interfaces_ref` (string), `expected_sha` (string), `set_info` (string, b64 JSON); on `publish_tag` ≠ '' it pushes `ghcr.io/rammp-org/rammp-base:<publish_tag>` and `ghcr.io/rammp-org/rammp-cuda:<publish_tag>` and nothing else. `promote.yml` dispatchable with input `source_tag`; re-points `:nightly` on both images.

- [ ] **Step 1: Branch**

```bash
cd ~/atdev/RAMMP-docker && git fetch origin && git switch -c ci/nightly-dispatch-contract origin/main
```

- [ ] **Step 2: Add dispatch inputs and guards to build-push.yml**

Replace the bare `workflow_dispatch:` trigger (line 31) with:

```yaml
  workflow_dispatch:
    inputs:
      publish_tag:
        description: "nightly tag to publish (nightly-YYYYMMDD[-rcN]); empty = ordinary unpublished CI build"
        required: false
        default: ""
        type: string
      interfaces_ref:
        description: "rammp-interfaces-ros2 ref to compile in (a SHA when dispatched by rammp-nightly)"
        required: false
        default: ""
        type: string
      expected_sha:
        description: "fail unless HEAD of the dispatched ref is exactly this SHA"
        required: false
        default: ""
        type: string
      set_info:
        description: "base64 JSON stamped at /etc/rammp-set.json"
        required: false
        default: ""
        type: string
```

Immediately after the `Check out` step, add:

```yaml
      - name: Branch did not move since the orchestrator resolved it
        if: inputs.expected_sha != ''
        env:
          EXPECTED: ${{ inputs.expected_sha }}
        run: |
          if [ "${GITHUB_SHA}" != "${EXPECTED}" ]; then
            echo "::error::HEAD is ${GITHUB_SHA} but the orchestrator resolved ${EXPECTED}; the branch moved mid-run. Re-run the set."
            exit 1
          fi
```

In the `Derive version` step, insert a nightly arm **before** the existing tag check (nightly wins only on dispatch; tag pushes never carry inputs):

```bash
          if [ -n "${{ inputs.publish_tag }}" ]; then
            if [[ ! "${{ inputs.publish_tag }}" =~ ^nightly-[0-9]{8}(-rc[0-9]+)?$ ]]; then
              echo "::error::publish_tag '${{ inputs.publish_tag }}' must be nightly-YYYYMMDD[-rcN]; refusing to write into release tag space."
              exit 1
            fi
            echo "version=${{ inputs.publish_tag }}" >> "$GITHUB_OUTPUT"
            echo "publish=nightly" >> "$GITHUB_OUTPUT"
          elif [[ "${GITHUB_REF}" == refs/tags/v* ]]; then
            ... (existing tag logic unchanged, still emits publish=true)
          else
            ... (existing fallback unchanged, publish=false)
          fi
```

Change the `Build images` step to pass the overrides through make only when set (an empty `INTERFACES_REF=` on the command line would clobber the Makefile default):

```yaml
      - name: Build images
        run: |
          EXTRA=""
          [ -n "${{ inputs.interfaces_ref }}" ] && EXTRA="INTERFACES_REF=${{ inputs.interfaces_ref }}"
          [ -n "${{ inputs.set_info }}" ] && EXTRA="$EXTRA SET_INFO=${{ inputs.set_info }}"
          make base cuda VERSION=${{ steps.version.outputs.version }} $EXTRA
          df -h /
```

Login condition becomes `if: steps.version.outputs.publish != 'false'`. The existing `Publish release tags` step keeps `if: steps.version.outputs.publish == 'true'`; add beside it:

```yaml
      - name: Publish nightly tag (dated tag only, no :humble alias)
        if: steps.version.outputs.publish == 'nightly'
        run: |
          make push-version \
            VERSION=${{ steps.version.outputs.version }} \
            REGISTRY=${{ env.REGISTRY }}/${{ github.repository_owner }}
```

- [ ] **Step 3: Makefile — SET_INFO build-arg and push-version target**

In the `BUILD =` definition add one more build-arg line (default empty → stamp is a no-op):

```make
SET_INFO ?=
BUILD = docker build --build-arg VERSION=$(VERSION) \
                     --build-arg INTERFACES_REF=$(INTERFACES_REF) \
                     --build-arg INTERFACES_VERSION=$(INTERFACES_REF) \
                     --build-arg SET_INFO=$(SET_INFO)
```

After the existing `push:` target add:

```make
# Nightly publish: the dated tag only. `push` also publishes the :$(DISTRO)
# alias, which belongs to releases; a nightly must never move it.
push-version: 
	for img in rammp-base rammp-cuda; do \
	  docker tag  $$img:$(VERSION) $(REGISTRY)/$$img:$(VERSION); \
	  docker push $(REGISTRY)/$$img:$(VERSION); \
	done
```

(No `cuda` prerequisite: the workflow has already built and smoke-tested both images.)

- [ ] **Step 4: Stamp in docker/base/Dockerfile**

Next to the existing `ARG VERSION` block add (rammp-cuda is `FROM rammp-base`, so it inherits the file):

```dockerfile
# Which nightly set this image belongs to, if any. Empty outside the nightly
# channel. base64 because JSON does not survive make -> docker quoting.
ARG SET_INFO=
RUN if [ -n "${SET_INFO}" ]; then echo "${SET_INFO}" | base64 -d > /etc/rammp-set.json; fi
```

- [ ] **Step 5: Create promote.yml**

```yaml
name: promote nightly

# Re-points the :nightly alias at an already-pushed dated tag. Dispatched by
# rammp-nightly only after EVERY leg of the set went green — this repo never
# decides on its own that a set is blessed.
on:
  workflow_dispatch:
    inputs:
      source_tag:
        description: "existing dated tag to promote (nightly-YYYYMMDD[-rcN])"
        required: true
        type: string

permissions:
  contents: read
  packages: write

jobs:
  promote:
    runs-on: ubuntu-24.04
    steps:
      - name: Validate tag shape
        run: |
          [[ "${{ inputs.source_tag }}" =~ ^nightly-[0-9]{8}(-rc[0-9]+)?$ ]] \
            || { echo "::error::source_tag must be nightly-YYYYMMDD[-rcN]"; exit 1; }
      - uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - name: Re-point :nightly
        run: |
          for img in rammp-base rammp-cuda; do
            docker buildx imagetools create \
              -t "ghcr.io/${{ github.repository_owner }}/${img}:nightly" \
              "ghcr.io/${{ github.repository_owner }}/${img}:${{ inputs.source_tag }}"
          done
```

- [ ] **Step 6: Lint the workflows**

```bash
cd ~/atdev/RAMMP-docker && docker run --rm -v "$PWD":/repo -w /repo rhysd/actionlint:latest -color
```
Expected: no findings in the two touched workflows (pre-existing findings elsewhere, if any, are out of scope).

- [ ] **Step 7: Commit, push, PR to main**

```bash
git add .github/workflows/build-push.yml .github/workflows/promote.yml Makefile docker/base/Dockerfile
git commit -m "ci: nightly dispatch contract (publish_tag/interfaces_ref/expected_sha/set_info) + promote"
git push -u origin ci/nightly-dispatch-contract
gh pr create --repo rammp-org/RAMMP-docker --base main --title "ci: nightly dispatch contract + promote" \
  --body "Adds the rammp-nightly dispatch contract per kinova-gen3-driver docs/superpowers/specs/2026-10-02-nightly-integration-channel-design.md. Push/PR/tag behavior unchanged."
```
(RAMMP-docker builds from main — no dev leg needed here.)

- [ ] **Step 8: Verify the PR build is byte-identical to today's behavior**

On the PR run: `Derive version` must take the fallback arm (`0.0.0-<sha>`, publish=false), no login, no publish. After merge, rehearse the contract:

```bash
gh workflow run build-push.yml --repo rammp-org/RAMMP-docker --ref main \
  -f publish_tag=nightly-19700101-rc1 -f interfaces_ref=dev
gh run watch --repo rammp-org/RAMMP-docker $(gh run list --repo rammp-org/RAMMP-docker -w build-push.yml -L1 --json databaseId -q '.[0].databaseId')
```
Expected: green; `ghcr.io/rammp-org/rammp-base:nightly-19700101-rc1` and `rammp-cuda:nightly-19700101-rc1` exist; `:humble` alias untouched. Then rehearse promote with `-f source_tag=nightly-19700101-rc1` and confirm `:nightly` resolves to the same digest (`docker buildx imagetools inspect`). Delete the rehearsal package versions afterwards (ghcr → package → versions, or `gh api -X DELETE`).

---

### Task 2: RAMMP-CuRobo — rebase the planner image onto rammp-cuda

**Files:**
- Modify: `RAMMP-CuRobo/docker/Dockerfile`

**Interfaces:**
- Consumes: `ghcr.io/rammp-org/rammp-cuda:1.1.0-jp6` (ROS 2 Humble + Cyclone selected + interfaces + CUDA 12.6 runtime + torch 2.10.0/torchvision 0.25.0 from jp6/cu126, openblas/CUPTI/cuDSS already resolved).
- Produces: the same planner image contract (entrypoint, launch command, `/opt/rammp_curobo`), now `FROM ${BASE_IMAGE}` with `ARG BASE_IMAGE` — which Task 3's workflow injects.

This is the spec's "enabling work": today's image is `FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0` and reinstalls what the base family already carries. Deletions and survivals, by current line numbers:

| lines | layer | fate |
| --- | --- | --- |
| 21 | `FROM l4t-jetpack` | → `ARG BASE_IMAGE=ghcr.io/rammp-org/rammp-cuda:1.1.0-jp6` + `FROM ${BASE_IMAGE}` |
| 25–40 | ROS 2 Humble install | delete ros-base/pip/colcon (in base); KEEP `ros-humble-control-msgs ros-humble-controller-manager-msgs ros-humble-example-interfaces` + `git` in a small apt layer (not in rammp-base) |
| 42–45 | torch/torchvision install | delete — rammp-cuda pins the same torch 2.10.0/torchvision 0.25.0 |
| 47–75 | openblas + cuDSS fixes | delete — rammp-cuda resolves both; keep only the cheap fail-loud gate `python3 -c "import torch; ..."` |
| 77–93 | cuRobo v0.7.8 source build | keep, but prepend the toolchain: rammp-cuda ships CUDA **runtime** only, no `nvcc` |
| 95–133 | repo build, PYTHONUTF8, Cyclone RMW, entrypoint | keep repo build/UTF8/entrypoint; delete the Cyclone layer (122–125) — rammp-base installs and selects Cyclone |

- [ ] **Step 1: Branch**

```bash
cd ~/atdev/RAMMP-CuRobo && git fetch origin && git switch -c ci/image-on-rammp-cuda origin/dev
```

- [ ] **Step 2: Rewrite the head of docker/Dockerfile**

Replace lines 21–75 with:

```dockerfile
ARG BASE_IMAGE=ghcr.io/rammp-org/rammp-cuda:1.1.0-jp6
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

# rammp-cuda supplies ROS 2 Humble (Cyclone selected), the interfaces, the
# CUDA 12.6 runtime and torch 2.10.0 from the jp6/cu126 index — including the
# openblas/cuDSS fixes this file used to carry. What it deliberately does NOT
# ship is build tooling: no nvcc. The cuRobo sm_87 kernel compile below needs
# the toolkit, so this image installs it itself, matching the base's 12.6.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        ros-humble-control-msgs \
        ros-humble-controller-manager-msgs \
        ros-humble-example-interfaces \
        cuda-toolkit-12-6 \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -c "import torch; print('torch from base OK:', torch.__version__)"
ENV CUDA_HOME=/usr/local/cuda-12.6
ENV PATH=${CUDA_HOME}/bin:${PATH}
```

Keep the cuRobo build block (old lines 77–93) and the repo build block (95–111) verbatim. Delete the Cyclone layer (113–125, both the apt install and `ENV RMW_IMPLEMENTATION` — the base sets it). Keep entrypoint/CMD. At the end add the stamp:

```dockerfile
# Which nightly set this image belongs to, if any (see rammp-nightly).
ARG SET_INFO=
RUN if [ -n "${SET_INFO}" ]; then echo "${SET_INFO}" | base64 -d > /etc/rammp-set.json; fi
```

Update the header comment (lines 1–19): the build no longer happens "ON the Jetson because nvcc comes from the base image" — nvcc now comes from the `cuda-toolkit-12-6` layer and the build runs anywhere arm64.

- [ ] **Step 3: Verify via the repo's image gate (no local arm64 build exists)**

```bash
git add docker/Dockerfile && git commit -m "docker: build the planner image FROM rammp-cuda, not l4t-jetpack" && git push -u origin ci/image-on-rammp-cuda
gh workflow run image.yml --repo rammp-org/RAMMP-CuRobo --ref ci/image-on-rammp-cuda
```
Expected: the ~1 h build-and-link gate goes green (deps resolve, cuRobo v0.7.8 compiles sm_87 against the base's torch, both ament packages build). This gate cannot prove GPU execution — note in the PR that the first nightly consumed on the Jetson is the functional check, same boundary as today.

If the apt layer fails on `cuda-toolkit-12-6`: rammp-cuda's apt sources already carry the NVIDIA repo (it installs `cuda-libraries-12-6` from it); the package name is the standard toolkit meta-package for 12.6. Diagnose with the build log before substituting — do not silently fall back to l4t-jetpack.

- [ ] **Step 4: PR to dev (review required)**

```bash
gh pr create --repo rammp-org/RAMMP-CuRobo --base dev --title "docker: planner image builds FROM rammp-cuda" \
  --body "Deletes the duplicated ROS/torch/cuDSS layers (now supplied by rammp-cuda), adds the CUDA toolkit for the sm_87 compile, parameterizes the base for the nightly channel. Image gate green on the branch: <run link>. Functional GPU check remains the Jetson."
```
**[HUMAN]** CuRobo's ruleset requires a review; Swapnil reviews/admin-merges.

---

### Task 3: RAMMP-CuRobo — nightly dispatch contract + promote

**Files:**
- Modify: `RAMMP-CuRobo/.github/workflows/image.yml`
- Create: `RAMMP-CuRobo/.github/workflows/promote.yml`

**Interfaces:**
- Consumes: Task 2's `ARG BASE_IMAGE` / `ARG SET_INFO`.
- Produces (for Task 6): `image.yml` dispatchable with inputs `publish_tag`, `base_image`, `expected_sha`, `set_info`; on `publish_tag` ≠ '' it pushes `ghcr.io/rammp-org/rammp-curobo:<publish_tag>`. `promote.yml` with input `source_tag`.

- [ ] **Step 1: Branch from the Task-2 branch's merge target**

```bash
cd ~/atdev/RAMMP-CuRobo && git fetch origin && git switch -c ci/nightly-dispatch-contract origin/dev
```
(If Task 2 has not merged yet, branch from `ci/image-on-rammp-cuda` instead and say so in the PR.)

- [ ] **Step 2: image.yml — inputs, guards, nightly publish**

Replace the bare `workflow_dispatch:` (line 13) with inputs `publish_tag`, `base_image`, `expected_sha`, `set_info` — same block shape as Task 1 Step 2, with `base_image` described as "rammp-cuda image to build FROM (the nightly dated tag)" replacing `interfaces_ref`.

After the checkout step add the same `Branch did not move…` guard as Task 1 Step 2, then a tag-shape guard:

```yaml
      - name: Nightly tag stays out of release space
        if: inputs.publish_tag != ''
        run: |
          [[ "${{ inputs.publish_tag }}" =~ ^nightly-[0-9]{8}(-rc[0-9]+)?$ ]] \
            || { echo "::error::publish_tag must be nightly-YYYYMMDD[-rcN]"; exit 1; }
```

In the `meta` step's tags, append a raw-tag line so metadata-action emits the dated tag on nightly dispatches and nothing on other events:

```yaml
          tags: |
            type=semver,pattern={{version}}
            type=semver,pattern={{major}}.{{minor}}
            type=semver,pattern={{major}}
            type=raw,value=${{ inputs.publish_tag }},enable=${{ inputs.publish_tag != '' }}
```

Login step condition becomes `if: github.ref_type == 'tag' || inputs.publish_tag != ''`. The build-push step's `outputs:` ternary gains the same clause:

```yaml
          outputs: ${{ (github.ref_type == 'tag' || inputs.publish_tag != '') && 'type=image,push=true' || 'type=cacheonly' }}
```

and its `build-args:` pass the contract through (empty values are harmless: `BASE_IMAGE`'s Dockerfile default holds because docker treats an empty `--build-arg` as set — so only pass them when non-empty):

```yaml
          build-args: |
            ${{ inputs.base_image != '' && format('BASE_IMAGE={0}', inputs.base_image) || '' }}
            ${{ inputs.set_info != '' && format('SET_INFO={0}', inputs.set_info) || '' }}
```

The `A v* tag must point at a commit on main` step is tag-only and untouched. The `Show what was published` step's condition becomes `if: github.ref_type == 'tag' || inputs.publish_tag != ''` with `VERSION: ${{ inputs.publish_tag || steps.meta.outputs.version }}`.

- [ ] **Step 3: promote.yml**

Same file as Task 1 Step 5 with the single-image loop body:

```yaml
          docker buildx imagetools create \
            -t "ghcr.io/rammp-org/rammp-curobo:nightly" \
            "ghcr.io/rammp-org/rammp-curobo:${{ inputs.source_tag }}"
```

- [ ] **Step 4: Lint, commit, PR**

```bash
docker run --rm -v "$PWD":/repo -w /repo rhysd/actionlint:latest -color
git add .github/workflows/image.yml .github/workflows/promote.yml
git commit -m "ci: nightly dispatch contract (publish_tag/base_image/expected_sha/set_info) + promote"
git push -u origin ci/nightly-dispatch-contract
gh pr create --repo rammp-org/RAMMP-CuRobo --base dev --title "ci: nightly dispatch contract + promote" --body "Per the nightly-integration spec. Push/PR/tag behavior unchanged."
```
**[HUMAN]** review + merge to dev; then open the workflows-only PR to main (`git switch -c ci/nightly-contract-main origin/main && git checkout ci/nightly-dispatch-contract -- .github/workflows && git commit -m "ci: nightly dispatch contract (workflows only, for dispatchability)" && gh pr create --base main ...`) so the Actions API can resolve the filenames.

- [ ] **Step 5: Verify behavior is unchanged on ordinary events**

Watch the next push-to-dev run of image.yml: meta emits no tags beyond semver patterns (all disabled), outputs is `type=cacheonly`, no login. Expected: identical to pre-change runs.

---

### Task 4: kinova-gen3-ros2 — BASE_IMAGE/CUROBO_REF/stamp + nightly dispatch contract + promote

**Files:**
- Modify: `kinova-gen3-ros2/Dockerfile`
- Modify: `kinova-gen3-ros2/.github/workflows/build.yml`
- Create: `kinova-gen3-ros2/.github/workflows/promote.yml`

**Interfaces:**
- Produces (for Task 6): `build.yml` dispatchable with inputs `publish_tag`, `base_image`, `core_ref`, `curobo_ref`, `expected_sha`, `set_info`; on `publish_tag` ≠ '' the `kortex` job pushes `ghcr.io/rammp-org/kinova-gen3-ros2:<publish_tag>` directly (single-arch, no digest/manifest dance). `promote.yml` with input `source_tag`.

- [ ] **Step 1: Branch**

```bash
cd ~/atdev/kinova-gen3-ros2 && git fetch origin && git switch -c ci/nightly-dispatch-contract origin/dev
```

- [ ] **Step 2: Dockerfile — three ARGs**

Line 25, parameterize the base (default = today's exact pin, so every existing build is unchanged):

```dockerfile
ARG BASE_IMAGE=ghcr.io/rammp-org/rammp-base:1.1.0-jp6
FROM ${BASE_IMAGE}
```

After the existing `CORE_REF` block (lines 57–61), add the CuRobo analog of the same escape hatch:

```dockerfile
# CUROBO_REF overrides the RAMMP-CuRobo ref pinned in kinova_gen3.repos, the
# same escape hatch CORE_REF is for the driver. Used by the nightly channel.
ARG CUROBO_REF=
RUN if [ -n "${CUROBO_REF}" ]; then git -C src/RAMMP-CuRobo fetch --depth 1 origin "${CUROBO_REF}" && \
        git -C src/RAMMP-CuRobo checkout FETCH_HEAD; fi
```

At the end of the file, the stamp (overwrites the base's stamp with the identical set JSON — the orchestrator passes one `set_info` to every leg):

```dockerfile
ARG SET_INFO=
RUN if [ -n "${SET_INFO}" ]; then echo "${SET_INFO}" | base64 -d > /etc/rammp-set.json; fi
```

- [ ] **Step 3: build.yml — inputs, guards, nightly publish in the kortex job**

Replace the bare `workflow_dispatch:` (line 9) with inputs `publish_tag`, `base_image`, `core_ref`, `curobo_ref`, `expected_sha`, `set_info` (same shape as Task 1 Step 2).

In the `kortex` job, after checkout, add the `Branch did not move…` and `Nightly tag stays out of release space` guards verbatim from Task 3 Step 2. Extend the login condition to `if: github.ref_type == 'tag' || inputs.publish_tag != ''`. Extend the build step:

```yaml
          build-args: |
            KINOVA_ENABLE_KORTEX=ON
            ${{ inputs.base_image != '' && format('BASE_IMAGE={0}', inputs.base_image) || '' }}
            ${{ inputs.core_ref != '' && format('CORE_REF={0}', inputs.core_ref) || '' }}
            ${{ inputs.curobo_ref != '' && format('CUROBO_REF={0}', inputs.curobo_ref) || '' }}
            ${{ inputs.set_info != '' && format('SET_INFO={0}', inputs.set_info) || '' }}
          tags: ${{ inputs.publish_tag != '' && format('ghcr.io/{0}:{1}', steps.image.outputs.name, inputs.publish_tag) || '' }}
          outputs: ${{ github.ref_type == 'tag' && format('type=image,name=ghcr.io/{0},push-by-digest=true,name-canonical=true,push=true', steps.image.outputs.name) || (inputs.publish_tag != '' && 'type=image,push=true') || 'type=cacheonly' }}
```

The digest-export/upload steps and the `kortex-manifest` job stay tag-only (`if: github.ref_type == 'tag'`) — a nightly is single-arch and pushes its one tag directly, so the digest/manifest-list machinery never runs for it.

- [ ] **Step 4: promote.yml**

Task 1 Step 5's file with the image loop body replaced by the single image `ghcr.io/rammp-org/kinova-gen3-ros2`.

- [ ] **Step 5: Lint, commit, PR to dev, workflows-only PR to main**

```bash
docker run --rm -v "$PWD":/repo -w /repo rhysd/actionlint:latest -color
git add Dockerfile .github/workflows/build.yml .github/workflows/promote.yml
git commit -m "ci: nightly dispatch contract (publish_tag/base_image/core_ref/curobo_ref) + promote"
git push -u origin ci/nightly-dispatch-contract
gh pr create --repo rammp-org/kinova-gen3-ros2 --base dev --title "ci: nightly dispatch contract + promote" --body "Per the nightly-integration spec. Pins untouched; push/PR/tag behavior unchanged."
```
Then the same workflows-only PR to main as Task 3 Step 4 (build.yml + promote.yml; NOT the Dockerfile — it reaches main at the next release as usual, and nightly dispatches run on `ref: dev` where the Dockerfile ARGs live).

- [ ] **Step 6: Verify behavior unchanged**

Next ordinary push-to-dev run: `kortex` builds `type=cacheonly`, no login, no tags. Expected: identical to pre-change runs. Also confirm locally that the pins still rule when no ARG is passed: `grep -n 'version: v' kinova_gen3.repos` is untouched by this task.

---

### Task 5: rammp-nightly — repo bootstrap, stack definition, manifest tooling (TDD)

**Files:**
- Create: repo `rammp-org/rammp-nightly` with `README.md`, `pyproject.toml`, `stacks/kinova-arm.yml`, `scripts/stack_members.py`, `scripts/write_manifest.py`, `tests/test_write_manifest.py`, `tests/test_stack_members.py`

**Interfaces:**
- Produces (for Task 6):
  - `stacks/kinova-arm.yml` schema: `stack` (str), `members` (ordered map name → `{url, branch}`), `images` (list of ghcr package names).
  - `python3 scripts/stack_members.py stacks/kinova-arm.yml` → JSON `{"<name>": {"url": "...", "branch": "..."}}` on stdout.
  - `python3 scripts/write_manifest.py --stack <yml> --set-id <id> --shas <json> --digests <json> --out-dir <dir>` → writes `<id>.repos` + `<id>.images.json`; exit 1 if either exists or a member lacks a SHA.

- [ ] **Step 1 [HUMAN]: Create the repo and the credential**

```bash
gh repo create rammp-org/rammp-nightly --public \
  --description "Nightly integration sets: orchestrates, awaits and blesses; owns no builds"
```
Then (Swapnil, in the GitHub UI): a fine-grained PAT, resource owner rammp-org, repos `RAMMP-docker`, `RAMMP-CuRobo`, `kinova-gen3-ros2` (+ `rammp-interfaces-ros2`, `kinova-gen3-driver` read), permissions **Actions: read/write**, **Contents: read**; store as secret `RAMMP_NIGHTLY_TOKEN` in rammp-nightly. The token triggers workflows only; it never touches packages.

- [ ] **Step 2: Scaffold**

```bash
git clone git@github.com:rammp-org/rammp-nightly.git ~/atdev/rammp-nightly && cd ~/atdev/rammp-nightly
```

`pyproject.toml`:

```toml
[project]
name = "rammp-nightly"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["pyyaml>=6"]

[dependency-groups]
dev = ["pytest>=8"]
```

`stacks/kinova-arm.yml`:

```yaml
# The kinova arm stack. Members are everything a set pins; images are the
# ghcr packages a set publishes. Build ORDER lives in the stack's workflow
# (.github/workflows/kinova-arm.yml), where GitHub needs it anyway — this
# file is the data both the workflow and the manifest tooling read.
stack: kinova-arm
members:
  rammp-interfaces-ros2:
    url: https://github.com/rammp-org/rammp-interfaces-ros2.git
    branch: dev
  RAMMP-docker:
    url: https://github.com/rammp-org/RAMMP-docker.git
    branch: main
  kinova-gen3-driver:
    url: https://github.com/rammp-org/kinova-gen3-driver.git
    branch: dev
  RAMMP-CuRobo:
    url: https://github.com/rammp-org/RAMMP-CuRobo.git
    branch: dev
  kinova-gen3-ros2:
    url: https://github.com/rammp-org/kinova-gen3-ros2.git
    branch: dev
images:
  - rammp-base
  - rammp-cuda
  - rammp-curobo
  - kinova-gen3-ros2
```

- [ ] **Step 3: Write the failing tests**

`tests/test_stack_members.py`:

```python
import json, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def test_members_json_round_trips():
    out = subprocess.run(
        [sys.executable, ROOT / "scripts" / "stack_members.py", ROOT / "stacks" / "kinova-arm.yml"],
        capture_output=True, text=True, check=True)
    members = json.loads(out.stdout)
    assert set(members) == {"rammp-interfaces-ros2", "RAMMP-docker",
                            "kinova-gen3-driver", "RAMMP-CuRobo", "kinova-gen3-ros2"}
    assert members["RAMMP-docker"]["branch"] == "main"
    assert members["kinova-gen3-driver"]["url"].endswith("kinova-gen3-driver.git")
```

`tests/test_write_manifest.py`:

```python
import json, subprocess, sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
STACK = ROOT / "stacks" / "kinova-arm.yml"
SHAS = {n: f"{i:040x}" for i, n in enumerate(
    ["rammp-interfaces-ros2", "RAMMP-docker", "kinova-gen3-driver",
     "RAMMP-CuRobo", "kinova-gen3-ros2"], 1)}
DIGESTS = {img: f"sha256:{i:064x}" for i, img in enumerate(
    ["rammp-base", "rammp-cuda", "rammp-curobo", "kinova-gen3-ros2"], 1)}

def run(tmp_path, shas=SHAS, digests=DIGESTS, set_id="nightly-20261002"):
    (tmp_path / "shas.json").write_text(json.dumps(shas))
    (tmp_path / "digests.json").write_text(json.dumps(digests))
    return subprocess.run(
        [sys.executable, ROOT / "scripts" / "write_manifest.py",
         "--stack", STACK, "--set-id", set_id,
         "--shas", tmp_path / "shas.json", "--digests", tmp_path / "digests.json",
         "--out-dir", tmp_path / "out"],
        capture_output=True, text=True)

def test_writes_vcstool_repos_and_images_json(tmp_path):
    assert run(tmp_path).returncode == 0
    repos = yaml.safe_load((tmp_path / "out" / "nightly-20261002.repos").read_text())
    assert repos["repositories"]["kinova-gen3-driver"] == {
        "type": "git",
        "url": "https://github.com/rammp-org/kinova-gen3-driver.git",
        "version": SHAS["kinova-gen3-driver"]}
    images = json.loads((tmp_path / "out" / "nightly-20261002.images.json").read_text())
    assert images["set"] == "nightly-20261002" and images["stack"] == "kinova-arm"
    assert images["images"] == DIGESTS

def test_refuses_existing_set(tmp_path):
    assert run(tmp_path).returncode == 0
    second = run(tmp_path)
    assert second.returncode == 1 and "already exists" in second.stderr

def test_refuses_missing_member_sha(tmp_path):
    incomplete = {k: v for k, v in SHAS.items() if k != "RAMMP-CuRobo"}
    r = run(tmp_path, shas=incomplete, set_id="nightly-20261003")
    assert r.returncode == 1 and "RAMMP-CuRobo" in r.stderr
```

- [ ] **Step 4: Run the tests, watch them fail**

```bash
cd ~/atdev/rammp-nightly && uv run pytest -q
```
Expected: FAIL / errors — the scripts don't exist yet.

- [ ] **Step 5: Implement the two scripts**

`scripts/stack_members.py`:

```python
#!/usr/bin/env python3
"""Print a stack's members as JSON: {name: {url, branch}}."""
import json, sys
import yaml

def main() -> int:
    stack = yaml.safe_load(open(sys.argv[1]))
    print(json.dumps(stack["members"]))
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

`scripts/write_manifest.py`:

```python
#!/usr/bin/env python3
"""Write a blessed set's manifest: <set>.repos (vcstool, exact SHAs) + <set>.images.json.

Refuses to overwrite: a blessed set is immutable, so an existing file for the
set id is an error, not something to replace.
"""
import argparse, datetime, json, sys
from pathlib import Path
import yaml

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--stack", required=True)
    p.add_argument("--set-id", required=True)
    p.add_argument("--shas", required=True, help="JSON file: {member: sha}")
    p.add_argument("--digests", required=True, help="JSON file: {image: digest}")
    p.add_argument("--out-dir", required=True)
    a = p.parse_args()

    stack = yaml.safe_load(open(a.stack))
    shas = json.load(open(a.shas))
    digests = json.load(open(a.digests))

    missing = [m for m in stack["members"] if m not in shas]
    if missing:
        print(f"no SHA resolved for member(s): {', '.join(missing)}", file=sys.stderr)
        return 1

    out = Path(a.out_dir)
    repos_path = out / f"{a.set_id}.repos"
    images_path = out / f"{a.set_id}.images.json"
    for f in (repos_path, images_path):
        if f.exists():
            print(f"{f} already exists; blessed sets are immutable", file=sys.stderr)
            return 1
    out.mkdir(parents=True, exist_ok=True)

    repos = {"repositories": {
        name: {"type": "git", "url": member["url"], "version": shas[name]}
        for name, member in stack["members"].items()}}
    repos_path.write_text(yaml.safe_dump(repos, sort_keys=False))
    images_path.write_text(json.dumps({
        "set": a.set_id,
        "stack": stack["stack"],
        "generated": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "images": digests}, indent=2) + "\n")
    print(f"wrote {repos_path} and {images_path}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6: Tests pass**

```bash
uv run pytest -q
```
Expected: 4 passed.

- [ ] **Step 7: README (the collaborator-facing page) and commit**

`README.md` must cover, briefly: what a set is and its guarantee (CI-grade, no RT/arm validation); how to consume (`docker pull ghcr.io/rammp-org/<image>:nightly` or pin a date; `vcs import < manifests/kinova-arm/<set>.repos` for source builds); where freshness is visible (newest file in `manifests/kinova-arm/`, the tracking issue when red); that dated sets are immutable and `:nightly` only moves on green; and the dispatch contract (the four inputs + promote) a repo implements to join a stack.

```bash
git add -A && git commit -m "feat: kinova-arm stack definition + manifest tooling

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>" && git push -u origin main
```

---

### Task 6: rammp-nightly — the orchestration workflow

**Files:**
- Create: `rammp-nightly/.github/workflows/kinova-arm.yml`

**Interfaces:**
- Consumes: Task 1/3/4 dispatch contracts; Task 5 scripts and stack file; secret `RAMMP_NIGHTLY_TOKEN`.
- Produces: the nightly channel itself — on green, dated tags promoted and a manifest commit on rammp-nightly main.

- [ ] **Step 1: Write the workflow**

```yaml
name: kinova-arm nightly set

# Orchestrates only: resolves SHAs, dispatches each repo's OWN build workflow
# in dependency waves, awaits each leg, and blesses the set (promote + manifest)
# only when every leg is green. Owns no builds; writes to no ghcr package.
on:
  schedule:
    - cron: "0 9 * * *"   # 09:00 UTC ≈ 4–5 am US Eastern
  workflow_dispatch:
    inputs:
      # Pre-flight overrides: build the set with a feature branch in one or
      # more members before anything merges to dev. Empty = the stack branch.
      interfaces_ref: {description: "override rammp-interfaces-ros2 ref", required: false, default: "", type: string}
      docker_ref:     {description: "override RAMMP-docker ref",          required: false, default: "", type: string}
      driver_ref:     {description: "override kinova-gen3-driver ref",    required: false, default: "", type: string}
      curobo_ref:     {description: "override RAMMP-CuRobo ref",          required: false, default: "", type: string}
      node_ref:       {description: "override kinova-gen3-ros2 ref",      required: false, default: "", type: string}
      rc:             {description: "rc suffix (e.g. rc1): rehearsal/pre-flight set, kept out of the plain date", required: false, default: "", type: string}

permissions:
  contents: write   # the record job commits the manifest
  issues: write     # the red/green tracking issue

concurrency:
  group: kinova-arm-nightly
  cancel-in-progress: false

jobs:
  resolve:
    runs-on: ubuntu-24.04
    outputs:
      set_id: ${{ steps.resolve.outputs.set_id }}
      shas: ${{ steps.resolve.outputs.shas }}
      refs: ${{ steps.resolve.outputs.refs }}
      set_info: ${{ steps.resolve.outputs.set_info }}
    steps:
      - uses: actions/checkout@v4
      - name: Resolve every member at one instant
        id: resolve
        env:
          GH_TOKEN: ${{ secrets.RAMMP_NIGHTLY_TOKEN }}
          OVR_INTERFACES: ${{ inputs.interfaces_ref }}
          OVR_DOCKER: ${{ inputs.docker_ref }}
          OVR_DRIVER: ${{ inputs.driver_ref }}
          OVR_CUROBO: ${{ inputs.curobo_ref }}
          OVR_NODE: ${{ inputs.node_ref }}
          RC: ${{ inputs.rc }}
        run: |
          python3 -m pip install --quiet pyyaml
          members=$(python3 scripts/stack_members.py stacks/kinova-arm.yml)
          declare -A OVR=( [rammp-interfaces-ros2]="$OVR_INTERFACES" [RAMMP-docker]="$OVR_DOCKER" \
                           [kinova-gen3-driver]="$OVR_DRIVER" [RAMMP-CuRobo]="$OVR_CUROBO" \
                           [kinova-gen3-ros2]="$OVR_NODE" )
          shas="{" ; refs="{"
          for name in $(echo "$members" | python3 -c 'import json,sys; print(" ".join(json.load(sys.stdin)))'); do
            branch=$(echo "$members" | python3 -c "import json,sys; print(json.load(sys.stdin)['$name']['branch'])")
            ref="${OVR[$name]:-$branch}"
            sha=$(gh api "repos/rammp-org/${name}/commits/${ref}" --jq .sha)
            shas+="\"$name\":\"$sha\"," ; refs+="\"$name\":\"$ref\","
          done
          shas="${shas%,}}" ; refs="${refs%,}}"
          set_id="nightly-$(date -u +%Y%m%d)${RC:+-$RC}"
          if git ls-files "manifests/kinova-arm/${set_id}.repos" | grep -q .; then
            echo "::error::${set_id} already has a blessed manifest; sets are immutable. Use rc: for a rehearsal."
            exit 1
          fi
          set_info=$(printf '{"set":"%s","stack":"kinova-arm","shas":%s}' "$set_id" "$shas" | base64 -w0)
          { echo "set_id=$set_id"; echo "shas=$shas"; echo "refs=$refs"; echo "set_info=$set_info"; } >> "$GITHUB_OUTPUT"

  base:
    needs: resolve
    runs-on: ubuntu-24.04
    timeout-minutes: 120
    steps:
      - uses: convictional/trigger-workflow-and-wait@v1.6.5
        with:
          owner: rammp-org
          repo: RAMMP-docker
          github_token: ${{ secrets.RAMMP_NIGHTLY_TOKEN }}
          workflow_file_name: build-push.yml
          ref: ${{ fromJSON(needs.resolve.outputs.refs)['RAMMP-docker'] }}
          wait_interval: 60
          propagate_failure: true
          client_payload: >-
            {"publish_tag":"${{ needs.resolve.outputs.set_id }}",
             "interfaces_ref":"${{ fromJSON(needs.resolve.outputs.shas)['rammp-interfaces-ros2'] }}",
             "expected_sha":"${{ fromJSON(needs.resolve.outputs.shas)['RAMMP-docker'] }}",
             "set_info":"${{ needs.resolve.outputs.set_info }}"}

  planner:
    needs: [resolve, base]
    runs-on: ubuntu-24.04
    timeout-minutes: 180
    steps:
      - uses: convictional/trigger-workflow-and-wait@v1.6.5
        with:
          owner: rammp-org
          repo: RAMMP-CuRobo
          github_token: ${{ secrets.RAMMP_NIGHTLY_TOKEN }}
          workflow_file_name: image.yml
          ref: ${{ fromJSON(needs.resolve.outputs.refs)['RAMMP-CuRobo'] }}
          wait_interval: 60
          propagate_failure: true
          client_payload: >-
            {"publish_tag":"${{ needs.resolve.outputs.set_id }}",
             "base_image":"ghcr.io/rammp-org/rammp-cuda:${{ needs.resolve.outputs.set_id }}",
             "expected_sha":"${{ fromJSON(needs.resolve.outputs.shas)['RAMMP-CuRobo'] }}",
             "set_info":"${{ needs.resolve.outputs.set_info }}"}

  node:
    needs: [resolve, base]
    runs-on: ubuntu-24.04
    timeout-minutes: 120
    steps:
      - uses: convictional/trigger-workflow-and-wait@v1.6.5
        with:
          owner: rammp-org
          repo: kinova-gen3-ros2
          github_token: ${{ secrets.RAMMP_NIGHTLY_TOKEN }}
          workflow_file_name: build.yml
          ref: ${{ fromJSON(needs.resolve.outputs.refs)['kinova-gen3-ros2'] }}
          wait_interval: 60
          propagate_failure: true
          client_payload: >-
            {"publish_tag":"${{ needs.resolve.outputs.set_id }}",
             "base_image":"ghcr.io/rammp-org/rammp-base:${{ needs.resolve.outputs.set_id }}",
             "core_ref":"${{ fromJSON(needs.resolve.outputs.shas)['kinova-gen3-driver'] }}",
             "curobo_ref":"${{ fromJSON(needs.resolve.outputs.shas)['RAMMP-CuRobo'] }}",
             "expected_sha":"${{ fromJSON(needs.resolve.outputs.shas)['kinova-gen3-ros2'] }}",
             "set_info":"${{ needs.resolve.outputs.set_info }}"}

  promote:
    needs: [resolve, base, planner, node]
    runs-on: ubuntu-24.04
    strategy:
      matrix:
        repo: [RAMMP-docker, RAMMP-CuRobo, kinova-gen3-ros2]
    steps:
      - uses: convictional/trigger-workflow-and-wait@v1.6.5
        with:
          owner: rammp-org
          repo: ${{ matrix.repo }}
          github_token: ${{ secrets.RAMMP_NIGHTLY_TOKEN }}
          workflow_file_name: promote.yml
          ref: ${{ fromJSON(needs.resolve.outputs.refs)[matrix.repo] }}
          wait_interval: 15
          propagate_failure: true
          client_payload: '{"source_tag":"${{ needs.resolve.outputs.set_id }}"}'

  record:
    needs: [resolve, promote]
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - name: Collect image digests
        id: digests
        env:
          SET: ${{ needs.resolve.outputs.set_id }}
        run: |
          d="{"
          for img in rammp-base rammp-cuda rammp-curobo kinova-gen3-ros2; do
            digest=$(docker buildx imagetools inspect "ghcr.io/rammp-org/${img}:${SET}" --format '{{json .Manifest}}' | python3 -c 'import json,sys; print(json.load(sys.stdin)["digest"])')
            d+="\"$img\":\"$digest\","
          done
          echo "digests=${d%,}}" >> "$GITHUB_OUTPUT"
      - name: Write and commit the manifest
        env:
          SET: ${{ needs.resolve.outputs.set_id }}
          SHAS: ${{ needs.resolve.outputs.shas }}
          DIGESTS: ${{ steps.digests.outputs.digests }}
        run: |
          python3 -m pip install --quiet pyyaml
          echo "$SHAS" > /tmp/shas.json && echo "$DIGESTS" > /tmp/digests.json
          python3 scripts/write_manifest.py --stack stacks/kinova-arm.yml --set-id "$SET" \
            --shas /tmp/shas.json --digests /tmp/digests.json --out-dir manifests/kinova-arm
          git config user.name "rammp-nightly" && git config user.email "nightly@rammp.org"
          git add manifests && git commit -m "bless ${SET}" && git push
      - name: Close the red issue if one is open
        env:
          GH_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          GH_REPO: ${{ github.repository }}
          SET: ${{ needs.resolve.outputs.set_id }}
        run: |
          n=$(gh issue list --label nightly-red --state open --limit 1 --json number -q '.[0].number')
          [ -n "$n" ] && gh issue close "$n" --comment "Green again: ${SET} blessed." || true

  report-failure:
    needs: [resolve, base, planner, node, promote, record]
    if: failure()
    runs-on: ubuntu-24.04
    steps:
      - name: File or update the nightly-red issue
        env:
          GH_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          GH_REPO: ${{ github.repository }}
          RUN_URL: ${{ github.server_url }}/${{ github.repository }}/actions/runs/${{ github.run_id }}
        run: |
          body="The kinova-arm nightly did not bless a set. ${RUN_URL} — the failing leg's own logs are linked from its job. :nightly still serves the last green set."
          n=$(gh issue list --label nightly-red --state open --limit 1 --json number -q '.[0].number')
          if [ -n "$n" ]; then gh issue comment "$n" --body "$body"; else
            gh issue create --label nightly-red --title "kinova-arm nightly is red" --body "$body"; fi
```

- [ ] **Step 2: Create the `nightly-red` label, lint, commit**

```bash
cd ~/atdev/rammp-nightly
gh label create nightly-red --repo rammp-org/rammp-nightly --color B60205 --description "no blessed set last night" || true
docker run --rm -v "$PWD":/repo -w /repo rhysd/actionlint:latest -color
git add .github/workflows/kinova-arm.yml && git commit -m "feat: kinova-arm nightly orchestration" && git push
```
Expected: actionlint clean. Note the cron is live from this push — acceptable: until Tasks 1–4 are merged, a scheduled run fails at dispatch and files the red issue, which is the system working. If that noise is unwanted before rollout, comment the `schedule:` block out in this commit and restore it in Task 7.

---

### Task 7: First end-to-end set, rehearsal of the failure modes, rollout

**Files:** none new — this task exercises everything and flips the channel on.

- [ ] **Step 1: Pre-flight rehearsal set (-rc1)**

Prerequisites: Tasks 1–6 merged (contract workflows present on each repo's main).

```bash
gh workflow run kinova-arm.yml --repo rammp-org/rammp-nightly -f rc=rc1
gh run watch --repo rammp-org/rammp-nightly $(gh run list --repo rammp-org/rammp-nightly -w kinova-arm.yml -L1 --json databaseId -q '.[0].databaseId')
```
Expected: resolve → base → {planner, node} → promote×3 → record, all green. Verify the bless:

```bash
SET=nightly-$(date -u +%Y%m%d)-rc1
for img in rammp-base rammp-cuda rammp-curobo kinova-gen3-ros2; do
  docker buildx imagetools inspect ghcr.io/rammp-org/$img:$SET --format '{{.Manifest.Digest}}'
done
git -C ~/atdev/rammp-nightly pull && cat manifests/kinova-arm/$SET.images.json
docker run --rm ghcr.io/rammp-org/kinova-gen3-ros2:$SET cat /etc/rammp-set.json
```
Expected: four digests matching the committed `images.json`; the stamp names the set and the five SHAs.

- [ ] **Step 2: Rehearse failure mode 1+4 (stale SHA stops the chain before promote)**

```bash
gh workflow run build-push.yml --repo rammp-org/RAMMP-docker --ref main \
  -f publish_tag=nightly-19700102-rc1 -f expected_sha=0000000000000000000000000000000000000000
```
Expected: the run fails in the `Branch did not move…` step within a minute. Then confirm the orchestrator-side behavior from the Step 1 run's structure: `promote` lists `needs: [resolve, base, planner, node]` — a red leg can never reach it (statically true; the stale-SHA run shows the leg-level failure is loud and early).

- [ ] **Step 3: Rehearse failure mode 2 (release-space tag refused)**

```bash
gh workflow run build-push.yml --repo rammp-org/RAMMP-docker --ref main -f publish_tag=1.9.9
```
Expected: fails in `Derive version` with the "refusing to write into release tag space" error, before anything builds.

- [ ] **Step 4: Rehearse failure mode 3 (same set id refused)**

Re-run Step 1's command with the same `rc=rc1` on the same day. Expected: the `resolve` job fails with "already has a blessed manifest; sets are immutable".

- [ ] **Step 5: Clean up rehearsal artifacts**

Delete the `nightly-*-rc1` and `nightly-19700101/2-rc1` package versions from the four ghcr packages (GitHub UI, or `gh api -X DELETE /orgs/rammp-org/packages/container/<img>/versions/<id>`). Reset `:nightly` if a rehearsal moved it and a real set hasn't yet: re-run each repo's promote with the last real dated tag, or leave it — the first true nightly re-points it. The rc manifest files stay in git — they are history, and the date-guard only blocks exact-id reuse.

- [ ] **Step 6: Turn the channel on**

If the cron was commented out in Task 6: uncomment, commit, push. Otherwise nothing to do — the next 09:00 UTC run is the first real set. After it blesses:

```bash
git -C ~/atdev/rammp-nightly pull && ls manifests/kinova-arm/
```

- [ ] **Step 7: Tell the collaborators**

Point them at rammp-nightly's README (Task 5 Step 7): pull `:nightly` or pin a date; `vcs import` the manifest for source builds; the tracking issue is the red light. Remind them of the guarantee boundary: CI-grade, not arm-validated — that remains what releases are for.
