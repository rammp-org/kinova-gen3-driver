# Arbitration — who may command the arm

The driver accepts commands from one owner at a time. A **task orchestrator**
asks the driver for a grant, the driver mints a **capability token** and returns
it, and the orchestrator passes that token to the module it is handing the arm
to. Commands that do not carry the live token are refused.

The same mechanism expresses the degenerate case — *nobody* may command — which
is what an emergency stop is.

This layer lives in `kinova::interface::Arbiter`, which decorates both
`CommandSink` and `StreamSink` — the same token gates the streaming tier too.
It knows nothing about control: no `ControlMode`, no `RtExecutor`, no
`Dynamics`. Every transport (ROS2, socket, ATOS) inherits it for free.

## The token lifecycle

- **The driver mints tokens**, not the orchestrator. `grant(owner_id)` returns a
  fresh random 128-bit token. This makes the handover ordering race impossible:
  the orchestrator has nothing to hand a module until the driver has already
  issued it, so a module can never start commanding before the driver knows
  about it.
- **Every grant mints a new token.** A stale token from a previous grant simply
  does not match, so a zombie node that comes back to life is refused.
- **A driver restart voids every token.** After a restart the arm's state is
  unknown and nothing should hold ownership across it — the orchestrator must
  re-grant.
- **`generation`** increments per grant. It is not used for the check (a stale
  token already fails to match); it exists so a rejection is diagnosable.

## States

| State | Meaning |
|---|---|
| no owner | no valid grant; commands are refused in `enforced` mode |
| owned | exactly one owner: token, `owner_id`, `generation` |
| e-stopped | **latched**; everything is refused, `disabled` mode included |

A command is admitted iff the bypass is on, or it carries the live token.
`on_query_state()` is never gated — **reads are always open**, in every state.
Cancel *is* gated: a stranger must not be able to stop your motion. The
emergency path is e-stop, not cancel.

## Handover stops the arm

Revocation is **hard**: the in-flight goal is cancelled and the arm holds at its
**measured** position, so the next owner always starts from a stationary arm in
a known state. Re-granting to a different owner is revoke-then-grant — never a
silent swap under a moving arm.

Every goal that was accepted and then dropped settles with
`result_code::kHalted`, including goals still queued behind the active one.
Nothing is left orphaned waiting for a result.

The orchestrator controls *when* this happens. If it wants a clean handover with
no mid-motion stop, it waits for the goal to finish before revoking.

Halt takes effect within one sampler period plus one RT cycle — **≤5 ms** at the
default `sampler_hz = 250`.

## Emergency stop

`estop()` drops all grants, halts, and **latches**. Every command is refused
until an explicit `estop_clear()`, which exits to *no owner* — never straight
back to owned, so someone must deliberately re-grant. Clear works in any
arbitration mode, so you cannot strand yourself.

!!! warning "This is a software e-stop"
    It depends on the driver process being alive and scheduled. It is **not** a
    hardware safety chain and not a substitute for a hardware E-stop.

## Modes

| Mode | Behaviour |
|---|---|
| `enforced` | commands must carry the live token |
| `disabled` | any command is admitted — for experimentation without an orchestrator |

`disabled` is reported in `ArbitrationStatus`, so a client can *see* that the
driver is running unarbitrated rather than trusting that someone read a startup
log. E-stop still latches in `disabled` — it is the one thing the bypass does
not bypass.

## Known gaps

- **Orchestrator death freezes ownership.** Grants do not expire. If the
  orchestrator dies holding one, ownership persists until something calls
  `revoke()` or `estop()`. Process supervision is expected to cover this; a
  heartbeat was considered and deliberately left out.
- **The token is a mistake boundary, not a security boundary.** There is no
  SROS2 on this bus — anything that can publish can observe or replay a token.
  The design keeps two well-behaved systems from fighting over the arm. It does
  not defend against a hostile one, and no safety argument should rest on it.
- **Rejections are not logged by the core.** The library has no logging
  facility ([issue #24](https://github.com/rammp-org/kinova-gen3-driver/issues/24)).
  The Arbiter counts them in `ArbitrationStatus::rejected_count` and returns a
  distinguishable code; the transport backend does the logging.
