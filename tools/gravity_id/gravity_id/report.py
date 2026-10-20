"""The fit report: every family's held-out residual, the chosen correction, the gate."""

from __future__ import annotations

import numpy as np

from gravity_id.fit import FAMILIES, Selection
from gravity_id.model import NUM_JOINTS, ArmModel
from gravity_id.urdf_write import LinkChange

# Static gate (spec, Phase V): held-out RMS / max per joint, N*m.
GATE_RMS = np.array([0.5, 0.5, 0.5, 0.5, 0.2, 0.2, 0.2])
GATE_MAX = np.array([1.0, 1.0, 1.0, 1.0, 0.4, 0.4, 0.4])
GATE_J2_REDUCTION = 0.5
OFFSET_WARN = 0.3  # N*m: above this an offset matters, and it lands nowhere yet


def _row(label: str, v: np.ndarray, fmt: str = "{:.3f}") -> str:
    return f"| {label} | " + " | ".join(fmt.format(x) for x in v) + " |"


def tip_in_ee_frame(arm: ArmModel, dpi: np.ndarray) -> tuple[float, np.ndarray] | None:
    """Body 7's correction as a point mass in the EE frame: (dm kg, com m), or None."""
    dm, dh = dpi[24], dpi[25:28]
    if abs(dm) < 1e-9:
        return None
    ee = arm.model.frames[arm.ee_frame_id].placement  # EE frame -> joint 7 frame
    return dm, ee.rotation.T @ (dh / dm - ee.translation)


def render(arm: ArmModel, sel: Selection, family: str, changes: list[LinkChange], n_fit: int, n_val: int) -> str:
    joints = " | ".join(f"j{i}" for i in range(1, NUM_JOINTS + 1))
    sep = "|---" * (NUM_JOINTS + 1) + "|"
    out = [
        "# Gravity identification report",
        "",
        f"Fit holds: {n_fit}. Held-out (val) holds: {n_val}. "
        f"Chosen family: {sel.chosen}" + (f" (report and URDF use forced family {family})" if family != sel.chosen else ""),
        "",
        "## Held-out residual per family [N*m]",
        "",
        f"| family, RMS | {joints} |",
        sep,
    ]
    out += [_row(f, sel.val_rms[f]) for f in FAMILIES]
    out += ["", f"| family, max | {joints} |", sep]
    out += [_row(f, sel.val_max[f]) for f in FAMILIES]
    rejected = [f"- {f}: {sel.fits[f].rejected}" for f in FAMILIES if sel.fits[f].rejected]
    out += ["", "Rejected families (failed a physical check, never clipped):", *(rejected or ["- none"])]

    fit = sel.fits[family]
    out += ["", f"## {family} parameters", "", f"| | {joints} |", sep]
    out += [_row("offset [N*m]", fit.offsets, "{:+.3f}"), _row("± std", fit.std_offsets)]
    bodies = FAMILIES[family][0]
    if bodies:
        out += ["", "| body | dm [kg] | d(m*c) x, y, z [kg*m] | ± std (dm) |", "|---|---|---|---|"]
        for j in bodies:
            k = 4 * (j - 1)
            d = fit.dpi[k : k + 4]
            out.append(f"| {j} | {d[0]:+.4f} | {d[1]:+.4f}, {d[2]:+.4f}, {d[3]:+.4f} | {fit.std_dpi[k]:.4f} |")
    tip = tip_in_ee_frame(arm, fit.dpi)
    if tip:
        dm, c = tip
        out += [
            "",
            f"Tip correction as a point mass: {dm:+.3f} kg at ({c[0] * 1e3:+.0f}, {c[1] * 1e3:+.0f}, "
            f"{c[2] * 1e3:+.0f}) mm in the EE frame. Compare with the D3 scale weights. A static fit "
            "cannot say which body is wrong; agreement with the weights is what attributes it to the tip.",
        ]
    out += ["", "## URDF changes", ""]
    out += [
        f"- `{c.link}`: mass {c.mass_old:.4f} -> {c.mass_new:.4f} kg, COM "
        f"{np.round(c.com_old, 5).tolist()} -> {np.round(c.com_new, 5).tolist()} m (link frame)"
        for c in changes
    ] or ["- none (offsets are not a URDF quantity; see the spec's open decisions)"]

    rms, mx = sel.urdf_rms[family], sel.urdf_max[family]
    j2_drop = 1.0 - rms[1] / sel.urdf_rms["F0"][1] if sel.urdf_rms["F0"][1] > 0 else 0.0
    big = np.flatnonzero(np.abs(fit.offsets) > OFFSET_WARN)
    ok = bool(np.all(rms <= GATE_RMS) and np.all(mx <= GATE_MAX))
    out += [
        "",
        "## Static gate (held-out poses, corrected.urdf alone)",
        "",
        "Judged on what `corrected.urdf` delivers: the fitted offsets are NOT in it (a URDF "
        "cannot carry them), so they are left out of these residuals. The family tables above "
        "include them, because that is the fair way to compare families.",
        "",
        f"| | {joints} |",
        sep,
        _row("RMS", rms),
        _row("RMS limit", GATE_RMS, "{:.1f}"),
        _row("max", mx),
        _row("max limit", GATE_MAX, "{:.1f}"),
        "",
        *(
            [
                f"**WARNING: offsets above {OFFSET_WARN} N*m on "
                + ", ".join(f"j{i + 1} ({fit.offsets[i]:+.2f})" for i in big)
                + ". They are real torque error the URDF cannot fix; where they land is an open "
                "decision (spec). The gate below shows what remains without them.**",
                "",
            ]
            if len(big)
            else []
        ),
        f"Per-joint limits: {'PASS' if ok else 'FAIL'}. j2 RMS reduction vs F0: {j2_drop:.0%} "
        f"(needs >= {GATE_J2_REDUCTION:.0%}; if F0 already passes the limits, the model was fine "
        "and drift is the damping = 0 cause). Limits are loosened to 2x the measured repeatability "
        "where that is larger; see `parse` output.",
    ]
    return "\n".join(out) + "\n"
