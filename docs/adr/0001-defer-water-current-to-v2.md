# ADR-0001: Defer water current / tidal stream to v2

- **Status:** Accepted
- **Date:** 2026-06-21

## Context

The tool is a browser-based 2D dynamic simulation for teaching the physics of
sailboat spring lines, engine (saildrive) and wind during docking. The user
specified configurable **wind** as a v1 requirement; **water current / tidal
stream** was not requested but is a significant real-world docking factor
(stream running along a pontoon, wind-against-tide situations).

Adding current is cheap given our chosen physics model:

- Hull drag is **anisotropic linear damping computed against the water**
  (see core physics model). It is therefore already water-relative — a uniform
  current is simply "water velocity ≠ 0".
- Rudder authority uses **water-relative flow speed**, so it reacts to current
  automatically.

So the marginal *implementation* cost is roughly: one current speed/direction
control, a visual indicator, and using `v_rel = v_boat - v_current` in the drag
and rudder terms.

The real cost of any new force field is **calibration and validation**, not
code. The core spring-line / engine / wind / hull-contact story is already a
complete, shippable teaching tool on its own.

## Decision

Exclude water current from v1. Ship v1 with wind, lines, saildrive propulsion,
1-or-2 rudder steering, and hull–quay contact. Add a uniform current field in
v2 as a clean bolt-on, leveraging the fact that drag and rudder forces are
already water-relative.

## Consequences

- v1 stays focused: fewer forces to tune and validate before the core feels right.
- The physics core MUST keep hull drag and rudder authority expressed against
  **water velocity** (not ground velocity), so that current drops in without
  refactoring. This is a binding constraint on the v1 implementation.
- We lose the wind-against-tide teaching scenario until v2.
- No user-facing current control, indicator, or persisted setting in v1.
