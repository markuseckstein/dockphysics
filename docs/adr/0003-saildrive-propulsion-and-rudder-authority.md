# ADR-0003: Saildrive propulsion and rudder authority

- **Status:** Accepted
- **Date:** 2026-06-21

## Context

The boat always has a **saildrive** (centreline prop, mounted forward of the
rudder, under the sole). Rudder configuration is user-selectable: **one centre
rudder** or **two outboard rudders**. The defining real-world behaviour the sim
must reproduce: with a saildrive the prop wash does **not** blow strongly over
the rudder the way a shaft drive does, so low-/zero-speed steering is weak — and
*how* weak depends on rudder configuration.

Physics, established during design:

- **Single centre rudder:** sits on the centreline behind the prop, so *some*
  wash reaches it. Non-zero but modest standstill steering (wash has spread/risen
  by the time it reaches the aft-mounted rudder).
- **Twin outboard rudders:** mounted off-centreline; wash passes between them and
  reaches neither. Standstill steering ~ zero — only boat-speed flow steers them.

## Decision

- **Thrust:** throttle (forward/reverse) -> force along the boat centreline,
  applied at the saildrive point. Magnitude scales with throttle.
- **Rudder side-force** = f(rudder angle) x (water-relative flow speed at the
  rudder)^2. The flow-speed term is **boat speed only** (water-relative, per
  ADR-0001) — *no* general prop-wash term. Rudder bites when moving, limp at rest.
- **Prop-wash standstill coupling** — the one configuration-dependent steering
  term — is exposed as a **slider** ("standstill steering authority"):
  - default **modest, non-zero** for single centre rudder;
  - **locked at zero** for twin outboard rudders (physically cannot get wash).
- **Prop walk:** small saildrive-grade lateral/yaw kick in reverse (constant,
  possibly near-zero), reflecting that saildrives walk little.

## Consequences

- Reproduces the signature saildrive trait: you cannot kick the stern over with
  throttle at a standstill (especially twin-rudder) — a core teaching point.
- The rudder-config switch is meaningfully physical, not cosmetic: it changes the
  standstill-coupling default and locks it to zero for twins.
- Rudder authority stays water-relative, consistent with ADR-0001/0002.
