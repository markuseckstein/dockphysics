# ADR-0005: Dock geometry and boat parameterization

- **Status:** Accepted
- **Date:** 2026-06-21

## Context

Two scene-definition decisions that shape geometry, contact, and UI.

## Decision

### Dock

The dock is a **list of straight wall segments**, each with a cleat-able edge and
participating in penalty contact (ADR-0002). v1 ships an **L-berth**: one long
main quay + one perpendicular finger at one end, with the boat lying **alongside
the main quay**. Line dock-ends attach **free anywhere along a segment edge**
(boat-cleat -> dock-point); boat cleats are fixed (below).

Because the dock is already a segment list, a configurable finger
(length/position) and a pure slip layout are near-free follow-ups (v2).

### Boat

**Length (37–49 ft) is the only user-facing boat parameter.** Everything else is
derived by interpolating a **Bavaria Cruiser–class reference**: beam,
displacement (~7.5 t @ 37 ft -> ~12 t @ 49 ft), yaw inertia (slender-body
approx from mass x length), windage area + forward windage centre, and the
positions of the six fixed cleats (bow / midship-spring / stern, port+stbd),
the rudder(s), and the saildrive. All derived values live as tunable internal
constants anchored to the reference.

## Consequences

- One honest slider; no asking sailors for numbers they don't know.
- Large parameter space avoided -> far less to validate.
- Contact handling must cope with **inside corners** (fender touching two
  segments) from day one because of the L.
- Power-user parameter knobs (beam, displacement, windage) are a deferred
  extension, not designed-out.
