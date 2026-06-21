# ADR-0002: Core 2D rigid-body physics model

- **Status:** Accepted
- **Date:** 2026-06-21

## Context

The tool teaches spring-line / engine / wind docking physics by simulating a
moving boat with live force-and-moment overlays. It must be qualitatively
correct, numerically stable at docking speeds, and transparent enough to teach
from. It is not a calibrated naval-architecture simulator.

## Decision

Model the boat as a **2D rigid body with 3 degrees of freedom**: surge (x),
sway (y), yaw (heading). Time-step it explicitly and draw force arrows from the
same per-step force accumulation.

Forces, all summed each step into a net force + net moment about the centre of
mass:

- **Hull water resistance: anisotropic *linear* damping**, separate coefficients
  for surge (low), sway (high), yaw (high). Computed against **water velocity**
  (see ADR-0001 — keeps current a clean v2 bolt-on).
- **Added mass** on sway and yaw as constant additions to the effective
  mass/inertia (the dominant reason a boat responds sluggishly sideways at dock
  speeds). Constant, not configuration-dependent.
- **Lines: stiff one-way penalty springs.** Force = stiffness x stretch (+ light
  damping) only when end-to-end distance > rest length; zero (slack) otherwise.
  Tension is therefore a directly available quantity to display.
- **Hull–quay contact: one-way penalty normal force + Coulomb friction.** The
  dock is a list of straight wall segments (ADR on dock geometry); the hull edge
  / fender points collide with them. Inside corners may contact two segments.
- **Wind:** angle-dependent projected area x apparent-wind-speed^2, applied at a
  windage centre forward of midships (ADR on wind).
- **Propulsion / rudder:** see ADR-0003.

Integration: explicit (semi-implicit/symplectic Euler preferred for damped
springs), with stiffness kept sane and timestep capped so penalty springs and
contacts stay stable.

## Consequences

- Stable and cheap; tunable by eye against the four acceptance scenarios.
- Quantitatively approximate (linear drag ignores v^2 growth; penalty springs
  permit small visible stretch). Acceptable for a lines-teaching tool.
- Tension and contact forces are penalty outputs, trivially available for the
  overlay — the whole point.
- A future high-fidelity model (quadratic drag / MMG, rigid-constraint lines)
  would be a substantial rewrite of this core, knowingly deferred.
