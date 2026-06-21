# ADR-0004: Tech stack

- **Status:** Accepted
- **Date:** 2026-06-21

## Context

Browser-based 2D tool with a hand-rolled physics loop and per-frame force-vector
overlays. Must deploy as static files and stay transparent/tunable.

## Decision

- **Rendering:** HTML5 **Canvas 2D**, full redraw each frame (hull, dock
  segments, lines, fenders, force arrows, pivot point).
- **Language:** **TypeScript** — vector math + many tunable constants benefit
  from types; guards against unit/sign bugs.
- **Physics:** **hand-rolled**, no physics/game engine. Model is small (3 DOF,
  penalty springs/contacts, explicit integrator) and must stay transparent.
- **Build:** **Vite**; static output deployable anywhere.
- **UI:** **vanilla TS + DOM** for the control panel. No React/reactive layer —
  the meaningful state is the imperatively-redrawn sim.

## Consequences

- Lightweight, dependency-light, single static bundle.
- No component framework: control-panel wiring is manual DOM. Accepted as low
  cost for this scope; revisit only if UI complexity grows substantially.
- Canvas (not SVG/WebGL) — right for per-frame arrows at this geometry count.
