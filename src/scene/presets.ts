import { makeBody } from '../physics/integrator'
import type { RigidBody } from '../physics/integrator'
import { boatFromLength } from './boat'
import type { BoatGeometry } from './boat'
import { makeLine, cleatWorld } from '../physics/lines'
import type { Line } from '../physics/lines'
import type { Controls } from '../physics/propulsion'

export type PresetId =
  | 'weathercock'
  | 'spring-stern-in'
  | 'spring-off-bow'
  | 'standstill-steering'

export interface PresetInfo {
  id: PresetId
  label: string
  description: string
}

// The four acceptance presets (PRD §"Acceptance criteria").
export const PRESETS: PresetInfo[] = [
  { id: 'weathercock',         label: '1 — Weathercock',         description: 'Beam wind, no lines/engine → bow blows off downwind.' },
  { id: 'spring-stern-in',     label: '2 — Spring the stern in', description: 'Forward bow spring, ahead + helm → stern walks in onto the fenders.' },
  { id: 'spring-off-bow',      label: '3 — Spring off the bow',  description: 'Aft quarter spring, astern → bow swings out.' },
  { id: 'standstill-steering', label: '4 — Standstill steering', description: 'Zero speed, ahead + helm → single rudder kicks, twin barely responds.' },
]

export interface PresetState {
  id: PresetId
  lengthFt: number
  wind: { speed: number; dirToward: number }  // m/s, world radians (direction blown toward)
  body: RigidBody
  controls: Partial<Controls>
  lines: Line[]
}

const DEFAULT_LENGTH_FT = 40

// Standard berth geometry, anchored to the boat size.
export function quayLength(geo: BoatGeometry): number { return geo.lengthM * 2.5 }
export function fingerLength(geo: BoatGeometry): number { return geo.beamM * 4 }

// Boat lying alongside the main quay (quay on the −y side, just below the hull).
const BERTH_GAP = 0.5
function alongsideBody(geo: BoatGeometry): RigidBody {
  return makeBody(geo.lengthM * 0.8, geo.beamM / 2 + BERTH_GAP, 0)
}

// Boat sitting in open water, well clear of the dock (for wind / standstill demos).
function openWaterBody(geo: BoatGeometry): RigidBody {
  return makeBody(quayLength(geo) / 2, fingerLength(geo) + geo.lengthM, 0)
}

// Index of the bow / stern cleat on the quay side (smallest body-frame y).
function quaySideCleat(geo: BoatGeometry, which: 'bow' | 'stern'): number {
  const candidates = which === 'bow' ? [0, 1] : [4, 5]
  return candidates.reduce((a, b) => (geo.cleats[b].y < geo.cleats[a].y ? b : a))
}

export function buildPreset(id: PresetId, lengthFt: number = DEFAULT_LENGTH_FT): PresetState {
  const geo = boatFromLength(lengthFt)

  switch (id) {
    case 'weathercock': {
      return {
        id, lengthFt,
        wind: { speed: 12, dirToward: -Math.PI / 2 },  // beam wind blowing toward −y
        body: openWaterBody(geo),
        controls: { throttle: 0, helm: 0 },
        lines: [],
      }
    }

    case 'spring-stern-in': {
      const body = alongsideBody(geo)
      const cleat = quaySideCleat(geo, 'bow')
      // After bow spring: led AFT from the bow cleat to the main quay (y = 0).
      // Engine ahead loads it; the bow is held and the stern walks in, pivoting
      // about the spring cleat.
      const cw = cleatWorld(body, geo, cleat)
      const dockPoint = { x: cw.x - geo.lengthM * 0.6, y: 0 }
      return {
        id, lengthFt,
        wind: { speed: 0, dirToward: 0 },
        body,
        // Engine ahead loads the spring; helm hard over kicks the stern in (prop
        // wash over the centre rudder), so the boat pivots about the bow cleat.
        controls: { throttle: 0.5, helm: -1, rudderConfig: 'single', standstillAuthority: 0.6 },
        lines: [makeLine(body, geo, cleat, dockPoint)],
      }
    }

    case 'spring-off-bow': {
      const body = alongsideBody(geo)
      const cleat = quaySideCleat(geo, 'stern')
      // Quarter spring led FORWARD from the stern cleat to the main quay. Engine
      // astern loads it; the stern is held and the bow swings out off the dock.
      const cw = cleatWorld(body, geo, cleat)
      const dockPoint = { x: cw.x + geo.lengthM * 0.6, y: 0 }
      return {
        id, lengthFt,
        wind: { speed: 0, dirToward: 0 },
        body,
        controls: { throttle: -0.6, helm: 0, rudderConfig: 'single' },
        lines: [makeLine(body, geo, cleat, dockPoint)],
      }
    }

    case 'standstill-steering': {
      return {
        id, lengthFt,
        wind: { speed: 0, dirToward: 0 },
        body: openWaterBody(geo),
        controls: { throttle: 1, helm: 1, rudderConfig: 'single', standstillAuthority: 0.3 },
        lines: [],
      }
    }
  }
}
