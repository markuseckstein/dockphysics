import type { RigidBody, Forces } from './integrator'
import type { BoatGeometry, Vec2 } from '../scene/boat'

// ── Control & configuration state ────────────────────────────────────────────

export type RudderConfig = 'single' | 'twin'

export interface Controls {
  throttle: number          // -1 (full astern) .. 0 (neutral) .. +1 (full ahead)
  helm: number              // -1 .. +1, maps to rudder angle ±maxRudderAngle
  rudderConfig: RudderConfig
  standstillAuthority: number // 0..1, prop-wash standstill steering; locked 0 for twin
}

export function defaultStandstillAuthority(config: RudderConfig): number {
  // Single centre rudder gets some wash; twin outboard rudders get none.
  return config === 'single' ? 0.3 : 0
}

// The twin-rudder configuration can physically never receive prop wash between
// its outboard blades, so its standstill authority is always clamped to zero.
export function effectiveStandstillAuthority(c: Controls): number {
  if (c.rudderConfig === 'twin') return 0
  return Math.max(0, Math.min(1, c.standstillAuthority))
}

// ── Tuning ───────────────────────────────────────────────────────────────────

export interface PropulsionTuning {
  maxThrust: number       // N at full throttle
  rudderCoeff: number     // N per (unit sin(angle) · (m/s)²)
  maxRudderAngle: number  // rad at full helm
  washSpeed2: number      // (m/s)² of equivalent flow from full prop wash
  propWalk: number        // N·m yaw moment at full astern (saildrive-grade, small)
}

const V_MAX = 2.5             // m/s target terminal surge at full throttle (~5 kn)
const MAX_RUDDER_ANGLE = 0.611 // rad ≈ 35°

// Derive tuning from boat geometry so behaviour scales with boat size.
export function propulsionTuning(geo: BoatGeometry): PropulsionTuning {
  const maxThrust = geo.dampSurge * V_MAX
  return {
    maxThrust,
    // At cruise (~2 m/s, q≈4) full helm produces roughly 2× thrust of side-force.
    rudderCoeff: (2 * maxThrust) / (Math.sin(MAX_RUDDER_ANGLE) * 4),
    maxRudderAngle: MAX_RUDDER_ANGLE,
    washSpeed2: 2.0,
    propWalk: geo.massKg * 0.05,
  }
}

// ── Force components ─────────────────────────────────────────────────────────

// A single named force, expressed in the BODY frame and applied at a point in
// the body frame. Shared by the integrator feed and the overlay renderer.
export interface ForceComponent {
  label: 'thrust' | 'rudder' | 'prop-walk'
  point: Vec2  // application point, body frame (origin at CoM)
  fx: number   // body-frame surge force, N
  fy: number   // body-frame sway force, N (port +y positive)
}

export function rudderAngle(c: Controls, t: PropulsionTuning): number {
  return c.helm * t.maxRudderAngle
}

// Compute every propulsion/steering force component for the current state.
// Rudder side-force ∝ sin(angle) × (water-relative flow)², where the flow term
// is boat surge speed (sign-preserving) plus a throttle-driven prop-wash term
// that only exists for the single centre rudder (standstill authority).
export function controlForces(
  body: RigidBody,
  geo: BoatGeometry,
  controls: Controls,
  t: PropulsionTuning,
): ForceComponent[] {
  const components: ForceComponent[] = []

  // Surge (forward) speed through the water, body frame.
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  const vSurge = body.vx * cos + body.vy * sin

  // Thrust: centreline force at the saildrive, scales with throttle.
  const thrust = controls.throttle * t.maxThrust
  if (thrust !== 0) {
    components.push({ label: 'thrust', point: geo.saildrive, fx: thrust, fy: 0 })
  }

  // Rudder side-force. Dynamic-pressure term q (sign carries flow direction so
  // the rudder reverses correctly when going astern). Prop wash only adds flow
  // under forward throttle and only reaches a single centre rudder.
  const angle = rudderAngle(controls, t)
  const authority = effectiveStandstillAuthority(controls)
  const wash = authority * t.washSpeed2 * Math.max(0, controls.throttle)
  const q = vSurge * Math.abs(vSurge) + wash
  const sideForce = t.rudderCoeff * Math.sin(angle) * q  // total side-force, N

  if (sideForce !== 0) {
    if (controls.rudderConfig === 'twin') {
      // Split equally across the two outboard rudders.
      components.push({ label: 'rudder', point: { x: geo.rudderX, y:  geo.rudderOutboardY }, fx: 0, fy: sideForce / 2 })
      components.push({ label: 'rudder', point: { x: geo.rudderX, y: -geo.rudderOutboardY }, fx: 0, fy: sideForce / 2 })
    } else {
      components.push({ label: 'rudder', point: { x: geo.rudderX, y: 0 }, fx: 0, fy: sideForce })
    }
  }

  // Prop walk: small saildrive-grade lateral kick at the saildrive, in astern only.
  if (controls.throttle < 0) {
    const walk = t.propWalk * controls.throttle  // N·m equivalent; sign set below
    // Apply as a small sway force at the saildrive so it yaws the stern slightly.
    const fyWalk = walk / Math.max(0.5, Math.abs(geo.saildrive.x))
    components.push({ label: 'prop-walk', point: geo.saildrive, fx: 0, fy: fyWalk })
  }

  return components
}

// Reduce force components to a net body-frame force + yaw moment about the CoM.
// Moment (z) of a force F applied at r is r.x·F.y − r.y·F.x.
export function netForces(components: ForceComponent[]): Forces {
  let fx = 0, fy = 0, mz = 0
  for (const c of components) {
    fx += c.fx
    fy += c.fy
    mz += c.point.x * c.fy - c.point.y * c.fx
  }
  return { fx, fy, mz }
}
