export interface RigidBody {
  x: number        // world position, m
  y: number
  heading: number  // rad, 0 = east (+x), CCW positive
  vx: number       // world velocity, m/s
  vy: number
  yawRate: number  // rad/s, CCW positive
}

export interface BoatParams {
  mass: number           // kg
  addedMassSway: number  // kg — constant added mass on sway
  inertia: number        // kg·m²
  addedInertiaYaw: number
  dampSurge: number      // N·s/m — linear drag coefficient
  dampSway: number
  dampYaw: number        // N·m·s/rad
}

// Forces and moment in BODY frame (surge = forward, sway = port).
export interface Forces {
  fx: number  // surge force, N
  fy: number  // sway force, N (port positive)
  mz: number  // yaw moment, N·m (CCW positive)
}

export function makeBody(x: number, y: number, heading: number): RigidBody {
  return { x, y, heading, vx: 0, vy: 0, yawRate: 0 }
}

// Anisotropic linear hull damping in the BODY frame, computed against water
// (assumed still). Returned as a resisting force/moment so it can be both fed
// into the integrator and drawn as an overlay.
export function computeHullDrag(b: RigidBody, p: BoatParams): Forces {
  const cos = Math.cos(b.heading)
  const sin = Math.sin(b.heading)
  const vSurge =  b.vx * cos + b.vy * sin
  const vSway  = -b.vx * sin + b.vy * cos
  return {
    fx: -p.dampSurge * vSurge,
    fy: -p.dampSway  * vSway,
    mz: -p.dampYaw   * b.yawRate,
  }
}

// Semi-implicit (symplectic) Euler: velocity updated first, position second.
// All forces in body frame; hull damping computed against water (assumed still).
export function step(b: RigidBody, p: BoatParams, bodyForce: Forces, dt: number): RigidBody {
  const mSurge = p.mass
  const mSway  = p.mass + p.addedMassSway
  const Iyaw   = p.inertia + p.addedInertiaYaw

  const cos = Math.cos(b.heading)
  const sin = Math.sin(b.heading)

  // Decompose world velocity into body-frame components
  const vSurge =  b.vx * cos + b.vy * sin
  const vSway  = -b.vx * sin + b.vy * cos

  // Net body-frame forces: applied forces plus anisotropic hull damping
  const drag = computeHullDrag(b, p)
  const fSurge = bodyForce.fx + drag.fx
  const fSway  = bodyForce.fy + drag.fy
  const mYaw   = bodyForce.mz + drag.mz

  // Symplectic: update velocities first (body frame)
  const newVSurge  = vSurge  + (fSurge / mSurge) * dt
  const newVSway   = vSway   + (fSway  / mSway)  * dt
  const newYawRate = b.yawRate + (mYaw  / Iyaw)  * dt

  // Rotate updated body-frame velocity to world frame
  const newVx = newVSurge * cos - newVSway * sin
  const newVy = newVSurge * sin + newVSway * cos

  // Update world position with new velocity
  const newX       = b.x       + newVx      * dt
  const newY       = b.y       + newVy      * dt
  const newHeading = b.heading + newYawRate * dt

  return { x: newX, y: newY, heading: newHeading, vx: newVx, vy: newVy, yawRate: newYawRate }
}
