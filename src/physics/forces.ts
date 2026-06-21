import type { RigidBody, Forces } from './integrator'
import type { Vec2 } from '../scene/boat'

// A force expressed in the WORLD frame and applied at a world-frame point.
// Lines, hull–quay contact and wind are all most naturally computed in world
// coordinates; this is the shared currency the overlay draws and the integrator
// consumes (after reduction to the body frame).
export interface WorldForce {
  label: string
  point: Vec2  // application point, world frame
  fx: number   // world-frame force x, N
  fy: number   // world-frame force y, N
}

// Reduce a list of world-frame forces to a net BODY-frame force + yaw moment
// about the centre of mass, ready to feed into `step`.
//
// The body-frame surge/sway components are the world force rotated by −heading.
// The yaw moment is a scalar (frame independent): mz = r × F about the CoM.
export function worldForcesToBody(body: RigidBody, forces: WorldForce[]): Forces {
  const cos = Math.cos(body.heading)
  const sin = Math.sin(body.heading)
  let fx = 0, fy = 0, mz = 0
  for (const f of forces) {
    fx +=  f.fx * cos + f.fy * sin
    fy += -f.fx * sin + f.fy * cos
    const rx = f.point.x - body.x
    const ry = f.point.y - body.y
    mz += rx * f.fy - ry * f.fx
  }
  return { fx, fy, mz }
}
