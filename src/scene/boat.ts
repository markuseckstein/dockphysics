import { ftToM } from '../units'

export interface Vec2 { x: number; y: number }

export interface BoatGeometry {
  lengthM: number
  beamM: number
  massKg: number
  inertiaYaw: number     // kg·m²
  addedMassSway: number  // kg
  addedInertiaYaw: number
  dampSurge: number      // N·s/m
  dampSway: number
  dampYaw: number        // N·m·s/rad
  cleats: Vec2[]         // in boat body frame, origin at CoM
  fenders: Vec2[]        // contact points around the hull, body frame
  saildrive: Vec2        // centreline thrust point, body frame (forward of rudder)
  rudderX: number        // longitudinal rudder position (aft, body frame x)
  rudderOutboardY: number // lateral offset of each twin outboard rudder (|y|)
  windageFrontalArea: number // m², projected area bow-on
  windageLateralArea: number // m², projected area beam-on (much larger)
  windageCentre: Vec2        // body frame; forward of midships so the boat weathercocks
}

// Bavaria Cruiser class reference: 37 ft → 7500 kg, 49 ft → 12000 kg
// Beam: ~30 % of length. Linear interpolation on length for mass.
export function boatFromLength(lengthFt: number): BoatGeometry {
  const L = ftToM(lengthFt)
  const beamM = L * 0.315

  // Mass: linear between (37 ft, 7500 kg) and (49 ft, 12000 kg)
  const massKg = 7500 + (lengthFt - 37) / (49 - 37) * (12000 - 7500)

  // Slender-body yaw inertia: I ≈ m * L² / 12
  const inertiaYaw = massKg * L * L / 12

  // Added mass ≈ 50 % of mass on sway, 35 % of inertia on yaw
  const addedMassSway    = massKg * 0.50
  const addedInertiaYaw  = inertiaYaw * 0.35

  // Linear damping coefficients tuned for docking-speed stability.
  // Surge: moderate so the engine bites — the surge time-constant is
  // mass/dampSurge = 1/0.20 ≈ 5 s (was 0.06 ≈ 17 s, which felt sluggish).
  // Top speed is unaffected because maxThrust scales with dampSurge.
  // Sway: high (resists lateral skating). Yaw: lowered (0.80 → 0.60) so the
  // boat builds and holds a turn faster while still settling cleanly.
  const dampSurge = massKg * 0.20
  const dampSway  = (massKg + addedMassSway) * 0.80
  const dampYaw   = (inertiaYaw + addedInertiaYaw) * 0.60

  // Six cleats: bow/midship/stern × port/stbd
  // x is longitudinal (positive = bow), y is lateral (positive = port/+y)
  const halfL = L / 2
  const halfB = beamM / 2
  const cleats: Vec2[] = [
    { x:  halfL,         y:  halfB * 0.6 },  // bow stbd
    { x:  halfL,         y: -halfB * 0.6 },  // bow port
    { x:  0,             y:  halfB },          // midship stbd
    { x:  0,             y: -halfB },          // midship port
    { x: -halfL * 0.85,  y:  halfB * 0.6 },   // stern stbd
    { x: -halfL * 0.85,  y: -halfB * 0.6 },   // stern port
  ]

  // Fender / contact points around the hull edge: bow tip, bow shoulders,
  // midship, and the stern quarters. The dock pushes back at whichever of these
  // penetrate a wall segment.
  const fenders: Vec2[] = [
    { x:  halfL,        y:  0     },  // bow tip
    { x:  halfL * 0.85, y:  halfB },  // bow stbd shoulder
    { x:  halfL * 0.85, y: -halfB },  // bow port shoulder
    { x:  0,            y:  halfB },  // midship stbd
    { x:  0,            y: -halfB },  // midship port
    { x: -halfL,        y:  halfB },  // stern stbd quarter
    { x: -halfL,        y: -halfB },  // stern port quarter
  ]

  // Saildrive sits on the centreline, under the sole, forward of the rudder.
  // Rudder(s) sit near the stern; twin outboard rudders are offset to ~half-beam.
  const saildrive: Vec2 = { x: -halfL * 0.30, y: 0 }
  const rudderX = -halfL * 0.90
  const rudderOutboardY = halfB * 0.55

  // Windage: above-water profile. Beam-on (lateral) exposes the whole side; bow-on
  // (frontal) only the beam-width slice. Freeboard ~ 6 % of length is a rough
  // Bavaria-class anchor. The windage centre sits forward of midships so a beam
  // wind yaws the bow off downwind (weathercocking).
  const freeboard = L * 0.06 + 1.0
  const windageLateralArea = L * freeboard
  const windageFrontalArea = beamM * freeboard
  const windageCentre: Vec2 = { x: halfL * 0.18, y: 0 }

  return {
    lengthM: L, beamM, massKg, inertiaYaw, addedMassSway, addedInertiaYaw,
    dampSurge, dampSway, dampYaw, cleats, fenders,
    saildrive, rudderX, rudderOutboardY,
    windageFrontalArea, windageLateralArea, windageCentre,
  }
}
