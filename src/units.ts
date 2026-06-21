// Distances: metres. Boat length input: feet. Speed: knots internally displayed.
// 1 pixel = SCALE_PX_PER_M metres at the nominal zoom level.

export const SCALE_PX_PER_M = 8  // 8 px per metre → 40 m boat ≈ 320 px

export const ftToM = (ft: number): number => ft * 0.3048
export const mToFt = (m: number): number => m / 0.3048
export const knotsToMs = (kn: number): number => kn * 0.51444
export const msToKnots = (ms: number): number => ms / 0.51444
