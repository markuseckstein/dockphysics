export interface Segment {
  x1: number; y1: number
  x2: number; y2: number
  nx: number; ny: number  // unit outward normal (pointing into the water / berth)
}

function makeSegment(x1: number, y1: number, x2: number, y2: number, normalSide: 1 | -1): Segment {
  const dx = x2 - x1
  const dy = y2 - y1
  const len = Math.hypot(dx, dy)
  // Perpendicular to (dx,dy): rotate 90° CCW → (-dy, dx)
  const nx = (-dy / len) * normalSide
  const ny = (dx  / len) * normalSide
  return { x1, y1, x2, y2, nx, ny }
}

// L-berth: main quay runs along +x axis; finger extends in +y at x=0.
// The berth pocket is in the +y half (water side is +y for main quay).
// Origin = inner corner of the L.
export function lBerth(quayLength: number, fingerLength: number): Segment[] {
  // Main quay: from (0, 0) to (quayLength, 0), outward normal pointing +y (into berth)
  const main = makeSegment(0, 0, quayLength, 0, 1)

  // Finger: from (0, 0) to (0, fingerLength), outward normal pointing +x (into berth)
  const finger = makeSegment(0, 0, 0, fingerLength, -1)

  return [main, finger]
}
