/**
 * core.mjs — pure geometry engine for mesh-lod-studio.
 *
 * No three.js, no DOM, no Node APIs. Runs unchanged in node and in the browser.
 *
 * Pipeline:
 *   weldMesh()        index repair: merge coincident vertices, drop degenerate/duplicate faces
 *   decimate()        quadric error metric (QEM) edge-collapse simplification
 *   analyzeTopology() Euler characteristic, boundary/non-manifold edges, signed volume
 *   measureDeviation() symmetric surface distance (area-weighted sampling + AABB grid)
 *
 * Every public function takes/returns plain typed arrays so results are hashable and testable.
 */

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

/* ------------------------------------------------------------------ */
/* vector helpers                                                      */
/* ------------------------------------------------------------------ */

export function bboxOf(positions) {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1], z = positions[i + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  return { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] };
}

export function bboxDiagonal(positions) {
  const b = bboxOf(positions);
  const dx = b.max[0] - b.min[0], dy = b.max[1] - b.min[1], dz = b.max[2] - b.min[2];
  return Math.hypot(dx, dy, dz);
}

export function triangleCount(mesh) {
  return mesh.indices.length / 3;
}

/* ------------------------------------------------------------------ */
/* deterministic RNG                                                   */
/* ------------------------------------------------------------------ */

export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/* ------------------------------------------------------------------ */
/* weld / index repair                                                 */
/* ------------------------------------------------------------------ */

/**
 * Merge vertices closer than epsilon, drop degenerate triangles and duplicate faces.
 * epsilon defaults to 1e-6 * bbox diagonal (relative, so it works at any scale).
 */
export function weldMesh(positions, indices, epsilon) {
  const n = positions.length / 3;
  const diag = bboxDiagonal(positions) || 1;
  const eps = epsilon ?? diag * 1e-6;
  const inv = 1 / eps;
  const map = new Map();
  const remap = new Int32Array(n);
  const outPos = [];
  let merged = 0;

  for (let i = 0; i < n; i++) {
    const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
    const key = `${Math.round(x * inv)},${Math.round(y * inv)},${Math.round(z * inv)}`;
    const hit = map.get(key);
    if (hit === undefined) {
      const id = outPos.length / 3;
      map.set(key, id);
      outPos.push(x, y, z);
      remap[i] = id;
    } else {
      remap[i] = hit;
      merged++;
    }
  }

  const seen = new Set();
  const outIdx = [];
  let degenerate = 0, duplicate = 0;
  const tri = new Uint32Array(3);
  for (let f = 0; f < indices.length; f += 3) {
    const a = remap[indices[f]], b = remap[indices[f + 1]], c = remap[indices[f + 2]];
    if (a === b || b === c || a === c) { degenerate++; continue; }
    tri[0] = a; tri[1] = b; tri[2] = c;
    tri.sort();
    const key = `${tri[0]},${tri[1]},${tri[2]}`;
    if (seen.has(key)) { duplicate++; continue; }
    seen.add(key);
    outIdx.push(a, b, c);
  }

  return {
    positions: Float64Array.from(outPos),
    indices: Uint32Array.from(outIdx),
    stats: { inputVertices: n, vertices: outPos.length / 3, merged, degenerate, duplicate },
  };
}

/* ------------------------------------------------------------------ */
/* topology analysis                                                   */
/* ------------------------------------------------------------------ */

export function analyzeTopology(mesh) {
  const pos = mesh.positions, idx = mesh.indices;
  const V = pos.length / 3, F = idx.length / 3;
  const edgeMap = new Map();
  let degenerate = 0;

  for (let f = 0; f < F; f++) {
    const a = idx[f * 3], b = idx[f * 3 + 1], c = idx[f * 3 + 2];
    if (a === b || b === c || a === c) { degenerate++; continue; }
    const edges = [[a, b], [b, c], [c, a]];
    for (const [i, j] of edges) {
      const lo = i < j ? i : j, hi = i < j ? j : i;
      const key = lo * V + hi;
      const rec = edgeMap.get(key);
      if (rec) rec.count++; else edgeMap.set(key, { count: 1, a: lo, b: hi });
    }
  }

  let boundaryEdges = 0, nonManifoldEdges = 0;
  for (const rec of edgeMap.values()) {
    if (rec.count === 1) boundaryEdges++;
    else if (rec.count > 2) nonManifoldEdges++;
  }

  // connected components over face-adjacent vertices
  const parent = new Int32Array(V);
  for (let i = 0; i < V; i++) parent[i] = i;
  const find = (x) => { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
  const union = (a, b) => { const ra = find(a), rb = find(b); if (ra !== rb) parent[rb] = ra; };
  for (let f = 0; f < F; f++) {
    const a = idx[f * 3], b = idx[f * 3 + 1], c = idx[f * 3 + 2];
    union(a, b); union(b, c);
  }
  const used = new Set();
  for (let f = 0; f < F; f++) { used.add(find(idx[f * 3])); }
  const components = used.size;

  // signed volume (positive = outward winding for a closed mesh)
  let volume = 0;
  for (let f = 0; f < F; f++) {
    const a = idx[f * 3] * 3, b = idx[f * 3 + 1] * 3, c = idx[f * 3 + 2] * 3;
    const ax = pos[a], ay = pos[a + 1], az = pos[a + 2];
    const bx = pos[b], by = pos[b + 1], bz = pos[b + 2];
    const cx = pos[c], cy = pos[c + 1], cz = pos[c + 2];
    volume += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }

  return {
    vertices: V,
    faces: F,
    edges: edgeMap.size,
    boundaryEdges,
    nonManifoldEdges,
    degenerateFaces: degenerate,
    euler: V - edgeMap.size + F,
    closed: boundaryEdges === 0 && nonManifoldEdges === 0,
    components,
    volume,
  };
}

/* ------------------------------------------------------------------ */
/* quadric error metric                                                */
/* ------------------------------------------------------------------ */

function quadricPlane(nx, ny, nz, d, w) {
  return Float64Array.of(
    w * nx * nx, w * nx * ny, w * nx * nz, w * nx * d,
    w * ny * ny, w * ny * nz, w * ny * d,
    w * nz * nz, w * nz * d,
    w * d * d,
  );
}

function quadricAdd(target, add) {
  for (let i = 0; i < 10; i++) target[i] += add[i];
}

function quadricEval(q, x, y, z) {
  return (
    q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x +
    q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y +
    q[7] * z * z + 2 * q[8] * z +
    q[9]
  );
}

function optimalPoint(q, ax, ay, az, bx, by, bz) {
  const a2 = q[0], ab = q[1], ac = q[2], ad = q[3];
  const b2 = q[4], bc = q[5], bd = q[6];
  const c2 = q[7], cd = q[8], d2 = q[9];
  const det = a2 * (b2 * c2 - bc * bc) - ab * (ab * c2 - bc * ac) + ac * (ab * bc - b2 * ac);

  let best = [ax, ay, az];
  let bestCost = quadricEval(q, ax, ay, az);

  const consider = (x, y, z) => {
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;
    const cost = quadricEval(q, x, y, z);
    if (cost < bestCost) { bestCost = cost; best = [x, y, z]; }
  };

  consider((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5);
  consider(bx, by, bz);

  if (Math.abs(det) > 1e-14) {
    const inv = 1 / det;
    const x = -(ad * (b2 * c2 - bc * bc) - ab * (bd * c2 - bc * cd) + ac * (bd * bc - b2 * cd)) * inv;
    const y = -(a2 * (bd * c2 - bc * cd) - ad * (ab * c2 - bc * ac) + ac * (ab * cd - bd * ac)) * inv;
    const z = -(a2 * (b2 * cd - bd * bc) - ab * (ab * cd - bd * ac) + ad * (ab * bc - b2 * ac)) * inv;
    consider(x, y, z);
  }
  return { point: best, cost: bestCost };
}

class MinHeap {
  constructor(cmp) { this.a = []; this.cmp = cmp; }
  get size() { return this.a.length; }
  push(v) {
    const a = this.a; a.push(v); let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.cmp(a[i], a[p]) >= 0) break;
      [a[i], a[p]] = [a[p], a[i]]; i = p;
    }
  }
  pop() {
    const a = this.a;
    if (!a.length) return null;
    const top = a[0], last = a.pop();
    if (a.length) {
      a[0] = last; let i = 0;
      for (;;) {
        const l = i * 2 + 1, r = l + 1;
        let m = i;
        if (l < a.length && this.cmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && this.cmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        [a[i], a[m]] = [a[m], a[i]]; i = m;
      }
    }
    return top;
  }
}

/* ------------------------------------------------------------------ */
/* decimation                                                          */
/* ------------------------------------------------------------------ */

/**
 * QEM edge-collapse decimation.
 * @param {{positions:ArrayLike<number>, indices:ArrayLike<number>}} input
 * @param {{targetRatio?:number, targetTriangles?:number, boundaryWeight?:number}} opts
 */
export function decimate(input, opts = {}) {
  const t0 = now();
  const weld = weldMesh(input.positions, input.indices);
  const pos = weld.positions, idx = weld.indices;
  const inputTriangles = idx.length / 3;
  const targetTriangles = Math.max(
    4,
    Math.min(
      inputTriangles,
      opts.targetTriangles ?? Math.round(inputTriangles * (opts.targetRatio ?? 0.25)),
    ),
  );
  const diag = bboxDiagonal(pos) || 1;
  const areaEps = diag * diag * 1e-12;
  const boundaryWeight = opts.boundaryWeight ?? 100;

  const verts = [];
  for (let i = 0; i < pos.length; i += 3) {
    verts.push({
      x: pos[i], y: pos[i + 1], z: pos[i + 2],
      q: new Float64Array(10),
      faces: new Set(),
      alive: true,
      ver: 0,
    });
  }
  const faces = [];
  for (let i = 0; i < idx.length; i += 3) {
    faces.push({ v: [idx[i], idx[i + 1], idx[i + 2]], alive: true });
    verts[idx[i]].faces.add(faces.length - 1);
    verts[idx[i + 1]].faces.add(faces.length - 1);
    verts[idx[i + 2]].faces.add(faces.length - 1);
  }

  const faceNormal = (f) => {
    const [a, b, c] = f.v;
    const A = verts[a], B = verts[b], C = verts[c];
    const ux = B.x - A.x, uy = B.y - A.y, uz = B.z - A.z;
    const vx = C.x - A.x, vy = C.y - A.y, vz = C.z - A.z;
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const len = Math.hypot(nx, ny, nz);
    return { nx, ny, nz, len, area: len * 0.5 };
  };

  // face quadrics (area weighted)
  for (const f of faces) {
    const n = faceNormal(f);
    if (n.area <= areaEps || n.len === 0) continue;
    const [a, b, c] = f.v;
    const A = verts[a];
    const d = -(n.nx * A.x + n.ny * A.y + n.nz * A.z) / n.len;
    const q = quadricPlane(n.nx / n.len, n.ny / n.len, n.nz / n.len, d, n.area);
    quadricAdd(verts[a].q, q);
    quadricAdd(verts[b].q, q);
    quadricAdd(verts[c].q, q);
  }

  // edge -> incident faces
  const edgeMap = new Map();
  const edgeKey = (a, b) => (a < b ? a * verts.length + b : b * verts.length + a);
  for (let fi = 0; fi < faces.length; fi++) {
    const [a, b, c] = faces[fi].v;
    for (const [i, j] of [[a, b], [b, c], [c, a]]) {
      const k = edgeKey(i, j);
      let rec = edgeMap.get(k);
      if (!rec) { rec = { a: i < j ? i : j, b: i < j ? j : i, faces: [] }; edgeMap.set(k, rec); }
      rec.faces.push(fi);
    }
  }

  // boundary constraint quadrics: plane through edge, perpendicular to its face
  for (const rec of edgeMap.values()) {
    if (rec.faces.length !== 1) continue;
    const f = faces[rec.faces[0]];
    const n = faceNormal(f);
    if (n.len === 0) continue;
    const A = verts[rec.a], B = verts[rec.b];
    const ex = B.x - A.x, ey = B.y - A.y, ez = B.z - A.z;
    let cx = ey * n.nz - ez * n.ny;
    let cy = ez * n.nx - ex * n.nz;
    let cz = ex * n.ny - ey * n.nx;
    const cl = Math.hypot(cx, cy, cz);
    if (cl === 0) continue;
    cx /= cl; cy /= cl; cz /= cl;
    const d = -(cx * A.x + cy * A.y + cz * A.z);
    const w = boundaryWeight * (ex * ex + ey * ey + ez * ez);
    const q = quadricPlane(cx, cy, cz, d, w);
    quadricAdd(A.q, q); quadricAdd(B.q, q);
  }

  const heap = new MinHeap((p, q) => p.cost - q.cost);
  const pushEdge = (a, b) => {
    const A = verts[a], B = verts[b];
    if (!A.alive || !B.alive) return;
    const q = new Float64Array(10);
    for (let i = 0; i < 10; i++) q[i] = A.q[i] + B.q[i];
    const { point, cost } = optimalPoint(q, A.x, A.y, A.z, B.x, B.y, B.z);
    heap.push({ a, b, cost, point, va: A.ver, vb: B.ver });
  };

  for (const rec of edgeMap.values()) pushEdge(rec.a, rec.b);

  let triangles = inputTriangles;
  let collapses = 0, rejected = 0, stale = 0;
  const maxIterations = Math.max(1000, edgeMap.size * 8);

  const neighbors = (vi) => {
    const s = new Set();
    for (const fi of verts[vi].faces) {
      const f = faces[fi];
      if (!f.alive) continue;
      for (const v of f.v) if (v !== vi) s.add(v);
    }
    return s;
  };

  const tryCollapse = (a, b, shared, pt) => {
    const A = verts[a], B = verts[b];
    const na = neighbors(a), nb = neighbors(b);
    const opp = new Set();
    for (const fi of shared) {
      for (const v of faces[fi].v) if (v !== a && v !== b) opp.add(v);
    }
    let inter = 0;
    for (const v of na) if (nb.has(v)) inter++;
    if (inter !== opp.size) return false;
    for (const v of opp) if (!na.has(v) || !nb.has(v)) return false;

    const triKey = (x, y, z) => {
      const t = [x, y, z].sort((p, q) => p - q);
      return `${t[0]},${t[1]},${t[2]}`;
    };
    const occupied = new Set();
    for (const fi of A.faces) {
      if (!faces[fi].alive) continue;
      const f = faces[fi];
      if (f.v.includes(b)) continue;
      occupied.add(triKey(f.v[0], f.v[1], f.v[2]));
    }
    for (const fi of B.faces) {
      if (!faces[fi].alive) continue;
      const f = faces[fi];
      if (f.v.includes(a)) continue;
      const t = f.v.map((v) => (v === b ? a : v));
      if (new Set(t).size !== 3) return false;
      if (occupied.has(triKey(t[0], t[1], t[2]))) return false;
      occupied.add(triKey(t[0], t[1], t[2]));
    }

    // reject collapses that flip or flatten any incident face
    for (const vi of [a, b]) {
      for (const fi of verts[vi].faces) {
        const f = faces[fi];
        if (!f.alive) continue;
        if (f.v.includes(a) && f.v.includes(b)) continue;
        const old = faceNormal(f);
        if (old.len === 0) continue;
        const p = f.v.map((v) => (v === b ? a : v));
        const [i0, i1, i2] = p;
        const P0 = i0 === a ? pt : verts[i0];
        const P1 = i1 === a ? pt : verts[i1];
        const P2 = i2 === a ? pt : verts[i2];
        const ux = P1.x - P0.x, uy = P1.y - P0.y, uz = P1.z - P0.z;
        const vx = P2.x - P0.x, vy = P2.y - P0.y, vz = P2.z - P0.z;
        const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const len = Math.hypot(nx, ny, nz);
        if (len * 0.5 <= areaEps) return false;
        const dot = (nx * old.nx + ny * old.ny + nz * old.nz) / (len * old.len);
        if (dot < 0.05) return false;
      }
    }

    // commit
    for (const fi of shared) {
      faces[fi].alive = false;
      for (const v of faces[fi].v) verts[v].faces.delete(fi);
    }
    A.x = pt[0]; A.y = pt[1]; A.z = pt[2];
    for (let i = 0; i < 10; i++) A.q[i] += B.q[i];
    A.ver++;
    for (const fi of [...B.faces]) {
      const f = faces[fi];
      if (!f.alive) continue;
      for (let k = 0; k < 3; k++) if (f.v[k] === b) f.v[k] = a;
      A.faces.add(fi);
    }
    B.faces.clear();
    B.alive = false;
    B.ver++;

    const touched = new Set();
    for (const fi of A.faces) {
      if (!faces[fi].alive) continue;
      for (const v of faces[fi].v) if (v !== a) touched.add(v);
    }
    for (const v of touched) pushEdge(a, v);
    return true;
  };

  let iterations = 0;
  while (triangles > targetTriangles && heap.size && iterations < maxIterations) {
    iterations++;
    const e = heap.pop();
    const A = verts[e.a], B = verts[e.b];
    if (!A.alive || !B.alive || A.ver !== e.va || B.ver !== e.vb) { stale++; continue; }

    const shared = [];
    for (const fi of A.faces) {
      const f = faces[fi];
      if (f.alive && f.v.includes(e.b)) shared.push(fi);
    }
    if (shared.length === 0 || shared.length > 2) { rejected++; continue; }
    if (!tryCollapse(e.a, e.b, shared, e.point)) { rejected++; continue; }
    triangles -= shared.length;
    collapses++;
  }

  // compact
  const outMap = new Int32Array(verts.length).fill(-1);
  const outPos = [];
  for (let i = 0; i < verts.length; i++) {
    if (!verts[i].alive) continue;
    outMap[i] = outPos.length / 3;
    outPos.push(verts[i].x, verts[i].y, verts[i].z);
  }
  const outIdx = [];
  for (const f of faces) {
    if (!f.alive) continue;
    outIdx.push(outMap[f.v[0]], outMap[f.v[1]], outMap[f.v[2]]);
  }

  return {
    positions: Float64Array.from(outPos),
    indices: Uint32Array.from(outIdx),
    stats: {
      inputTriangles,
      outputTriangles: outIdx.length / 3,
      targetTriangles,
      collapses,
      rejected,
      stale,
      iterations,
      timeMs: now() - t0,
      weld: weld.stats,
    },
  };
}

/* ------------------------------------------------------------------ */
/* distance queries                                                    */
/* ------------------------------------------------------------------ */

/** Squared distance from point to triangle (Ericson, Real-Time Collision Detection). */
function pointTriangleDist2(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz) {
  const abx = bx - ax, aby = by - ay, abz = bz - az;
  const acx = cx - ax, acy = cy - ay, acz = cz - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz;
  const d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0 && d2 <= 0) return apx * apx + apy * apy + apz * apz;

  const bpx = px - bx, bpy = py - by, bpz = pz - bz;
  const d3 = abx * bpx + aby * bpy + abz * bpz;
  const d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) return bpx * bpx + bpy * bpy + bpz * bpz;

  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    const qx = ax + v * abx, qy = ay + v * aby, qz = az + v * abz;
    return (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2;
  }

  const cpx = px - cx, cpy = py - cy, cpz = pz - cz;
  const d5 = abx * cpx + aby * cpy + abz * cpz;
  const d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) return cpx * cpx + cpy * cpy + cpz * cpz;

  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    const qx = ax + w * acx, qy = ay + w * acy, qz = az + w * acz;
    return (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2;
  }

  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    const qx = bx + w * (cx - bx), qy = by + w * (cy - by), qz = bz + w * (cz - bz);
    return (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2;
  }

  const denom = 1 / (va + vb + vc);
  const v = vb * denom, w = vc * denom;
  const qx = ax + abx * v + acx * w;
  const qy = ay + aby * v + acy * w;
  const qz = az + abz * v + acz * w;
  return (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2;
}

/** Uniform AABB grid over triangles, for repeated nearest-surface queries. */
export function buildTriangleGrid(mesh, cellsPerAxis = 40) {
  const pos = mesh.positions, idx = mesh.indices;
  const b = bboxOf(pos);
  const extent = Math.max(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2], 1e-9);
  const cellSize = extent / cellsPerAxis;
  const nx = Math.max(1, Math.ceil((b.max[0] - b.min[0]) / cellSize));
  const ny = Math.max(1, Math.ceil((b.max[1] - b.min[1]) / cellSize));
  const nz = Math.max(1, Math.ceil((b.max[2] - b.min[2]) / cellSize));
  const cells = new Array(nx * ny * nz);
  const triCount = idx.length / 3;

  for (let f = 0; f < triCount; f++) {
    let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
    for (let k = 0; k < 3; k++) {
      const v = idx[f * 3 + k] * 3;
      const x = pos[v], y = pos[v + 1], z = pos[v + 2];
      if (x < x0) x0 = x; if (x > x1) x1 = x;
      if (y < y0) y0 = y; if (y > y1) y1 = y;
      if (z < z0) z0 = z; if (z > z1) z1 = z;
    }
    const ix0 = Math.max(0, Math.min(nx - 1, Math.floor((x0 - b.min[0]) / cellSize)));
    const ix1 = Math.max(0, Math.min(nx - 1, Math.floor((x1 - b.min[0]) / cellSize)));
    const iy0 = Math.max(0, Math.min(ny - 1, Math.floor((y0 - b.min[1]) / cellSize)));
    const iy1 = Math.max(0, Math.min(ny - 1, Math.floor((y1 - b.min[1]) / cellSize)));
    const iz0 = Math.max(0, Math.min(nz - 1, Math.floor((z0 - b.min[2]) / cellSize)));
    const iz1 = Math.max(0, Math.min(nz - 1, Math.floor((z1 - b.min[2]) / cellSize)));
    for (let iz = iz0; iz <= iz1; iz++) {
      for (let iy = iy0; iy <= iy1; iy++) {
        for (let ix = ix0; ix <= ix1; ix++) {
          const c = (iz * ny + iy) * nx + ix;
          if (!cells[c]) cells[c] = [];
          cells[c].push(f);
        }
      }
    }
  }

  return { mesh, cells, nx, ny, nz, cellSize, min: b.min, max: b.max };
}

export function distanceToGrid(grid, px, py, pz) {
  const { mesh, cells, nx, ny, nz, cellSize, min } = grid;
  const pos = mesh.positions, idx = mesh.indices;
  const cx = Math.max(0, Math.min(nx - 1, Math.floor((px - min[0]) / cellSize)));
  const cy = Math.max(0, Math.min(ny - 1, Math.floor((py - min[1]) / cellSize)));
  const cz = Math.max(0, Math.min(nz - 1, Math.floor((pz - min[2]) / cellSize)));

  let best2 = Infinity;
  const maxRing = Math.max(nx, ny, nz);
  for (let r = 0; r <= maxRing; r++) {
    if (best2 < Infinity && best2 <= ((r - 1) * cellSize) ** 2) break;
    const zLo = Math.max(0, cz - r), zHi = Math.min(nz - 1, cz + r);
    const yLo = Math.max(0, cy - r), yHi = Math.min(ny - 1, cy + r);
    const xLo = Math.max(0, cx - r), xHi = Math.min(nx - 1, cx + r);
    for (let iz = zLo; iz <= zHi; iz++) {
      const onZ = iz === cz - r || iz === cz + r;
      for (let iy = yLo; iy <= yHi; iy++) {
        const onY = iy === cy - r || iy === cy + r;
        for (let ix = xLo; ix <= xHi; ix++) {
          if (!onZ && !onY && ix !== cx - r && ix !== cx + r) continue;
          const list = cells[(iz * ny + iy) * nx + ix];
          if (!list) continue;
          for (const f of list) {
            const a = idx[f * 3] * 3, b = idx[f * 3 + 1] * 3, c = idx[f * 3 + 2] * 3;
            const d2 = pointTriangleDist2(
              px, py, pz,
              pos[a], pos[a + 1], pos[a + 2],
              pos[b], pos[b + 1], pos[b + 2],
              pos[c], pos[c + 1], pos[c + 2],
            );
            if (d2 < best2) best2 = d2;
          }
        }
      }
    }
  }

  if (!Number.isFinite(best2)) {
    for (let f = 0; f < idx.length / 3; f++) {
      const a = idx[f * 3] * 3, b = idx[f * 3 + 1] * 3, c = idx[f * 3 + 2] * 3;
      const d2 = pointTriangleDist2(
        px, py, pz,
        pos[a], pos[a + 1], pos[a + 2],
        pos[b], pos[b + 1], pos[b + 2],
        pos[c], pos[c + 1], pos[c + 2],
      );
      if (d2 < best2) best2 = d2;
    }
  }
  return Math.sqrt(best2);
}

/** Area-weighted deterministic surface sampling. */
export function sampleSurface(mesh, count, seed = 12345) {
  const pos = mesh.positions, idx = mesh.indices;
  const F = idx.length / 3;
  const cdf = new Float64Array(F);
  let total = 0;
  for (let f = 0; f < F; f++) {
    const a = idx[f * 3] * 3, b = idx[f * 3 + 1] * 3, c = idx[f * 3 + 2] * 3;
    const ux = pos[b] - pos[a], uy = pos[b + 1] - pos[a + 1], uz = pos[b + 2] - pos[a + 2];
    const vx = pos[c] - pos[a], vy = pos[c + 1] - pos[a + 1], vz = pos[c + 2] - pos[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    total += Math.hypot(nx, ny, nz) * 0.5;
    cdf[f] = total;
  }
  const rnd = mulberry32(seed);
  const out = new Float64Array(count * 3);
  for (let s = 0; s < count; s++) {
    const target = rnd() * total;
    let lo = 0, hi = F - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (cdf[mid] < target) lo = mid + 1; else hi = mid;
    }
    const f = lo;
    const a = idx[f * 3] * 3, b = idx[f * 3 + 1] * 3, c = idx[f * 3 + 2] * 3;
    let u = rnd(), v = rnd();
    if (u + v > 1) { u = 1 - u; v = 1 - v; }
    out[s * 3] = pos[a] + u * (pos[b] - pos[a]) + v * (pos[c] - pos[a]);
    out[s * 3 + 1] = pos[a + 1] + u * (pos[b + 1] - pos[a + 1]) + v * (pos[c + 1] - pos[a + 1]);
    out[s * 3 + 2] = pos[a + 2] + u * (pos[b + 2] - pos[a + 2]) + v * (pos[c + 2] - pos[a + 2]);
  }
  return out;
}

/**
 * Symmetric surface deviation: original -> simplified and simplified -> original.
 * Returns absolute values and percentages of the original bbox diagonal.
 */
export function measureDeviation(original, simplified, opts = {}) {
  const t0 = now();
  const samples = opts.samples ?? 4000;
  const diag = bboxDiagonal(original.positions) || 1;
  const gridA = buildTriangleGrid(simplified, opts.cells ?? 40);
  const gridB = buildTriangleGrid(original, opts.cells ?? 40);
  const ptsA = sampleSurface(original, samples, opts.seed ?? 12345);
  const ptsB = sampleSurface(simplified, samples, opts.seed ?? 54321);

  let maxA = 0, sumA = 0, maxB = 0, sumB = 0;
  for (let i = 0; i < samples; i++) {
    const dA = distanceToGrid(gridA, ptsA[i * 3], ptsA[i * 3 + 1], ptsA[i * 3 + 2]);
    if (dA > maxA) maxA = dA; sumA += dA;
    const dB = distanceToGrid(gridB, ptsB[i * 3], ptsB[i * 3 + 1], ptsB[i * 3 + 2]);
    if (dB > maxB) maxB = dB; sumB += dB;
  }
  const max = Math.max(maxA, maxB);
  const mean = (sumA + sumB) / (samples * 2);

  return {
    samples,
    max,
    mean,
    maxPct: (max / diag) * 100,
    meanPct: (mean / diag) * 100,
    diag,
    timeMs: now() - t0,
  };
}

/* ------------------------------------------------------------------ */
/* procedural sample geometry                                          */
/* ------------------------------------------------------------------ */

export function generateSphere(segments = 128, rings = 96, radius = 1) {
  const positions = [], indices = [];
  for (let iy = 0; iy <= rings; iy++) {
    const theta = (iy / rings) * Math.PI;
    const y = Math.cos(theta), r = Math.sin(theta);
    for (let ix = 0; ix < segments; ix++) {
      const phi = (ix / segments) * Math.PI * 2;
      positions.push(r * Math.cos(phi) * radius, y * radius, r * Math.sin(phi) * radius);
    }
  }
  const row = segments;
  for (let iy = 0; iy < rings; iy++) {
    for (let ix = 0; ix < segments; ix++) {
      const a = iy * row + ix;
      const b = iy * row + ((ix + 1) % segments);
      const c = (iy + 1) * row + ix;
      const d = (iy + 1) * row + ((ix + 1) % segments);
      indices.push(a, b, c, b, d, c);
    }
  }
  return weldMesh(positions, indices);
}

export function generateTorus(radial = 128, tubular = 48, R = 1, r = 0.38) {
  const positions = [], indices = [];
  for (let i = 0; i < radial; i++) {
    const u = (i / radial) * Math.PI * 2;
    for (let j = 0; j < tubular; j++) {
      const v = (j / tubular) * Math.PI * 2;
      positions.push((R + r * Math.cos(v)) * Math.cos(u), r * Math.sin(v), (R + r * Math.cos(v)) * Math.sin(u));
    }
  }
  for (let i = 0; i < radial; i++) {
    for (let j = 0; j < tubular; j++) {
      const a = i * tubular + j;
      const b = ((i + 1) % radial) * tubular + j;
      const c = i * tubular + ((j + 1) % tubular);
      const d = ((i + 1) % radial) * tubular + ((j + 1) % tubular);
      indices.push(a, c, b, b, c, d);
    }
  }
  return weldMesh(positions, indices);
}

function hash2(x, y) {
  let h = Math.imul(x | 0, 374761393) ^ Math.imul(y | 0, 668265263);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

function smoothNoise(x, y) {
  const ix = Math.floor(x), iy = Math.floor(y);
  const fx = x - ix, fy = y - iy;
  const sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
  const n00 = hash2(ix, iy), n10 = hash2(ix + 1, iy);
  const n01 = hash2(ix, iy + 1), n11 = hash2(ix + 1, iy + 1);
  return (n00 * (1 - sx) + n10 * sx) * (1 - sy) + (n01 * (1 - sx) + n11 * sx) * sy;
}

export function generateTerrain(n = 120, size = 2, height = 0.42) {
  const positions = [], indices = [];
  for (let iz = 0; iz <= n; iz++) {
    for (let ix = 0; ix <= n; ix++) {
      const x = (ix / n - 0.5) * size;
      const z = (iz / n - 0.5) * size;
      let h = 0, amp = 1, freq = 2.2;
      for (let o = 0; o < 4; o++) {
        h += amp * (smoothNoise(x * freq + 13.7, z * freq + 7.1) * 2 - 1);
        amp *= 0.5; freq *= 2.05;
      }
      positions.push(x, h * height, z);
    }
  }
  for (let iz = 0; iz < n; iz++) {
    for (let ix = 0; ix < n; ix++) {
      const a = iz * (n + 1) + ix;
      const b = a + 1;
      const c = a + (n + 1);
      const d = c + 1;
      indices.push(a, c, b, b, c, d);
    }
  }
  return weldMesh(positions, indices);
}

/* ------------------------------------------------------------------ */
/* per-vertex deviation (for the browser heatmap)                      */
/* ------------------------------------------------------------------ */

export function vertexDistances(mesh, reference, cells = 40) {
  const grid = buildTriangleGrid(reference, cells);
  const out = new Float32Array(mesh.positions.length / 3);
  for (let i = 0; i < out.length; i++) {
    out[i] = distanceToGrid(grid, mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]);
  }
  return out;
}

/** Brute-force nearest-surface distance (used by tests to validate the grid). */
export function distanceToMeshBrute(mesh, px, py, pz) {
  const pos = mesh.positions, idx = mesh.indices;
  let best2 = Infinity;
  for (let f = 0; f < idx.length; f += 3) {
    const a = idx[f] * 3, b = idx[f + 1] * 3, c = idx[f + 2] * 3;
    const d2 = pointTriangleDist2(
      px, py, pz,
      pos[a], pos[a + 1], pos[a + 2],
      pos[b], pos[b + 1], pos[b + 2],
      pos[c], pos[c + 1], pos[c + 2],
    );
    if (d2 < best2) best2 = d2;
  }
  return Math.sqrt(best2);
}
