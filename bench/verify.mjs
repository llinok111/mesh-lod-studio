/**
 * verify.mjs — hard assertions on the pure geometry core. No browser involved.
 * Exit code 0 only if every check passes.
 */
import {
  weldMesh, generateSphere, generateTorus, generateTerrain,
  decimate, analyzeTopology, measureDeviation,
  buildTriangleGrid, distanceToGrid, distanceToMeshBrute, sampleSurface,
} from '../src/core.mjs';

let passed = 0, failed = 0;
function check(name, cond, detail = '') {
  if (cond) { passed++; console.log(`  PASS  ${name}${detail ? '  (' + detail + ')' : ''}`); }
  else { failed++; console.log(`  FAIL  ${name}${detail ? '  (' + detail + ')' : ''}`); }
}
function near(actual, expected, tol) { return Math.abs(actual - expected) <= tol; }
function fnv1a(typed) {
  const bytes = new Uint8Array(typed.buffer, typed.byteOffset, typed.byteLength);
  let h = 2166136261 >>> 0;
  for (let i = 0; i < bytes.length; i++) { h ^= bytes[i]; h = Math.imul(h, 16777619); }
  return h >>> 0;
}
function finitePositions(mesh) {
  for (let i = 0; i < mesh.positions.length; i++) if (!Number.isFinite(mesh.positions[i])) return false;
  return true;
}

console.log('\n[1] weld / index repair');
{
  const positions = [
    0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, // quad 1
    1, 0, 0, 1, 1, 0, 2, 0, 0, 2, 1, 0, // quad 2 (two coincident verts)
  ];
  const indices = [
    0, 1, 2, 1, 3, 2,
    4, 5, 6, 5, 7, 6,
    0, 0, 1,       // degenerate
    2, 1, 0,       // duplicate of face (0,1,2)
  ];
  const w = weldMesh(positions, indices);
  check('merged coincident vertices', w.stats.merged === 2, `merged=${w.stats.merged}`);
  check('dropped degenerate triangle', w.stats.degenerate === 1, `degenerate=${w.stats.degenerate}`);
  check('dropped duplicate face', w.stats.duplicate === 1, `duplicate=${w.stats.duplicate}`);
  check('kept 6 unique vertices', w.positions.length / 3 === 6, `verts=${w.positions.length / 3}`);
  check('kept 4 valid triangles', w.indices.length / 3 === 4, `tris=${w.indices.length / 3}`);
}

console.log('\n[2] sphere: QEM 10% LOD, watertight and bounded error');
const sphere = generateSphere(128, 96);
const sphereTopo = analyzeTopology(sphere);
const sphereLod = decimate(sphere, { targetRatio: 0.1 });
const sphereLodTopo = analyzeTopology(sphereLod);
const sphereDev = measureDeviation(sphere, sphereLod, { samples: 3000 });
{
  check('source sphere is a closed manifold', sphereTopo.closed && sphereTopo.euler === 2,
    `euler=${sphereTopo.euler} boundary=${sphereTopo.boundaryEdges}`);
  check('source has 24,320 triangles', sphereTopo.faces === 24320, `faces=${sphereTopo.faces}`);
  check('LOD hits the 10% target', sphereLodTopo.faces === 2432, `faces=${sphereLodTopo.faces}`);
  check('LOD stays a closed manifold', sphereLodTopo.closed, `boundary=${sphereLodTopo.boundaryEdges}`);
  check('LOD keeps Euler characteristic 2', sphereLodTopo.euler === 2, `euler=${sphereLodTopo.euler}`);
  check('LOD has no non-manifold edges', sphereLodTopo.nonManifoldEdges === 0, `nm=${sphereLodTopo.nonManifoldEdges}`);
  check('LOD has no degenerate faces', sphereLodTopo.degenerateFaces === 0, `deg=${sphereLodTopo.degenerateFaces}`);
  check('LOD has no NaN positions', finitePositions(sphereLod));
  check('max surface error < 0.15% of bbox diagonal', sphereDev.maxPct < 0.15, `max=${sphereDev.maxPct.toFixed(4)}%`);
  check('mean surface error < 0.05% of bbox diagonal', sphereDev.meanPct < 0.05, `mean=${sphereDev.meanPct.toFixed(4)}%`);
  const volErr = Math.abs((sphereLodTopo.volume - sphereTopo.volume) / sphereTopo.volume) * 100;
  check('enclosed volume drift < 1%', volErr < 1, `drift=${volErr.toFixed(3)}%`);
  check('decimation under 5 s on this machine', sphereLod.stats.timeMs < 5000, `t=${sphereLod.stats.timeMs.toFixed(0)}ms`);
}

console.log('\n[3] torus: genus-1 topology preserved');
{
  const torus = generateTorus(128, 48);
  const lod = decimate(torus, { targetRatio: 0.1 });
  const topo = analyzeTopology(lod);
  const dev = measureDeviation(torus, lod, { samples: 2000 });
  check('Euler characteristic stays 0 (genus 1)', topo.euler === 0, `euler=${topo.euler}`);
  check('torus LOD is closed', topo.closed, `boundary=${topo.boundaryEdges}`);
  check('torus LOD max error < 0.5%', dev.maxPct < 0.5, `max=${dev.maxPct.toFixed(4)}%`);
}

console.log('\n[4] terrain: open boundary preserved without non-manifold edges');
{
  const terrain = generateTerrain(120);
  const lod = decimate(terrain, { targetRatio: 0.1 });
  const topo = analyzeTopology(lod);
  const dev = measureDeviation(terrain, lod, { samples: 2000 });
  check('terrain LOD keeps an open boundary', topo.boundaryEdges > 0, `boundary=${topo.boundaryEdges}`);
  check('terrain LOD has no non-manifold edges', topo.nonManifoldEdges === 0, `nm=${topo.nonManifoldEdges}`);
  check('terrain LOD max error < 1%', dev.maxPct < 1, `max=${dev.maxPct.toFixed(4)}%`);
  check('terrain LOD has no degenerate faces', topo.degenerateFaces === 0, `deg=${topo.degenerateFaces}`);
}

console.log('\n[5] deviation metric: AABB grid equals brute force');
{
  const grid = buildTriangleGrid(sphereLod, 40);
  const pts = sampleSurface(sphere, 150, 99);
  let maxDiff = 0;
  for (let i = 0; i < 150; i++) {
    const a = distanceToGrid(grid, pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]);
    const b = distanceToMeshBrute(sphereLod, pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]);
    maxDiff = Math.max(maxDiff, Math.abs(a - b));
  }
  check('grid vs brute-force max difference < 1e-9', maxDiff < 1e-9, `diff=${maxDiff.toExponential(2)}`);
}

console.log('\n[6] determinism and degenerate input');
{
  const a = decimate(sphere, { targetRatio: 0.25 });
  const b = decimate(sphere, { targetRatio: 0.25 });
  check('same input -> identical indices', fnv1a(a.indices) === fnv1a(b.indices),
    `hash=${fnv1a(a.indices).toString(16)}`);
  check('same input -> identical positions', fnv1a(a.positions) === fnv1a(b.positions));

  const flat = weldMesh([0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 1, 2, 0, 0, 1]);
  const empty = decimate({ positions: flat.positions, indices: flat.indices }, { targetRatio: 0.1 });
  check('all-degenerate input returns empty mesh without throwing', empty.indices.length === 0,
    `tris=${empty.indices.length / 3}`);
}

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed === 0 ? 0 : 1);
