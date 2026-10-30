/**
 * bench.mjs — real measurements, median of 5 runs, written to bench/result.json.
 * Run: node bench/bench.mjs
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import os from 'node:os';
import {
  generateSphere, generateTorus, generateTerrain,
  decimate, analyzeTopology, measureDeviation,
} from '../src/core.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const REPEATS = 5;
const RATIO = 0.1;

const cases = [
  { name: 'sphere', mesh: generateSphere(128, 96) },
  { name: 'torus', mesh: generateTorus(128, 48) },
  { name: 'terrain', mesh: generateTerrain(120) },
];

const results = [];
for (const { name, mesh } of cases) {
  const before = analyzeTopology(mesh);
  const times = [];
  let last = null;
  for (let i = 0; i < REPEATS; i++) {
    const t0 = performance.now();
    last = decimate(mesh, { targetRatio: RATIO });
    times.push(performance.now() - t0);
  }
  const sorted = [...times].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)];
  const after = analyzeTopology(last);
  const dev = measureDeviation(mesh, last, { samples: 3000 });

  results.push({
    case: name,
    inputTriangles: before.faces,
    outputTriangles: after.faces,
    reductionPct: +((1 - after.faces / before.faces) * 100).toFixed(2),
    medianMs: +median.toFixed(1),
    minMs: +sorted[0].toFixed(1),
    maxMs: +sorted[sorted.length - 1].toFixed(1),
    maxDeviationPctDiag: +dev.maxPct.toFixed(4),
    meanDeviationPctDiag: +dev.meanPct.toFixed(4),
    closed: after.closed,
    euler: after.euler,
    boundaryEdges: after.boundaryEdges,
    nonManifoldEdges: after.nonManifoldEdges,
    degenerateFaces: after.degenerateFaces,
    volumeErrorPct: +(((after.volume - before.volume) / Math.abs(before.volume)) * 100).toFixed(3),
    targetTriangles: last.stats.targetTriangles,
    rejectedCollapses: last.stats.rejected,
    samples: dev.samples,
  });
  console.log(
    `${name.padEnd(8)} ${before.faces} -> ${after.faces} tris  ` +
    `median ${median.toFixed(0)} ms  max ${dev.maxPct.toFixed(3)}%  mean ${dev.meanPct.toFixed(3)}%  ` +
    `closed=${after.closed} euler=${after.euler}`,
  );
}

const out = {
  generatedAt: new Date().toISOString(),
  node: process.version,
  platform: `${os.platform()} ${os.arch()}`,
  cpu: os.cpus()[0]?.model ?? 'unknown',
  repeats: REPEATS,
  targetRatio: RATIO,
  results,
};
mkdirSync(here, { recursive: true });
writeFileSync(join(here, 'result.json'), JSON.stringify(out, null, 2) + '\n');
console.log(`\nwrote ${join(here, 'result.json')}`);
