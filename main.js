/**
 * main.js — rendering + interaction only.
 * All geometry decisions (decimation, topology, deviation) live in src/core.mjs,
 * the same module that bench/verify.mjs exercises in node.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  generateSphere, generateTorus, generateTerrain,
  decimate, analyzeTopology, measureDeviation, vertexDistances, bboxOf,
} from './src/core.mjs';

const $ = (id) => document.getElementById(id);
const fmt = (n) => n.toLocaleString('en-US');
const state = {
  kind: 'sphere',
  ratio: 0.1,
  heatmap: true,
  wireframe: false,
  autoRotate: false,
  busy: false,
  pending: false,
};

/* ------------------------------------------------------------------ */
/* scene                                                               */
/* ------------------------------------------------------------------ */

let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
} catch (err) {
  $('fatal').classList.add('on');
  throw err;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.08;
$('fatal').before(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x080b11);
scene.fog = new THREE.Fog(0x080b11, 9, 22);

const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);
camera.position.set(0, 1.4, 6.4);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.rotateSpeed = 0.8;
controls.zoomSpeed = 0.9;
controls.minDistance = 2;
controls.maxDistance = 24;
controls.autoRotateSpeed = 0.9;

scene.add(new THREE.HemisphereLight(0xbfe9ff, 0x0a0f17, 1.25));
const key = new THREE.DirectionalLight(0xffffff, 2.3);
key.position.set(4, 6.5, 5);
scene.add(key);
const rim = new THREE.DirectionalLight(0x22d3ee, 1.35);
rim.position.set(-5.5, 2.5, -4.5);
scene.add(rim);
scene.add(new THREE.AmbientLight(0x2b3a50, 0.9));

const grid = new THREE.GridHelper(16, 32, 0x1d3048, 0x111c29);
grid.position.y = -1.36;
grid.material.transparent = true;
grid.material.opacity = 0.55;
scene.add(grid);

const root = new THREE.Group();
scene.add(root);
const sourceGroup = new THREE.Group();
const lodGroup = new THREE.Group();
root.add(sourceGroup, lodGroup);

const sourceMaterial = new THREE.MeshStandardMaterial({
  color: 0x536178, roughness: 0.9, metalness: 0.05,
  flatShading: true, transparent: false, opacity: 1,
});
const lodMaterial = new THREE.MeshStandardMaterial({
  color: 0x2dd4bf, roughness: 0.34, metalness: 0.16,
  flatShading: true, emissive: 0x073b36, emissiveIntensity: 0.55,
  vertexColors: false,
});

let sourceMeshObj = null;
let lodMeshObj = null;
let lodGeometry = null;
let lodCore = null;
let sourceCore = null;
let sourceTopo = null;
let lodTopo = null;
let lastDeviation = null;
let lodColors = null;
let lodColorMax = 0;
let lastFitKind = null;

const cache = new Map();
function getSource(kind) {
  if (!cache.has(kind)) {
    const t0 = performance.now();
    const mesh = kind === 'sphere' ? generateSphere(128, 96)
      : kind === 'torus' ? generateTorus(128, 48)
        : generateTerrain(120);
    mesh.buildMs = performance.now() - t0;
    cache.set(kind, mesh);
  }
  return cache.get(kind);
}

function toGeometry(mesh) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(Float32Array.from(mesh.positions), 3));
  g.setIndex(new THREE.BufferAttribute(Uint32Array.from(mesh.indices), 1));
  g.computeVertexNormals();
  g.computeBoundingSphere();
  return g;
}

function setStatus(on, text) {
  $('status').classList.toggle('on', on);
  if (text) $('statusText').textContent = text;
}

/* ------------------------------------------------------------------ */
/* heatmap                                                             */
/* ------------------------------------------------------------------ */

const RAMP = [
  [0.0, new THREE.Color(0x0ea5e9)],
  [0.38, new THREE.Color(0x22d3ee)],
  [0.7, new THREE.Color(0xfacc15)],
  [1.0, new THREE.Color(0xef4444)],
];
const tmpA = new THREE.Color(), tmpB = new THREE.Color();

function ramp(t) {
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < RAMP.length; i++) {
    if (t <= RAMP[i][0]) {
      const [t0, c0] = RAMP[i - 1], [t1, c1] = RAMP[i];
      tmpA.copy(c0); tmpB.copy(c1);
      return tmpA.lerp(tmpB, (t - t0) / (t1 - t0));
    }
  }
  return RAMP[RAMP.length - 1][1];
}

function paintHeatmap() {
  if (!lodGeometry || !lodColors) return;
  const count = lodGeometry.getAttribute('position').count;
  const colors = new Float32Array(count * 3);
  const denom = Math.max(lodColorMax, 1e-9);
  for (let i = 0; i < count; i++) {
    const t = Math.pow(lodColors[i] / denom, 0.72);
    const c = ramp(t);
    colors[i * 3] = c.r; colors[i * 3 + 1] = c.g; colors[i * 3 + 2] = c.b;
  }
  lodGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  lodMaterial.vertexColors = state.heatmap;
  lodMaterial.color.set(state.heatmap ? 0xffffff : 0x2dd4bf);
  lodMaterial.needsUpdate = true;
}

/* ------------------------------------------------------------------ */
/* layout / camera                                                     */
/* ------------------------------------------------------------------ */

function layout(mesh, refit) {
  const b = bboxOf(mesh.positions);
  const w = Math.max(b.max[0] - b.min[0], 0.6);
  const h = b.max[1] - b.min[1];
  const gap = Math.max(0.22, w * 0.11);
  const offset = w / 2 + gap / 2;
  sourceGroup.position.x = -offset;
  lodGroup.position.x = offset;

  if (!refit) { controls.update(); return; }

  const totalW = w * 2 + gap;
  const vFov = (camera.fov * Math.PI) / 180;
  const distH = totalW / (2 * Math.tan(vFov / 2));
  const distV = Math.max(h * 1.5, 1.6) / (2 * Math.tan(vFov / 2));
  const dist = Math.max(distH, distV) * 1.18 + 0.6;
  camera.position.set(0, Math.max(0.55, h * 0.32 + 0.55), dist);
  camera.near = Math.max(0.05, dist / 200);
  camera.far = dist * 8;
  camera.updateProjectionMatrix();
  controls.target.set(0, 0, 0);
  controls.minDistance = Math.max(1.2, w * 0.55);
  controls.maxDistance = dist * 3.2;
  controls.update();
  grid.position.y = b.min[1] - 0.16;
}

/* ------------------------------------------------------------------ */
/* stats                                                               */
/* ------------------------------------------------------------------ */

function renderStats() {
  if (!lodCore) return;
  const dev = lastDeviation;
  $('sTris').textContent = fmt(lodTopo.faces);
  $('sReduction').textContent = `${(100 * (1 - lodTopo.faces / sourceTopo.faces)).toFixed(1)}%`;
  $('sTime').textContent = `${lodCore.stats.timeMs.toFixed(0)} ms`;
  $('sMax').textContent = `${dev.maxPct.toFixed(3)}%`;
  $('sMean').textContent = `${dev.meanPct.toFixed(3)}%`;
  if (lodTopo.closed && Math.abs(sourceTopo.volume) > 1e-9) {
    const drift = ((lodTopo.volume - sourceTopo.volume) / Math.abs(sourceTopo.volume)) * 100;
    $('sVol').textContent = `${drift >= 0 ? '+' : ''}${drift.toFixed(2)}%`;
    $('sVol').className = 'v';
  } else {
    $('sVol').textContent = 'n/a · open surface';
    $('sVol').className = 'v dim';
  }
  $('sEuler').textContent = String(lodTopo.euler);

  const chip = $('topoChip');
  if (lodTopo.closed) {
    chip.classList.remove('warn');
    $('topoText').textContent =
      `closed manifold · 0 boundary · 0 non-manifold · ${lodTopo.components} component${lodTopo.components === 1 ? '' : 's'}`;
  } else {
    chip.classList.add('warn');
    $('topoText').textContent =
      `open surface · ${fmt(lodTopo.boundaryEdges)} boundary edges · 0 non-manifold`;
  }

  $('srcLabel').textContent = `${fmt(sourceTopo.faces)} tris`;
  $('lodLabel').textContent = `${fmt(lodTopo.faces)} tris`;
  $('legendMax').textContent = `${((lodColorMax / (dev.diag || 1)) * 100).toFixed(3)}% diag`;
  $('legend').classList.toggle('hidden', !state.heatmap);
}

/* ------------------------------------------------------------------ */
/* pipeline                                                            */
/* ------------------------------------------------------------------ */

function run() {
  const src = getSource(state.kind);
  sourceCore = src;
  sourceTopo = analyzeTopology(src);

  const target = Math.max(4, Math.round(sourceTopo.faces * state.ratio));
  const t0 = performance.now();
  const lod = decimate(src, { targetRatio: state.ratio });
  lodCore = lod;
  lodTopo = analyzeTopology(lod);
  lastDeviation = measureDeviation(src, lod, { samples: 1200, cells: 40 });
  lodColors = vertexDistances(lod, src, 40);
  lodColorMax = 0;
  for (let i = 0; i < lodColors.length; i++) if (lodColors[i] > lodColorMax) lodColorMax = lodColors[i];

  const tTotal = performance.now() - t0;

  // rebuild meshes
  if (sourceMeshObj) { sourceGroup.remove(sourceMeshObj); sourceMeshObj.geometry.dispose(); }
  if (lodMeshObj) { lodGroup.remove(lodMeshObj); lodGeometry.dispose(); }
  sourceMeshObj = new THREE.Mesh(toGeometry(src), sourceMaterial);
  lodGeometry = toGeometry(lod);
  lodMeshObj = new THREE.Mesh(lodGeometry, lodMaterial);
  sourceGroup.add(sourceMeshObj);
  lodGroup.add(lodMeshObj);

  lodMaterial.wireframe = state.wireframe;
  paintHeatmap();
  layout(src, state.kind !== lastFitKind);
  lastFitKind = state.kind;

  $('sTris').textContent = fmt(lodTopo.faces);
  $('sReduction').textContent = `${(100 * (1 - lodTopo.faces / sourceTopo.faces)).toFixed(1)}%`;
  $('sTime').textContent = `${lod.stats.timeMs.toFixed(0)} ms`;
  $('sMax').textContent = `${lastDeviation.maxPct.toFixed(3)}%`;
  $('sMean').textContent = `${lastDeviation.meanPct.toFixed(3)}%`;
  renderStats();

  $('ratioOut').textContent = String(Math.round(state.ratio * 100));
  $('ratioFrom').textContent = `of ${fmt(sourceTopo.faces)} tris`;
  $('targetSub').textContent = `${fmt(lodTopo.faces)} tris`;
  const pct = ((state.ratio * 100 - 5) / 45) * 100;
  $('ratio').style.setProperty('--fill', `${pct}%`);
  $('srcLabel').textContent = `${fmt(sourceTopo.faces)} tris`;
  $('lodLabel').textContent = `${fmt(lodTopo.faces)} tris`;

  console.info(
    `[mesh-lod-studio] ${state.kind} ${sourceTopo.faces} -> ${lodTopo.faces} tris in ${lod.stats.timeMs.toFixed(0)} ms; ` +
    `max ${lastDeviation.maxPct.toFixed(3)}% mean ${lastDeviation.meanPct.toFixed(3)}% of diag; ` +
    `closed=${lodTopo.closed} euler=${lodTopo.euler} (total ${tTotal.toFixed(0)} ms)`,
  );
}

function schedule(reason) {
  if (state.busy) { state.pending = true; return; }
  state.busy = true;
  const src = getSource(state.kind);
  const est = Math.max(4, Math.round(analyzeTopology(src).faces * state.ratio));
  setStatus(true, `${reason} · ${fmt(est)} tris`);
  requestAnimationFrame(() => setTimeout(() => {
    try {
      run();
    } catch (err) {
      console.error(err);
      $('fatal').classList.add('on');
      $('fatal').querySelector('h2').textContent = 'Geometry pipeline failed';
      $('fatal').querySelector('p').textContent = String(err && err.message ? err.message : err);
    } finally {
      setStatus(false);
      state.busy = false;
      if (state.pending) { state.pending = false; schedule('updating'); }
    }
  }, 24));
}

/* ------------------------------------------------------------------ */
/* UI wiring                                                           */
/* ------------------------------------------------------------------ */

$('models').addEventListener('click', (e) => {
  const btn = e.target.closest('button[data-kind]');
  if (!btn) return;
  state.kind = btn.dataset.kind;
  for (const b of $('models').querySelectorAll('button')) b.classList.toggle('active', b === btn);
  schedule('loading mesh');
});

const ratioEl = $('ratio');
ratioEl.addEventListener('input', () => {
  state.ratio = Number(ratioEl.value) / 100;
  $('ratioOut').textContent = ratioEl.value;
  const pct = ((Number(ratioEl.value) - 5) / 45) * 100;
  ratioEl.style.setProperty('--fill', `${pct}%`);
  const src = getSource(state.kind);
  $('targetSub').textContent = `${fmt(Math.round(analyzeTopology(src).faces * state.ratio))} tris`;
});

ratioEl.addEventListener('change', () => schedule('decimating'));

$('heatmap').addEventListener('change', (e) => {
  state.heatmap = e.target.checked;
  if (lodGeometry) paintHeatmap();
  $('legend').classList.toggle('hidden', !state.heatmap);
});

$('wire').addEventListener('change', (e) => {
  state.wireframe = e.target.checked;
  lodMaterial.wireframe = state.wireframe;
});

$('spin').addEventListener('change', (e) => {
  state.autoRotate = e.target.checked;
  controls.autoRotate = state.autoRotate;
});

$('export').addEventListener('click', () => {
  if (!lodCore) return;
  const lines = [
    '# mesh-lod-studio — QEM decimation',
    `# source: ${state.kind} ${sourceTopo.faces} tris`,
    `# lod: ${lodTopo.faces} tris, max error ${lastDeviation.maxPct.toFixed(4)}% of bbox diagonal`,
    `# closed=${lodTopo.closed} euler=${lodTopo.euler}`,
  ];
  const p = lodCore.positions;
  for (let i = 0; i < p.length; i += 3) lines.push(`v ${p[i].toFixed(6)} ${p[i + 1].toFixed(6)} ${p[i + 2].toFixed(6)}`);
  const idx = lodCore.indices;
  for (let i = 0; i < idx.length; i += 3) lines.push(`f ${idx[i] + 1} ${idx[i + 1] + 1} ${idx[i + 2] + 1}`);
  const blob = new Blob([lines.join('\n') + '\n'], { type: 'text/plain' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `lod-${state.kind}-${Math.round(state.ratio * 100)}pct.obj`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 4000);
});

/* ------------------------------------------------------------------ */
/* resize + loop                                                       */
/* ------------------------------------------------------------------ */

function resize() {
  const stage = renderer.domElement.parentElement;
  const w = stage.clientWidth, h = stage.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(renderer.domElement.parentElement);
window.addEventListener('resize', resize);

renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});

resize();
schedule('decimating');
