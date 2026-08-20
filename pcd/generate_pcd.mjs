import { writeFileSync } from 'fs';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';

const points = [];

// Ground plane: 6m x 8m room, y=0, 0.5m grid spacing
for (let x = -3; x <= 3; x += 0.5) {
  for (let z = 0; z <= 8; z += 0.5) {
    points.push([x, 0.0, z]);
  }
}

// Back wall: z=8, y from 0 to 2.5
for (let x = -3; x <= 3; x += 0.5) {
  for (let y = 0; y <= 2.5; y += 0.5) {
    points.push([x, y, 8.0]);
  }
}

// Left wall: x=-3, y from 0 to 2.5
for (let z = 0; z <= 8; z += 0.5) {
  for (let y = 0; y <= 2.5; y += 0.5) {
    points.push([-3.0, y, z]);
  }
}

// Right wall: x=3, y from 0 to 2.5
for (let z = 0; z <= 8; z += 0.5) {
  for (let y = 0; y <= 2.5; y += 0.5) {
    points.push([3.0, y, z]);
  }
}

// Box obstacle at center (1, 0, 4), size 1x1x1
const boxCx = 1.0, boxCz = 4.0;
const bw = 0.5, bh = 1.0, bd = 0.5;
for (let u = -bw; u <= bw; u += 0.25) {
  for (let v = 0; v <= bh; v += 0.25) {
    points.push([boxCx + u, v, boxCz + bd]); // front
    points.push([boxCx + u, v, boxCz - bd]); // back
  }
  for (let w = -bd; w <= bd; w += 0.25) {
    points.push([boxCx - bw, u + bw, boxCz + w]); // left
    points.push([boxCx + bw, u + bw, boxCz + w]); // right
    points.push([boxCx + u, bh, boxCz + w]);      // top
  }
}

// Cylinder pillar at (-1, 0, 6), radius=0.4, height=2
const cylX = -1.0, cylZ = 6.0, cylR = 0.4, cylH = 2.0;
for (let theta = 0; theta < 2 * Math.PI; theta += 0.3) {
  for (let y = 0; y <= cylH; y += 0.25) {
    points.push([
      cylX + cylR * Math.cos(theta),
      y,
      cylZ + cylR * Math.sin(theta)
    ]);
  }
}

// Write PCD
const lines = [
  'VERSION 0.7',
  'FIELDS x y z',
  'SIZE 4 4 4',
  'TYPE F F F',
  'COUNT 1 1 1',
  `WIDTH ${points.length}`,
  'HEIGHT 1',
  'VIEWPOINT 0 0 0 1 0 0 0',
  `POINTS ${points.length}`,
  'DATA ascii',
];
for (const [x, y, z] of points) {
  lines.push(`${x.toFixed(6)} ${y.toFixed(6)} ${z.toFixed(6)}`);
}

const outPath = join(dirname(fileURLToPath(import.meta.url)), 'sample.pcd');
writeFileSync(outPath, lines.join('\n') + '\n');
console.log(`Generated ${points.length} points -> ${outPath}`);
