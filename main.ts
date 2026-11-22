// Sokoban solver: decompose each level into the rooms off its hallway, then solve
// each room push-optimally with A* (min-matching heuristic + deadlock pruning).
// The player starts in the hallway, enters a room through its one door, solves it,
// exits, and moves on; rooms are independent, so a level's optimum is the sum of
// its rooms' optima. Deterministic: every budget is a node count, never wall clock.
import { mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// u, d, l, r. Opposite of direction d is d ^ 1 by this ordering.
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

// Node-count budgets (never wall clock, so two runs write identical bytes).
const ASTAR_BUDGET = 120000;
const GREEDY_BUDGET = 60000;
const GLOBAL_BUDGET = 1500000;
let globalExpansions = 0;

/** Parse an XSB level. */
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const floor = new Uint8Array(w * h);
  const goal = new Uint8Array(w * h);
  const box = new Uint8Array(w * h);
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = rows[y][x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      floor[i] = 1;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') box[i] = 1;
      if (ch === '@' || ch === '+') player = i;
    }
  }
  return { w, h, floor, goal, box, player };
}

/**
 * Find the rooms: the hallway is the player's row; a door is a floor cell directly
 * above/below a hallway cell; each room is the flood fill from its door over floor
 * cells outside the hallway row. Returns [{door, entry, cells}].
 */
function findRooms(level) {
  const { w, h, floor, player } = level;
  const py = Math.floor(player / w);
  const doors = [];
  for (let x = 0; x < w; x++) {
    if (!floor[py * w + x]) continue;
    for (const dy of [-1, 1]) {
      const y = py + dy;
      if (y >= 0 && y < h && floor[y * w + x]) doors.push({ door: y * w + x, entry: py * w + x });
    }
  }
  const seen = new Uint8Array(w * h);
  const rooms = [];
  for (const d of doors) {
    if (seen[d.door]) continue;
    const cells = [];
    const stack = [d.door];
    seen[d.door] = 1;
    while (stack.length) {
      const c = stack.pop();
      cells.push(c);
      const cx = c % w;
      const cy = Math.floor(c / w);
      for (const dir of DIRS) {
        const nx = cx + dir.dx;
        const ny = cy + dir.dy;
        if (ny === py) continue;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (!seen[ni] && floor[ni]) {
          seen[ni] = 1;
          stack.push(ni);
        }
      }
    }
    rooms.push({ door: d.door, entry: d.entry, cells });
  }
  return rooms;
}

/**
 * Build a room's search model. The searchable region is the room's cells plus the
 * hallway entry cell; the player starts on the entry cell and boxes may only ever
 * sit on room cells (a box pushed into the hallway could never come back).
 */
function buildRoom(level, roomCells, doorCell, entryCell) {
  const region = [...roomCells, entryCell].sort((a, b) => a - b);
  const n = region.length;
  const loc = new Map(region.map((g, i) => [g, i]));
  const xs = region.map((g) => g % level.w);
  const ys = region.map((g) => Math.floor(g / level.w));
  const neigh = [];
  for (let i = 0; i < n; i++) {
    const arr = [];
    for (const d of DIRS) {
      const g = (ys[i] + d.dy) * level.w + (xs[i] + d.dx);
      arr.push(loc.has(g) ? loc.get(g) : -1);
    }
    neigh.push(arr);
  }
  // All-pairs shortest paths ignoring boxes (for the matching heuristic).
  const dist = new Int32Array(n * n).fill(-1);
  for (let s = 0; s < n; s++) {
    const q = [s];
    dist[s * n + s] = 0;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      for (let d = 0; d < 4; d++) {
        const nb = neigh[c][d];
        if (nb >= 0 && dist[s * n + nb] < 0) {
          dist[s * n + nb] = dist[s * n + c] + 1;
          q.push(nb);
        }
      }
    }
  }
  const goals = [];
  const boxes0 = [];
  for (let i = 0; i < n; i++) {
    if (level.goal[region[i]]) goals.push(i);
    if (level.box[region[i]]) boxes0.push(i);
  }
  goals.sort((a, b) => a - b);
  boxes0.sort((a, b) => a - b);
  // Statically dead cells: corners (non-goal) and cells from which no goal is reachable.
  const staticDead = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    if (goals.includes(i)) continue;
    const up = neigh[i][0] < 0;
    const dn = neigh[i][1] < 0;
    const lf = neigh[i][2] < 0;
    const rt = neigh[i][3] < 0;
    if ((up && lf) || (up && rt) || (dn && lf) || (dn && rt)) staticDead[i] = 1;
    let any = false;
    for (const g of goals) {
      if (dist[i * n + g] >= 0) {
        any = true;
        break;
      }
    }
    if (!any) staticDead[i] = 1;
  }
  // Every 2x2 square of the region (a cell outside the region counts as wall).
  const byCoord = new Map(region.map((g, i) => [`${g % level.w},${Math.floor(g / level.w)}`, i]));
  const squares = [];
  for (let i = 0; i < n; i++) {
    const x = xs[i];
    const y = ys[i];
    const at = (xx, yy) => (byCoord.has(`${xx},${yy}`) ? byCoord.get(`${xx},${yy}`) : -1);
    squares.push([i, at(x + 1, y), at(x, y + 1), at(x + 1, y + 1)]);
  }
  return {
    n,
    region,
    neigh,
    dist,
    goals,
    boxes0,
    doorLocal: loc.get(doorCell),
    entryLocal: loc.get(entryCell),
    staticDead,
    squares,
  };
}

/** BFS from a cell avoiding boxes. parent[i] = previous cell, -2 for the start, -1 unvisited. */
function bfsFrom(room, start, boxAt) {
  const parent = new Int32Array(room.n).fill(-1);
  const pdir = new Int8Array(room.n).fill(-1);
  const q = [start];
  parent[start] = -2;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    for (let d = 0; d < 4; d++) {
      const nb = room.neigh[c][d];
      if (nb >= 0 && parent[nb] === -1 && !boxAt[nb]) {
        parent[nb] = c;
        pdir[nb] = d;
        q.push(nb);
      }
    }
  }
  return { parent, pdir };
}

/** The walk (as move letters) from the BFS start to a target cell. */
function pathTo(bfs, target) {
  const letters = [];
  let c = target;
  while (bfs.parent[c] >= 0) {
    letters.push(DIRS[bfs.pdir[c]].ch);
    c = bfs.parent[c];
  }
  letters.reverse();
  return letters.join('');
}

/** Smallest cell index the player can reach from `player` without moving a box. */
function regionMin(room, boxes, player) {
  const boxAt = new Uint8Array(room.n);
  for (const b of boxes) boxAt[b] = 1;
  const bfs = bfsFrom(room, player, boxAt);
  let min = player;
  for (let i = 0; i < room.n; i++) {
    if (bfs.parent[i] !== -1 && i < min) min = i;
  }
  return min;
}

/**
 * Deadlock test for a box set. Sound prunings only: static-dead cells, frozen boxes
 * (fixpoint: a box is frozen only when every push direction is blocked by a wall or
 * by a box that is itself frozen — a movable neighbour may always move away), and
 * 2x2 blocks of walls/boxes holding a box off its goal.
 */
function isDead(room, boxes) {
  const boxAt = new Uint8Array(room.n);
  for (const b of boxes) boxAt[b] = 1;
  const goalSet = new Set(room.goals);
  for (const b of boxes) if (room.staticDead[b]) return true;
  const frozen = new Uint8Array(room.n);
  let changed = true;
  while (changed) {
    changed = false;
    for (const b of boxes) {
      if (frozen[b] || goalSet.has(b)) continue;
      let movable = false;
      for (let d = 0; d < 4; d++) {
        const t = room.neigh[b][d];
        const s = room.neigh[b][d ^ 1];
        const tBlocked = t < 0 || (boxAt[t] && frozen[t]);
        const sBlocked = s < 0 || (boxAt[s] && frozen[s]);
        if (!tBlocked && !sBlocked) {
          movable = true;
          break;
        }
      }
      if (!movable) {
        frozen[b] = 1;
        changed = true;
      }
    }
  }
  for (const b of boxes) if (frozen[b]) return true;
  for (const sq of room.squares) {
    let hasBadBox = false;
    let allBlocked = true;
    for (const cell of sq) {
      if (cell < 0) continue; // wall counts as blocked
      if (boxAt[cell]) {
        if (!goalSet.has(cell)) hasBadBox = true;
      } else {
        allBlocked = false;
      }
    }
    if (hasBadBox && allBlocked) return true;
  }
  return false;
}

/** Minimum bipartite matching cost between boxes and goals (Infinity if impossible). */
function heuristic(room, boxes) {
  const { dist, goals, n } = room;
  const m = boxes.length;
  if (m === 0) return 0;
  let best = Infinity;
  const used = new Array(goals.length).fill(false);
  const rec = (i, sum) => {
    if (sum >= best) return;
    if (i === m) {
      best = sum;
      return;
    }
    for (let j = 0; j < goals.length; j++) {
      if (used[j]) continue;
      const d = dist[boxes[i] * n + goals[j]];
      if (d < 0) continue;
      used[j] = true;
      rec(i + 1, sum + d);
      used[j] = false;
    }
  };
  rec(0, 0);
  return best;
}

/**
 * Push-optimal A* over push-states of one room (greedy=true turns it into a
 * best-first fallback that dives for any solution). The player starts on the
 * entry cell. Returns {ok, pushes, moves, endPlayer} or {ok:false, reason}.
 * With exitRequired the returned moves end with the player back on the entry cell.
 */
function searchRoom(room, exitRequired, budget, greedy) {
  const { n, goals, boxes0, entryLocal } = room;
  const goalKey = goals.join(',');
  let seq = 0;
  const heap = [];
  const cmp = (a, b) => a.f - b.f || a.h - b.h || a.seq - b.seq;
  const heapPush = (node) => {
    heap.push(node);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmp(heap[p], heap[i]) <= 0) break;
      const tmp = heap[p];
      heap[p] = heap[i];
      heap[i] = tmp;
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && cmp(heap[l], heap[m]) < 0) m = l;
        if (r < heap.length && cmp(heap[r], heap[m]) < 0) m = r;
        if (m === i) break;
        const tmp = heap[m];
        heap[m] = heap[i];
        heap[i] = tmp;
        i = m;
      }
    }
    return top;
  };
  const closed = new Map();
  const makeNode = (boxes, player, g, parent, moves, h) => {
    const canon = regionMin(room, boxes, player);
    return {
      boxes,
      player,
      g,
      h,
      f: greedy ? h : g + h,
      parent,
      moves,
      seq: seq++,
      key: `${boxes.join(',')}|${canon}`,
    };
  };
  const startH = heuristic(room, boxes0);
  if (!Number.isFinite(startH)) return { ok: false, reason: 'exhausted' };
  if (isDead(room, boxes0)) return { ok: false, reason: 'exhausted' };
  heapPush(makeNode(boxes0, entryLocal, 0, null, '', startH));
  let expansions = 0;
  while (heap.length) {
    const node = heapPop();
    const prevG = closed.get(node.key);
    if (prevG !== undefined && prevG <= node.g) continue;
    closed.set(node.key, node.g);
    if (node.boxes.join(',') === goalKey) {
      let endPlayer = node.player;
      let extra = '';
      let solved = true;
      if (exitRequired) {
        const boxAt = new Uint8Array(n);
        for (const b of node.boxes) boxAt[b] = 1;
        const bfs = bfsFrom(room, node.player, boxAt);
        if (bfs.parent[entryLocal] === -1) {
          // All boxes are home but the player is walled in: do not discard this
          // state — a box may have to leave a goal and come back so the player
          // can get out. Fall through and expand it instead.
          solved = false;
        } else {
          extra = pathTo(bfs, entryLocal);
          endPlayer = entryLocal;
        }
      }
      if (solved) {
        const parts = [];
        let cur = node;
        while (cur) {
          parts.push(cur.moves);
          cur = cur.parent;
        }
        parts.reverse();
        return { ok: true, pushes: node.g, moves: parts.join('') + extra, endPlayer };
      }
    }
    if (++expansions > budget || ++globalExpansions > GLOBAL_BUDGET) {
      return { ok: false, reason: 'budget' };
    }
    const boxAt = new Uint8Array(n);
    for (const b of node.boxes) boxAt[b] = 1;
    const bfs = bfsFrom(room, node.player, boxAt);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const c = node.boxes[bi];
      for (let d = 0; d < 4; d++) {
        const t = room.neigh[c][d];
        if (t < 0 || boxAt[t]) continue;
        const s = room.neigh[c][d ^ 1];
        if (s < 0 || boxAt[s]) continue;
        if (bfs.parent[s] === -1) continue; // player cannot reach the pushing spot
        const newBoxes = [...node.boxes];
        newBoxes[bi] = t;
        newBoxes.sort((a, b) => a - b);
        if (isDead(room, newBoxes)) continue;
        const h = heuristic(room, newBoxes);
        if (!Number.isFinite(h)) continue;
        const g2 = node.g + 1;
        const moves = pathTo(bfs, s) + DIRS[d].ch;
        const child = makeNode(newBoxes, c, g2, node, moves, h);
        const prev = closed.get(child.key);
        if (prev !== undefined && prev <= g2) continue;
        heapPush(child);
      }
    }
  }
  return { ok: false, reason: 'exhausted' };
}

/** Walk on the whole level from `from` to `to` avoiding boxes; null if unreachable. */
function walkPath(level, from, to, boxNow) {
  const { w, h, floor } = level;
  const parent = new Int32Array(w * h).fill(-1);
  const pdir = new Int8Array(w * h).fill(-1);
  const q = [from];
  parent[from] = -2;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    if (c === to) break;
    const cx = c % w;
    const cy = Math.floor(c / w);
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d].dx;
      const ny = cy + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const ni = ny * w + nx;
      if (parent[ni] === -1 && floor[ni] && !boxNow[ni]) {
        parent[ni] = c;
        pdir[ni] = d;
        q.push(ni);
      }
    }
  }
  if (parent[to] === -1) return null;
  const letters = [];
  let c = to;
  while (parent[c] >= 0) {
    letters.push(DIRS[pdir[c]].ch);
    c = parent[c];
  }
  letters.reverse();
  return letters.join('');
}

/** Solve one level; '' when the level cannot be solved under this model. */
function solveLevel(level) {
  const rooms = findRooms(level);
  const totalBoxes = [...level.box].reduce((a, b) => a + b, 0);
  const totalGoals = [...level.goal].reduce((a, b) => a + b, 0);
  const roomBoxes = rooms.reduce(
    (sum, r) => sum + r.cells.filter((c) => level.box[c]).length,
    0,
  );
  const roomGoals = rooms.reduce(
    (sum, r) => sum + r.cells.filter((c) => level.goal[c]).length,
    0,
  );
  // Fallback: if the room split does not account for every box and goal, treat
  // the whole level as a single room (player starts where it stands).
  if (rooms.length === 0 || roomBoxes !== totalBoxes || roomGoals !== totalGoals) {
    const allCells = [];
    for (let i = 0; i < level.w * level.h; i++) if (level.floor[i]) allCells.push(i);
    const room = buildRoom(level, allCells, level.player, level.player);
    let res = searchRoom(room, false, ASTAR_BUDGET, false);
    if (!res.ok && res.reason === 'budget') res = searchRoom(room, false, GREEDY_BUDGET, true);
    return res.ok ? res.moves : '';
  }
  const infos = rooms.map((r) => {
    const room = buildRoom(level, r.cells, r.door, r.entry);
    let un = searchRoom(room, false, ASTAR_BUDGET, false);
    if (!un.ok && un.reason === 'budget') un = searchRoom(room, false, GREEDY_BUDGET, true);
    let ex = searchRoom(room, true, ASTAR_BUDGET, false);
    if (!ex.ok && ex.reason === 'budget') ex = searchRoom(room, true, GREEDY_BUDGET, true);
    return { room, door: r.door, entry: r.entry, un, ex };
  });
  for (const info of infos) if (!info.un.ok) return '';
  // Pick the room order minimising total pushes: every room but the last pays its
  // exit-feasible optimum, the last pays its unconstrained optimum. A room with no
  // exit-feasible solution must go last; two such rooms cannot be ordered at all.
  const mustLast = infos.map((info, i) => (info.ex.ok ? -1 : i)).filter((i) => i >= 0);
  if (mustLast.length > 1) return '';
  const idx = infos.map((_, i) => i);
  let bestOrder = null;
  let bestTotal = Infinity;
  const permute = (arr, k) => {
    if (k === arr.length) {
      let total = 0;
      for (let j = 0; j < arr.length; j++) {
        const info = infos[arr[j]];
        if (j === arr.length - 1) {
          total += info.un.pushes;
        } else if (info.ex.ok) {
          total += info.ex.pushes;
        } else {
          return; // a room that cannot be exited sits mid-order
        }
      }
      if (total < bestTotal) {
        bestTotal = total;
        bestOrder = [...arr];
      }
      return;
    }
    for (let i = k; i < arr.length; i++) {
      [arr[k], arr[i]] = [arr[i], arr[k]];
      permute(arr, k + 1);
      [arr[k], arr[i]] = [arr[i], arr[k]];
    }
  };
  permute(idx, 0);
  if (!bestOrder) return '';
  let moves = '';
  let cur = level.player;
  const boxNow = new Uint8Array(level.box);
  for (let k = 0; k < bestOrder.length; k++) {
    const info = infos[bestOrder[k]];
    const last = k === bestOrder.length - 1;
    const sol = last ? info.un : info.ex.ok ? info.ex : info.un;
    const w = walkPath(level, cur, info.entry, boxNow);
    if (w === null) return '';
    moves += w;
    moves += sol.moves;
    cur = info.room.region[sol.endPlayer];
    for (const b of info.room.boxes0) boxNow[info.room.region[b]] = 0;
    for (const g of info.room.goals) boxNow[info.room.region[g]] = 1;
  }
  return moves;
}

for (const id of readdirSync(INPUT).sort()) {
  let moves = '';
  try {
    if (statSync(join(INPUT, id)).isDirectory()) {
      moves = solveLevel(parseLevel(readFileSync(join(INPUT, id, 'level.txt'), 'utf8')));
    }
  } catch {
    moves = '';
  }
  mkdirSync(join(OUTPUT, id), { recursive: true });
  writeFileSync(join(OUTPUT, id, 'solution.txt'), `${moves}\n`);
}
