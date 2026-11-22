// A push-optimal Sokoban solver, written for make.fun's sokoban-solver-01-run-20261121.
//
// Approach
// --------
// Generated levels are a hallway with independent rooms off it (3-4 boxes each).
// Boxes never leave their room, so the level's push-optimum is the sum of the
// rooms' optima. This solver therefore
//   1. splits a level into rooms (flood-fill after cutting the hallway's dead
//      squares out of the floor), and
//   2. solves each room push-optimally, in turn, with the other rooms' boxes
//      treated as walls, the player walking on from where the last room left it.
//
// Each room is solved by A* over pushes:
//   * a state is the sorted box squares plus the player's reachable region
//     (named by its lowest square) -- the transposition table keys on both;
//   * the heuristic is the minimum-cost assignment (Hungarian, brute-forced
//     over <= 4 boxes) of boxes to goals, where a box-goal cost is the exact
//     minimum number of pushes to move that box alone onto that goal
//     (precomputed by a reverse push-BFS from every goal, ignoring the other
//     boxes). One push moves one box one square, so the assignment total is a
//     lower bound on the pushes still needed: admissible, hence optimal;
//   * deadlocks are pruned: a box on a dead square (one from which no push
//     sequence, ignoring the other boxes, can reach a goal) and a box caught
//     in a frozen 2x2 block of walls and boxes with a box off its goal.
// A push-BFS with the same pruning is kept as a fallback if A* ever exhausts
// its work budget on a room.
//
// Determinism: every budget is counted in expanded states, never in wall
// time; no clock is read, no randomness is used, and every tie is broken by
// fixed direction order (u, d, l, r) and insertion order, so two runs of the
// program on one set write byte-identical answers.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets (expanded states, never the clock): a room the generator keeps
// is solvable by plain push-BFS within 250,000 states, and A* with the
// assignment heuristic expands a subset of that, so these caps leave margin.
const ASTAR_EXPANSIONS = 400_000;
const BFS_STATES = 250_000;

// The four directions: [dx, dy, step letter, push letter]. Fixed order u, d,
// l, r everywhere: BFS/heap tie-breaking, walk reconstruction, deadlocks.
const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];
const INF = 1e9;

type Level = {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall
  goals: number[]; // sorted
  boxes: number[]; // sorted
  player: number;
};

/** Parse XSB text (# wall, space floor, . goal, $ box, * box on a goal, @ player, + player on a goal). */
function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  // The outermost frame is wall whatever it shows: nothing leaves the board.
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals: goals.sort((a, b) => a - b), boxes: boxes.sort((a, b) => a - b), player };
}

/**
 * Dead squares: a square is live when a box standing on it can be pushed onto
 * some goal, other boxes ignored. Computed by pulling boxes back from the
 * goals: a box on y can be pushed to x (direction d) when the player can stand
 * on y - d, so y is live when x is live and both y and y - d are floor.
 * A corner or wall-line square with no goal on its line is dead by this.
 */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/**
 * Push distances: dist[g][s] = the fewest pushes to move one box from s onto
 * goal g, other boxes ignored. Reverse BFS from each goal: a box on y can be
 * pushed to x (direction d) when the player can stand on y - d, so from a
 * reached square x, the predecessor y = x - d is one push farther when y and
 * y - d are both floor.
 */
function pushDistances(level: Level): Int32Array[] {
  const { w, wall, goals } = level;
  const size = wall.length;
  const out: Int32Array[] = [];
  for (const g of goals) {
    const dist = new Int32Array(size).fill(-1);
    dist[g] = 0;
    const queue = [g];
    for (let qi = 0; qi < queue.length; qi++) {
      const x = queue[qi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const y = x - d;
        if (!wall[y] && !wall[y - d] && dist[y] < 0) {
          dist[y] = dist[x]! + 1;
          queue.push(y);
        }
      }
    }
    out.push(dist);
  }
  return out;
}

/** The player's reachable region from `from` with `occupied` boxes in the way; returns its lowest square. */
function regionLow(level: Level, occupied: Uint8Array, from: number, seen: Int32Array, stamp: number, stack: Int32Array): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes holding a box off its goal. */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/**
 * The minimum-cost assignment of boxes to goals (brute force over the <= 4!
 * permutations): the sum of the boxes' push distances to their assigned goals.
 * INF when no assignment reaches every goal -- the state is dead.
 */
function assignmentCost(boxes: number[], dists: Int32Array[], perm: number[]): number {
  const n = boxes.length;
  let best = INF;
  // Heap's algorithm over permutations of goal indices, deterministic order.
  const idx = [...perm];
  const c = new Array<number>(n).fill(0);
  const score = (): number => {
    let s = 0;
    for (let i = 0; i < n; i++) {
      const d = dists[idx[i]!]![boxes[i]!]!;
      if (d < 0) return INF;
      s += d;
      if (s >= best) return INF; // early cut: cannot improve
    }
    return s;
  };
  best = score();
  let i = 0;
  while (i < n) {
    if (c[i]! < i) {
      const j = i % 2 === 0 ? 0 : c[i]!;
      [idx[j], idx[i]] = [idx[i]!, idx[j]!];
      const s = score();
      if (s < best) best = s;
      c[i]!++;
      i = 0;
    } else {
      c[i] = 0;
      i++;
    }
  }
  return best;
}

/** A binary heap of [f, h, sequence] ordered by f, then h, then insertion sequence -- fully deterministic. */
class Heap {
  private a: [number, number, number, number][] = []; // [f, h, seq, state]
  get size(): number {
    return this.a.length;
  }
  private less(x: [number, number, number, number], y: [number, number, number, number]): boolean {
    return x[0] < y[0] || (x[0] === y[0] && (x[1] < y[1] || (x[1] === y[1] && x[2] < y[2])));
  }
  push(e: [number, number, number, number]): void {
    const a = this.a;
    let i = a.length;
    a.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(a[i]!, a[p]!)) break;
      [a[i], a[p]] = [a[p]!, a[i]!];
      i = p;
    }
  }
  pop(): [number, number, number, number] {
    const a = this.a;
    const top = a[0]!;
    const last = a.pop()!;
    if (a.length > 0) {
      a[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= a.length) break;
        if (c + 1 < a.length && this.less(a[c + 1]!, a[c]!)) c++;
        if (!this.less(a[c]!, a[i]!)) break;
        [a[i], a[c]] = [a[c]!, a[i]!];
        i = c;
      }
    }
    return top;
  }
}

// Numeric state key: the region's lowest square and the sorted box squares,
// as digits in base `size` (exact while size ** (boxes + 1) stays under 2 ** 53).
function keyOf(boxes: number[], low: number, size: number): number {
  let k = low;
  for (const b of boxes) k = k * size + b;
  return k;
}

/**
 * Push-optimal A* for one (sub-)level: {pushes, moves} (moves in LURD, a
 * lower letter a step, an upper one a push), or null when the work budget runs
 * out or the level has no solution. `maxExpansions` bounds the work.
 */
function solveAStar(level: Level, maxExpansions: number): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  const dists = pushDistances(level);
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (level.boxes.some((b) => !live[b])) return null;

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  // States: boxes, the player's square after the push that made them, parent,
  // and g (pushes so far). Index 0 is the start.
  const statesBoxes: number[][] = [level.boxes];
  const statesPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const gScore: number[] = [0];
  const best = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  const startLow = regionLow(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  const startH = assignmentCost(level.boxes, dists, goals.map((_, i) => i));
  if (startH >= INF) return null;
  best.set(keyOf(level.boxes, startLow, size), 0);

  const heap = new Heap();
  let seq = 0;
  heap.push([startH, startH, seq++, 0]);
  let expanded = 0;
  let found = solved(level.boxes) ? 0 : -1;

  while (heap.size > 0 && found < 0) {
    const [, , , s] = heap.pop();
    const boxes = statesBoxes[s]!;
    const g = gScore[s]!;
    const k0 = keyOf(boxes, regionLow(level, (() => {
      for (const b of boxes) occupied[b] = 1;
      return occupied;
    })(), statesPlayer[s]!, seen, ++stamp, stack), size);
    // The key above recomputed the region; restore occupied afterwards.
    for (const b of boxes) occupied[b] = 0;
    if (best.get(k0) !== g) continue; // a better route to this state was found
    if (expanded >= maxExpansions) return null;
    expanded++;

    for (const b of boxes) occupied[b] = 1;
    const here = ++stamp;
    regionLow(level, occupied, statesPlayer[s]!, seen, here, stack);
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = -1;
        let next: number[] | null = null;
        let h = INF;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          h = assignmentCost(next, dists, goals.map((_, i) => i));
          if (h < INF) key = keyOf(next, regionLow(level, occupied, b, mark, ++stamp, stack), size);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || h >= INF || key < 0) continue;
        const prev = best.get(key);
        if (prev !== undefined && prev <= g + 1) continue;
        best.set(key, g + 1);
        const ns = statesBoxes.length;
        statesBoxes.push(next!);
        statesPlayer.push(b);
        parent.push(s);
        gScore.push(g + 1);
        if (solved(next!)) {
          found = ns;
          break;
        }
        heap.push([g + 1 + h, h, seq++, ns]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  return { pushes: gScore[found]!, moves: reconstruct(level, statesBoxes, statesPlayer, parent, found) };
}

/**
 * Push-BFS fallback with the same deadlock pruning: expands states in order
 * of pushes made, so the first solved state is push-optimal. Kept separate
 * from A* so a room A* gives up on still gets a complete search.
 */
function solveBfs(level: Level, maxStates: number): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (level.boxes.some((b) => !live[b])) return null;

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const statesBoxes: number[][] = [level.boxes];
  const statesPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const index = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  index.set(keyOf(level.boxes, regionLow(level, occupied, level.player, seen, ++stamp, stack), size), 0);
  for (const b of level.boxes) occupied[b] = 0;
  let found = solved(level.boxes) ? 0 : -1;
  for (let head = 0; found < 0 && head < statesBoxes.length; head++) {
    const boxes = statesBoxes[head]!;
    for (const b of boxes) occupied[b] = 1;
    regionLow(level, occupied, statesPlayer[head]!, seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = -1;
        let next: number[] | null = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, regionLow(level, occupied, b, mark, ++stamp, stack), size);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || index.has(key)) continue;
        if (statesBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        index.set(key, statesBoxes.length);
        statesBoxes.push(next!);
        statesPlayer.push(b);
        parent.push(head);
        if (solved(next!)) {
          found = statesBoxes.length - 1;
          break;
        }
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  let pushes = 0;
  for (let s = found; s > 0; s = parent[s]!) pushes++;
  return { pushes, moves: reconstruct(level, statesBoxes, statesPlayer, parent, found) };
}

/** The shortest walk from `from` to `to` around walls and boxes, in lower LURD letters (u, d, l, r first on ties). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: the push square is out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** The square the player stands on after `moves` (LURD, case aside) from `from` on a board `w` wide. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/** Rebuild the move string of a found state from the parent chain: walk to each push, then push. */
function reconstruct(level: Level, statesBoxes: number[][], statesPlayer: number[], parent: number[], found: number): string {
  const { w, wall } = level;
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]!) path.push(s);
  path.reverse();
  const occupied = new Uint8Array(wall.length);
  let moves = '';
  let player = statesPlayer[0]!;
  let boxes = statesBoxes[0]!;
  for (const s of path) {
    const nextBoxes = statesBoxes[s]!;
    const from = boxes.find((b) => !nextBoxes.includes(b))!;
    const to = nextBoxes.find((b) => !boxes.includes(b))!;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d)!;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[3];
    player = from;
    boxes = nextBoxes;
  }
  return moves;
}

type Room = { boxes: number[]; goals: number[] };

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway:
 * a box there reaches no goal) out of the floor, and each part left that holds
 * a box is a room. Null when some part holds more or fewer boxes than goals,
 * or a box starts in the hallway: then the level is no set of rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (goals.some((g) => part[g]! < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/**
 * Solve a whole level: room by room when it splits into rooms, else one
 * search over the whole level. Each room's solution runs with the other
 * rooms' boxes standing as walls (on their goals once their room is done),
 * the player walking on from where the last room left it. A box never leaves
 * its room, so the rooms' optima add up to the level's.
 */
function solveLevel(level: Level): string {
  const rooms = roomsOf(level);
  if (rooms === null) {
    const whole = solveAStar(level, ASTAR_EXPANSIONS) ?? solveBfs(level, BFS_STATES);
    return whole === null ? '' : whole.moves;
  }
  let player = level.player;
  let moves = '';
  const done: boolean[] = rooms.map(() => false);
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri]!;
    const wall = level.wall.slice();
    for (let oj = 0; oj < rooms.length; oj++) {
      if (oj === ri) continue;
      for (const b of done[oj]! ? rooms[oj]!.goals : rooms[oj]!.boxes) wall[b] = 1;
    }
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solveAStar(sub, ASTAR_EXPANSIONS) ?? solveBfs(sub, BFS_STATES);
    if (solved === null) return '';
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    done[ri] = true;
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8')));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
