# sokoban-solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/acaiyaya/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/acaiyaya/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver: one program answers a whole set of 5 levels in a single run, writing each level's moves (LURD, a lower letter a step, an upper one a push) to `/output/<id>/solution.txt`.

## How it works

Generated levels are a hallway with independent rooms off it (3–4 boxes each), and a box pushed out of a room can never come back — so a level's push-optimum is the sum of its rooms' optima. The solver therefore:

1. **Splits a level into rooms** — flood-fill after cutting the hallway's dead squares (squares from which no push sequence can reach a goal) out of the floor.
2. **Solves each room push-optimally, in turn**, with the other rooms' boxes treated as walls (on their goals once their room is done) and the player walking on from where the last room left it.

Each room is solved by **A\* over pushes**:

- a state is the sorted box squares plus the player's reachable region (named by its lowest square); the transposition table keys on both;
- the heuristic is the minimum-cost assignment of boxes to goals, where a box–goal cost is the exact minimum number of pushes to move that box alone onto that goal (precomputed by a reverse push-BFS from every goal, ignoring the other boxes). One push moves one box one square, so the assignment total is a lower bound on the pushes still needed — admissible, hence the search returns the push-optimal solution;
- deadlocks are pruned: a box on a dead square, and a box caught in a frozen 2×2 block of walls and boxes holding a box off its goal.

A push-BFS with the same pruning is kept as a fallback if A\* ever exhausts its work budget on a room.

## Determinism

Every budget is counted in expanded states, never in wall time; no clock is read, no randomness is used, and every tie is broken by fixed direction order (u, d, l, r) and insertion order — so two runs on one set write byte-identical answers.

## Results

- Practice set: **1,000,000 / 1,000,000** — every instance solved in exactly P\* pushes (13, 16, 29, 47, 60), in ~0.7 s for the whole set.
- 11 further sets drawn from the kit's own generator (55 instances): **1,000,000 on every set**, worst set runtime ~2 s against the 60 s budget.

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Node 22 runs `main.ts` directly (erasable TypeScript syntax only, no dependencies, no network, no native code).

---

<sub>Entered on [make.fun](https://make.fun/?ref=5ttpknjz&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
