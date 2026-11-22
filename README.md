# Sokoban Solver — make.fun entry

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/lions0810/sokoban-solver-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/lions0810/sokoban-solver-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

An entry to make.fun's [“Build the best Sokoban solver” bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121) ($100, top 10 × $10).

One program answers all 5 instances of a set in a single run: it writes the player's
moves (LURD) that push every box onto a goal in the push-optimal number of pushes.

## How it works

Every generated level is a one-square-wide hallway with one to four rooms off it,
each room opening onto the hallway through a single door, and a box pushed out of a
room can never be pushed back — so the rooms are independent and a level's optimum
is the sum of its rooms' optima. The solver exploits that:

1. **Decompose.** The hallway is the row the player starts on; each room is the
   flood fill from its door over the floor outside that row. (A room may start
   with a box sitting on its door cell, so the room search starts the player in
   the hallway, on the cell next to the door.)
2. **Solve each room push-optimally with A\*.** States are push-states: the sorted
   box positions plus the player's reachable region. The heuristic is the minimum
   bipartite matching between boxes and goals over box-free shortest paths, which
   is admissible, so the first goal popped is push-optimal. Deadlocks are pruned
   soundly: corner and unreachable cells, frozen boxes (fixpoint — a box counts
   as frozen only when every push direction is blocked by a wall or by a box
   that is itself frozen), and 2×2 blocks of walls/boxes holding a box off its goal.
3. **Order the rooms.** Every room but the last is solved with the extra
   requirement that the player can walk back out to its door; the last room pays
   its unconstrained optimum. The order minimising total pushes is chosen.
4. **Assemble.** The player walks the hallway between rooms (BFS, avoiding boxes),
   enters each room, solves it, and walks out.

All budgets are node counts, never wall clock, so two executions write
byte-identical output. No dependencies, no network, plain JavaScript run by
Node 22 as `node main.ts`.

## Practice result

`grade-local` on the kit's practice set: **1,000,000** — all 5 instances solved in
exactly the push-optimal counts (13, 16, 29, 47, 60 pushes), in about half a second.
