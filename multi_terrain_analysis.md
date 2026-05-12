# Multi-Terrain Robot Navigation — Analysis

## Part 1: Problem Formulation & Heuristic Design

### 1.1 State Space & Actions

**Environment:** A 7×7 2D grid where each cell represents a position and has an associated terrain type:

| Symbol | Terrain        | Movement Cost |
|--------|----------------|---------------|
| `C`    | Clear Path     | 1             |
| `S`    | Sandy Path     | 3             |
| `R`    | Rocky Mountain | 5             |
| `P`    | Start (Clear)  | 1             |
| `F`    | Goal  (Clear)  | 1             |

**Actions:** The robot may move to any of the 4 orthogonally adjacent cells — **Up, Down, Left, or Right**. Diagonal movement is not allowed.

**Initial State:** The cell marked `P` — position `(0, 0)`.  
**Goal State:** The cell marked `F` — position `(3, 6)`.

**Grid used in all experiments:**
```
 P  C  C  C  C  C  C
 C  C  S  S  S  S  C
 C  C  S  R  R  S  C
 C  C  S  R  R  S  F   <- Goal at (3,6)
 C  C  S  R  R  S  C
 C  C  S  S  S  S  C
 C  C  C  C  C  C  C
```

The central block of `R` and `S` terrain forms a high-cost zone. The cheap path goes right along row 0 then down column 6 (all `C`). Greedy's h(n) lures it straight through the expensive terrain.

---

### 1.2 Heuristics Formulation

Two admissible heuristic functions estimate the cost from node $n$ to the goal $G$:

**h1 — Manhattan Distance:**
$$h_1(n) = |n_{row} - G_{row}| + |n_{col} - G_{col}|$$

**h2 — Euclidean Distance:**
$$h_2(n) = \sqrt{(n_{row} - G_{row})^2 + (n_{col} - G_{col})^2}$$

---

### 1.3 Admissibility Proof

A heuristic $h$ is **admissible** iff $h(n) \le h^*(n)$ for all $n$, where $h^*(n)$ is the true minimum cost from $n$ to the goal.

**Proof for h1 (Manhattan Distance):**  
The minimum possible cost to traverse any single cell is **1** (a `C` cell). Since the robot can only move in 4 directions, the minimum number of steps to reach the goal from $n$ is exactly the Manhattan distance. Even if all cells were `C`, the true cost equals the step count, i.e., $h^*(n) \ge h_1(n)$. Any `S` or `R` terrain only increases the true cost further. Therefore $h_1$ never overestimates — **admissible**. ✅

**Proof for h2 (Euclidean Distance):**  
By the triangle inequality, the straight-line distance between two points is always less than or equal to the sum of the absolute differences along each axis:
$$h_2(n) = \sqrt{\Delta r^2 + \Delta c^2} \le |\Delta r| + |\Delta c| = h_1(n)$$
Since $h_2(n) \le h_1(n)$ and $h_1(n) \le h^*(n)$, by transitivity $h_2(n) \le h^*(n)$ — **admissible**. ✅

---

## Part 2: Source Code Overview

The implementation is in **`multi_terrain.py`**, structured as follows and matching the style of `main.py` and `informed.py`:

| Component | Description |
|-----------|-------------|
| `terrain_costs` dict | Maps each cell symbol to its movement cost |
| `grid` | 7×7 2D list defining the environment |
| `find_start(grid)` | Scans for the `'P'` cell |
| `find_goal(grid)` | Scans for the `'F'` cell |
| `get_neighbors(row, col, grid)` | Returns the 4 valid adjacent cells |
| `reconstruct_path(parent, start, goal)` | Traces back through the `parent` dict |
| `h1(node, goal)` | Manhattan distance heuristic |
| `h2(node, goal)` | Euclidean distance heuristic |
| `GreedyBestFirst(grid, heuristic_func)` | Greedy search — uses only `h(n)` |
| `AStar(grid, heuristic_func)` | A* search — uses `f(n) = g(n) + h(n)` |

**Key design decisions:**
- Both algorithms accept the **grid and heuristic function as parameters**, making them generic and reusable.
- A `counter` is used as a tie-breaker in the priority queue to avoid comparing tuples of nodes.
- A* uses a `g_cost` dict to track the best known cost to each node, and skips stale priority queue entries (`if g_cost[current] < g: continue`).
- Nodes expanded, path, steps, and true cost are all tracked and printed for comparison.

---

## Part 3: In-depth Performance Analysis

### 3.1 Greedy vs. A* — Why Greedy Fails

**Test case output (same grid, both using h1):**

| Algorithm | Path | Steps | True Cost | Nodes Expanded |
|-----------|------|-------|-----------|----------------|
| Greedy    | `(0,0)→(1,0)→(2,0)→(3,0)→(3,1)→(3,2)→(3,3)→(3,4)→(3,5)→(3,6)` | 9 | **21** | 10 |
| A*        | `(0,0)→(0,1)→(0,2)→(0,3)→(0,4)→(0,5)→(0,6)→(1,6)→(2,6)→(3,6)` | 9 | **9**  | 16 |

**Greedy path (marked with `*`):**
```
 P  C  C  C  C  C  C
 *  C  S  S  S  S  C
 *  C  S  R  R  S  C
 *  *  *  *  *  *  F   <- cuts through S and R terrain
 C  C  S  R  R  S  C
 C  C  S  S  S  S  C
 C  C  C  C  C  C  C
```

**A\* path (marked with `*`):**
```
 P  *  *  *  *  *  *
 C  C  S  S  S  S  *
 C  C  S  R  R  S  *
 C  C  S  R  R  S  F   <- goes along cheap perimeter
 C  C  S  R  R  S  C
 C  C  S  S  S  S  C
 C  C  C  C  C  C  C
```

**Explanation:**  
Greedy only evaluates $h(n)$ — the estimated distance to the goal. From `(3,0)`, the neighbor `(3,1)` has $h_1 = 5$, while `(4,0)` has $h_1 = 7$. Greedy picks `(3,1)` because it is geographically closer, and continues right across row 3 through `S` and `R` cells. It never "knows" the accumulated cost $g(n)$ is skyrocketing.

A* evaluates $f(n) = g(n) + h(n)$. After a few steps, A* notices that going right along row 0 (all `C`, cost=1 each) keeps $g(n)$ very low. Moving through `S` or `R` cells would make $g(n)$ spike, raising $f(n)$ above the cheap perimeter path. A* correctly avoids the expensive zone and finds the optimal cost of **9**.

> **Greedy saves nodes expanded (10 vs 16), but pays a cost penalty of +12 — a classic greedy short-sightedness failure.**

---

### 3.2 A*(h1) vs. A*(h2) — Comparing Heuristics

**Test case output:**

| Heuristic | Path | True Cost | Nodes Expanded |
|-----------|------|-----------|----------------|
| h1 (Manhattan) | top-row + right-column | **9** | **16** |
| h2 (Euclidean) | top-row + right-column | **9** | **16** |

Both heuristics find the **same optimal path** at the **same cost**, because both are admissible.

On this particular 7×7 grid, both heuristics also expand the same number of nodes (16). This happens because the optimal path is "unambiguous" — the top-row + right-column route dominates so clearly that both $h_1$ and $h_2$ agree on which nodes to prioritize at every step.

**Why h1 is theoretically more informed:**  
Since $h_2(n) \le h_1(n)$ always (Euclidean ≤ Manhattan), $h_1$ gives a **tighter lower bound** on the true cost. A tighter bound means fewer nodes receive an artificially low $f$-score and get expanded unnecessarily. On **larger or denser grids** with many competing paths of similar cost, A*(h1) will expand fewer nodes than A*(h2), because h1 can better distinguish which paths are promising.

**Conclusion:** Both heuristics are correct and find optimal paths. h1 is the **preferred heuristic** for 4-directional grid navigation since it is strictly more informed than h2 in this movement model.

---

## Part 4: Discussion

**Why this problem?**  
Multi-terrain robot navigation is an ideal benchmark because it combines both a **search problem** (finding a path) and a **cost optimization problem** (minimizing terrain traversal costs). This directly highlights the fundamental difference between Greedy and A*.

**Heuristic choices:**  
- **h1 (Manhattan)** was chosen as the primary heuristic because it perfectly matches the movement model (4-directional, unit steps) and provides the tightest admissible bound.
- **h2 (Euclidean)** was added as a second heuristic to satisfy the "two different heuristics" requirement and to demonstrate that admissibility alone does not guarantee equal performance — informativeness matters too.

**Data Structures used:**
- `heapq` (min-heap / priority queue) — efficiently extracts the node with lowest priority.
- `dict` (`g_cost`, `parent`) — O(1) lookup for best-known costs and path reconstruction.
- `set` (`visited` in Greedy) — O(1) membership test to avoid re-expanding nodes.

**Key takeaway:**  
Greedy Best-First Search is fast but **not optimal** in weighted environments. A* is optimal (when h is admissible) and only marginally slower on this problem. The heuristic quality (informativeness) affects efficiency but not correctness.
