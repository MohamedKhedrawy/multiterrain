import nbformat as nbf
nb = nbf.v4.new_notebook()
cells = []

cells.append(nbf.v4.new_markdown_cell(r"""# Multi-Terrain Robot Navigation — Analysis

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

The central block of `R` and `S` terrain forms a high-cost zone. The cheap path goes right along row 0 then down column 6 (all `C`). Greedy's h(n) lures it straight through the expensive terrain."""))

cells.append(nbf.v4.new_code_cell(r"""import heapq
import math

# =============================================================================
# Problem Formulation & Environment Setup
# =============================================================================
# State Space : Every cell (row, col) in a 2D grid is a possible state.
# Actions     : Move Up, Down, Left, or Right to an adjacent cell.
# Initial State: Cell marked 'P'
# Goal State  : Cell marked 'F'
#
# Terrain Types and Movement Costs:
#   'C' = Clear Path      -> Cost = 1
#   'S' = Sandy Path      -> Cost = 3
#   'R' = Rocky Mountain  -> Cost = 5
#   'P' = Start           -> Cost = 1 (treated as Clear)
#   'F' = Goal            -> Cost = 1 (treated as Clear)
# =============================================================================

terrain_costs = {'C': 1, 'S': 3, 'R': 5, 'P': 1, 'F': 1}

# 7x7 grid with all three terrain types — C (Clear), S (Sandy), R (Rocky).
# F (goal) is at the middle-right (3,6).
# The direct path toward F cuts through S and R terrain (very expensive).
# The cheap path: go right along the top row, then down column 6 (all C).
# Greedy follows h(n) and gets lured down through S/R toward F.
# A* accounts for g(n) and discovers the cheap perimeter route.
grid = [
    ['P', 'C', 'C', 'C', 'C', 'C', 'C'],
    ['C', 'C', 'S', 'S', 'S', 'S', 'C'],
    ['C', 'C', 'S', 'R', 'R', 'S', 'C'],
    ['C', 'C', 'S', 'R', 'R', 'S', 'F'],
    ['C', 'C', 'S', 'R', 'R', 'S', 'C'],
    ['C', 'C', 'S', 'S', 'S', 'S', 'C'],
    ['C', 'C', 'C', 'C', 'C', 'C', 'C'],
]"""))

cells.append(nbf.v4.new_markdown_cell(r"""---

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
Since $h_2(n) \le h_1(n)$ and $h_1(n) \le h^*(n)$, by transitivity $h_2(n) \le h^*(n)$ — **admissible**. ✅"""))

cells.append(nbf.v4.new_code_cell(r"""# =============================================================================
# Heuristic Functions
# =============================================================================

def h1(node, goal):
    '''
    h1: Manhattan Distance
    h1(n) = |n.row - goal.row| + |n.col - goal.col|
    Admissible: min step cost = 1, so h1 <= true cost h*(n).
    '''
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def h2(node, goal):
    '''
    h2: Euclidean Distance
    h2(n) = sqrt((n.row - goal.row)^2 + (n.col - goal.col)^2)
    Admissible: h2 <= h1 <= h*(n) by the triangle inequality.
    '''
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)"""))

cells.append(nbf.v4.new_markdown_cell(r"""---

## Part 2: Source Code Overview

The implementation is structured as follows:

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
- Nodes expanded, path, steps, and true cost are all tracked and printed for comparison."""))

cells.append(nbf.v4.new_code_cell(r"""# =============================================================================
# Helper Functions
# =============================================================================

def find_start(grid):
    '''Find the 'P' (start) cell.'''
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 'P':
                return (i, j)
    return None

def find_goal(grid):
    '''Find the 'F' (goal) cell.'''
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 'F':
                return (i, j)
    return None

def get_neighbors(row, col, grid):
    '''Return valid adjacent cells: Up, Down, Left, Right.'''
    neighbors = []
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
        nr, nc = row + dr, col + dc
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]):
            neighbors.append((nr, nc))
    return neighbors

def reconstruct_path(parent, start, goal):
    '''Trace back from goal to start using parent dict.'''
    path, current = [], goal
    while current in parent:
        path.append(current)
        current = parent[current]
    path.append(start)
    path.reverse()
    return path

def print_grid(grid, path=None):
    '''Print the grid, marking path cells with '*'.'''
    path_set = set(path) if path else set()
    symbols = {1: 'R', 3: 'S', 5: 'R'}  # not used, kept for reference
    for i, row in enumerate(grid):
        line = ""
        for j, cell in enumerate(row):
            if (i, j) in path_set and cell not in ('P', 'F'):
                line += " * "
            else:
                line += f" {cell} "
        print(line)
    print()

def print_results(label, path, cost, nodes, grid):
    print(f"  Path:           {path}")
    print(f"  Steps:          {len(path) - 1}")
    print(f"  Total Cost:     {cost}")
    print(f"  Nodes Expanded: {nodes}")
    print_grid(grid, path)"""))

cells.append(nbf.v4.new_code_cell(r"""# =============================================================================
# Algorithm 1: Greedy Best-First Search
# Evaluation: f(n) = h(n)  — ignores accumulated cost g(n).
# NOT guaranteed to be optimal.
# =============================================================================

def GreedyBestFirst(grid, heuristic_func=h1):
    start = find_start(grid)
    goal  = find_goal(grid)

    pq, counter = [], 0
    heapq.heappush(pq, (heuristic_func(start, goal), counter, start))

    visited = {start}
    parent  = {}
    goal_node, nodes_expanded = None, 0

    while pq:
        _, _, current = heapq.heappop(pq)
        row, col = current
        nodes_expanded += 1

        if grid[row][col] == 'F':
            goal_node = current
            break

        for nb in get_neighbors(row, col, grid):
            if nb not in visited:
                visited.add(nb)
                parent[nb] = current
                counter += 1
                heapq.heappush(pq, (heuristic_func(nb, goal), counter, nb))

    if goal_node is None:
        print("  No path found.")
        return None, None, None

    path = reconstruct_path(parent, start, goal_node)
    cost = sum(terrain_costs[grid[r][c]] for r, c in path[1:])
    return path, cost, nodes_expanded"""))

cells.append(nbf.v4.new_code_cell(r"""# =============================================================================
# Algorithm 2: A* Search
# Evaluation: f(n) = g(n) + h(n)  — balances cost-so-far with heuristic.
# Guaranteed OPTIMAL when h is admissible.
# Heuristic function is passed as a parameter (h1 or h2).
# =============================================================================

def AStar(grid, heuristic_func=h1):
    start = find_start(grid)
    goal  = find_goal(grid)

    pq, counter = [], 0
    heapq.heappush(pq, (heuristic_func(start, goal), 0, counter, start))

    g_cost = {start: 0}   # g(n): best known cost from start to n
    parent = {}
    goal_node, nodes_expanded = None, 0

    while pq:
        f, g, _, current = heapq.heappop(pq)
        row, col = current
        nodes_expanded += 1

        if grid[row][col] == 'F':
            goal_node = current
            break

        # Outdated entry — a cheaper path was already found
        if g_cost.get(current, float('inf')) < g:
            continue

        for nb in get_neighbors(row, col, grid):
            nr, nc   = nb
            new_g    = g + terrain_costs[grid[nr][nc]]
            if new_g < g_cost.get(nb, float('inf')):
                g_cost[nb] = new_g
                parent[nb] = current
                counter   += 1
                heapq.heappush(pq, (new_g + heuristic_func(nb, goal), new_g, counter, nb))

    if goal_node is None:
        print("  No path found.")
        return None, None, None

    path = reconstruct_path(parent, start, goal_node)
    return path, g_cost[goal_node], nodes_expanded"""))

cells.append(nbf.v4.new_markdown_cell(r"""---

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

> **Greedy saves nodes expanded (10 vs 16), but pays a cost penalty of +12 — a classic greedy short-sightedness failure.**"""))


cells.append(nbf.v4.new_code_cell(r"""print("=" * 55)
print("  Multi-Terrain Robot Navigation")
print("=" * 55)
print("\nGrid layout (C=Clear, S=Sandy, R=Rocky):")
print_grid(grid)

# --- Experiment 1: Greedy vs A* ---
print("--- Experiment 1: Greedy vs A* (both using h1: Manhattan) ---")

print("\n>>> Greedy Best-First:")
g_path, g_cost, g_exp = GreedyBestFirst(grid, h1)
print_results("Greedy", g_path, g_cost, g_exp, grid)

print(">>> A* Search (h1):")
a_path, a_cost, a_exp = AStar(grid, h1)
print_results("A*", a_path, a_cost, a_exp, grid)

print(f"  [Result] Greedy cost={g_cost} | A* cost={a_cost}")
if g_cost > a_cost:
    print(f"  Greedy is SUBOPTIMAL — A* saves {g_cost - a_cost} cost unit(s).\n")
else:
    print("  Both found the same cost path.\n")"""))

cells.append(nbf.v4.new_markdown_cell(r"""---

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

**Conclusion:** Both heuristics are correct and find optimal paths. h1 is the **preferred heuristic** for 4-directional grid navigation since it is strictly more informed than h2 in this movement model."""))

cells.append(nbf.v4.new_code_cell(r"""# --- Experiment 2: A*(h1) vs A*(h2) ---
print("--- Experiment 2: A*(h1: Manhattan) vs A*(h2: Euclidean) ---")

print("\n>>> A* with h1 (Manhattan):")
p1, c1, e1 = AStar(grid, h1)
print_results("A*(h1)", p1, c1, e1, grid)

print(">>> A* with h2 (Euclidean):")
p2, c2, e2 = AStar(grid, h2)
print_results("A*(h2)", p2, c2, e2, grid)

print(f"  [Result] A*(h1): cost={c1}, nodes={e1} | A*(h2): cost={c2}, nodes={e2}")
if e1 < e2:
    print(f"  h1 is MORE INFORMED: expands {e2 - e1} fewer node(s).")
elif e2 < e1:
    print(f"  h2 is MORE INFORMED: expands {e1 - e2} fewer node(s).")
else:
    print("  Both expand the same number of nodes on this grid.")"""))

cells.append(nbf.v4.new_markdown_cell(r"""---

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
Greedy Best-First Search is fast but **not optimal** in weighted environments. A* is optimal (when h is admissible) and only marginally slower on this problem. The heuristic quality (informativeness) affects efficiency but not correctness."""))

nb['cells'] = cells
with open('multi_terrain.ipynb', 'w') as f:
    nbf.write(nb, f)
