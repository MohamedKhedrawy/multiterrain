import heapq
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
]

# =============================================================================
# Helper Functions
# =============================================================================

def find_start(grid):
    """Find the 'P' (start) cell."""
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 'P':
                return (i, j)
    return None

def find_goal(grid):
    """Find the 'F' (goal) cell."""
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 'F':
                return (i, j)
    return None

def get_neighbors(row, col, grid):
    """Return valid adjacent cells: Up, Down, Left, Right."""
    neighbors = []
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
        nr, nc = row + dr, col + dc
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]):
            neighbors.append((nr, nc))
    return neighbors

def reconstruct_path(parent, start, goal):
    """Trace back from goal to start using parent dict."""
    path, current = [], goal
    while current in parent:
        path.append(current)
        current = parent[current]
    path.append(start)
    path.reverse()
    return path

def print_grid(grid, path=None):
    """Print the grid, marking path cells with '*'."""
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
    print_grid(grid, path)

# =============================================================================
# Heuristic Functions
# =============================================================================

def h1(node, goal):
    """
    h1: Manhattan Distance
    h1(n) = |n.row - goal.row| + |n.col - goal.col|
    Admissible: min step cost = 1, so h1 <= true cost h*(n).
    """
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def h2(node, goal):
    """
    h2: Euclidean Distance
    h2(n) = sqrt((n.row - goal.row)^2 + (n.col - goal.col)^2)
    Admissible: h2 <= h1 <= h*(n) by the triangle inequality.
    """
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

# =============================================================================
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
    return path, cost, nodes_expanded

# =============================================================================
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
    return path, g_cost[goal_node], nodes_expanded

# =============================================================================
# Main
# =============================================================================

if __name__ == '__main__':
    print("=" * 55)
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
        print("  Both found the same cost path.\n")

    # --- Experiment 2: A*(h1) vs A*(h2) ---
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
        print("  Both expand the same number of nodes on this grid.")
