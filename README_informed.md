# 🟡 Pac-Man AI Pathfinding Project (Informed Search)

## 📌 Overview

This project implements a simplified Pac-Man navigation system using classical AI informed search algorithms. The agent must navigate a 2D grid to reach the food while avoiding walls and ghost boundaries.

The project demonstrates the behavior and differences between:

- Greedy Best-First Search
- A* Search
- Hill Climbing Algorithm
- Genetic Algorithm

---

# 🎯 Problem Description

The environment is represented as a 2D grid where:

| Symbol | Meaning         |
| ------ | --------------- |
| P      | Start (Pac-Man) |
| F      | Goal (Food)     |
| 0      | Free cell       |
| 1      | Wall (blocked)  |
| G      | Ghost (blocked) |

The agent can move in four directions:

- Up
- Down
- Left
- Right

---

# 🧠 Problem Formulation & PEAS

## 🔹 Performance Measure

- Reach the goal (F)
- Minimize heuristic distance to goal (Greedy, Hill Climbing)
- Minimize total path cost (A*, Genetic Algorithm)
- Avoid walls and ghosts

---

## 🔹 Environment

- 2D Grid
- Fully Observable
- Deterministic
- Static
- Discrete

---

## 🔹 Actuators

- Move Up / Down / Left / Right

---

## 🔹 Sensors

- Current position (row, col)
- Neighboring cells
- Cell type (0, 1, G, F)

---

## 🧩 State Space

- Represented as a 2D array (grid)
- Each state = (row, col)

---

## 🎯 States & Actions

### Initial State

- Position of Pac-Man (P)

### Goal State

- Position of Food (F)

### Valid Actions

- Move Up, Down, Left, Right
- Only if:
  - Inside grid
  - Not a wall
  - Not a ghost

---

# ⚙️ Algorithms Implemented

## 🟢 Greedy Best-First Search

- Uses a Priority Queue based on heuristic distance to the goal
- Explores paths that seem closer to the goal
- Does NOT guarantee optimal path (can be misled by obstacles)

---

## 🟡 A* Search

- Uses a Priority Queue based on f(n) = g(n) + h(n)
- Considers both accumulated path cost and heuristic distance to goal
- Guarantees optimal (least-cost) path

---

## 🔴 Hill Climbing Algorithm

- Local search algorithm
- Always moves to the neighbor with the best heuristic value
- Fast but can get stuck in local optima (though implemented with sideways moves to traverse plateaus)

---

## 🧬 Genetic Algorithm

- Evolutionary search algorithm
- Maintains a population of paths, evaluates them based on a fitness function (minimizing cost and distance)
- Uses crossover and mutation to evolve better paths over generations
- Not guaranteed to find optimal solution but explores the space globally

---

# 🧠 Data Structures Used

- Priority Queue (heapq) → Greedy Best-First, A*
- Set → visited nodes
- Dictionary → distance & parent tracking
- Lists → population and chromosome tracking (Genetic Algorithm)

---

# 📊 Performance Analysis

## 🔹 Optimality

| Algorithm            | Result                                     |
| -------------------- | ------------------------------------------ |
| Greedy Best-First    | Fast, but not guaranteed optimal           |
| A* Search            | Least-cost path (Optimal)                  |
| Hill Climbing        | Not optimal (prone to local optima)        |
| Genetic Algorithm    | Not guaranteed optimal, but can find good paths |

---

## 🔹 Efficiency

| Algorithm            | Behavior                                        |
| -------------------- | ----------------------------------------------- |
| Greedy Best-First    | Fast execution, but can explore sub-optimal paths|
| A* Search            | Balanced efficiency, avoids unnecessary paths   |
| Hill Climbing        | Very fast, memory efficient                     |
| Genetic Algorithm    | Slower due to population evaluation over generations |

---

## 🧠 Key Observations

- A* successfully combines the speed of Greedy Search with the optimality of Dijkstra's algorithm.
- Hill Climbing struggles with complex obstacles but is extremely fast for direct paths.
- Genetic Algorithms offer a completely different heuristic approach, optimizing entire paths rather than exploring node by node.

---

# 🧪 Example Output

```
=== Greedy Best-First ===
Path: [(0, 0), (1, 0), (2, 0), (3, 0)]
Steps: 3

=== A* Search ===
Path: [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1), (3, 1), (3, 0)]
Cost: 22

=== Hill Climbing ===
Path: [(0, 0), (1, 0), (2, 0), (3, 0)]
Steps: 3

=== Genetic Algorithm ===
Path: [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1), (3, 1), (3, 0)]
Steps: 7
Cost: 22
Status: Goal reached!
```

---

# 🛠️ Project Structure

```
Pacman-AI/
│
├── main.py (Uninformed Search)
├── informed.py (Informed Search)
├── README.md (Uninformed Documentation)
├── README_informed.md (Informed Documentation)
```

---

# 🚀 How to Run

```bash
python informed.py
```
