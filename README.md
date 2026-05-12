# 🟡 Pac-Man AI Pathfinding Project

## 📌 Overview

This project implements a simplified Pac-Man navigation system using classical AI search algorithms. The agent must navigate a 2D grid to reach the food while avoiding walls and ghost boundaries.

The project demonstrates the behavior and differences between:

- Breadth-First Search (BFS)
- Depth-First Search (DFS)
- Dijkstra’s Algorithm

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
- Minimize number of steps (BFS)
- Minimize total cost (Dijkstra)
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

## 🟢 Breadth-First Search (BFS)

- Uses Queue (FIFO)
- Explores level by level
- Guarantees shortest path (in steps)

---

## 🔴 Depth-First Search (DFS)

- Uses Stack (LIFO)
- Explores deeply before backtracking
- Does NOT guarantee optimal path

---

## 🟡 Dijkstra’s Algorithm

- Uses Priority Queue (Min Heap)
- Considers movement cost (weights)
- Finds least-cost path

---

# 🧠 Data Structures Used

- Queue → BFS
- Stack → DFS
- Priority Queue (heapq) → Dijkstra
- Set → visited nodes
- Dictionary → distance & parent tracking

---

# 📊 Performance Analysis

## 🔹 Optimality

| Algorithm | Result                |
| --------- | --------------------- |
| BFS       | Shortest path (steps) |
| DFS       | Not optimal           |
| Dijkstra  | Least-cost path       |

---

## 🔹 Efficiency

| Algorithm | Behavior                              |
| --------- | ------------------------------------- |
| BFS       | Fast in unweighted grids              |
| DFS       | May explore unnecessary paths         |
| Dijkstra  | Slower but more accurate with weights |

---

## 🧠 Key Observations

- BFS and Dijkstra produce the same result in unweighted environments
- DFS may produce longer paths depending on traversal order
- Dijkstra becomes more effective when weights are introduced

---

# 🧪 Example Output

```
=== BFS ===
Path: [(0,0), (1,0), (2,0), (3,0)]
Steps: 3

=== DFS ===
Path: [...]
Steps: 9

=== Dijkstra ===
Path: [...]
Cost: 22
```

---

# 🧠 Discussion

- BFS guarantees shortest path in terms of steps
- DFS explores deeply and is not optimal
- Dijkstra considers cost and chooses the most efficient path

### ❓ Why BFS = Dijkstra sometimes?

Because all movements have equal cost (unweighted grid)

### ❓ Why Dijkstra is important?

It handles weighted environments and avoids expensive paths

---

# 🛠️ Project Structure

```
Pacman-AI/
│
├── main.py
├── README.md
```

---

# 🚀 How to Run

```bash
python main.py
```

---

# 📌 Future Improvements

- GUI using Tkinter or Pygame
- Path animation
- Random grid generator
- A\* algorithm implementation

---

# 👨‍💻 Author

Mohamed Ekramy
Moahmed El Motaz
Mahmoud Sayed
