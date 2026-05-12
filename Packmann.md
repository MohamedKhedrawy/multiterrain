# 🟡 Problem Formulation & PEAS

## 🧠 PEAS Description

### 🔹 Performance Measure

- Reach the food (F) successfully
- Minimize number of steps (BFS)
- Minimize total cost (Dijkstra)
- Avoid invalid cells (walls and ghosts)

---

### 🔹 Environment

- Type: 2D Grid
- Observable: Fully Observable
- Deterministic: Yes
- Static: Yes
- Discrete: Yes

---

### 🔹 Actuators

- Move Up
- Move Down
- Move Left
- Move Right

---

### 🔹 Sensors

- Current position (row, col)
- Neighboring cells
- Type of each cell (0, 1, G, F)

---

## 🧩 State Space

- Represented as a 2D Grid (Array)
- Each state is a position:

  State = (row, col)

---

## 🎯 States & Actions

### 🔹 Initial State

- Position of Pac-Man (P)

### 🔹 Goal State

- Position of Food (F)

### 🔹 Valid Actions

- Up → (row - 1, col)
- Down → (row + 1, col)
- Left → (row, col - 1)
- Right → (row, col + 1)

Actions are valid only if:

- Inside grid bounds
- Not a wall (1)
- Not a ghost (G)

# 💻 Source Code Description

The project implements three search algorithms:

## 🔹 Breadth-First Search (BFS)

- Uses a Queue (FIFO)
- Explores level by level
- Guarantees shortest path in terms of steps

## 🔹 Depth-First Search (DFS)

- Uses a Stack (LIFO)
- Explores deeply before backtracking
- Does not guarantee optimal solution

## 🔹 Dijkstra Algorithm

- Uses a Priority Queue (Min Heap)
- Considers path cost (weights)
- Finds the least-cost path

---

## 🧠 Data Structures Used

- Queue → BFS
- Stack → DFS
- Priority Queue (heapq) → Dijkstra
- Dictionary → distance tracking
- Set → visited nodes
- Parent mapping → path reconstruction

---

## ✨ Clean Code Practices

- Each algorithm is implemented in a separate function
- Meaningful variable names
- Reusable helper functions (e.g., get_neighbors)
- Comments added to explain logic

# 📊 Performance Analysis

## 🔹 Optimality

| Algorithm | Optimality                     |
| --------- | ------------------------------ |
| BFS       | Finds shortest path (in steps) |
| DFS       | Not guaranteed optimal         |
| Dijkstra  | Finds least-cost path          |

---

## 🔹 Efficiency

| Algorithm | Efficiency                                                  |
| --------- | ----------------------------------------------------------- |
| BFS       | Efficient for unweighted grids                              |
| DFS       | May explore unnecessary paths                               |
| Dijkstra  | Slower due to priority queue but more accurate with weights |

---

## 🧠 Observations

- BFS and Dijkstra may produce the same result in unweighted grids
- DFS often produces longer paths
- Dijkstra outperforms BFS when weights are introduced
