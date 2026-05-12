# 🟡 Problem Formulation & PEAS (Informed Search)

## 🧠 PEAS Description

### 🔹 Performance Measure

- Reach the food (F) successfully
- Minimize heuristic distance to goal (Greedy, Hill Climbing)
- Minimize total path cost (A*, Genetic Algorithm)
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

The `informed.py` script implements four informed search algorithms:

## 🔹 Greedy Best-First Search

- Evaluates states using only a heuristic function (Manhattan distance)
- Fast but not guaranteed to find the shortest or cheapest path

## 🔹 A* Search

- Evaluates states by combining cost so far and heuristic distance
- Finds the guaranteed least-cost path

## 🔹 Hill Climbing

- Evaluates immediate neighbors and picks the one with the best heuristic
- Fast and memory-efficient but can get stuck in local optima

## 🔹 Genetic Algorithm

- Evaluates paths using a fitness function that considers path cost
- Evolves solutions over multiple generations using crossover and mutation

---

## 🧠 Data Structures Used

- Priority Queue (heapq) → Greedy Best-First, A*
- Dictionary → distance tracking
- Set → visited nodes
- Parent mapping → path reconstruction
- Lists → population and chromosome tracking (Genetic Algorithm)

---

## ✨ Clean Code Practices

- Each algorithm is implemented in a separate function
- Meaningful variable names
- Reusable helper functions (e.g., get_neighbors, heuristic)
- Comments added to explain logic

# 📊 Performance Analysis

## 🔹 Optimality

| Algorithm            | Optimality                               |
| -------------------- | ---------------------------------------- |
| Greedy Best-First    | Not guaranteed optimal                   |
| A* Search            | Finds least-cost path (Optimal)          |
| Hill Climbing        | Not guaranteed optimal (Local search)    |
| Genetic Algorithm    | Not guaranteed optimal (Heuristic search)|

---

## 🔹 Efficiency

| Algorithm            | Efficiency                                                  |
| -------------------- | ----------------------------------------------------------- |
| Greedy Best-First    | Fast but can explore suboptimal paths                       |
| A* Search            | Balanced, avoids unnecessary exploration while being optimal|
| Hill Climbing        | Highly efficient in terms of memory and speed               |
| Genetic Algorithm    | Slower due to generation processing and fitness evaluation  |

---

## 🧠 Observations

- A* Search reliably finds the lowest cost path (Cost: 22) in this environment.
- Hill Climbing quickly finds a path (Cost: 25) but misses the optimal route because it doesn't look far ahead.
- The Genetic Algorithm is capable of finding the optimal path given enough generations, demonstrating its global search capabilities.
