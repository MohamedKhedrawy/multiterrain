import heapq
import random

grid = [
 ['P', 0, 0, 0],
 [0, 1, 0, 0],
 [0, 0, 0, 0],
 ['F', 0, 0, 0]
]

weights = {
    (0,1): 5,
    (0,2): 2,
    (0,3): 5,
    (1,0): 7,
    (1,1): 5,
    (1,2): 2,
    (1,3): 5,
    (2,0): 17,
    (2,1): 5,
    (2,2): 2,
    (2,3): 5,
    (3,0): 1,
    (3,1): 5,
    (3,2): 7,
    (3,3): 5,
}

def find_start(grid):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 'P':
                return (i,j)

def find_goal(grid):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 'F':
                return (i,j)

def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def get_neighbors(row, col, grid):
    directions = [(1,0), (0,1), (0,-1), (-1,0)]
    grid = grid
    neighbors = []
    for dr, dc in directions:
        new_row = row + dr
        new_col = col + dc
        if 0 <= new_row < len(grid):
            if 0 <= new_col < len(grid[0]):
                if grid[new_row][new_col] != 1:
                    if grid[new_row][new_col] != 'G':
                        neighbors.append((new_row,new_col))
    return neighbors

def GreedyBestFirst(vertex):
    goal_pos = find_goal(grid)
    if not goal_pos:
        print("No goal found")
        return

    pq = []
    heapq.heappush(pq, (heuristic(vertex, goal_pos), vertex))

    visited = set()
    visited.add(vertex)
    parent = {}
    goal = None

    while pq:
        h, current_vertex = heapq.heappop(pq)
        row, col = current_vertex

        if grid[row][col] == 'F':
            goal = current_vertex
            break

        for neighbor in get_neighbors(row, col, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current_vertex
                heapq.heappush(pq, (heuristic(neighbor, goal_pos), neighbor))

    if goal is None:
        print("No path found")
        return

    path = []
    current_vertex = goal

    while current_vertex in parent:
        path.append(current_vertex)
        current_vertex = parent[current_vertex]

    path.append(vertex)
    path.reverse()

    print("Path:", path)
    print("Steps:", len(path)-1)
    return path


def AStar(vertex):
    goal_pos = find_goal(grid)
    if not goal_pos:
        print("No goal found")
        return

    pq = []
    heapq.heappush(pq, (heuristic(vertex, goal_pos), 0, vertex)) # (f, g, node)

    distance = {vertex: 0}
    parent = {}
    goal = None

    while pq:
        f, g, current_vertex = heapq.heappop(pq)
        row, col = current_vertex

        if grid[row][col] == 'F':
            goal = current_vertex
            break
            
        if distance.get(current_vertex, float('inf')) < g:
            continue

        for neighbor in get_neighbors(row, col, grid):
            if neighbor in weights:
                cost_to_neighbor = weights[neighbor]
            else:
                cost_to_neighbor = 1
                
            new_cost = g + cost_to_neighbor
            
            if neighbor not in distance or new_cost < distance[neighbor]:
                distance[neighbor] = new_cost
                parent[neighbor] = current_vertex
                f_score = new_cost + heuristic(neighbor, goal_pos)
                heapq.heappush(pq, (f_score, new_cost, neighbor))

    if goal is None:
        print("No path found")
        return

    path = []
    current_vertex = goal

    while current_vertex in parent:
        path.append(current_vertex)
        current_vertex = parent[current_vertex]

    path.append(vertex)
    path.reverse()

    print("Path:", path)
    print("Cost:", distance[goal])

    return path


def HillClimbing(vertex):
    goal_pos = find_goal(grid)
    current_vertex = vertex
    path = [current_vertex]
    visited = {current_vertex}

    while True:
        row, col = current_vertex
        if grid[row][col] == 'F':
            print("Path:", path)
            print("Steps:", len(path)-1)
            return path

        neighbors = get_neighbors(row, col, grid)
        best_neighbor = None
        best_h = float('inf')

        for neighbor in neighbors:
            if neighbor not in visited:
                h = heuristic(neighbor, goal_pos)
                if h < best_h:
                    best_h = h
                    best_neighbor = neighbor

        if best_neighbor is None or best_h > heuristic(current_vertex, goal_pos):
            print("Stuck in local optimum")
            print("Path:", path)
            print("Steps:", len(path)-1)
            return path

        current_vertex = best_neighbor
        visited.add(current_vertex)
        path.append(current_vertex)


def GeneticAlgorithm(vertex):
    goal_pos = find_goal(grid)
    if not goal_pos:
        print("No goal found")
        return

    directions = [(1,0), (0,1), (0,-1), (-1,0)]
    pop_size = 50
    generations = 100
    path_length = 20

    population = [[random.choice(directions) for _ in range(path_length)] for _ in range(pop_size)]

    def evaluate(chromosome):
        row, col = vertex
        path = [(row, col)]
        cost = 0
        for dr, dc in chromosome:
            if grid[row][col] == 'F':
                break
            new_row = row + dr
            new_col = col + dc
            if 0 <= new_row < len(grid) and 0 <= new_col < len(grid[0]) and grid[new_row][new_col] != 1 and grid[new_row][new_col] != 'G':
                step_cost = weights.get((new_row, new_col), 1)
                cost += step_cost
                row, col = new_row, new_col
            path.append((row, col))
        
        dist = heuristic((row, col), goal_pos)
        if grid[row][col] == 'F':
            return 1000 - cost
        return -dist - cost * 0.1

    best_path = None
    best_cost = None
    
    for gen in range(generations):
        fitnesses = [(evaluate(chrom), chrom) for chrom in population]
        fitnesses.sort(key=lambda x: x[0], reverse=True)
        
        best_fitness, best_chrom = fitnesses[0]
        
        row, col = vertex
        current_best_path = [(row, col)]
        for dr, dc in best_chrom:
            if grid[row][col] == 'F':
                break
            new_row = row + dr
            new_col = col + dc
            if 0 <= new_row < len(grid) and 0 <= new_col < len(grid[0]) and grid[new_row][new_col] != 1 and grid[new_row][new_col] != 'G':
                row, col = new_row, new_col
            if current_best_path[-1] != (row, col):
                current_best_path.append((row, col))
            
        if grid[row][col] == 'F':
            current_cost = sum(weights.get(node, 1) for node in current_best_path[1:])
            if best_cost is None or current_cost < best_cost:
                best_cost = current_cost
                best_path = current_best_path
        else:
            if best_path is None:
                best_path = current_best_path

        next_gen = [chrom for fit, chrom in fitnesses[:5]]
        
        while len(next_gen) < pop_size:
            parent1 = max(random.sample(fitnesses, 3), key=lambda x: x[0])[1]
            parent2 = max(random.sample(fitnesses, 3), key=lambda x: x[0])[1]
            
            crossover_point = random.randint(1, path_length - 1)
            child = parent1[:crossover_point] + parent2[crossover_point:]
            
            for i in range(path_length):
                if random.random() < 0.1:
                    child[i] = random.choice(directions)
                    
            next_gen.append(child)
            
        population = next_gen

    print("Path:", best_path)
    print("Steps:", len(best_path)-1)
    if best_path and grid[best_path[-1][0]][best_path[-1][1]] == 'F':
        cost = sum(weights.get(node, 1) for node in best_path[1:])
        print("Cost:", cost)
        print("Status: Goal reached!")
    else:
        print("Status: Goal not reached. Best path ends at:", best_path[-1] if best_path else vertex)
    return best_path


start = find_start(grid)

print("\n=== Greedy Best-First ===")
GreedyBestFirst(start)

print("\n=== A* Search ===")
AStar(start)

print("\n=== Hill Climbing ===")
HillClimbing(start)

print("\n=== Genetic Algorithm ===")
GeneticAlgorithm(start)
