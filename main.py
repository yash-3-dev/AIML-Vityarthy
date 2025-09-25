import heapq
import json
import random
import time
from collections import deque
from typing import List, Tuple, Optional

# ---------------------------------------------
# Environment class modeling 2D grid with obstacles
# ---------------------------------------------
class GridEnvironment:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = [[1 for _ in range(width)] for _ in range(height)]  # Terrain costs, ≥1
        self.static_obstacles = set()  # Cells that cannot be crossed
        self.dynamic_obstacles = dict()  # id -> list of positions per timestep
        self.start = None
        self.goal = None

    def set_terrain_cost(self, x: int, y: int, cost: int):
        if 0 <= x < self.width and 0 <= y < self.height and cost >= 1:
            self.grid[y][x] = cost

    def add_static_obstacle(self, x: int, y: int):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.static_obstacles.add((x, y))

    def add_dynamic_obstacle(self, obs_id: str, path: List[Tuple[int, int]]):
        self.dynamic_obstacles[obs_id] = path

    def is_passable(self, x: int, y: int, timestep: int = 0) -> bool:
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        if (x, y) in self.static_obstacles:
            return False
        for path in self.dynamic_obstacles.values():
            if timestep < len(path) and path[timestep] == (x, y):
                return False
        return True

    def get_neighbors(self, x: int, y: int, timestep: int = 0) -> List[Tuple[int, int]]:
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_passable(nx, ny, timestep):
                neighbors.append((nx, ny))
        return neighbors

    def get_cost(self, x: int, y: int):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x]
        return float('inf')

    def load_from_file(self, filename: str):
        with open(filename, 'r') as file:
            data = json.load(file)
        self.width = data['width']
        self.height = data['height']
        self.grid = data['grid']
        self.static_obstacles = set(tuple(o) for o in data['static_obstacles'])
        self.start = tuple(data['start'])
        self.goal = tuple(data['goal'])
        self.dynamic_obstacles = {k: [tuple(pos) for pos in v] for k, v in data.get('dynamic_obstacles', {}).items()}

# ---------------------------------------------
# Pathfinding algorithms
# ---------------------------------------------

def bfs(env: GridEnvironment):
    start, goal = env.start, env.goal
    queue = deque([(start, 0)])
    visited = {start: None}
    nodes_expanded = 0

    while queue:
        current, timestep = queue.popleft()
        nodes_expanded += 1
        if current == goal:
            break
        for neighbor in env.get_neighbors(*current, timestep):
            if neighbor not in visited:
                visited[neighbor] = current
                queue.append((neighbor, timestep + 1))

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = visited.get(node)
    path.reverse()
    return {'path': path, 'nodes_expanded': nodes_expanded, 'cost': sum(env.get_cost(x, y) for x, y in path)}

def uniform_cost_search(env: GridEnvironment):
    start, goal = env.start, env.goal
    pq = [(0, start, 0)]  # (cost_so_far, position, timestep)
    visited = {}
    came_from = {}
    nodes_expanded = 0

    while pq:
        cost, current, timestep = heapq.heappop(pq)
        if current in visited and visited[current] <= cost:
            continue
        visited[current] = cost
        nodes_expanded += 1
        if current == goal:
            break
        for nx, ny in env.get_neighbors(*current, timestep):
            new_cost = cost + env.get_cost(nx, ny)
            if visited.get((nx, ny), float('inf')) > new_cost:
                came_from[(nx, ny)] = current
                heapq.heappush(pq, (new_cost, (nx, ny), timestep + 1))

    # Reconstruct path
    path = []
    node = goal
    if node not in came_from and node != start:
        return {'path': [], 'nodes_expanded': nodes_expanded, 'cost': float('inf')}
    while node != start:
        path.append(node)
        node = came_from.get(node, start)
    path.append(start)
    path.reverse()
    total_cost = sum(env.get_cost(x, y) for x, y in path)
    return {'path': path, 'nodes_expanded': nodes_expanded, 'cost': total_cost}

def manhattan_heuristic(a: Tuple[int, int], b: Tuple[int, int]):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(env: GridEnvironment):
    start, goal = env.start, env.goal
    pq = [(0 + manhattan_heuristic(start, goal), 0, start, 0)]  # (f, cost_so_far, pos, timestep)
    came_from = {}
    cost_so_far = {start: 0}
    nodes_expanded = 0

    while pq:
        _, cost, current, timestep = heapq.heappop(pq)
        nodes_expanded += 1
        if current == goal:
            break
        for nx, ny in env.get_neighbors(*current, timestep):
            new_cost = cost + env.get_cost(nx, ny)
            if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                cost_so_far[(nx, ny)] = new_cost
                priority = new_cost + manhattan_heuristic((nx, ny), goal)
                came_from[(nx, ny)] = current
                heapq.heappush(pq, (priority, new_cost, (nx, ny), timestep + 1))

    # Reconstruct path
    path = []
    node = goal
    if node not in came_from and node != start:
        return {'path': [], 'nodes_expanded': nodes_expanded, 'cost': float('inf')}
    while node != start:
        path.append(node)
        node = came_from.get(node, start)
    path.append(start)
    path.reverse()
    total_cost = sum(env.get_cost(x, y) for x, y in path)
    return {'path': path, 'nodes_expanded': nodes_expanded, 'cost': total_cost}

# -------------------------------------------
# Hill Climbing for local search replanning
# -------------------------------------------
def hill_climbing(env: GridEnvironment, start_pos: Tuple[int, int], goal: Tuple[int, int], max_iters=100):
    current = start_pos
    path = [current]
    nodes_expanded = 0

    for _ in range(max_iters):
        neighbors = env.get_neighbors(*current)
        if not neighbors:
            break
        # Choose the neighbor closest to goal by heuristic
        neighbors = sorted(neighbors, key=lambda n: manhattan_heuristic(n, goal))
        best = neighbors[0]
        nodes_expanded += 1
        if best == goal:
            path.append(best)
            return {'path': path, 'nodes_expanded': nodes_expanded, 'cost': sum(env.get_cost(x, y) for x, y in path)}
        if manhattan_heuristic(best, goal) >= manhattan_heuristic(current, goal):
            # Local optima reached
            break
        path.append(best)
        current = best

    return {'path': path, 'nodes_expanded': nodes_expanded, 'cost': sum(env.get_cost(x, y) for x, y in path)}

# ---------------------------------------------
# Simple CLI to demo the algorithms
# ---------------------------------------------
import argparse

def main():
    parser = argparse.ArgumentParser(description='Autonomous Delivery Agent on 2D Grid')
    parser.add_argument('--map', type=str, default='sample_map.json', help='JSON file with map data')
    parser.add_argument('--algorithm', type=str, choices=['bfs', 'ucs', 'astar', 'hill_climb'], default='astar')
    args = parser.parse_args()

    env = GridEnvironment(0, 0)
    env.load_from_file(args.map)

    env.start = tuple(env.start)
    env.goal = tuple(env.goal)

    start_time = time.time()
    if args.algorithm == 'bfs':
        result = bfs(env)
    elif args.algorithm == 'ucs':
        result = uniform_cost_search(env)
    elif args.algorithm == 'astar':
        result = a_star(env)
    elif args.algorithm == 'hill_climb':
        result = hill_climbing(env, env.start, env.goal)
    else:
        print("Unknown algorithm selected.")
        return

    end_time = time.time()
    print(f"Algorithm: {args.algorithm.upper()}")
    print(f"Path length: {len(result['path'])}")
    print(f"Total cost: {result['cost']}")
    print(f"Nodes expanded: {result['nodes_expanded']}")
    print(f"Time taken: {end_time - start_time:.4f} seconds")
    print("Path:", result['path'])

if __name__ == "__main__":
    main()
