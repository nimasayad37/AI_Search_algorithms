"""
This file contains function headers for various pathfinding algorithms.
Students are responsible for implementing these algorithms.

IMPORTANT:
- Do NOT include any graphical elements in this file.
- Use `update_visualization function` to update the visualization.
"""

# imports are here 
import heapq
from collections import defaultdict
import time
import matplotlib as plt
from graphics import update_visualization, reconstruct_path
from math import sqrt
import queue
import stack_data
def heuristic(cell, goal):
    return sqrt((cell[0] - goal[0]) ** 2 + (cell[1] - goal[1]) ** 2)

def A_star(graph, start, goal, pathMap, ax2, ax3):
    f = defaultdict(lambda: float('inf'))
    g = defaultdict(lambda: float('inf'))
    parent = {}
    open_list = []
    visited = []
    curr = start
    g[start] = 0
    f[start] = heuristic(start, goal)
    then = time.time()
    heapq.heappush(open_list, (f[start], start))
    while open_list:
        curr = heapq.heappop(open_list)[1]
        if curr == goal:
            pathMap[goal[0], goal[1]] = 255
            update_visualization(pathMap, ax2, "Searching complete")
            path = []
            curr = goal
            while curr != start:
                path.append(curr)
                curr = parent[curr]
            path.append(start)
            path.reverse()
            for x, y in visited:
                pathMap[x, y] = 13
            for x, y in path: 
                pathMap[x, y] = 200
            now = time.time() 
            update_visualization(pathMap, ax3, "Final path")
            print("length of the path", len(path), "\n number of nodes explored ", len(visited))
            return path, visited, now - then
        visited.append(curr)
        for n in graph[curr]:
            currs_child_g = g[curr] + 1
            if currs_child_g < g[n] and n not in parent:
                parent[n] = curr
                g[n] = currs_child_g
                f[n] = g[n] + heuristic(n, goal)
                if n not in [ol[1] for ol in open_list]:
                    heapq.heappush(open_list, (f[n], n))
        pathMap[curr[0], curr[1]] = 100
        #pathMap[start[0], start[1]] = 255
        update_visualization(pathMap, ax2, "Searching")
        now = time.time()
    return [], visited, now - then
def bfs(graph, start, goal, pathMap, ax2, ax3):
    q = queue.Queue()
    q.put(start)
    visited = set()
    parent = {}
    then = time.time()
    while not q.empty():
        curr = q.get()
        visited.add(curr)
        if curr == goal:
            path = []
            while curr in parent:
                path.append(curr)
                curr = parent[curr]
            path.append(start)
            path.reverse
            for x, y in path:
                pathMap[x, y] = 200
            update_visualization(pathMap, ax3, "Path")
            now = time.time()
            print("length of the path", len(path), "\n number of nodes explored ", len(visited))
            return path, visited, now - then 
        pathMap[curr[0], curr[1]] = 13
        update_visualization(pathMap, ax2, "yaahoo")
        for n in graph[curr]:
            if n not in visited and n not in parent:
                parent[n] = curr
                q.put(n)
                update_visualization(pathMap, ax2, "Searching")
    now = time.time()
    return [], visited, now - then

def dfs(graph, start, goal, pathMap, ax2, ax3):
    stack = []
    stack.append(start)
    visited = set()
    parent = {}
    then = time.time()
    while len(stack) != 0: 
        curr = stack.pop()
        visited.add(curr)
        if curr == goal:
            path = []
            while curr in parent:
                path.append(curr)
                curr = parent[curr]
            path.append(start)
            path.reverse
            for x, y in path:
                pathMap[x, y] = 200
            update_visualization(pathMap, ax3, "DFS path")
            now = time.time()
            print("length of the path", len(path), "\n number of nodes explored ", len(visited))
            return path, visited, now - then
        pathMap[curr[0], curr[1]] = 13
        update_visualization(pathMap, ax2, "DFS in progress")
        for n in graph[curr]:
            if n not in visited:
                parent[n] = curr
                stack.append(n)
    now = time.time()
    return [], visited, now - then
def greedy(graph, start, goal, pathMap, ax2, ax3):
    f = defaultdict(lambda: float('inf'))
    parent = {}
    open_list = []
    visited = []
    curr = start
    f[start] = heuristic(start, goal)
    then = time.time()
    heapq.heappush(open_list, (f[start], start))
    while open_list:
        curr = heapq.heappop(open_list)[1]
        if curr == goal:
            pathMap[curr[0], curr[1]] = 200
            update_visualization(pathMap, ax2, "Greedy complete")
            path = []
            curr = goal
            while curr != start:
                path.append(curr)
                curr = parent[curr]
            path.append(start)
            path.reverse()
            for x, y in visited:
                if (x, y) in path:
                    pathMap[x, y] = 200
                else:
                    pathMap[x, y] = 13
            now = time.time() 
            update_visualization(pathMap, ax3, "Final path")
            print("length of the path", len(path), "\n number of nodes explored ", len(visited))
            return path, visited, now - then
        visited.append(curr)
        for n in graph[curr]:
            if n not in visited:
                parent[n] = curr
                f[n] = heuristic(n, goal)
                if n not in [ol[1] for ol in open_list]:
                    heapq.heappush(open_list, (f[n], n))
        pathMap[curr[0], curr[1]] = 100
        #pathMap[start[0], start[1]] = 255
        update_visualization(pathMap, ax2, "seaching")
        now = time.time()
    return [], visited, now - then
def depth_limited_search(graph, current, goal, depth, came_from, pathMap, ax2, expanded_nodes):
    if depth == 0:
        pathMap[current[0], current[1]] = 200
        update_visualization(pathMap, ax2, "Searching")
        return [current], came_from
    else:
        l = []
        for n in expanded_nodes:
            l = l + graph[n]
            for c in graph[n]:
                if c not in came_from:
                    came_from[c] = n
                    pathMap[c[0], c[1]] = 13
            update_visualization(pathMap, ax2, "Searching")
        return l, came_from
def iterative_deepening_search(graph, start, goal, pathMap, ax2, ax3):
    depth = 0
    then = time.time()
    curr = start
    open_set = set()
    open_set.add(start)
    parent = {}
    visited = []
    while True:
        next_depth, parent = depth_limited_search(graph, curr, goal, depth, parent, pathMap, ax2, open_set)
        for n in next_depth:
            if n in visited:
                next_depth.remove(n)
        visited = visited + next_depth
        if goal in next_depth:
            update_visualization(pathMap, ax2, "Search complete")
            curr = goal
            path = []
            while curr != start:
                path.append(curr)
                curr = parent[curr]
            path.append(start)
            path.reverse()
            for x, y in path:
                pathMap[x, y] = 200
            update_visualization(pathMap, ax3, "PATH")
            now = time.time()
            print("length of the path", len(path), "\n number of nodes explored ", len(visited))
            return path, visited, now - then
        open_set = next_depth
        depth = depth + 1