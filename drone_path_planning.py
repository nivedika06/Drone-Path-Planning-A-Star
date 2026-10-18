import tkinter as tk
from tkinter import messagebox
import heapq
import time
ROWS = 15
COLS = 15
CELL = 35
start = None
goal = None
obstacles = set()
root = tk.Tk()
root.title("Drone Path Planning using A*")
root.resizable(False, False)
title = tk.Label(
    root,
    text="DRONE PATH PLANNING USING A*",
    font=("Arial", 18, "bold")
)
title.pack(pady=10)
instruction = tk.Label(
    root,
    text="Click Start → Click Destination → Click cells for Obstacles",
    font=("Arial", 11)
)
instruction.pack(pady=5)
canvas = tk.Canvas(
    root,
    width=COLS * CELL,
    height=ROWS * CELL,
    bg="white"
)
canvas.pack(padx=10, pady=10)
def draw_grid():
    canvas.delete("all")
    for row in range(ROWS):
        for col in range(COLS):
            x1 = col * CELL
            y1 = row * CELL
            x2 = x1 + CELL
            y2 = y1 + CELL
            cell = (row, col)
            if cell == start:
                color = "green"
            elif cell == goal:
                color = "red"
            elif cell in obstacles:
                color = "black"
            else:
                color = "white"
            canvas.create_rectangle(
                x1, y1, x2, y2,
                fill=color,
                outline="gray"
            )
def mouse_click(event):
    global start, goal
    col = int(event.x / CELL)
    row = int(event.y / CELL)
    if row < 0 or row >= ROWS or col < 0 or col >= COLS:
        return
    cell = (row, col)
    if start is None:
        start = cell
        status.config(
            text="Start selected. Now click the destination."
        )
        draw_grid()
        return
    if goal is None:
        if cell == start:
            status.config(
                text="Choose a different cell for destination."
            )
            return
        goal = cell
        status.config(
            text="Destination selected. Now add obstacles."
        )
        draw_grid()
        return
    if cell != start and cell != goal:
        if cell in obstacles:
            obstacles.remove(cell)
        else:
            obstacles.add(cell)
        status.config(
            text="Obstacle added. Click Find Path when ready."
        )
        draw_grid()
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
def get_neighbors(node):
    row, col = node
    directions = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1)
    ]
    neighbors = []
    for dr, dc in directions:
        new_row = row + dr
        new_col = col + dc
        if 0 <= new_row < ROWS and 0 <= new_col < COLS:
            new_cell = (new_row, new_col)
            if new_cell not in obstacles:
                neighbors.append(new_cell)
    return neighbors
def a_star():
    open_list = []
    heapq.heappush(
        open_list,
        (0, start)
    )
    came_from = {}
    g_cost = {
        start: 0
    }
    nodes_explored = 0
    while open_list:
        current = heapq.heappop(open_list)[1]
        nodes_explored += 1
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, nodes_explored
        for neighbor in get_neighbors(current):
            new_cost = g_cost[current] + 1
            if neighbor not in g_cost or new_cost < g_cost[neighbor]:
                g_cost[neighbor] = new_cost
                h = heuristic(neighbor, goal)
                f = new_cost + h
                heapq.heappush(
                    open_list,
                    (f, neighbor)
                )
                came_from[neighbor] = current
    return None, nodes_explored
def find_path():
    if start is None:
        messagebox.showwarning(
            "Start Missing",
            "Please select the starting point."
        )
        return
    if goal is None:
        messagebox.showwarning(
            "Destination Missing",
            "Please select the destination."
        )
        return
    start_time = time.perf_counter()
    path, nodes = a_star()
    end_time = time.perf_counter()
    execution_time = (end_time - start_time) * 1000
    if path is None:
        messagebox.showinfo(
            "No Path",
            "No possible path found."
        )
        return
    for row, col in path:
        if (row, col) == start or (row, col) == goal:
            continue
        x1 = col * CELL
        y1 = row * CELL
        x2 = x1 + CELL
        y2 = y1 + CELL
        canvas.create_rectangle(
            x1, y1, x2, y2,
            fill="skyblue",
            outline="gray"
        )
    row, col = start
    canvas.create_rectangle(
        col * CELL,
        row * CELL,
        (col + 1) * CELL,
        (row + 1) * CELL,
        fill="green",
        outline="gray"
    )
    row, col = goal
    canvas.create_rectangle(
        col * CELL,
        row * CELL,
        (col + 1) * CELL,
        (row + 1) * CELL,
        fill="red",
        outline="gray"
    )
    path_length = len(path) - 1
    status.config(
        text=
        f"Path Length: {path_length} | "
        f"Nodes Explored: {nodes} | "
        f"Time: {execution_time:.3f} ms"
    )
def reset():
    global start, goal, obstacles
    start = None
    goal = None
    obstacles.clear()
    draw_grid()
    status.config(
        text="Click any cell to select the starting point."
    )
button_frame = tk.Frame(root)
button_frame.pack(pady=5)
find_button = tk.Button(
    button_frame,
    text="Find Path",
    width=12,
    command=find_path
)
find_button.pack(
    side=tk.LEFT,
    padx=5
)
reset_button = tk.Button(
    button_frame,
    text="Reset",
    width=12,
    command=reset
)
reset_button.pack(
    side=tk.LEFT,
    padx=5
)
status = tk.Label(
    root,
    text="Click any cell to select the starting point.",
    font=("Arial", 10)
)
status.pack(pady=8)
canvas.bind("<Button-1>", mouse_click)
draw_grid()
root.mainloop()