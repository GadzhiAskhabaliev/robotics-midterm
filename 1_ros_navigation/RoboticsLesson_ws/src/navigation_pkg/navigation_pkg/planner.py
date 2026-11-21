"""Grid A* on an inflated occupancy grid. No ROS dependency."""

import heapq
import math

import numpy as np


class GridPlanner:
    def __init__(self, resolution=0.25, robot_radius=0.40):
        self.resolution = float(resolution)
        self.robot_radius = float(robot_radius)

    def planning(self, start, goal, obstacles=None):
        start = np.asarray(start, dtype=float).reshape(-1)[:2]
        goal = np.asarray(goal, dtype=float).reshape(-1)[:2]
        obstacles = self._obstacle_xy(obstacles)

        if len(obstacles) == 0:
            return self._straight(start, goal)

        origin, width, height, blocked = self._build_grid(start, goal, obstacles)
        start_cell = self._nearest_free(self._cell(start, origin), blocked, width, height)
        goal_cell = self._nearest_free(self._cell(goal, origin), blocked, width, height)
        if start_cell is None or goal_cell is None:
            return self._straight(start, goal)

        cells = self._astar(start_cell, goal_cell, blocked, width, height)
        if not cells:
            return self._straight(start, goal)

        path = [self._meters(cell, origin) for cell in cells]
        path[0] = (float(start[0]), float(start[1]))
        path[-1] = (float(goal[0]), float(goal[1]))
        return path

    def _obstacle_xy(self, obstacles):
        if obstacles is None:
            return np.zeros((0, 2), dtype=float)
        points = np.asarray(obstacles, dtype=float)
        if points.size == 0:
            return np.zeros((0, 2), dtype=float)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        return points[:, :2]

    def _straight(self, start, goal):
        distance = float(np.linalg.norm(goal - start))
        steps = max(1, int(math.ceil(distance / self.resolution)))
        path = []
        for i in range(steps + 1):
            t = i / steps
            point = (1.0 - t) * start + t * goal
            path.append((float(point[0]), float(point[1])))
        return path

    def _build_grid(self, start, goal, obstacles):
        margin = self.robot_radius + self.resolution
        mins = np.minimum(np.minimum(start, goal), obstacles.min(axis=0)) - margin
        maxs = np.maximum(np.maximum(start, goal), obstacles.max(axis=0)) + margin
        origin = mins
        width = int(math.ceil((maxs[0] - mins[0]) / self.resolution)) + 1
        height = int(math.ceil((maxs[1] - mins[1]) / self.resolution)) + 1
        blocked = np.zeros((width, height), dtype=bool)

        cells_x = np.floor((obstacles[:, 0] - origin[0]) / self.resolution).astype(int)
        cells_y = np.floor((obstacles[:, 1] - origin[1]) / self.resolution).astype(int)
        inside = (cells_x >= 0) & (cells_y >= 0) & (cells_x < width) & (cells_y < height)
        cells_x = cells_x[inside]
        cells_y = cells_y[inside]

        radius_cells = max(1, int(math.ceil(self.robot_radius / self.resolution)))
        offsets = [
            (dx, dy)
            for dx in range(-radius_cells, radius_cells + 1)
            for dy in range(-radius_cells, radius_cells + 1)
            if dx * dx + dy * dy <= radius_cells * radius_cells
        ]
        for x, y in zip(cells_x.tolist(), cells_y.tolist()):
            for dx, dy in offsets:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    blocked[nx, ny] = True
        return origin, width, height, blocked

    def _cell(self, point, origin):
        x = int(math.floor((point[0] - origin[0]) / self.resolution))
        y = int(math.floor((point[1] - origin[1]) / self.resolution))
        return x, y

    def _meters(self, cell, origin):
        x = origin[0] + (cell[0] + 0.5) * self.resolution
        y = origin[1] + (cell[1] + 0.5) * self.resolution
        return float(x), float(y)

    def _nearest_free(self, cell, blocked, width, height):
        if self._free(cell, blocked, width, height):
            return cell
        cx, cy = cell
        for radius in range(1, max(width, height)):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidate = (cx + dx, cy + dy)
                    if self._free(candidate, blocked, width, height):
                        return candidate
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidate = (cx + dx, cy + dy)
                    if self._free(candidate, blocked, width, height):
                        return candidate
        return None

    def _free(self, cell, blocked, width, height):
        x, y = cell
        return 0 <= x < width and 0 <= y < height and not blocked[x, y]

    def _astar(self, start, goal, blocked, width, height):
        moves = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                moves.append((dx, dy, math.hypot(dx, dy)))

        open_set = [(math.hypot(goal[0] - start[0], goal[1] - start[1]), 0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}

        while open_set:
            _, cost, current = heapq.heappop(open_set)
            if cost > cost_so_far.get(current, float("inf")):
                continue
            if current == goal:
                return self._reconstruct(came_from, current)

            for dx, dy, step_cost in moves:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._free(nxt, blocked, width, height):
                    continue
                if dx != 0 and dy != 0:
                    if not self._free((current[0] + dx, current[1]), blocked, width, height):
                        continue
                    if not self._free((current[0], current[1] + dy), blocked, width, height):
                        continue
                new_cost = cost + step_cost
                if new_cost >= cost_so_far.get(nxt, float("inf")):
                    continue
                cost_so_far[nxt] = new_cost
                priority = new_cost + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_set, (priority, new_cost, nxt))
                came_from[nxt] = current
        return []

    def _reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
