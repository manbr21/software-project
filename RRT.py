import numpy as np
import math
import random

# Node class representing a state in the space
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

# RRT* algorithm
class RRTStar:
    def __init__(self, start, goal, obstacles, map_size, step_size=1.0, max_iter=500):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.goal_region_radius = 0.18
        self.search_radius = 0.5
        self.path = None
        self.goal_reached = False

    def plan(self):
        """Main RRT* planning loop."""
        for i in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if self.is_collision_free(new_node):
                neighbors = self.find_neighbors(new_node)
                new_node = self.choose_parent(neighbors, nearest_node, new_node)
                self.node_list.append(new_node)
                self.rewire(new_node, neighbors)
                self.next_node = new_node

            if self.reached_goal(new_node):
                self.path = self.generate_final_path(new_node)
                self.goal_reached = True
                #print("ACHOU CAMINHO")
                return
        
    def get_random_node(self):
        """Generate a random node in the map."""
        if random.random() > 0.2:
            rand_node = Node(random.uniform(-self.map_size[0], self.map_size[0]), random.uniform(-self.map_size[1], self.map_size[1]))
        else:
            rand_node = Node(self.goal.x, self.goal.y)
        return rand_node

    def steer(self, from_node, to_node):
        """Steer from one node to another, step-by-step."""
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = Node(from_node.x + self.step_size * math.cos(theta),
                        from_node.y + self.step_size * math.sin(theta))
        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node
        return new_node

    def is_collision_free(self, node):
        """Check if the node is collision-free with respect to obstacles."""
        for (ox, oy, size) in self.obstacles:
            if (node.x - ox) ** 2 + (node.y - oy) ** 2 <= size ** 2:
                return False
        return True

    def find_neighbors(self, new_node):
        """Find nearby nodes within the search radius."""
        return [node for node in self.node_list
                if np.linalg.norm([node.x - new_node.x, node.y - new_node.y]) < self.search_radius]

    def choose_parent(self, neighbors, nearest_node, new_node):
        """Choose the best parent for the new node based on cost."""
        min_cost = nearest_node.cost + np.linalg.norm([new_node.x - nearest_node.x, new_node.y - nearest_node.y])
        best_node = nearest_node

        for neighbor in neighbors:
            cost = neighbor.cost + np.linalg.norm([new_node.x - neighbor.x, new_node.y - neighbor.y])
            if cost < min_cost and self.is_collision_free(neighbor):
                best_node = neighbor
                min_cost = cost

        new_node.cost = min_cost
        new_node.parent = best_node
        return new_node

    def rewire(self, new_node, neighbors):
        """Rewire the tree by checking if any neighbor should adopt the new node as a parent."""
        for neighbor in neighbors:
            cost = new_node.cost + np.linalg.norm([neighbor.x - new_node.x, neighbor.y - new_node.y])
            if cost < neighbor.cost and self.is_collision_free(neighbor):
                neighbor.parent = new_node
                neighbor.cost = cost

    def reached_goal(self, node):
        """Check if the goal has been reached."""
        return np.linalg.norm([node.x - self.goal.x, node.y - self.goal.y]) < self.goal_region_radius

    def generate_final_path(self, goal_node):
        """Generate the final path from the start to the goal."""
        path = []
        node = goal_node
        #print(goal_node.x, goal_node.y)
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]  # Reverse the path

    def get_nearest_node(self, node_list, rand_node):
        """Find the nearest node in the tree to the random node."""
        distances = [np.linalg.norm([node.x - rand_node.x, node.y - rand_node.y]) for node in node_list]
        nearest_node = node_list[np.argmin(distances)]
        return nearest_node
    