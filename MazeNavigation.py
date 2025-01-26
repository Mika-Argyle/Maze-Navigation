#Modified code from Assignment 2
#By:
#Mika Argyle - 101043314
#and
#

from collections import deque
import heapq
def print_forest(forest):
    for row in forest:
        print(" ".join(row))
    print()
class SearchAgent:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
    def is_valid(self, position):
        row, col = position
        return 0 <= row < self.rows and 0 <= col < self.cols and self.grid[row][col] != '#'
    def get_cost(self, position):
        cell = self.grid[position[0]][position[1]]
        if cell == 'f':
            return 3
        elif cell == 'F':
            return 5
        return 1
    def print_path(self, path):
        temp_grid = [row[:] for row in self.grid] # Create a copy of the grid
        for row, col in path[1:-1]: # Avoid overriding start and goal
            temp_grid[row][col] = 'P'
        for row in temp_grid:
            print(" ".join(row))
    def bfs(self):

        return None
# Implement BFS logic: return exploration steps, path cost, and path length, or None if no path is found. 
# To be implemented by group member
    def dfs(self):

        return None
# Implement DFS logic: return exploration steps, path cost, and path length, or None if no path is found.
# To be implemented by group member
    def ucs(self):
# Implement UCS logic: return exploration steps, path cost, and path length, or None if no path is found.
# Orders by path cost (backwards cost) - most basic algorithm that factors in cost
        

        #Initialize step count
        exploration_steps = 0

        #Create Fringe and begin adding to it
        fringe = []
        heapq.heappush(fringe, (0,self.start))
        previous_node = {self.start: None}
        accumulated_cost = {self.start: 0}
        

        #Pop the cheapest node from the fringe (fringe will be organized accordingly)
        while fringe:
            exploration_steps += 1 #new step 
            current_cost, current_node = heapq.heappop(fringe)

            #If we have found the goal
            if current_node == self.goal: 
                #We construct optimized path, this is what should be visualized
                path = []
                #We are starting from the current node (the goal), and working our way back to the start
                while current_node != self.start: 
                    path.append(current_node)
                    current_node = previous_node[current_node]
                #Once we reach the start, leaving the loop, we add the start to our path, and reverse it, since we build the path backwards
                path.append(self.start)
                path.reverse()

                self.print_path(path)

                #Return the parameters we need
                return len(path), exploration_steps, accumulated_cost[self.goal]
            
            #If we have not found the goal
            #Determine valid neighbouring nodes, and add them by cost to the fringe

            row, col = current_node

            #The four neighbouring nodes
            for r, c in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]:
                neighbouring_node = (r, c)
                #If the node is valid
                if self.is_valid(neighbouring_node):
                    #Add the current cost to the cost of the new node
                    new_cost = current_cost + self.get_cost(neighbouring_node)

                    #If the neighbouring node isn't already in the path or the new cost is cheaper 
                    if neighbouring_node not in accumulated_cost or new_cost < accumulated_cost[neighbouring_node]:
                        #update the cost
                        accumulated_cost[neighbouring_node] = new_cost
                        #add it to the fringe
                        heapq.heappush(fringe, (new_cost, neighbouring_node))
                        #set it's previous node (so we can follow path)
                        previous_node[neighbouring_node] = current_node

        return None



 
    
        
    def astar(self, heuristic=None):
        if heuristic is None:
            def heuristic(pos):
                return abs(pos[0] - self.goal[0]) + abs(pos[1] - self.goal[1])
            
        
        
        #Initialize step count
        exploration_steps = 0

        #Create Fringe and begin adding to it
        fringe = []
        heapq.heappush(fringe, (0 + heuristic(self.start),self.start))
        previous_node = {self.start: None}
        accumulated_cost = {self.start: 0}
        

        #Pop the cheapest node from the fringe (fringe will be organized accordingly)
        while fringe:
            exploration_steps += 1 #new step 
            _, current_node = heapq.heappop(fringe) #pops the cheapest (cost + heuristic) node from the fringe

            #If we have found the goal
            if current_node == self.goal: 
                #We construct optimized path, this is what should be visualized
                path = []
                #We are starting from the current node (the goal), and working our way back to the start
                while current_node != self.start: 
                    path.append(current_node)
                    current_node = previous_node[current_node]
                #Once we reach the start, leaving the loop, we add the start to our path, and reverse it, since we build the path backwards
                path.append(self.start)
                path.reverse()

                #Print the path
                self.print_path(path)

                #Return the parameters we need
                return len(path), exploration_steps, accumulated_cost[self.goal]
            
            #If we have not found the goal
            #Determine valid neighbouring nodes, and add them by cost to the fringe

            row, col = current_node

            #The four neighbouring nodes
            for r, c in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]:
                neighbouring_node = (r, c)
                #If the node is valid
                if self.is_valid(neighbouring_node):
                    #Add the current cost to the cost of the new node
                    new_cost = accumulated_cost[current_node] + self.get_cost(neighbouring_node)

                    #If the neighbouring node isn't already in the path or the new cost is cheaper 
                    if neighbouring_node not in accumulated_cost or new_cost < accumulated_cost[neighbouring_node]:
                        #update the cost
                        accumulated_cost[neighbouring_node] = new_cost
                        #Calculate A* Priority (cost + heuristic)
                        priority = new_cost + heuristic(neighbouring_node)
                        #Add it to the fringe based on this priority
                        heapq.heappush(fringe, (priority, neighbouring_node))
                        #Set it's previous node (so we can follow path)
                        previous_node[neighbouring_node] = current_node

        return None    
            




# Implement A* logic: return exploration steps, path cost, and path length, or None if no path is found.
def test_search_agent(agent):
    results = {}
    '''print("\n--- BFS ---")
    path_length, exploration_steps, cost_length = agent.bfs()
    results['BFS'] = (path_length, exploration_steps, cost_length)
    print(f"Path Length: {path_length} steps")
    print(f"Exploration Steps: {exploration_steps}")
    print(f"Cost Length: {cost_length}")
    print("\n--- DFS ---")
    path_length, exploration_steps, cost_length = agent.dfs()
    results['DFS'] = (path_length, exploration_steps, cost_length)
    print(f"Path Length: {path_length} steps")
    print(f"Exploration Steps: {exploration_steps}")
    print(f"Cost Length: {cost_length}")'''
    print("\n--- UCS ---")
    path_length, exploration_steps, cost_length = agent.ucs()
    results['UCS'] = (path_length, exploration_steps, cost_length)
    print(f"Path Length: {path_length} steps")
    print(f"Exploration Steps: {exploration_steps}")
    print(f"Cost Length: {cost_length}")
    print("\n--- A* ---")
    path_length, exploration_steps, cost_length = agent.astar(lambda pos: abs(pos[0] - agent.goal[0]) +
    abs(pos[1] - agent.goal[1]))
    results['A*'] = (path_length, exploration_steps, cost_length)
    print(f"Path Length: {path_length} steps")
    print(f"Exploration Steps: {exploration_steps}")
    print(f"Cost Length: {cost_length}")
    return results

# This is a good grid for testing UCS benefits
grid = [
    ['S', 'F', 'F', 'F', 'F'],
    ['.', '#', '#', '#', 'F'],
    ['.', '#', '.', '.', '.'],
    ['.', '#', '.', '#', '.'],
    ['.', '.', '.', '#', 'G']
]
start = (0, 0)  # Starting position
goal = (4, 4)   # Goal position

agent1 = SearchAgent(start, goal, grid)

grid = [
    ['S', '.', '.', 'f', '.', '.', '.'],
    ['#', '#', '#', '#', '#', '#', 'f'],
    ['#', '.', '.', '.', '.', '.', '.'],
    ['#', '.', '#', '#', '#', '.', '#'],
    ['#', '.', '#', 'f', '#', '.', 'F'],
    ['#', '#', '#', 'F', '#', '#', 'G']
]
start = (0,0)
goal = (5,6)


# Creates an instance of a search agent
agent2 = SearchAgent(start, goal, grid)

#Tests the search agent
print("\n\nTESTING MAZE ONE")
test_search_agent(agent1)
print("\n\nTESTING MAZE TWO")
test_search_agent(agent2)

#Does it work ? 