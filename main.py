import sys
import queue
import math
import heapq
import math
# python3 main.py searchType, -A city -B city fileName
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 main.py searchType, -A city -B city mapFile")
        return
    cities, edges, coords = parseFile(sys.argv[-1])
    start, goal = getTargets(sys.argv)
    searchType = sys.argv[1]
    if searchType == "astar":
        astar(cities,edges,coords,start,goal)
    elif searchType == "dls":
        dls(cities,edges,coords,start,goal, sys.argv[2])
    elif searchType == "ucs":
        ucs(cities,edges,coords,start,goal)
    else:
        bfs(cities,edges,coords,start,goal)
    # print(edges)
    # print(cities)
    # print(coords)
    # print(start)
    # print(goal)

def getTargets(args):
    start = ""
    goal = ""
    for arg in range(len(sys.argv)):
        if sys.argv[arg] == "-A":
            start = sys.argv[arg+1]
        elif sys.argv[arg] == "-B":
            goal = sys.argv[arg+1]
    
    return start, goal



def parseFile(fileName):
    cities = []
    edges = {}
    coords = {}
    
    with open(fileName, "r") as file:
        for line in file:
            parts = line.strip().split("-->")
            currentCity = parts[0].strip().split(" ")[0]
            cities.append(currentCity)
            coords[currentCity] = parts[0].strip().split(" ")[1:]
            paths = parts[1].strip().split(" ")
            edges[currentCity] = [[], []]
            for i in range(0, len(paths)):
                if i % 2 == 0:
                    edges[currentCity][0].append(paths[i].replace('va-', '', 1)) # Add the city
                else:
                    edges[currentCity][1].append(paths[i]) # Add the cost
    return cities, edges, coords


def astar(cities,edges,coords,start,goal):
    frontier = queue.PriorityQueue()
    visited = []
    frontier.put((0, start)) 
    currentDist = 0
    while not frontier.empty():
        weight, current = frontier.get()

        if current == goal:
            print(current+": Goal Found")
            return
        print("Current: ", current)
        if current not in visited:
            visited.append(current)
            for i in range(len(edges[current][0])):
                h = getHeuristic(coords[edges[current][0][i]], coords[goal])
                g = float(edges[current][1][i])
                city = edges[current][0][i]
                frontier.put((g+h, city))  

def getHeuristic(start, goal):
    return math.sqrt((float(start[0])-float(goal[0]))**2 + (float(start[1])-float(goal[1]))**2)

def bfs(cities,edges,coords,start,goal):
    frontier = queue.Queue()
    visited = []
    frontier.put(start)
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            print(current+": Goal Found")
            return
        if current not in visited:
            visited.append(current)
            for i in range(len(edges[current][0])):
                frontier.put(edges[current][0][i])


def dls(cities,edges,coords,start,goal, maxDepth):
    depth = 0
    frontier = queue.Queue()
    visited = []
    frontier.put(start)
    while not frontier.empty():
        current = frontier.get()
        print("Current: ", current)
        if current == goal:
            print(current+": Goal Found")
            return
        if current not in visited:
            visited.append(current)
            for i in range(len(edges[current][0])):
                if edges[current][0][i] not in visited:
                    frontier.put(edges[current][0][i])

        if depth == maxDepth:
            print("Max Depth Reached")
            return
        depth += 1



def ucs(cities,edges,coords,start,goal):
    frontier = []
    visited = []
    costs = {}
    frontier.append(start)
    while frontier:
        current = frontier.pop(0)
        if current not in visited:
            visited.append(current)
            if current == goal:
                print(current+": Goal Found")
                return
            print("Current: ", current)
            for i in range(len(edges[current][0])):
                child = edges[current][0][i]
                child_cost = edges[current][1][i]
                if child not in visited:
                    if child in frontier:
                        index = frontier.index(child)
                        if costs[child] > child_cost:
                            frontier[index] = child
                            costs[child] = child_cost
                    else:
                        frontier.append(child)
                        costs[child] = child_cost

if __name__ == "__main__":
    main()