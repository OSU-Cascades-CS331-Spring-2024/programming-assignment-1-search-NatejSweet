import sys
import queue
import math
import heapq
import math
import Map as Map
# python3 main.py searchType, -A city -B city fileName
def main():
    start, goal = getTargets(sys.argv)
    cities, edges, coords = parseFile(sys.argv[-1])
    map = Map.Map(edges, cities, coords)
    if '-A' not in sys.argv and '-B' not in sys.argv:
        return defaultSearches(sys.argv,map)
    # print("Cities: ", cities)
    # print("Edges: ", edges)
    # print("Coords: ", coords)
    searchType = sys.argv[1]
    if searchType == "astar":
        aPath, aCost, aNodesExplored, aNodesExpanded, aNodesMaintained = astar(map, start, goal)
        print(start+" to "+goal)
        print("A* Path: ", aPath)
        print("A* Cost: ", aCost)
        print("A* Nodes Explored: ", aNodesExplored)
        print("A* Nodes Expanded: ", aNodesExpanded)
        print("A* Nodes Maintained: ", aNodesMaintained)
    elif searchType == "dls":
        if int(sys.argv[2].isdigit()):
            dPath, dCost, dNodesExplored, dNodesExpanded, dNodesMaintained = dls(map, start, goal, sys.argv[2])
        else:
            dPath, dCost, dNodesExplored, dNodesExpanded, dNodesMaintained = dls(map, start, goal, 20)
        print(start+" to "+goal)
        print("DLS Path: ", dPath)
        print("DLS Cost: ", dCost)
        print("DLS Nodes Explored: ", dNodesExplored)
        print("DLS Nodes Expanded: ", dNodesExpanded)
        print("DLS Nodes Maintained: ", dNodesMaintained)
    elif searchType == "ucs":
        uPath, uCost, uNodesExplored, uNodesExpanded, uNodesMaintained = ucs(map ,start, goal)
        print(start+" to "+goal)
        print("UCS Path: ", uPath)
        print("UCS Cost: ", uCost)
        print("UCS Nodes Explored: ", uNodesExplored)   
        print("UCS Nodes Expanded: ", uNodesExpanded)
        print("UCS Nodes Maintained: ", uNodesMaintained)
    else:
        bPath, bCost, bNodesExplored, bNodesExpanded, bNodesMaintained = bfs(map, start, goal)
        print(start+" to "+goal)
        print("BFS Path: ", bPath)
        print("BFS Cost: ", bCost)
        print("BFS Nodes Explored: ", bNodesExplored)
        print("BFS Nodes Expanded: ", bNodesExpanded)
        print("BFS Nodes Maintained: ", bNodesMaintained)

def defaultSearches(argv,map):
    cities, edges, coords = parseFile(argv[-1])
    paths = ["brest-nice", 'montpellier-calais', 'strasbourg-bordeaux', 'paris-grenoble', 'grenoble-paris', 'brest-grenoble', 'nice-nantes', 'caen-strasbourg']
    aAvgCost, aAvgNodesExplored, aAvgNodesExpanded, aAvgNodesMaintained  = 0, 0, 0, 0
    bAvgCost, bAvgNodesExplored, bAvgNodesExpanded, bAvgNodesMaintained = 0, 0, 0, 0
    dAvgCost, dAvgNodesExplored, dAvgNodesExpanded , dAvgNodesMaintained = 0, 0, 0, 0
    uAvgCost, uAvgNodesExplored, uAvgNodesExpanded, uAvgNodesMaintained = 0, 0, 0, 0
    aWins, bWins, dWins, uWins = 0,0,0,0
    with open('solutions.txt', 'w') as file:
        for path in paths:
            
            start = path.split("-")[0]
            goal = path.split("-")[1]
            aPath, aCost, aNodesExplored, aNodesExpanded, aNodesMaintained = astar(map,start,goal)
            bPath, bCost, bNodesExplored, bNodesExpanded, bNodesMaintained = bfs(map,start,goal)
            dPath, dCost, dNodesExplored, dNodesExpanded, dNodesMaintained = dls(map,start,goal, 30)
            uPath, uCost, uNodesExplored, uNodesExpanded, uNodesMaintained = ucs(map,start,goal)
            if aNodesExplored <= bNodesExplored and aNodesExplored <= dNodesExplored and aNodesExplored <= uNodesExplored:
                aWins += 1
            if bNodesExplored <= aNodesExplored and bNodesExplored <= dNodesExplored and bNodesExplored <= uNodesExplored:
                bWins += 1
            if dNodesExplored <= aNodesExplored and dNodesExplored <= bNodesExplored and dNodesExplored <= uNodesExplored:
                dWins += 1
            if uNodesExplored <= aNodesExplored and uNodesExplored<= bNodesExplored and uNodesExplored <= dNodesExplored:
                uWins += 1
            aAvgCost += aCost
            aAvgNodesExplored += aNodesExplored
            aAvgNodesExpanded += aNodesExpanded
            aAvgNodesMaintained += aNodesMaintained
            bAvgCost += bCost
            bAvgNodesExplored += bNodesExplored
            bAvgNodesExpanded += bNodesExpanded
            bAvgNodesMaintained += bNodesMaintained
            dAvgCost += dCost
            dAvgNodesExplored += dNodesExplored
            dAvgNodesExpanded += dNodesExpanded
            dAvgNodesMaintained += dNodesMaintained
            uAvgCost += uCost
            uAvgNodesExplored += uNodesExplored
            uAvgNodesExpanded += uNodesExpanded
            uAvgNodesMaintained += uNodesMaintained
            file.write(path+":\n")
            file.write("    A* Path: "+str(aPath)+"\n")
            file.write("    A* Cost: "+str(aCost)+"\n")
            file.write("    A* Nodes Explored: "+str(aNodesExplored)+"\n")
            file.write("    A* Nodes Expanded: "+str(aNodesExpanded)+"\n")
            file.write("    A* Nodes Maintained: "+str(aNodesMaintained)+"\n")
            file.write("\n")
            file.write("    BFS Path: "+str(bPath)+"\n")
            file.write("    BFS Cost: "+str(bCost)+"\n")
            file.write("    BFS Nodes Explored: "+str(bNodesExplored)+"\n")
            file.write("    BFS Nodes Expanded: "+str(bNodesExpanded)+"\n")
            file.write("    BFS Nodes Maintained: "+str(bNodesMaintained)+"\n")
            file.write("\n")
            file.write("    DLS Path: "+str(dPath)+"\n")
            file.write("    DLS Cost: "+str(dCost)+"\n")
            file.write("    DLS Nodes Explored: "+str(dNodesExplored)+"\n")
            file.write("    DLS Nodes Expanded: "+str(dNodesExpanded)+"\n")
            file.write("    DLS Nodes Maintained: "+str(dNodesMaintained)+"\n")
            file.write("\n")
            file.write("    UCS Path: "+str(uPath)+"\n")
            file.write("    UCS Cost: "+str(uCost)+"\n")
            file.write("    UCS Nodes Explored: "+str(uNodesExplored)+"\n")
            file.write("    UCS Nodes Expanded: "+str(uNodesExpanded)+"\n")
            file.write("    UCS Nodes Maintained: "+str(uNodesMaintained)+"\n")
            file.write("\n")
            file.write("\n")
    aAvgCost = aAvgCost/len(paths)
    aAvgNodesExplored = aAvgNodesExplored/len(paths)
    aAvgNodesExpanded = aAvgNodesExpanded/len(paths)
    aAvgNodesMaintained = aAvgNodesMaintained/len(paths)
    bAvgCost = bAvgCost/len(paths)
    bAvgNodesExplored = bAvgNodesExplored/len(paths)
    bAvgNodesExpanded = bAvgNodesExpanded/len(paths)
    bAvgNodesMaintained = bAvgNodesMaintained/len(paths)
    dAvgCost = dAvgCost/len(paths)
    dAvgNodesExplored = dAvgNodesExplored/len(paths)
    dAvgNodesExpanded = dAvgNodesExpanded/len(paths)
    dAvgNodesMaintained = dAvgNodesMaintained/len(paths)
    uAvgCost = uAvgCost/len(paths)
    uAvgNodesExplored = uAvgNodesExplored/len(paths)
    uAvgNodesExpanded = uAvgNodesExpanded/len(paths)
    uAvgNodesMaintained = uAvgNodesMaintained/len(paths)
    with open('READEME.txt', 'w') as file:
        file.write("A* Average Cost: "+str(aAvgCost)+"\n")
        file.write("A* Average Nodes Explored: "+str(aAvgNodesExplored)+"\n")
        file.write("A* Average Nodes Expanded: "+str(aAvgNodesExpanded)+"\n")
        file.write("A* Average Nodes Maintained: "+str(aAvgNodesMaintained)+"\n")
        file.write("A* Wins: "+str(aWins)+"\n")
        file.write("\n")
        file.write("BFS Average Cost: "+str(bAvgCost)+"\n")
        file.write("BFS Average Nodes Explored: "+str(bAvgNodesExplored)+"\n")
        file.write("BFS Average Nodes Expanded: "+str(bAvgNodesExpanded)+"\n")
        file.write("BFS Average Nodes Maintained: "+str(bAvgNodesMaintained)+"\n")
        file.write("BFS Wins: "+str(bWins)+"\n")
        file.write("\n")
        file.write("DLS Average Cost: "+str(dAvgCost)+"\n")
        file.write("DLS Average Nodes Explored: "+str(dAvgNodesExplored)+"\n")
        file.write("DLS Average Nodes Expanded: "+str(dAvgNodesExpanded)+"\n")
        file.write("DLS Average Nodes Maintained: "+str(dAvgNodesMaintained)+"\n")
        file.write("DLS Wins: "+str(dWins)+"\n")
        file.write("\n")
        file.write("UCS Average Cost: "+str(uAvgCost)+"\n")
        file.write("UCS Average Nodes Explored: "+str(uAvgNodesExplored)+"\n")
        file.write("UCS Average Nodes Expanded: "+str(uAvgNodesExpanded)+"\n")
        file.write("UCS Average Nodes Maintained: "+str(uAvgNodesMaintained)+"\n")
        file.write("UCS Wins: "+str(uWins)+"\n")
        file.write("\n")

#Source wikipedia: https://en.wikipedia.org/wiki/A*_search_algorithm
def astar(map, start, goal):
    frontier = queue.PriorityQueue()
    frontier.put((0, start))
    visited = []
    cameFrom = {}
    gScore = {}
    gScore[start] = 0
    fScore = {}
    fScore[start] = map.getHeuristic(start, goal)
    numNodesExpanded = 0
    numNodesMaintained = 1
    while not frontier.empty():
        current = frontier.get()[1]
        numNodesMaintained -= 1
        if current == goal:
            return map.reconstructPath(cameFrom, current), map.getCost(cameFrom, current, start), len(visited), numNodesExpanded, numNodesMaintained
        visited.append(current)
        for neighbor in map.getNeighbors(current)[0]:
            tempGScore = gScore[current] + int(map.getEdgeCost(current, neighbor))
            if gScore.get(neighbor, None) is None or tempGScore < gScore[neighbor]:
                if neighbor not in gScore:
                    numNodesExpanded += 1
                cameFrom[neighbor] = current
                gScore[neighbor] = tempGScore
                fScore[neighbor] = gScore[neighbor] + map.getHeuristic(neighbor, goal)
                if neighbor not in visited:
                    frontier.put((fScore[neighbor], neighbor))
                    numNodesMaintained += 1
    print("No Path Found")
    return None, None, None, None, None

def bfs(map, start, goal):
    numNodesMaintained = 1
    numNodesExpanded = 0
    frontier = []
    visited = []
    frontier.append(start)
    cameFrom = {}
    while len(frontier) > 0:
        current = frontier.pop()
        numNodesMaintained -=1
        if current == goal:
            return map.reconstructPath(cameFrom, current), map.getCost(cameFrom, current, start), len(visited), len(cameFrom), numNodesMaintained
        if current not in visited:
            visited.append(current)
            for neighbor in map.getNeighbors(current)[0]:
                if neighbor not in visited:
                    frontier.append(neighbor)
                    numNodesMaintained += 1
                    numNodesExpanded += 1
                    cameFrom[neighbor] = current
    return None, None, None, None, None

def dls(map, start, goal, maxDepth):
    numNodesMaintained = 1
    cameFrom = {}
    depth = 0
    frontier = []
    visited = []
    frontier.append(start)
    while len(frontier) > 0:
        current = frontier.pop()
        numNodesMaintained -= 1
        if current == goal:
            return map.reconstructPath(cameFrom, current), map.getCost(cameFrom, current, start), len(visited), len(cameFrom), numNodesMaintained
        if current not in visited:
            visited.append(current)
            for i in range(len(map.getNeighbors(current)[0])):
                if map.getNeighbors(current)[0][i] not in visited:
                    frontier.append(map.getNeighbors(current)[0][i])
                    numNodesMaintained += 1
                    cameFrom[map.getNeighbors(current)[0][i]] = current

        if depth >= int(maxDepth):
            print("Max Depth Reached")
            return None, None, None, None, None
        depth += 1

def ucs(map, start, goal):
    numNodesMaintained = 1
    frontier = []
    heapq.heappush(frontier, (0, start))
    visited = []
    cameFrom = {}
    gScore = {}
    gScore[start] = 0
    while frontier:
        cost, current = heapq.heappop(frontier)
        numNodesMaintained -= 1
        if current not in visited:
            if current == goal:
                return map.reconstructPath(cameFrom, current), map.getCost(cameFrom, current, start), len(visited), len(cameFrom), numNodesMaintained
            visited.append(current)
            for neighbor in map.getNeighbors(current)[0]:
                tempGScore = gScore[current] + int(map.getEdgeCost(current, neighbor))
                if gScore.get(neighbor, None) is None or tempGScore < gScore[neighbor]:
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tempGScore
                    heapq.heappush(frontier, (gScore[neighbor], neighbor))
                    numNodesMaintained += 1
    print("No Path Found")
    return None, None, None, None, None

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

if __name__ == "__main__":
    main()