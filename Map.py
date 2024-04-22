import City
import math
class Map:
 
    def __init__(self, edges = {}, nodes = [], coordinates = {}):
        self.edges = edges
        self.nodes = []
        self.coordinates = coordinates
        for node in nodes:
            self.nodes.append(City.City(node, coordinates[node], edges[node][0]))
    
    def getNeighbors(self, node):
        for n in self.nodes:
            if n.name == node:
                return [n.neighbors]
    
    def getCoordinates(self, node):
        for n in self.nodes:
            if n.name == node:
                return n.coordinates

    def getHeuristic(self,current, goal):
        # current and goal are formatted as [degrees, minutes, seconds, direction, degrees, minutes, seconds, direction]
        currentCoords = self.getCoordinates(current)
        goalCoords = self.getCoordinates(goal)
        currentLat = self.getDegrees(currentCoords[0:3])
        currentLong = self.getDegrees(currentCoords[4:7])
        goalLat = self.getDegrees(goalCoords[0:3])
        goalLong = self.getDegrees(goalCoords[4:7])
        return math.sqrt((currentLat-goalLat)**2 + (currentLong-goalLong)**2)

    def getDegrees(self,coords):
        return float(coords[0]) + (float(coords[1])/60) + (float(coords[2])/3600)

    def getEdgeCost(self, node1, node2):
        for edge in range(0,len(self.edges[node1])):
            if self.edges[node1][0][edge] == node2:
                return self.edges[node1][1][edge]
        return 0

    def reconstructPath(self,cameFrom, current):
        totalPath = [current]
        while current in cameFrom.keys():
            current = cameFrom[current]
            totalPath.insert(0, current)
        return totalPath


    def getCost(self,cameFrom, goal, start):
        totalCost = 0 
        current = goal
        while current != start:
            for i in range(len(self.edges[current][0])):
                if self.edges[current][0][i] == cameFrom[current]:
                    totalCost += int(self.edges[current][1][i])
                    current = cameFrom[current]
                    break
        return totalCost


