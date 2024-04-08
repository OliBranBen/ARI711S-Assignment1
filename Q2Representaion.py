class Location:
    # Dictionary to store distances to other locations
    def __init__(self, name):
        self.name = name
        self.distances = {}  

    def add_distance(self, location, distance):
        self.distances[location] = distance

    def get_distance(self, location):
        return self.distances.get(location, float('inf'))


class DistanceGraph:
    def __init__(self):
        self.locations = []

    def add_location(self, name):
        location = Location(name)
        self.locations.append(location)
        return location

    def add_distance(self, location1, location2, distance):
        location1.add_distance(location2, distance)
        location2.add_distance(location1, distance)

    def get_distance(self, location1, location2):
        return location1.get_distance(location2)
    
    def calculate_total_distance(self, route):
        total_distance = 0
        for i in range(len(route) - 1):
            current_location = route[i]
            next_location = route[i + 1]
            distance = self.get_distance(current_location, next_location)
            total_distance += distance
        return total_distance

graph = DistanceGraph()

# Adding locations
A = graph.add_location("Dorado Park")
B = graph.add_location("Khomasdal")
C = graph.add_location("Katutura")
D = graph.add_location("Eros")
E = graph.add_location("Klein Windhoek")

# Adding distances between locations
#Dorado
graph.add_distance(A, A, 0)
graph.add_distance(A, B, 7)
graph.add_distance(A,C,20)
graph.add_distance(A,D,15)
graph.add_distance(A,E,12)

#Khomasdal
graph.add_distance(B, A, 10)
graph.add_distance(B, B, 0)
graph.add_distance(B,C,6)
graph.add_distance(B,D,14)
graph.add_distance(B,E,18)

#Katutura
graph.add_distance(C, A, 20)
graph.add_distance(C, B, 6)
graph.add_distance(C,C,0)
graph.add_distance(C,D,15)
graph.add_distance(C,E,30)

#Eros
graph.add_distance(D, A, 15)
graph.add_distance(D, B, 14)
graph.add_distance(D,C,25)
graph.add_distance(D,D,0)
graph.add_distance(D,E,2)

#Klein Windhoek
graph.add_distance(E, A, 12)
graph.add_distance(E, B, 18)
graph.add_distance(E,C,30)
graph.add_distance(E,D,2)
graph.add_distance(E,E,0)

# Calculate total distance for a given route
route = [A, B, C, D, E]
total_distance = graph.calculate_total_distance(route)


# Getting distance between two locations and total distance
print("Distance from Dorado Park to Kelin Windhoek:", graph.get_distance(A, E))
print("Total distance for the given route:", total_distance)