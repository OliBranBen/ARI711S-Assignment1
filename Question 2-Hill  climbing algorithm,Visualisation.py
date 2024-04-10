import random
import matplotlib.pyplot as plt

# User input for total number of places,user wants to visit
total_places = int(input('How many places do you want to visit?: '))

# Set of places user wants to visit
places = ["Khomasdal", "Zoo Park", "Namibia Craft Centre", "National Museum", "Heroes Acre", "Christuskirche"]

# List of  distances between the places mentioned above
distances = [
    [0, 5.7, 5.3, 5.5, 15.4, 6.4],
    [5.8, 0, 0.8, 0.5, 11, 0.45],
    [5.3, 0.8, 0, 1.5, 10.5, 0.95],
    [5.5, 0.5, 1.6, 0, 12.9, 0.55],
    [15.4, 12.5, 10.6, 12.8, 0, 12.5],
    [6.4, 0.45, 0.95, 0.5, 12.5, 0]
]


# Function to  randomly select initial route
def randomize_place():
    selected_initial_route = places.copy()
    place = (random.sample(selected_initial_route, k=total_places))

    return place


# Function to calculate total distance of a randomly selected route
def total_distance(place, places_list, distance_matrix):
    distance_total = 0
    # Loop to iterate through array list of places provided by user
    for i in range(len(place) - 1):
        current_location = places_list.index(place[i])
        next_location = places_list.index(place[i + 1])
        # Retrieves distance from distance list,using distance index inorder to calculate total distance
        distance = distance_matrix[current_location][next_location]
        distance_total += distance
    return distance_total


# Function to generate the neighboring routes of the randomly selected initial route
def route_neighbours(initial_route):
    neighbours = []
    for i in range(len(initial_route)):
        for j in range(i + 1, len(initial_route)):
            # Swap positions of two places that are randomly selected
            neighbour = initial_route.copy()
            neighbour[i], neighbour[j] = neighbour[j], neighbour[i]
            # Check if the new route contains unique places,used to eliminate duplicate places
            if len(set(neighbour)) == len(neighbour):
                neighbours.append(neighbour)
    return neighbours


# Hill climbing algorithm function to find the best route out of the generated neighboring routes
def hill_climbing():
    current_route = randomize_place()
    current_distance = total_distance(current_route, places, distances)

    while True:
        neighbouring_routes = route_neighbours(current_route)
        best_neighbor_distance = float('inf')
        best_neighbor_route = None

        for neighbour in neighbouring_routes:
            neighbour_distance = total_distance(neighbour, places, distances)
            if neighbour_distance < best_neighbor_distance:
                best_neighbor_distance = neighbour_distance
                best_neighbor_route = neighbour

        if best_neighbor_distance < current_distance:
            current_route = best_neighbor_route
            current_distance = best_neighbor_distance
        else:
            break

    return current_route, current_distance


final_route, final_distance = hill_climbing()
route = randomize_place()
total_length = total_distance(route, places, distances)
neighbouring_routes = route_neighbours(route)

# Print the initial route and its total distance
print("Initial route:", route)
print("Initial distance:", total_length)

# Print neighboring routes with their distances
print("\nNeighboring routes:")
for i, route in enumerate(neighbouring_routes):
    print(f"Neighbour {i + 1}: {route}, Distance: {round(total_distance(route, places, distances), 2)}")

print("\n Optimal route found from neighboring route evaluation")

# Print the final route and its distance
print("Final route:", final_route)
print("Final distance:", round(final_distance, 2))

# Coordinates of each place
coordinates = {
    "Khomasdal": (0, 0),
    "Zoo Park": (1, 2),
    "Namibia Craft Centre": (2, 4),
    "National Museum": (3, 3),
    "Heroes Acre": (4, 5),
    "Christuskirche": (5, 3.5)
}


# Function to plot a given route
def plot_route(route, color='b', label=None):
    x_coords = [coordinates[place][0] for place in route]
    y_coords = [coordinates[place][1] for place in route]
    plt.plot(x_coords, y_coords, marker='o', color=color, linestyle='--', linewidth=2, markersize=8, label=label)


x_positions_labels = [0, 1, 2, 3, 4, 5]

# Plot of the initial and final routes on a graph
plt.figure(figsize=(5, 5))
plot_route(route, color='r', label='Initial Route')
plot_route(final_route, color='g', label='Final Route')
plt.title('Routes')
plt.xlabel('Places')
plt.ylabel('Distances')
plt.legend()
plt.grid(True)
plt.xticks(x_positions_labels,
           ['Khomasdal', 'Zoo Park', 'Namibia Craft Centre', 'National Museum', 'Heroes Acre', 'Christuskirche'],
           rotation=45)
plt.tight_layout()
plt.show()
