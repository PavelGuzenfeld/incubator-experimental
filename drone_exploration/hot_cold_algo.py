import numpy as np
import matplotlib.pyplot as plt
import imageio
import time

# Define the grid size and the radiation source
grid_size = 100
source_position = (70, 80)  # The coordinates of the radiation source


# Create a heatmap with a radiation source
def create_heatmap(grid_size, source_position):
    heatmap = np.zeros((grid_size, grid_size))
    for i in range(grid_size):
        for j in range(grid_size):
            distance = np.sqrt(
                (i - source_position[0]) ** 2 + (j - source_position[1]) ** 2
            )
            heatmap[i, j] = 1 / (
                distance + 1
            )  # Radiation intensity decreases with distance
    return heatmap


true_heatmap = create_heatmap(grid_size, source_position)


# Mock function to simulate getting RSSI value based on the drone's position
def get_rssi(position):
    x, y = position
    true_rssi = true_heatmap[x, y] * 100  # Base RSSI value from the heatmap
    noise = np.random.normal(
        0, 5
    )  # Add Gaussian noise with mean 0 and standard deviation 5
    return max(0, true_rssi + noise)  # Ensure RSSI is non-negative


# Calculate the density of high RSSI points around a position
def calculate_density(position, discovered_heatmap, radius=3):
    x, y = position
    neighborhood = discovered_heatmap[
        max(0, x - radius) : min(grid_size, x + radius + 1),
        max(0, y - radius) : min(grid_size, y + radius + 1),
    ]
    return np.sum(neighborhood > 0.5)  # Count points with normalized RSSI > 0.5


# Update the discovered heatmap with RSSI values for all points passed through
def update_discovered_heatmap(path, discovered_heatmap):
    for pos in path:
        rssi = get_rssi(pos)
        discovered_heatmap[pos[0], pos[1]] = rssi / 100.0  # Normalize RSSI for heatmap


# Simulate drone movement towards the source
def move_drone_towards_source(position, discovered_heatmap, visited_map, stuck_counter):
    x, y = position
    best_rssi = get_rssi((x, y))
    best_position = (x, y)
    best_density = calculate_density((x, y), discovered_heatmap)

    # Determine step size based on density (higher density, smaller steps)
    step_size = max(1, 10 - best_density)

    path = [position]

    for dx in [-step_size, 0, step_size]:
        for dy in [-step_size, 0, step_size]:
            if dx == 0 and dy == 0:
                continue
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < grid_size and 0 <= new_y < grid_size:
                new_rssi = get_rssi((new_x, new_y))
                density = calculate_density((new_x, new_y), discovered_heatmap)
                # Add penalty for revisiting areas
                if visited_map[new_x, new_y]:
                    new_rssi -= 20  # Penalty value, adjust as needed
                if (new_rssi > best_rssi) or (
                    new_rssi == best_rssi and density > best_density
                ):
                    best_rssi = new_rssi
                    best_density = density
                    best_position = (new_x, new_y)
                discovered_heatmap[new_x, new_y] = (
                    new_rssi / 100.0
                )  # Normalize RSSI for heatmap
                path.append((new_x, new_y))

    if best_position == position:
        stuck_counter += 1
    else:
        stuck_counter = 0

    # If stuck for too long, force random exploration
    if stuck_counter > 5:
        density = calculate_density(position, discovered_heatmap)
        step_size = max(1, 10 - density)  # Determine step size based on local density
        while visited_map[
            best_position[0], best_position[1]
        ]:  # Ensure it moves to an unvisited position
            best_position = (
                x + np.random.randint(-step_size, step_size + 1),
                y + np.random.randint(-step_size, step_size + 1),
            )
            best_position = (
                max(0, min(grid_size - 1, best_position[0])),  # Ensure within bounds
                max(0, min(grid_size - 1, best_position[1])),  # Ensure within bounds
            )
        path.append(best_position)
        stuck_counter = 0

    visited_map[best_position[0], best_position[1]] = (
        True  # Mark the new position as visited
    )

    return best_position, stuck_counter, path


# Plot the heatmap, discovered heatmap, and density map
def plot_heatmaps(true_heatmap, discovered_heatmap, density_map, path, frames):
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))

    axs[0].imshow(true_heatmap, cmap="hot", interpolation="nearest")
    axs[0].set_title("True Heatmap")

    axs[1].imshow(discovered_heatmap, cmap="hot", interpolation="nearest")
    path = np.array(path)
    if len(path) > 0:
        axs[1].plot(path[:, 1], path[:, 0], "bo-")  # Plot the path of the drone
    axs[1].set_title("Discovered Heatmap")

    axs[2].imshow(density_map, cmap="hot", interpolation="nearest")
    axs[2].set_title("Discovered Density Map")

    plt.savefig("frame.png")
    frames.append(imageio.imread("frame.png"))
    plt.close()


# Main function to find the radiation source
def find_source(initial_position, max_steps=200):
    position = initial_position
    path = [position]
    frames = []
    stuck_counter = 0

    discovered_heatmap = np.zeros((grid_size, grid_size))
    density_map = np.zeros((grid_size, grid_size))
    visited_map = np.zeros((grid_size, grid_size), dtype=bool)

    for step in range(max_steps):
        new_position, stuck_counter, new_path = move_drone_towards_source(
            position, discovered_heatmap, visited_map, stuck_counter
        )
        position = new_position
        path.extend(new_path)
        update_discovered_heatmap(new_path, discovered_heatmap)
        density_map[position[0], position[1]] = calculate_density(
            position, discovered_heatmap
        )
        plot_heatmaps(true_heatmap, discovered_heatmap, density_map, path, frames)
        time.sleep(0.1)  # Simulate time delay for drone movement

        # Check if the drone has found the source
        if position == source_position:
            print("Source found at position:", position)
            break

    imageio.mimsave("drone_exploration.gif", frames, fps=2)


if __name__ == "__main__":
    initial_position = (10, 10)  # Starting position of the drone
    find_source(initial_position)
