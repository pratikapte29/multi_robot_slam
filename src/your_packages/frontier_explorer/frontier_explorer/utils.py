import numpy as np
import math 
from scipy.ndimage import binary_dilation
from sklearn.cluster import DBSCAN


def cell_entropy(p: float) -> float:
    """_summary_

    Args:
        p (float): Probability of a cell being occupied

    Returns:
        float: Entropy of cell
    """

    if p <= 0 or p >= 1:
        return 0.0
    
    return -(p * math.log2(p) + (1 - p) * math.log2(1 - p))


# def map_entropy(map: np.ndarray) -> float:
#     """Computes total map entropy by only taking into consideration cells that are "unknown".
#     Map input is from nav_msgs/msg/OccupancyGrid Message which by convention has unknown regions valued at -1

#     Args:
#         map (np.ndarray): current available map 

#     Returns:
#         float: _description_
#     """

#     return float(np.sum(map == -1))


def compute_information_gain(
        map: np.ndarray,
        frontier_cells: np.ndarray,
        sensor_range_cells: int
) -> dict:
    """_summary_

    Args:
        map (np.ndarray): _description_
        frontier_cells (np.ndarray): _description_
        sensor_range_cells (int): _description_

    Returns:
        dict: Dictionary containing information gains for frontier cells
    """
    
    info_gains = {}

    for cell in frontier_cells:
        r, c = int(cell[0]), int(cell[1])
        r_min = max(0, r - sensor_range_cells)
        r_max = min(map.shape[0], r + sensor_range_cells + 1)
        c_min = max(0, c - sensor_range_cells)
        c_max = min(map.shape[1], c + sensor_range_cells + 1)

        # region to be checked for unknown cells:
        region = map[r_min: r_max, c_min: c_max]
        # check for total number of unknown cells within the region covered by lidar
        total = 0.0
        for val in region.flat:
            if val == -1:
                p = 0.5        # unknown → maximum uncertainty
            elif val == 0:
                p = 0.01       # free
            else:
                p = val / 100.0  # occupied
            total += cell_entropy(p)
        
        info_gains[tuple(cell)] = total

    return info_gains


def compute_motion_cost(robot_pos: tuple, subgoal_pos: tuple) -> float:
    """Computes motion cost to reach subgoals at the frontiers

    Args:
        robot_pos (tuple): Current position of robot
        subgoal_pos (tuple): Frontier cell position 

    Returns:
        float: motion cost
    """

    motion_cost = np.sqrt((subgoal_pos[0] - robot_pos[0])**2 + (subgoal_pos[1] - robot_pos[1])**2)

    return motion_cost


def find_frontiers(map: np.ndarray) -> np.ndarray:
    """_summary_

    Args:
        map (np.ndarray): _description_

    Returns:
        np.ndarray: _description_
    """

    free_mask = (map == 0)
    unknown_mask = (map == -1)

    # using 4-neighbor connectivity
    kernel = np.array([
        [0, 1, 0],
        [1, 1, 1],
        [0, 1, 0]
    ], dtype=bool)

    unknown_dilated = binary_dilation(unknown_mask, structure=kernel)
    occupied_mask = (map > 0)
    occupied_dilated = binary_dilation(occupied_mask, structure=kernel)
    frontier_mask = free_mask & unknown_dilated & ~occupied_dilated

    frontier_cells = np.argwhere(frontier_mask)

    return frontier_cells


def cluster_frontiers(
        frontier_cells: np.ndarray,
        cluster_size: int = 5
) -> list[np.ndarray]:
    if len(frontier_cells) == 0:
        return []
    
    labels = DBSCAN(eps=3.0, min_samples=cluster_size).fit_predict(frontier_cells)

    clusters = []
    for label in set(labels):
        if label == -1:
            continue
        clusters.append(frontier_cells[labels == label])

    return clusters


def frontier_centroid(cluster: np.ndarray) -> tuple:
    centroid = np.mean(cluster, axis=0)

    distances = np.linalg.norm(cluster - centroid, axis=1)

    best_cell = cluster[np.argmin(distances)]

    return tuple(best_cell)


def compute_utility(info_gain: float, motion_cost: float, lambda_: float = 0.4) -> float:
    return info_gain - lambda_ * motion_cost


def map_to_world(cell: tuple, map_origin: tuple, resolution: float) -> tuple:
    x = map_origin[0] + cell[1] * resolution
    y = map_origin[1] + cell[0] * resolution
    return (x, y)


def world_to_map(point: tuple, map_origin: tuple, resolution: float) -> tuple:
    col = int((point[0] - map_origin[0]) / resolution)
    row = int((point[1] - map_origin[1]) / resolution)
    return (row, col)