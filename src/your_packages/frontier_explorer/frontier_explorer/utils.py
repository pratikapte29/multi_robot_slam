import numpy as np
import math 


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


def map_entropy(map: np.ndarray) -> float:
    """Computes total map entropy by only taking into consideration cells that are "unknown".
    Map input is from nav_msgs/msg/OccupancyGrid Message which by convention has unknown regions valued at -1

    Args:
        map (np.ndarray): current available map 

    Returns:
        float: _description_
    """

    return float(np.sum(map == -1))


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
        info_gains[tuple(cell)] = float(np.sum(region == -1))  

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
