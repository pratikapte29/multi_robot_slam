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


