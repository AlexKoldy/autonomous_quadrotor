# from heapq import heapify, heappush, heappop  # Recommended.
# import numpy as np


# from flightsim.world import World

# from occupancy_map import OccupancyMap  # Recommended.


from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap  # Recommended.


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    x_g, y_g, z_g = occ_map.index_to_metric_center(goal_index)
    x_s, y_s, z_s = occ_map.index_to_metric_center(start_index)

    shape = occ_map.map.shape
    hash_map = {}
    pq = []
    obs = np.argwhere(occ_map.map)

    # for i in range(shape[0]):
    #     for j in range(shape[1]):
    #         for k in range(shape[2]):
    #             if not occ_map.map[i, j, k]:
    #                 node = (i, j, k)
    #                 hash_map[node] = {
    #                     "cost": float("inf"),
    #                     "parent": None,
    #                     "in_pq": False,
    #                 }

    h_s = np.sqrt((x_g - x_s) ** 2 + (y_g - y_s) ** 2 + (z_g - z_s) ** 2)
    hash_map[start_index] = {
        "cost": 0 + h_s if astar else 0,
        "parent": None,
        "in_pq": True,
    }
    heappush(pq, (0, start_index))
    visited = set()
    for row in obs:
        visited.add(tuple(row))
    while goal_index not in visited:
        cost, node = heappop(pq)
        if node == goal_index:
            break
        hash_map[node]["in_pq"] = False
        visited.add(node)

        for x in range(-1, 2):
            for y in range(-1, 2):
                for z in range(-1, 2):
                    x_n = node[0] + x
                    y_n = node[1] + y
                    z_n = node[2] + z
                    if (
                        x_n < 0
                        or x_n >= shape[0]
                        or y_n < 0
                        or y_n >= shape[1]
                        or z_n < 0
                        or z_n >= shape[2]
                    ):
                        continue

                    neighbor = (x_n, y_n, z_n)

                    if neighbor in visited:
                        continue
                    else:
                        x_u, y_u, z_u = occ_map.index_to_metric_center(node)
                        x_v, y_v, z_v = occ_map.index_to_metric_center(neighbor)
                        distance = np.sqrt(
                            (x_v - x_u) ** 2 + (y_v - y_u) ** 2 + (z_v - z_u) ** 2
                        )
                        d = hash_map[node]["cost"] + distance
                        h = 0
                        if astar:
                            h += np.sqrt(
                                (x_g - x_v) ** 2 + (y_g - y_v) ** 2 + (z_g - z_v) ** 2
                            )
                        if neighbor in hash_map:
                            if d < hash_map[neighbor]["cost"]:
                                hash_map[neighbor]["cost"] = d
                                hash_map[neighbor]["parent"] = node
                        else:
                            hash_map[neighbor] = {
                                "cost": d,
                                "parent": node,
                                "in_pq": False,
                            }

                        if not hash_map[neighbor]["in_pq"]:
                            heappush(pq, (d + h, neighbor))
                            hash_map[neighbor]["in_pq"] = True

    node = goal_index
    path = []
    path.append(goal)
    path.append(occ_map.index_to_metric_center(node))
    while node != start_index:
        node = hash_map[node]["parent"]
        path.append(occ_map.index_to_metric_center(node))
    path.append(start)
    path.reverse()
    # Return a tuple (path, nodes_expanded)
    return np.array(path), len(visited)
