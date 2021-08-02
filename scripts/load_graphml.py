import networkx as nx
import matplotlib.pyplot as plt
from icecream import ic
import numpy as np

def load_rrt_gragh_2_maps(filename):

    G = nx.read_graphml(filename)
    nx.draw(G)
    plt.show()

    # Save the relationship of nodes: key->target(son), value->source(father)
    node_pos_map = dict()  # key->node name, value->position, np.array->(x, y)
    son_father_map = dict()
    edge_weight_map = dict()
    edge_connect_father_son_map = dict()

    for node in G.nodes(data=True):

        coords = node[1]["coords"].split(",")
        x = float(coords[0])
        y = float(coords[1])
        yaw = float(coords[2])
        xy = np.array([[x, y]])
        node_pos_map[node[0]] = xy
    # ic(node_pos_map)

    for edge in G.edges(data=True):

        son_father_map[edge[1]] = edge[0]
        edge_weight_map[edge[2]["id"]] = edge[2]["weight"]
        edge_connect_father_son_map[edge[2]["id"]] = (edge[1], edge[0])
    # ic(son_father_map)
    # ic(edge_weight)
    # ic(edge_connect_father_son_map)

    return node_pos_map, son_father_map, edge_weight_map, edge_connect_father_son_map

if __name__ == "__main__":

    node_pos_map, son_father_map, edge_weight_map, edge_connect_father_son_map = load_rrt_gragh_2_maps(filename="graph.graphml")