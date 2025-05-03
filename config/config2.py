from func import *
import pickle

link12_part = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 12.dxf")[0]
link11_part = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 11.dxf")[0]
link10_part = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 10.dxf")[0]
link1_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 1.dxf")[0]
link2_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 2.dxf")[0]
link3_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 3.dxf")[0]
link4_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 4.dxf")[0]
link5_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 5.dxf")[0]
link6_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 6.dxf")[0]
link7_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 7.dxf")[0]
link8_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 8.dxf")[0]
link9_part  = load_dxf_as_shapely_polygons("/Users/Viggo/Robot arm/Arm/part 9.dxf")[0]
# Define pivot points for each shape

link12 = Part(link12_part, position=(0,0))
link11 = Part(link11_part, position=(0,21.643*1/0.9))
link10 = Part(link10_part, position=(0,21.643))
link1= Part(link1_part, position=(0,21.643*0.9))  # relative to base
link2 = Part(link2_part, position=(0,19.479))  # relative to base
link3 = Part(link3_part, position=(0, 17.531))
link4 = Part(link4_part, position=(0, 15.778))
link5 = Part(link5_part, position=(0,14.2))
link6 = Part(link6_part, position=(0,12.78))
link7 = Part(link7_part, position=(0, 11.502))
link8 = Part(link8_part, position=(0, 10.352))
link9 = Part(link9_part, position=(0, 9.317), name="tipp")

parts = [link12, link11, link10, link1, link2, link3, link4, link5, link6, link7, link8, link9]

if __name__ == "__main__":
    angle_steps=(35,0,-35)
    with open("points_dict.pkl", "rb") as f:
        possible_points = pickle.load(f)
    with open("points_tree.pkl", "rb") as f:
        tree = pickle.load(f)
    #possible_points, tree = calculate_possible_points(parts, angle_steps=angle_steps)
    neighbors = calculate_neighbors(possible_points, angle_steps)
    if neighbors==None:
        exit()
    # Plot them for visualization
    import matplotlib.pyplot as plt

    x_vals, y_vals = zip(*possible_points)
    plt.figure()
    plt.scatter(x_vals, y_vals, color='red', s=10)
    plt.gca().set_aspect('equal')
    plt.title("All Reachable Tip Positions")
    plt.savefig("reachable_tip_positions.png")
    plt.show()

    with open("points_dict.pkl", "wb") as f:
        pickle.dump(possible_points, f)
    with open("points_tree.pkl", "wb") as f:
        pickle.dump(tree, f)
    with open("neighbors.pkl", "wb") as f:
        pickle.dump(neighbors, f)