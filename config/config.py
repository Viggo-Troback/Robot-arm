from func import *
import pickle


standard_part = load_dxf_as_shapely_polygons("/Users/Viggo/Downloads/DXF_files/topp2.dxf")[0]
base_part = load_dxf_as_shapely_polygons("/Users/Viggo/Downloads/DXF_files/bas2.dxf")[0]
tipp_part = load_dxf_as_shapely_polygons("/Users/Viggo/DownloadsDXF_files//tipp.dxf")[0]
# Define pivot points for each shape
base = Part(base_part, position=(0, 0))
link1 = Part(standard_part, position=(0, 25))  # relative to base
link2 = Part(standard_part)  # relative to link1
tipp=Part(tipp_part, name="tipp")

parts = [base, link1, link2, Part(standard_part),Part(standard_part), Part(standard_part), Part(standard_part), Part(standard_part), tipp]

if __name__ == "__main__":
    angle_steps=(60,0-60)
    possible_points, tree = calculate_possible_points(parts, angle_steps=(-60, 0, 60))
    neighbors = calculate_neighbors(possible_points)

    # Plot them for visualization
    import matplotlib.pyplot as plt

    x_vals, y_vals = zip(*possible_points)
    plt.figure()
    plt.scatter(x_vals, y_vals, color='red', s=10)
    plt.gca().set_aspect('equal')
    plt.title("All Reachable Tip Positions")
    plt.show()


    with open("points_dict.pkl", "wb") as f:
        pickle.dump(possible_points, f)
