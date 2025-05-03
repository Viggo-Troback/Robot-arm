import ezdxf
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon as ShapelyPolygon, MultiPolygon
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
from shapely.ops import linemerge, unary_union, polygonize
from shapely.affinity import rotate, translate
from tqdm import tqdm
from matplotlib.textpath import TextPath
from matplotlib.patches import PathPatch
from scipy.spatial import KDTree

def to_xy(vec3):
    return (vec3.x, vec3.y)


def snap_coords(geom, precision=4):
    """
    Round coordinates to improve connectivity during polygonization.
    """
    def round_pt(pt):
        return tuple(round(coord, precision) for coord in pt)
    return LineString([round_pt(pt) for pt in geom.coords])


def load_dxf_as_shapely_polygons(filename, arc_segments=50, snap_precision=4):
    """
    Load a DXF file and convert its shapes to a list of shapely Polygon objects.
    Handles LINE, LWPOLYLINE, ARC, CIRCLE, SPLINE.
    """
    doc = ezdxf.readfile(filename)
    msp = doc.modelspace()
    segments = []

    for entity in msp:
        dtype = entity.dxftype()

        if dtype == 'LINE':
            start = to_xy(entity.dxf.start)
            end = to_xy(entity.dxf.end)
            segments.append(LineString([start, end]))

        elif dtype == 'LWPOLYLINE':
            points = [(point[0], point[1]) for point in entity]
            if entity.closed:
                points.append(points[0])
            segments.append(LineString(points))

        elif dtype == 'ARC':
            center = to_xy(entity.dxf.center)
            radius = entity.dxf.radius
            start_angle = np.deg2rad(entity.dxf.start_angle)
            end_angle = np.deg2rad(entity.dxf.end_angle)

            if end_angle < start_angle:
                end_angle += 2 * np.pi

            angles = np.linspace(start_angle, end_angle, arc_segments)
            arc_points = [(center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)) for a in angles]
            segments.append(LineString(arc_points))

        elif dtype == 'CIRCLE':
            center = to_xy(entity.dxf.center)
            radius = entity.dxf.radius
            angles = np.linspace(0, 2 * np.pi, arc_segments, endpoint=False)
            circle_points = [(center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)) for a in angles]
            circle_points.append(circle_points[0])  # close it
            segments.append(LineString(circle_points))

        elif dtype == 'SPLINE':
            spline_points = entity.approximate(arc_segments)
            points = [tuple(p) for p in spline_points]
            segments.append(LineString(points))

    if not segments:
        raise ValueError("No drawable geometry found in DXF.")

    # Snap all segments to help with precision issues
    segments = [snap_coords(seg, precision=snap_precision) for seg in segments]

    # Merge connected lines and polygonize
    merged = linemerge(unary_union(segments))
    polygons = list(polygonize(merged))

    return polygons

def apply_pivot_transform(polygon, pivot, angle_deg=0, position=(0, 0)):
    # Rotate around pivot
    rotated = rotate(polygon, angle_deg, origin=pivot, use_radians=False)
    # Translate to global position
    translated = translate(rotated, xoff=position[0], yoff=position[1])
    return translated

def plot_shapely_polygons_with_pivots(polygons, pivots, angles=None, positions=None):
    fig, ax = plt.subplots()
    patches = []

    if angles is None:
        angles = [0] * len(polygons)
    if positions is None:
        positions = [(0, 0)] * len(polygons)

    for i, polygon_group in enumerate(polygons):
        pivot = pivots[i]
        angle = angles[i]
        position = positions[i]

        for poly in polygon_group:
            if not poly.is_empty:
                transformed = apply_pivot_transform(poly, pivot, angle, position)
                x, y = transformed.exterior.xy
                patches.append(MplPolygon(list(zip(x, y)), closed=True))

        # Also show the pivot point (after translation)
        px, py = pivot
        ax.plot(px + position[0], py + position[1], 'ro')  # red dot for pivot

    collection = PatchCollection(patches, facecolor='lightblue', edgecolor='black', linewidths=1)
    ax.add_collection(collection)
    ax.autoscale()
    ax.set_aspect('equal')
    plt.title("Transformed Shapes with Pivots")
    plt.show()

class Part:
    def __init__(self, polygon, pivot=(0,0), angle=0, position=(0, 45.735), name="standard"):
        self.polygon = polygon
        self.pivot = pivot
        self.angle = angle
        self.name = name
        self.position = position  # Global position of pivot
        self.transformed_polygon = None

    def transform(self, parent_angle=0, parent_position=(0, 0)):
        # Total angle relative to base
        total_angle = parent_angle + self.angle

        # Position is relative to parent pivot
        x, y = self.position
        px, py = parent_position
        global_position = (
            px + x * np.cos(np.radians(parent_angle)) - y * np.sin(np.radians(parent_angle)),
            py + x * np.sin(np.radians(parent_angle)) + y * np.cos(np.radians(parent_angle))
        )

        from shapely.affinity import rotate, translate
        rotated = rotate(self.polygon, total_angle, origin=self.pivot, use_radians=False)
        translated = translate(rotated, xoff=global_position[0], yoff=global_position[1])
        self.transformed_polygon = translated

        return total_angle, global_position
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection

def plot_transformed_parts(parts, point):
    fig, ax = plt.subplots()
    patches = []

    for part in parts:
        poly = part.transformed_polygon
        if poly:
            x, y = poly.exterior.xy
            patches.append(MplPolygon(list(zip(x, y)), closed=True))
    plt.plot(point[0], point[1], 'ro')
    collection = PatchCollection(patches, facecolor='lightblue', edgecolor='black', linewidths=1)
    ax.add_collection(collection)
    ax.autoscale()
    ax.set_aspect('equal')
    plt.title("Transformed Robot Parts")
    plt.show()

from itertools import product
import numpy as np

def calculate_possible_points(parts, angle_steps):
    point_to_angles = {}

    n = len(parts)

    # Force the first angle to 0, vary the rest
    combos = list(product(angle_steps, repeat=n - 1))  # store to know total length for tqdm

    for rest_combo in tqdm(combos, desc="Calculating points"):
        combo = (0,) + rest_combo  # base part fixed at 0

        parts, global_tip = forward_kinematics(parts, combo)

        # Round position to 2 decimal places to use as key
        rounded_point = tuple(np.round(global_tip, 2))

        if rounded_point in point_to_angles:
            point_to_angles[rounded_point].append(combo)
        else:
            point_to_angles[rounded_point] = [combo]


    points = list(point_to_angles.keys())
    tree = KDTree(points)


    return point_to_angles, tree

def get_combo_neighbors(combo, angle_steps_set):
    neighbors = []
    for i in range(1, len(combo)):  # skip joint 0 (fixed at 0)
        for delta in [-1, 1]:
            new_angle = combo[i] + delta
            if new_angle in angle_steps_set:
                new_combo = list(combo)
                new_combo[i] = new_angle
                neighbors.append(tuple(new_combo))
    return neighbors

def calculate_neighbors(point_to_angles, angle_steps):
    reversed_dict = {}
    for k, v in point_to_angles.items():

        for key in v:
            if key in reversed_dict:
                reversed_dict[key].append(k)
            else:
                reversed_dict[key] = [k]

    neighbors={pt: [] for pt in point_to_angles}
    first_key = next(iter(point_to_angles))
    n = len(point_to_angles[first_key][0])
    sample = next(iter(reversed_dict))
     # Force the first angle to 0, vary the rest
    combos = list(product(angle_steps, repeat=n - 1))  # store to know total length for tqdm

    if len(combos) != len(reversed_dict):  #This has to be the length of combos to avoid issus later since keys needs to be exclusive.
        print("error, dict not correct length")
        return None
    for rest_combo in tqdm(combos, desc="Finding neighbours"):
        combo = (0,) + rest_combo  # base part fixed at 0
        key=(reversed_dict[combo])[0] #find the key for combo
        for a in range(1,n):
            for b in angle_steps:
                combo_perm=list(combo)
                combo_perm[a]=b
                combo_perm = tuple(combo_perm)
                neighbors[key].append(reversed_dict[combo_perm]) #This could raise issues if keys not exclusive
    return neighbors

def forward_kinematics(parts, angles):
    """Calculates the endpoint by adding one part at the time and then the tip"""
    #This could probably be done faster by some mathfunction for combos and partlengths? This works for now. 
    angle = 0
    position = (0, 0)
    for i, part in enumerate(parts):
        part.angle=angles[i]
        angle, position = part.transform(parent_angle=angle, parent_position=position)
        if part.name=="tipp":
            x, y = (0,11.74)
            px, py = position
            global_position = (
                px + x * np.cos(np.radians(angle)) - y * np.sin(np.radians(angle)),
                py + x * np.sin(np.radians(angle)) + y * np.cos(np.radians(angle))
            )
            # Local tip point relative to the pivot
            point=global_position
    return parts, point

def closest_match(aim, possible_points,tree):
    dist, idx = tree.query(aim)
    points = list(possible_points.keys())
    closest_point = points[idx]
    return closest_point

def points_within_radius(aim, possible_points, tree, radius):
    """Like closest_match but uses a radius to find more neighbours instead of only the closest"""
    indices = tree.query_ball_point(aim, radius)
    points = list(possible_points.keys())
    nearby_points =[]
    for i in indices:
        nearby_points.append(points[i])
    if len(nearby_points)==0:
        dist, idx = tree.query(aim) #guarentees atleast one neighbour.
        nearby_points=points_within_radius(aim,possible_points,tree, dist*1.1)
        
    return nearby_points

def text_to_points(text, scale, plot=True):
    tp = TextPath((0, 0), text, size=scale)
    vertices = tp.vertices  # Nx2 array of (x, y) points
    codes = tp.codes        # Path drawing instructions (MOVETO, LINETO, etc.)

    if plot==True:
        # Step 3: (Optional) visualize the path
        fig, ax = plt.subplots()
        patch = PathPatch(tp, facecolor='none', edgecolor='black')
        ax.add_patch(patch)
        ax.set_aspect('equal')
        ax.autoscale()
        plt.title("Text as Vector Path")
        plt.show()
    
    return vertices, codes


def map_to_polar_field(points, r1, r2, angle_start=np.pi, angle_end=0,origin=(0, 50), plot=True):
    points = np.array(points)
    xs, ys = points[:, 0], points[:, 1]

    # Normalize x and y
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    xs_norm = (xs - x_min) / (x_max - x_min)  # 0 to 1
    ys_norm = (ys - y_min) / (y_max - y_min)  # 0 to 1

    # Map to angle and radius
    angles = angle_start + xs_norm * (angle_end - angle_start)
    radii = r1 + ys_norm * (r2 - r1)

    # Polar to Cartesian, then apply origin offset
    new_xs = radii * np.cos(angles) + origin[0]
    new_ys = radii * np.sin(angles) + origin[1]

    new_points = list(zip(new_xs, new_ys))

    if plot==True:
        # Plotting the points in a polar band
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.autoscale()

        # Extract X and Y from points for plotting
        x_vals, y_vals = zip(*points)
        
        # Plot points
        ax.scatter(x_vals, y_vals, color='red', s=10)
        
        # Set labels, title, and other plot properties
        plt.title("Text as Points in Polar Band")
        plt.show()

    return new_points

def pathfinder(start, goal, neighbors): #BFS, could be done with something like A* maybe?
    queue = [[start]]
    visited = set([start])

    while queue:
        path = queue.pop(0)
        current = path[-1]

        if current == goal:
            return path  # Found the shortest path

        for neighbor in neighbors.get(current, []):
            if neighbor[0] not in visited:
                visited.add(neighbor[0])
                queue.append(path + [neighbor[0]])

    return None  # No path found

def normalize(x): 
    if x != 0:
        return x / abs(x)
    else:
        return 0
    
def combo_distance(a, b):
    diff=0
    for ai, bi in zip(a, b):
        diff += abs(normalize(ai) - normalize(bi)) #Normalize builds on steps (-value, 0, value) and calculates the steps between them. 
    return diff

def find_closest_pair(cluster_1, cluster_2, combo_dict):
    """Find the two closest points in the clusters based on the combo to reach them"""
    #This might not be the best way of finding a path that looks good, but it does make the job easier for the pathfinder as it makes it certain that the path is quite short in number of nodes.
    best_diff = float('inf')
    best_pair = (None, None)

    for point1 in cluster_1:
        for combo1 in combo_dict[point1]:
            for point2 in cluster_2:
                for combo2 in combo_dict[point2]:
                    diff = combo_distance(combo1, combo2)
                    if diff < best_diff:
                        best_diff = diff
                        best_pair = (point1, point2)

    return best_pair