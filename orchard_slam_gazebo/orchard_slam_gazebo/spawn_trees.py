#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import datetime as dt
import glob
import hashlib
import json
import numpy as np
import os


TREE_ID_MAX = 99


def unique_seed(*args):
    """Generate a unique, reproducible seed from any set of arguments"""
    data = str(args).encode("utf-8")
    return int.from_bytes(hashlib.sha256(data).digest()[:8], "big")


def generate_orchard_config(
    orchard_name: str = None,
    tree_namespace: str = "lpy",
    tree_type: str = "envy",
    n_rows: int = 5,
    orchard_seed: int = None, # needed for generating unique orchards given the same parameters
    avg_trees_per_row: int = 10,
    trees_per_row_std: float = 1.0,
    avg_tree_spacing: float = 2.0,  # average spacing between trees in a row, in meters
    tree_spacing_std: float = 0.25,
    avg_row_deviation: float = 0.0,  # how much the trees in a row deviate from a straight line, in meters
    std_row_deviation: float = 0.05,
    avg_row_spacing: float = 3.0,  # average spacing between rows, in meters
    row_spacing_std: float = 0.15,
    initial_offset: tuple = (0.0, 0.0),  # initial offset for the entire orchard, in meters
    
) -> dict:
    """Generates a set of trees based on number of rows with Gaussian distribution.

    :param n_rows: number of rows in the orchard
    :type n_rows: int
    :param avg_trees_per_row: average number of trees per row
    :type avg_trees_per_row: int
    :param trees_per_row_std: standard deviation of the number of trees per row
    :type trees_per_row_std: float
    :param avg_tree_spacing: average spacing between trees in a row, in meters
    :type avg_tree_spacing: float
    :param tree_spacing_std: standard deviation of the spacing between trees in a row, in meters
    :type tree_spacing_std: float
    :param avg_row_deviation: how much the trees in a row deviate from a straight line, in meters
    :type avg_row_deviation: float
    :param std_row_deviation: standard deviation of how much the trees in a row deviate from a straight line, in meters
    :type std_row_deviation: float
    :param avg_row_spacing: average spacing between rows, in meters
    :type avg_row_spacing: float
    :param row_spacing_std: standard deviation of the spacing between rows, in meters
    :type row_spacing_std: float
    :return: dict containing the orchard configuration
    :rtype: dict
    """
    # get a seed for reproducibility using function of the input parameters
    unique_layout_seed = unique_seed(
        n_rows,
        avg_trees_per_row,
        trees_per_row_std,
        avg_tree_spacing,
        tree_spacing_std,
        avg_row_deviation,
        std_row_deviation,
        avg_row_spacing,
        row_spacing_std,
        orchard_seed,
    )
    rgen = np.random.default_rng(unique_layout_seed)

    # generate the number of trees per row with Gaussian distribution
    trees_per_row = rgen.normal(avg_trees_per_row, trees_per_row_std, size=n_rows)
    trees_per_row = np.clip(trees_per_row, 0, None).astype(int)

    total_num_trees = np.sum(trees_per_row)
    if total_num_trees > TREE_ID_MAX:
        raise ValueError(f"Total number of trees ({total_num_trees}) exceeds maximum allowed ({TREE_ID_MAX}). Please adjust parameters to reduce the total number of trees.")

    tree_ids = rgen.choice(range(TREE_ID_MAX), size=np.sum(trees_per_row), replace=False)
    # self.id_str = f"{self.tree_namespace}_{self.tree_type}_{self.tree_id}"
    tree_id_strings = [f"{tree_namespace}_{tree_type}_{str(tree_id).zfill(5)}" for tree_id in tree_ids]    

    rows = []
    row_spacing = rgen.normal(avg_row_spacing, row_spacing_std, size=n_rows)  # spacing between rows
    for i in range(n_rows):
        n_trees = trees_per_row[i]
        tree_spacing = rgen.normal(avg_tree_spacing, tree_spacing_std, size=n_trees)
        tree_spacing = np.clip(tree_spacing, 0.1, None)  # minimum spacing of 0.1m to avoid overlap
        tree_x_positions = np.cumsum(tree_spacing)  # x positions of the trees in the row
        in_row_deviation = rgen.normal(
            avg_row_deviation, std_row_deviation, size=n_trees
        )  # how much each tree in the row deviates from a straight line
        row = np.zeros((n_trees, 2))  # initialize row with x and y positions

        row[:, 0] = tree_x_positions
        row[:, 1] = (
            in_row_deviation + i * row_spacing[i]
        )  # y positions of the trees in the row, with spacing between rows
        row += np.array(initial_offset)  # apply initial offset to the entire row
        rows.append(row)



    orchard_config = {
        'orchard_name': orchard_name,
        "orchard_seed": orchard_seed,
        "seed": unique_layout_seed,
        "n_rows": n_rows,
        "total_num_trees": total_num_trees,
        "avg_trees_per_row": avg_trees_per_row,
        "trees_per_row_std": trees_per_row_std,
        "avg_tree_spacing": avg_tree_spacing,
        "tree_spacing_std": tree_spacing_std,
        "avg_row_deviation": avg_row_deviation,
        "std_row_deviation": std_row_deviation,
        "avg_row_spacing": avg_row_spacing,
        "row_spacing_std": row_spacing_std,
        "rows": {},
    }
    
    # Use cumulative sum to get correct starting indices for each row
    cumsum_trees = np.concatenate([[0], np.cumsum(trees_per_row)])
    for i, row in enumerate(rows):
        orchard_config["rows"][f"row_{i}"] = {
            "n_trees": trees_per_row[i],
            "tree_positions": row.tolist(),
            "tree_ids": tree_id_strings[cumsum_trees[i]:cumsum_trees[i+1]],
        }
    
    # select tree models from the models directory
    return orchard_config


def get_trees_from_file(file_path: str):
    """Opens a json file containing the orchard configuration and returns a dict"""
    return


def save_orchard_to_file(orchard_config: dict, output_dir: str) -> str:
    """Saves the orchard configuration to a file.
    TODO: json format
    :param orchard_config: dict containing the orchard configuration
    :type orchard_config: dict
    :param file_name: name of the file to save the orchard configuration
    :type file_name: str
    :param dir: directory to save the orchard configuration file
    :type dir: str
    :type file_path: str
    :return: path to the saved orchard configuration file
    :rtype: str
    """
    if orchard_config.get("orchard_name") is not None:
        file_path = f"orchard_{orchard_config['orchard_name']}_config.json"
    else:
        file_path = f"orchard_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}_config.json"
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    output_filepath =  os.path.join(output_dir, file_path)
    if os.path.exists(output_filepath):
        raise FileExistsError(f"File {file_path} already exists in {output_dir}. Please choose a different name or directory.")
    
    # stringify everything in the orchard_config dict
    orchard_config = json.loads(json.dumps(orchard_config, default=str))
    
    with open(output_filepath, "w") as f:
        json.dump(orchard_config, f, indent=4)
    return output_filepath


def generate_tree_sdf(tree_id_str: str, tree_pose: tuple) -> str:
    """Generates an SDF string for a tree model with the given tree ID."""
    file_uri = get_package_share_directory("orchard_slam_gazebo") + f"/meshes/obj/{tree_id_str}.obj"
    sdf_str =  f"""<?xml version="1.0"?>
                    <sdf version="1.8">
                        <model name="{tree_id_str}">
                            <static>true</static>
                            <pose>{tree_pose[0]} {tree_pose[1]} 0 0 0 0</pose>
                            <link name="link">
                            <visual name="{tree_id_str}_visual">
                                <geometry>
                                    <mesh>
                                        <uri>file://{file_uri}</uri>
                                    </mesh>
                                </geometry>
                                <material>
                                    <ambient>0.43137254902 0.407843137255 0.356862745098 1</ambient>
                                    <diffuse>0.43137254902 0.407843137255 0.356862745098 1</diffuse>
                                    <specular>0.43137254902 0.407843137255 0.356862745098 1</specular>
                                </material>
                            </visual>
                            <collision name="{tree_id_str}_collision"><geometry><mesh><uri>file://{file_uri}</uri></mesh></geometry></collision>
                            </link>
                        </model>
                    </sdf>"""
    return sdf_str

def generate_all_tree_sdfs(orchard_config: dict) -> dict:
    """Generates SDF strings for all trees in the orchard configuration.
    
    Returns a dict with tree_id as key and dict containing 'sdf' and 'pose' as value.
    """
    tree_sdfs = {}
    for row_name, row in orchard_config["rows"].items():
        for i, tree_id in enumerate(row["tree_ids"]):
            tree_pose = row["tree_positions"][i]
            tree_sdfs[tree_id] = {
                'sdf': generate_tree_sdf(tree_id, tree_pose),
                'pose': tree_pose  # [x, y]
            }
    return tree_sdfs


if __name__ == "__main__":
    import plotly.graph_objects as go
    import pprint

    orchard_config = generate_orchard_config(
        n_rows=5,
        orchard_seed=42,
        avg_trees_per_row=10,
        trees_per_row_std=0.1,
        avg_tree_spacing=1.0,
        tree_spacing_std=0.10,
        avg_row_deviation=0.0,
        std_row_deviation=0.05,
        avg_row_spacing=3.0,
        row_spacing_std=0.10,
        initial_offset=(2.0, 2.0),
    )
    tree_sdfs = generate_all_tree_sdfs(orchard_config)
    test_tree = tree_sdfs['lpy_envy_00091']
    print(f"Tree SDF:\n{test_tree['sdf']}")
    print(f"Tree pose: x={test_tree['pose'][0]:.2f}, y={test_tree['pose'][1]:.2f}")
    # pprint.pprint(orchard_config)
    # get src/orchard-slam/data/orchards directory
    orchards_dir = os.path.join(os.path.dirname(__file__), "..", "..", "data", "orchards")
    # save_orchard_to_file(orchard_config, output_dir=orchards_dir)

    fig = go.Figure()
    for row_name, row in orchard_config["rows"].items():
        tree_positions = np.array(row["tree_positions"])
        fig.add_trace(
            go.Scatter(
                x=tree_positions[:, 0], 
                y=tree_positions[:, 1], 
                mode="markers", 
                marker=dict(size=10),
                customdata=np.column_stack((row["tree_ids"], [row_name]*len(row["tree_ids"]))),  # Add tree IDs and index as custom data
                hovertemplate=(
                    "%{customdata[1]}<br>"
                    "x: %{x:.2f} m<br>"
                    "y: %{y:.2f} m<br>"
                    "<extra>%{customdata[0]}<br></extra>"  # Hide the trace name in the hover tooltip
                )
            )
        )
    fig.update_layout(title="Orchard Configuration", xaxis_title="X Position (m)", yaxis_title="Y Position (m)")
    fig.show()
