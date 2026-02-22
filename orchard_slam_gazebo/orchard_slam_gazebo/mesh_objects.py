#!/usr/bin/env python3
import trimesh
import numpy as np
import os

class MeshObjects:
    @staticmethod
    def load_mesh(mesh_path: str, mesh_type: str = "ply") -> trimesh.Trimesh:
        """Load a mesh from a path

        :param mesh_path: Mesh path
        :type mesh_path: str
        :return: Trimesh object of the mesh
        :rtype: trimesh.Trimesh
        """
        if not os.path.exists(mesh_path):
            raise FileNotFoundError(f"Could not find file '{mesh_path}'.")
        mesh = trimesh.load_mesh(file_obj=mesh_path, file_type=mesh_type)
        return mesh

    @staticmethod
    def export_obj(
        mesh: trimesh.Trimesh,
        filename: str,
        dir_path: str,
    ) -> None:
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        
        # Ensure vertex normals are computed (required by Gazebo/DART)
        if not hasattr(mesh, 'vertex_normals') or len(mesh.vertex_normals) != len(mesh.vertices):
            mesh.fix_normals()
            _ = mesh.vertex_normals  # Force computation
        
        export_path = os.path.join(dir_path, filename)
        mesh.export(f"{export_path}", file_type="obj", include_normals=True)
        return
    

if __name__ == "__main__":
    import sys
    print("Converting PLY meshes to OBJ format...\n Are you sure you want to do this? This will overwrite any existing OBJ files in the target directory.")
    resp = input("(y/n): ")
    if resp.lower() != "y":
        print("Aborting...")
        sys.exit(0)

    __here__ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    meshes_dir = os.path.join(__here__, "meshes")
    assert os.path.exists(meshes_dir), f"Meshes directory does not exist at path '{meshes_dir}'"

    ply_dir = os.path.join(meshes_dir, "ply")
    obj_dir = os.path.join(meshes_dir, "obj")
    assert os.path.exists(ply_dir), f"PLY directory does not exist at path '{ply_dir}'"
    if not os.path.exists(obj_dir):
        os.makedirs(obj_dir)

    for filename in os.listdir(ply_dir):
        if filename.endswith(".ply"):
            mesh_path = os.path.join(ply_dir, filename)
            mesh = MeshObjects.load_mesh(mesh_path)
            MeshObjects.export_obj(mesh, filename.replace(".ply", ".obj"), obj_dir)