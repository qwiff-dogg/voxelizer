#pragma once

#include "../voxelizer.h"

/**
 * Load a mesh from a Wavefront OBJ file.
 *
 * @param filename The filename
 * @param shape_index The shape index
 * @return The mesh or <code>null</code> if it could not be loaded.
 *
 */
vx_mesh_t* load_obj_mesh(const char* filename, size_t shape_index = 0);

/**
 * Saves a mesh to a Wavefront OBJ file.
 *
 * @param filename
 * @param mesh
 * @return <code>true</code> if the file was written.
 */
bool save_obj_mesh(const char* filename, const vx_mesh_t* mesh);

/**
 * Saves a point cloud to a PCD file.
 *
 * @param filename
 * @param pc
 * @return <code>true</code> if the file was written.
 */
bool save_pcd_point_cloud(const char* filename, const vx_point_cloud_t* pc);

/**
 * Calculates the volume of the given mesh.
 *
 * @param mesh
 * @return The mesh volume.
 */
float calculate_volume(const vx_mesh_t* mesh);

/**
 * Calculates the sum volume of the voxels in the point cloud.
 *
 * @param pc The point cloud
 * @param voxel_size The voxel size
 * @return The voxel size
 */
float calculate_voxel_volume(const vx_point_cloud_t* pc, vx_vec3_t voxel_size);
