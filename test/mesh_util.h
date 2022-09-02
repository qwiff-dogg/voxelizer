#pragma once

#include "../voxelizer.h"

#include <vector>

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
 * @param pc The point cloud
 * @param overlay Optional overlay point cloud
 * @return <code>true</code> if the file was written.
 */
bool save_pcd_point_cloud(const char* filename, const vx_point_cloud_t* pc, const vx_point_cloud_t* overlay = nullptr);

/**
 * Calculates the volume of the given mesh.
 *
 * @param mesh
 * @return The mesh volume.
 */
float calculate_mesh_volume(const vx_mesh_t* mesh);

/**
 * Calculates the volume of the voxels of the given type.
 *
 * To calculate the total volume of a filled point cloud, invoke this function once for the shell and once for the interior.
 *
 * @param pc The point cloud
 * @param size The voxel size
 * @param count_shell Whether to count the shell or the interior voxels
 * @return
 */
float calculate_voxel_volume(vx_point_cloud_t* pc, vx_vec3_t size, bool count_shell);

