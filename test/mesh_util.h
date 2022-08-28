#pragma once

#include "../voxelizer.h"

/**
 * Load a mesh from a Wavefront OBJ file.
 * @param filename The filename
 * @param shape_index The shape index
 * @return The mesh or <code>null</code> if it could not be loaded.
 *
 */
vx_mesh_t* load_obj_mesh(const char* filename, size_t shape_index = 0);

/**
 * Saves a mesh to a Wavefront OBJ file.
 * @param filename
 * @param mesh
 * @return <code>true</code> if the file was written.
 */
bool save_obj_mesh(const char* filename, const vx_mesh_t* mesh);
