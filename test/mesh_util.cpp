#include "mesh_util.h"

#define TINYOBJLOADER_IMPLEMENTATION

#include "../example/tiny_obj_loader.h"

vx_mesh_t* load_obj_mesh(const char* filename, size_t shape_index) {
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool loaded = tinyobj::LoadObj(shapes, materials, err, filename, nullptr);
    if (!loaded) {
        return nullptr;
    }

    const auto& shape = shapes[shape_index];
    vx_mesh_t* mesh = vx_mesh_alloc(shape.mesh.positions.size(), shape.mesh.indices.size());

    for (size_t f = 0; f < shape.mesh.indices.size(); f++) {
        mesh->indices[f] = shape.mesh.indices[f];
    }

    for (size_t v = 0; v < shape.mesh.positions.size() / 3; v++) {
        mesh->vertices[v].x = shape.mesh.positions[3 * v + 0];
        mesh->vertices[v].y = shape.mesh.positions[3 * v + 1];
        mesh->vertices[v].z = shape.mesh.positions[3 * v + 2];
    }
    return mesh;
}

bool save_obj_mesh(const char* filename, const vx_mesh_t* mesh) {
    auto* file = fopen(filename, "wb");
    if (!file) {
        return false;
    }

    fprintf(file, "o mesh\n");

    for (size_t vertex_idx = 0; vertex_idx < mesh->nvertices; vertex_idx++) {
        const auto& vertex = mesh->vertices[vertex_idx];
        fprintf(file, "v %0.5f %0.5f %0.5f\n", vertex.x, vertex.y, vertex.z);
    }

    for (size_t normal_idx = 0; normal_idx < mesh->nnormals; normal_idx++) {
        const auto& normal = mesh->normals[normal_idx];
        fprintf(file, "vn %0.5f %0.5f %0.5f\n", normal.x, normal.y, normal.z);
    }

    for (size_t index_idx = 0; index_idx < mesh->nindices; index_idx += 3) {
        const auto i1 = mesh->indices[index_idx + 0] + 1;
        const auto i2 = mesh->indices[index_idx + 1] + 1;
        const auto i3 = mesh->indices[index_idx + 2] + 1;
        const auto n1 = mesh->normalindices[index_idx + 0] + 1;
        const auto n2 = mesh->normalindices[index_idx + 1] + 1;
        const auto n3 = mesh->normalindices[index_idx + 2] + 1;
        fprintf(file, "f %u//%u %u//%u %u//%u\n", i1, n1, i2, n2, i3, n3);
    }

    fclose(file);
    return true;
}
