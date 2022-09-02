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

struct Point {
    vx_vec3_t position;
    bool is_shell;
};

static std::vector<Point> to_points(const vx_point_cloud_t* pc) {
    auto points = std::vector<Point>(pc->nvertices);

    for (size_t vertex_idx = 0; vertex_idx < pc->nvertices; vertex_idx++) {
        points[vertex_idx] = Point{
                .position = pc->vertices[vertex_idx],
                .is_shell = pc->is_shell[vertex_idx],
        };
    }

    return points;
}

static bool save_pcd_point_cloud(const char* filename, const std::vector<Point>& points) {
    auto* file = fopen(filename, "wb");
    if (!file) {
        return false;
    }

    fprintf(file, "VERSION .5\n");
    fprintf(file, "FIELDS x y z\n");
    fprintf(file, "SIZE %lu %lu %lu\n", sizeof(float), sizeof(float), sizeof(float));
    fprintf(file, "TYPE F F F\n");
    fprintf(file, "COUNT 1 1 1\n");
    fprintf(file, "WIDTH %lu\n", points.size());
    fprintf(file, "HEIGHT 1\n");
    fprintf(file, "DATA ascii\n");

    for (const auto& point: points) {
        const auto& position = point.position;
        fprintf(file, "%0.5f %0.5f %0.5f\n", position.x, position.y, position.z);
    }

    fclose(file);
    return true;
}

bool save_pcd_point_cloud(const char* filename, const vx_point_cloud_t* pc, const vx_point_cloud_t* overlay) {
    auto points = to_points(pc);

    if (overlay) {
        auto overlay_points = to_points(overlay);
        const auto begin = std::remove_if(overlay_points.begin(), overlay_points.end(), [](const auto& point) {
            return point.is_shell;
        });
        overlay_points.erase(begin, overlay_points.end());
        for (const auto& point: overlay_points) {
            points.emplace_back(point);
        }
    }

    return save_pcd_point_cloud(filename, points);
}

// https://stackoverflow.com/questions/1406029/how-to-calculate-the-volume-of-a-3d-mesh-object-the-surface-of-which-is-made-up
static float signed_volume(vx_vec3_t p1, vx_vec3_t p2, vx_vec3_t p3) {
    float v321 = p3.x * p2.y * p1.z;
    float v231 = p2.x * p3.y * p1.z;
    float v312 = p3.x * p1.y * p2.z;
    float v132 = p1.x * p3.y * p2.z;
    float v213 = p2.x * p1.y * p3.z;
    float v123 = p1.x * p2.y * p3.z;
    return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
}

float calculate_mesh_volume(const vx_mesh_t* mesh) {
    float volume = 0.f;

    for (size_t i = 0; i < mesh->nindices; i += 3) {
        auto i1 = mesh->indices[i + 0];
        auto i2 = mesh->indices[i + 1];
        auto i3 = mesh->indices[i + 2];
        auto p1 = mesh->vertices[i1];
        auto p2 = mesh->vertices[i2];
        auto p3 = mesh->vertices[i3];
        volume += signed_volume(p1, p2, p3);
    }

    return volume;
}

float calculate_voxel_volume(vx_point_cloud_t* pc, vx_vec3_t size, bool count_shell) {
    float volume = 0.f;

    for (size_t i = 0; i < pc->nvertices; i++) {
        if (count_shell && pc->is_shell[i] || !count_shell && !pc->is_shell[i]) {
            volume += size.x * size.y * size.z;
        }
    }

    return volume;
}
