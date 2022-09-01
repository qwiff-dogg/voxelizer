#include "catch.hpp"
#include "mesh_util.h"
#include "../voxelizer.h"

TEST_CASE("Volume", "[voxel]") {
    static constexpr float PRECISION = 0.f;

    SECTION("Cube volume") {
        static constexpr float VOXEL_SIZE = 0.5f;
        static constexpr float OCCUPANCY_SAMPLES = 2;

        auto* mesh = load_obj_mesh("example/models/cube.obj");
        REQUIRE(mesh != nullptr);

        const float volume = calculate_volume(mesh);
        REQUIRE(volume == Approx(8.f));

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true, OCCUPANCY_SAMPLES);
        const float voxel_volume = calculate_voxel_volume(pc, {VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE});
        REQUIRE(voxel_volume == Approx(volume).margin(0.1f));
        vx_point_cloud_free(pc);

        vx_mesh_free(mesh);
    }

    SECTION("Sphere volume") {
        static constexpr float VOXEL_SIZE = 0.1f;
        static constexpr float OCCUPANCY_SAMPLES = 5;

        auto* mesh = load_obj_mesh("example/models/sphere.obj");
        REQUIRE(mesh != nullptr);

        const float volume = calculate_volume(mesh);
        REQUIRE(volume == Approx(4.188f).margin(0.1f));

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true, OCCUPANCY_SAMPLES);
        const float voxel_volume = calculate_voxel_volume(pc, {VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE});
        REQUIRE(voxel_volume == Approx(volume).margin(.5f));
        vx_point_cloud_free(pc);

        vx_mesh_free(mesh);
    }

}
