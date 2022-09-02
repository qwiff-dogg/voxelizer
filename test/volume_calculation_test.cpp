#include "catch.hpp"
#include "mesh_util.h"
#include "../voxelizer.h"

/**
 * Tests that demonstrate how voxel size affects the accuracy of volume calculation.
 *
 * The accuracy depends on the voxel coverage of the original mesh. A higher voxel resolution does not necessarily
 * result in better accuracy.
 *
 * A combination of a high voxel resolution for the shell and a lower resolution for the interior can be used to reduce
 * memory footprint. For multi-resolution a power-of-2 subdivision (0.5, 0.25, etc) prevents gaps in the voxel coverage.
 */
TEST_CASE("Volume calculation") {

    using VoxelSizeAccuracy = std::pair<float, float>;

    SECTION("Calculate Cube volume") {
        auto* mesh = load_obj_mesh("example/models/cube.obj");
        REQUIRE(mesh != nullptr);

        const float mesh_volume = calculate_mesh_volume(mesh);
        REQUIRE(mesh_volume == Approx(8.f));

        SECTION("using single voxel resolution") {
            const auto params = GENERATE(
                    VoxelSizeAccuracy{0.5, 0.96f},
                    VoxelSizeAccuracy{0.4, 2.52f},
                    VoxelSizeAccuracy{0.3, 0.16f}
            );
            const auto [size, accuracy] = params;

            auto* pc = vx_voxelize_pc(mesh, size, size, size, 0.f, true);

            auto voxel_volume = calculate_voxel_volume(pc, {size, size, size}, true);
            voxel_volume += calculate_voxel_volume(pc, {size, size, size}, false);
            REQUIRE(voxel_volume == Approx(mesh_volume).margin(mesh_volume * accuracy));

            vx_point_cloud_free(pc);
        }

        SECTION("using multiple voxel resolutions") {
            const auto params = GENERATE(
                    VoxelSizeAccuracy{0.5f, 0.18f},
                    VoxelSizeAccuracy{0.4f, 0.09f},
                    VoxelSizeAccuracy{0.3f, 0.03f}
            );
            const auto [interior_size, accuracy] = params;
            const auto shell_size = interior_size * 0.5f; // 2^-1

            auto* shell_pc = vx_voxelize_pc(mesh, shell_size, shell_size, shell_size, 0.f, false);
            auto* volume_pc = vx_voxelize_pc(mesh, interior_size, interior_size, interior_size, 0.f, true);
#ifndef NDEBUG
            char filename[128];
            sprintf(filename, "test/tmp/cube_multires_%.3f-%.3f.pcd", shell_size, interior_size);
            save_pcd_point_cloud(filename, shell_pc, volume_pc);
#endif

            auto voxel_volume = calculate_voxel_volume(shell_pc, {shell_size, shell_size, shell_size}, true);
            voxel_volume += calculate_voxel_volume(volume_pc, {interior_size, interior_size, interior_size}, false);
            REQUIRE(voxel_volume == Approx(mesh_volume).margin(mesh_volume * accuracy));

            vx_point_cloud_free(volume_pc);
            vx_point_cloud_free(shell_pc);
        }

        vx_mesh_free(mesh);
    }

    SECTION("Calculate Sphere volume") {
        auto* mesh = load_obj_mesh("example/models/sphere.obj");
        REQUIRE(mesh != nullptr);

        const float mesh_volume = calculate_mesh_volume(mesh);
        REQUIRE(mesh_volume == Approx(4.188f).margin(0.11f));

        SECTION("using single voxel resolution") {
            const auto params = GENERATE(
                    VoxelSizeAccuracy{0.5f, 1.34f},
                    VoxelSizeAccuracy{0.4f, 0.81f},
                    VoxelSizeAccuracy{0.3f, 0.73f}
            );
            const auto [size, accuracy] = params;

            auto* pc = vx_voxelize_pc(mesh, size, size, size, 0.f, true);

            auto voxel_volume = calculate_voxel_volume(pc, {size, size, size}, true);
            voxel_volume += calculate_voxel_volume(pc, {size, size, size}, false);
            REQUIRE(voxel_volume == Approx(mesh_volume).margin(mesh_volume * accuracy));

            vx_point_cloud_free(pc);
        }

        SECTION("using multiple voxel resolutions") {
            const auto params = GENERATE(
                    VoxelSizeAccuracy{0.3f, 0.26f},
                    VoxelSizeAccuracy{0.2f, 0.23f},
                    VoxelSizeAccuracy{0.1f, 0.42f}
            );
            const auto [interior_size, accuracy] = params;
            const auto shell_size = interior_size * 0.5f; // 2^-1

            auto* shell_pc = vx_voxelize_pc(mesh, shell_size, shell_size, shell_size, 0.f, false);
            auto* volume_pc = vx_voxelize_pc(mesh, interior_size, interior_size, interior_size, 0.f, true);
#ifndef NDEBUG
            char filename[128];
            sprintf(filename, "test/tmp/sphere_multires_%.3f-%.3f.pcd", shell_size, interior_size);
            save_pcd_point_cloud(filename, shell_pc, volume_pc);
#endif


            auto voxel_volume = calculate_voxel_volume(shell_pc, {shell_size, shell_size, shell_size}, true);
            voxel_volume += calculate_voxel_volume(volume_pc, {interior_size, interior_size, interior_size}, false);
            REQUIRE(voxel_volume == Approx(mesh_volume).margin(mesh_volume * accuracy));

            vx_point_cloud_free(volume_pc);
            vx_point_cloud_free(shell_pc);
        }

        vx_mesh_free(mesh);
    }

    SECTION("Calculate Cone volume") {
        auto* mesh = load_obj_mesh("example/models/cone.obj");
        REQUIRE(mesh != nullptr);

        const float mesh_volume = calculate_mesh_volume(mesh);
        REQUIRE(mesh_volume == Approx(2.09).margin(0.01f));

        SECTION("using single voxel resolution") {
            const auto params = GENERATE(
                    VoxelSizeAccuracy{0.3f, 0.93f},
                    VoxelSizeAccuracy{0.2f, 0.67f},
                    VoxelSizeAccuracy{0.1f, 0.09f}
            );
            const auto [size, accuracy] = params;

            auto* pc = vx_voxelize_pc(mesh, size, size, size, 0.f, true);

            auto voxel_volume = calculate_voxel_volume(pc, {size, size, size}, true);
            voxel_volume += calculate_voxel_volume(pc, {size, size, size}, false);
            REQUIRE(voxel_volume == Approx(mesh_volume).margin(mesh_volume * accuracy));

            vx_point_cloud_free(pc);
        }

        SECTION("using multiple voxel resolutions") {
            const auto params = GENERATE(
                    VoxelSizeAccuracy{0.3f, 0.47f},
                    VoxelSizeAccuracy{0.2f, 0.08f},
                    VoxelSizeAccuracy{0.1f, 0.27f}
            );
            const auto [interior_size, accuracy] = params;
            const auto shell_size = interior_size * 0.5f; // 2^-1

            auto* shell_pc = vx_voxelize_pc(mesh, shell_size, shell_size, shell_size, 0.f, false);
            auto* volume_pc = vx_voxelize_pc(mesh, interior_size, interior_size, interior_size, 0.f, true);
#ifndef NDEBUG
            char filename[128];
            sprintf(filename, "test/tmp/cone_multires_%.3f-%.3f.pcd", shell_size, interior_size);
            save_pcd_point_cloud(filename, shell_pc, volume_pc);
#endif

            auto voxel_volume = calculate_voxel_volume(shell_pc, {shell_size, shell_size, shell_size}, true);
            voxel_volume += calculate_voxel_volume(volume_pc, {interior_size, interior_size, interior_size}, false);
            REQUIRE(voxel_volume == Approx(mesh_volume).margin(mesh_volume * accuracy));

            vx_point_cloud_free(volume_pc);
            vx_point_cloud_free(shell_pc);
        }

        vx_mesh_free(mesh);
    }

}
