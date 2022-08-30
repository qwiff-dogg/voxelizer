#include "catch.hpp"
#include "mesh_util.h"
#include "../voxelizer.h"

TEST_CASE("Volume", "[volume]") {
    static constexpr float PRECISION_FACTOR = 0.f;

    SECTION("Cube") {
        static constexpr float VOXEL_SIZE = 0.5f;
        static constexpr float PRECISION = VOXEL_SIZE * PRECISION_FACTOR;
        static constexpr size_t SHELL_VOXELS = 98;
        static constexpr size_t INTERIOR_VOXELS = 27; // 3x3x3 voxels

        auto* mesh = load_obj_mesh("example/models/cube.obj");
        REQUIRE(mesh != nullptr);

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Bunny") {
        static constexpr float VOXEL_SIZE = 0.01f;
        static constexpr float PRECISION = VOXEL_SIZE * PRECISION_FACTOR;
        static constexpr size_t SHELL_VOXELS = 828;
        static constexpr size_t INTERIOR_VOXELS = 259;

        auto* mesh = load_obj_mesh("example/models/bunny.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        save_obj_mesh("test/tmp/bunny_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        save_obj_mesh("test/tmp/bunny_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Monkey") {
        static constexpr float VOXEL_SIZE = 0.05f;
        static constexpr float PRECISION = VOXEL_SIZE * PRECISION_FACTOR;
        static constexpr size_t SHELL_VOXELS = 7765;
        static constexpr size_t INTERIOR_VOXELS = 6556;

        auto* mesh = load_obj_mesh("example/models/suzanne.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        save_obj_mesh("test/tmp/monkey_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        save_obj_mesh("test/tmp/monkey_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Dragon") {
        static constexpr float VOXEL_SIZE = 0.05f;
        static constexpr float PRECISION = VOXEL_SIZE * PRECISION_FACTOR;
        static constexpr size_t SHELL_VOXELS = 4164;
        static constexpr size_t INTERIOR_VOXELS = 882;

        auto* mesh = load_obj_mesh("example/models/dragon.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        save_obj_mesh("test/tmp/dragon_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        save_obj_mesh("test/tmp/dragon_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, false);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, true);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }
}
