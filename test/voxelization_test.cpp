#include "catch.hpp"
#include "mesh_util.h"
#include "../voxelizer.h"

TEST_CASE("Voxelization") {

    SECTION("Cube") {
        static constexpr float VOXEL_SIZE = 0.5f;
        static constexpr size_t SHELL_VOXELS = 98;
        static constexpr size_t INTERIOR_VOXELS = 27; // 3x3x3 voxels

        auto* mesh = load_obj_mesh("example/models/cube.obj");
        REQUIRE(mesh != nullptr);

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, false);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/cube_shell.pcd", pc);
#endif
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_fill = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, true);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/cube_fill.pcd", pc_fill);
#endif
        REQUIRE(pc_fill->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_fill);

        vx_mesh_free(mesh);
    }

    SECTION("Sphere") {
        static constexpr float VOXEL_SIZE = 0.1f;
        static constexpr size_t SHELL_VOXELS = 2022;
        static constexpr size_t INTERIOR_VOXELS = 1351;

        auto* mesh = load_obj_mesh("example/models/sphere.obj");
        REQUIRE(mesh != nullptr);

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, false);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/sphere_shell.pcd", pc);
#endif
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_fill = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, true);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/sphere_fill.pcd", pc_fill);
#endif
        REQUIRE(pc_fill->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_fill);

        vx_mesh_free(mesh);
    }

    SECTION("Bunny") {
        static constexpr float VOXEL_SIZE = 0.01f;
        static constexpr size_t SHELL_VOXELS = 828;
        static constexpr size_t INTERIOR_VOXELS = 259;

        auto* mesh = load_obj_mesh("example/models/bunny.obj");
        REQUIRE(mesh != nullptr);

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, false);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/bunny_shell.pcd", pc);
#endif
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_fill = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, true);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/bunny_fill.pcd", pc_fill);
#endif
        REQUIRE(pc_fill->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_fill);

        vx_mesh_free(mesh);
    }

    SECTION("Monkey") {
        static constexpr float VOXEL_SIZE = 0.05f;
        static constexpr size_t SHELL_VOXELS = 7765;
        static constexpr size_t INTERIOR_VOXELS = 6556;

        auto* mesh = load_obj_mesh("example/models/suzanne.obj");
        REQUIRE(mesh != nullptr);

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, false);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/monkey_shell.pcd", pc);
#endif
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_fill = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, true);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/monkey_fill.pcd", pc_fill);
#endif
        REQUIRE(pc_fill->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_fill);

        vx_mesh_free(mesh);
    }

    SECTION("Dragon") {
        static constexpr float VOXEL_SIZE = 0.05f;
        static constexpr size_t SHELL_VOXELS = 4164;
        static constexpr size_t INTERIOR_VOXELS = 882;

        auto* mesh = load_obj_mesh("example/models/dragon.obj");
        REQUIRE(mesh != nullptr);

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, false);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/dragon_shell.pcd", pc);
#endif
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        auto* pc_fill = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, 0.f, true);
#ifndef NDEBUG
        save_pcd_point_cloud("test/tmp/dragon_fill.pcd", pc_fill);
#endif
        REQUIRE(pc_fill->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_fill);

        vx_mesh_free(mesh);
    }
}
