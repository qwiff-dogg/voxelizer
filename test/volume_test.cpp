#include "catch.hpp"
#include "mesh_util.h"
#include "../voxelizer.h"

TEST_CASE("Volume", "[volume]") {
    static constexpr float PRECISION = 0.f;

    SECTION("Cube") {
        static constexpr float VOXEL_SIZE = 0.5f;
        static constexpr size_t SHELL_VOXELS = 98;
        static constexpr size_t INTERIOR_VOXELS = 27; // 3x3x3 voxels

        auto* mesh = load_obj_mesh("example/models/cube.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        save_obj_mesh("test/tmp/cube_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        float debug_volume = 0.f;
        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &debug_volume);
        save_obj_mesh("test/tmp/cube_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        float volume = 0.f;
        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &volume);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        REQUIRE(volume == Approx(8.f));
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Sphere") {
        static constexpr float VOXEL_SIZE = 0.1f;
        static constexpr size_t SHELL_VOXELS = 2022;
        static constexpr size_t INTERIOR_VOXELS = 1351;

        auto* mesh = load_obj_mesh("example/models/sphere.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        save_obj_mesh("test/tmp/sphere_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        float debug_volume = 0.f;
        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &debug_volume);
        save_obj_mesh("test/tmp/sphere_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        float volume = 0.f;
        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &volume);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        REQUIRE(volume == Approx(4.188).margin(0.1));
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Bunny") {
        static constexpr float VOXEL_SIZE = 0.01f;
        static constexpr size_t SHELL_VOXELS = 828;
        static constexpr size_t INTERIOR_VOXELS = 259;

        auto* mesh = load_obj_mesh("example/models/bunny.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        save_obj_mesh("test/tmp/bunny_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        float debug_volume = 0.f;
        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &debug_volume);
        save_obj_mesh("test/tmp/bunny_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        float volume = 0.f;
        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &volume);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Monkey") {
        static constexpr float VOXEL_SIZE = 0.05f;
        static constexpr size_t SHELL_VOXELS = 7765;
        static constexpr size_t INTERIOR_VOXELS = 6556;

        auto* mesh = load_obj_mesh("example/models/suzanne.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        save_obj_mesh("test/tmp/monkey_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        float debug_volume = 0.f;
        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &debug_volume);
        save_obj_mesh("test/tmp/monkey_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        float volume = 0.f;
        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &volume);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }

    SECTION("Dragon") {
        static constexpr float VOXEL_SIZE = 0.05f;
        static constexpr size_t SHELL_VOXELS = 4164;
        static constexpr size_t INTERIOR_VOXELS = 882;

        auto* mesh = load_obj_mesh("example/models/dragon.obj");
        REQUIRE(mesh != nullptr);

#ifndef NDEBUG
        auto* shell_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        save_obj_mesh("test/tmp/dragon_shell.obj", shell_mesh);
        vx_mesh_free(shell_mesh);

        float debug_volume = 0.f;
        auto* volume_mesh = vx_voxelize(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &debug_volume);
        save_obj_mesh("test/tmp/dragon_volume.obj", volume_mesh);
        vx_mesh_free(volume_mesh);
#endif

        auto* pc = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, nullptr);
        REQUIRE(pc->nvertices == SHELL_VOXELS);
        vx_point_cloud_free(pc);

        float volume = 0.f;
        auto* pc_volume = vx_voxelize_pc(mesh, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, PRECISION, &volume);
        REQUIRE(pc_volume->nvertices == SHELL_VOXELS + INTERIOR_VOXELS);
        vx_point_cloud_free(pc_volume);

        vx_mesh_free(mesh);
    }
}
