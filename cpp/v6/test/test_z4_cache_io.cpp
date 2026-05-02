#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/lect/z4_ep_cache.h>
#include <sbf/lect/z4_grid_cache.h>
#include <sbf/lect/lect_cache_manager.h>
#include <sbf/voxel/voxel_grid.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
#include <unistd.h>

namespace {

std::filesystem::path make_temp_dir(const char* name) {
    auto dir = std::filesystem::temp_directory_path() /
        (std::string(name) + "_" + std::to_string(static_cast<long>(::getpid())));
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

sbf::voxel::SparseVoxelGrid make_grid(float lo, float hi, double delta = 0.05) {
    sbf::voxel::SparseVoxelGrid grid(delta, 0.0, 0.0, 0.0, 0.0);
    float aabb[6] = {lo, lo, lo, hi, hi, hi};
    grid.fill_aabb(aabb);
    return grid;
}

void check_same_grid(const sbf::voxel::SparseVoxelGrid& a,
                     const sbf::voxel::SparseVoxelGrid& b) {
    CHECK(a.num_bricks() == b.num_bricks());
    CHECK(a.count_occupied() == b.count_occupied());
    CHECK(a.bricks().size() == b.bricks().size());
    for (auto entry : a.bricks()) {
        auto it = b.bricks().find(entry.key);
        CHECK(it != b.bricks().end());
        if (it != b.bricks().end()) {
            for (int z = 0; z < 8; ++z) {
                CHECK(entry.value.words[z] == it.second().words[z]);
            }
        }
    }
}

}  // namespace

TEST_SUITE("Z4 cache IO") {

TEST_CASE("Z4EpCache round-trips endpoint and link-iAABB payloads") {
    auto dir = make_temp_dir("sbf_z4_ep_cache_io");
    const auto path = (dir / "ep.cache").string();

    sbf::Z4EpCache cache;
    REQUIRE(cache.open(path, 12, 6, 64));

    std::vector<float> ep(12);
    std::vector<float> liaabb(6);
    for (int i = 0; i < 12; ++i) ep[i] = 0.25f * static_cast<float>(i + 1);
    for (int i = 0; i < 6; ++i) liaabb[i] = -0.5f * static_cast<float>(i + 1);

    cache.insert(0x1234ULL, sbf::EndpointSource::IFK, ep.data(), liaabb.data());

    std::vector<float> ep_out(12, 0.0f);
    std::vector<float> liaabb_out(6, 0.0f);
    CHECK(cache.lookup_copy(0x1234ULL, sbf::EndpointSource::IFK,
                            ep_out.data(), liaabb_out.data()));
    CHECK(ep_out == ep);
    CHECK(liaabb_out == liaabb);
    CHECK(cache.probe_calls() > 0);
    CHECK(cache.lookup_copy_calls() == 1);
    CHECK(cache.lookup_copy_bytes() == static_cast<int64_t>((12 + 6) * sizeof(float)));
    CHECK(cache.insert_calls() == 1);
    CHECK(cache.insert_bytes() == static_cast<int64_t>((12 + 6) * sizeof(float)));

    cache.close();
    sbf::Z4EpCache reloaded;
    REQUIRE(reloaded.open(path, 12, 6, 64));
    std::fill(ep_out.begin(), ep_out.end(), 0.0f);
    std::fill(liaabb_out.begin(), liaabb_out.end(), 0.0f);
    CHECK(reloaded.lookup_copy(0x1234ULL, sbf::EndpointSource::IFK,
                               ep_out.data(), liaabb_out.data()));
    CHECK(ep_out == ep);
    CHECK(liaabb_out == liaabb);
    reloaded.close();

    REQUIRE(cache.open(path, 12, 6, 64));
    for (uint64_t i = 1; i <= 70; ++i) {
        cache.insert(0x4000ULL + i, sbf::EndpointSource::IFK,
                     ep.data(), liaabb.data());
    }
    cache.close();
    REQUIRE(reloaded.open(path, 12, 6, 64));
    CHECK(reloaded.capacity() >= 128);

    std::filesystem::remove_all(dir);
}

TEST_CASE("Z4GridCache round-trips grids and records IO counters") {
    auto dir = make_temp_dir("sbf_z4_grid_cache_io");
    const auto path = (dir / "grid.cache").string();

    sbf::GridQuality quality;
    quality.type = sbf::EnvelopeType::LinkIAABB_Grid;
    quality.resolution = 0.05f;
    quality.n_sub = 2;

    auto grid = make_grid(-0.10f, 0.18f, quality.resolution);

    sbf::Z4GridCache cache;
    REQUIRE(cache.open(path, 64, 0, 0));
    cache.insert(0x223344ULL, grid, quality);
    CHECK(cache.pwrite_calls() == 1);
    CHECK(cache.pwrite_bytes() > 0);
    CHECK(cache.dead_bytes() == 0);

    sbf::GridQuality coarse_req = quality;
    coarse_req.resolution = 0.10f;
    coarse_req.n_sub = 1;
    auto hit = cache.lookup(0x223344ULL, coarse_req);
    REQUIRE(hit != nullptr);
    check_same_grid(grid, *hit);

    sbf::GridQuality too_fine_req = quality;
    too_fine_req.resolution = 0.01f;
    CHECK(cache.lookup(0x223344ULL, too_fine_req) == nullptr);

    cache.close();
    sbf::Z4GridCache reloaded;
    REQUIRE(reloaded.open(path, 64, 0, 0));
    auto disk_hit = reloaded.lookup(0x223344ULL, coarse_req);
    REQUIRE(disk_hit != nullptr);
    check_same_grid(grid, *disk_hit);
    CHECK(reloaded.pread_calls() >= 1);
    CHECK(reloaded.pread_bytes() > 0);
    CHECK(reloaded.disk_hits() == 1);

    auto upgraded = make_grid(-0.20f, 0.20f, quality.resolution);
    sbf::GridQuality upgraded_quality = quality;
    upgraded_quality.n_sub = 3;
    reloaded.insert(0x223344ULL, upgraded, upgraded_quality);
    CHECK(reloaded.dead_bytes() > 0);
    auto upgraded_hit = reloaded.lookup(0x223344ULL, quality);
    REQUIRE(upgraded_hit != nullptr);
    check_same_grid(upgraded, *upgraded_hit);

    for (uint64_t i = 1; i <= 70; ++i) {
        reloaded.insert(0x990000ULL + i, grid, quality);
    }
    CHECK(reloaded.capacity() >= 128);
    CHECK(reloaded.grow_calls() >= 1);

    std::filesystem::remove_all(dir);
}

TEST_CASE("pre-sized cache tables avoid online growth") {
    auto dir = make_temp_dir("sbf_z4_cache_presize");

    std::vector<float> ep(12, 1.0f);
    std::vector<float> liaabb(6, 2.0f);
    sbf::Z4EpCache ep_small;
    sbf::Z4EpCache ep_large;
    REQUIRE(ep_small.open((dir / "ep_small.cache").string(), 12, 6, 64));
    REQUIRE(ep_large.open((dir / "ep_large.cache").string(), 12, 6, 512));
    for (uint64_t i = 1; i <= 160; ++i) {
        ep_small.insert(0x100000ULL + i, sbf::EndpointSource::IFK,
                        ep.data(), liaabb.data());
        ep_large.insert(0x100000ULL + i, sbf::EndpointSource::IFK,
                        ep.data(), liaabb.data());
    }
    CHECK(ep_small.grow_calls() > 0);
    CHECK(ep_large.grow_calls() == 0);
    CHECK(ep_small.capacity() > 64);
    CHECK(ep_large.capacity() == 512);

    sbf::GridQuality quality;
    quality.type = sbf::EnvelopeType::LinkIAABB_Grid;
    quality.resolution = 0.05f;
    quality.n_sub = 1;
    auto grid = make_grid(-0.08f, 0.08f, quality.resolution);

    sbf::Z4GridCache grid_small;
    sbf::Z4GridCache grid_large;
    REQUIRE(grid_small.open((dir / "grid_small.cache").string(), 64, 0, 0));
    REQUIRE(grid_large.open((dir / "grid_large.cache").string(), 512, 0, 0));
    for (uint64_t i = 1; i <= 160; ++i) {
        grid_small.insert(0x200000ULL + i, grid, quality);
        grid_large.insert(0x200000ULL + i, grid, quality);
    }
    CHECK(grid_small.grow_calls() > 0);
    CHECK(grid_large.grow_calls() == 0);
    CHECK(grid_small.capacity() > 64);
    CHECK(grid_large.capacity() == 512);

    std::filesystem::remove_all(dir);
}

TEST_CASE("LectCacheManager skips grid files for AABB-only payloads") {
    auto dir = make_temp_dir("sbf_lect_cache_manager_storage");

    sbf::LectCacheManager aabb_mgr;
    REQUIRE(aabb_mgr.init(0x55aaULL, "unit", 12,
                          sbf::EndpointSource::IFK,
                          sbf::EnvelopeType::LinkIAABB,
                          dir.string(), 0, 0, 64, 64));
    CHECK(aabb_mgr.ep_cache(0).is_open());
    CHECK(aabb_mgr.ep_cache(1).is_open());
    CHECK_FALSE(aabb_mgr.grid_cache(0).is_open());
    CHECK_FALSE(aabb_mgr.grid_cache(1).is_open());
    CHECK_FALSE(std::filesystem::exists(dir / "00000000000055aa" / "ep0_env0" / "grid_safe.cache"));
    CHECK_FALSE(std::filesystem::exists(dir / "00000000000055aa" / "ep0_env0" / "grid_unsafe.cache"));

    sbf::LectCacheManager grid_mgr;
    REQUIRE(grid_mgr.init(0x55aaULL, "unit", 12,
                          sbf::EndpointSource::IFK,
                          sbf::EnvelopeType::Hull16_Grid,
                          dir.string(), 0, 0, 64, 64));
    CHECK(grid_mgr.grid_cache(0).is_open());
    CHECK(grid_mgr.grid_cache(1).is_open());
    CHECK(std::filesystem::exists(dir / "00000000000055aa" / "ep0_env2" / "grid_safe.cache"));
    CHECK(std::filesystem::exists(dir / "00000000000055aa" / "ep0_env2" / "grid_unsafe.cache"));

    std::filesystem::remove_all(dir);
}

}  // TEST_SUITE
