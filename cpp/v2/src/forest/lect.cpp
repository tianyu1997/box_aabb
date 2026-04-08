// SafeBoxForest v2  LECT implementation
#include "sbf/forest/lect.h"
#include "sbf/forest/node_store.h"
#include "sbf/envelope/collision_policy.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/robot/interval_fk.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace sbf {
namespace forest {

using namespace envelope;

// -- Tag sidecar helpers (companion .meta text file) --------------------------

static std::string meta_path(const std::string& tree_path) {
    return tree_path + ".meta";
}

static std::string frames_path(const std::string& tree_path) {
    return tree_path + ".frames";
}

static std::string grids_path(const std::string& tree_path) {
    return tree_path + ".grids";
}

static void save_meta(const std::string& tree_path, const std::string& tag) {
    std::ofstream f(meta_path(tree_path));
    if (f) f << "tag=" << tag << "\n";
}

static std::string load_meta_tag(const std::string& tree_path) {
    std::ifstream f(meta_path(tree_path));
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("tag=", 0) == 0)
            return line.substr(4);
    }
    return "ifk";  // default for legacy caches without .meta
}

// -- Construction -------------------------------------------------------------

LECT::LECT(const Robot& robot, const std::string& tag)
    : tree_(robot)
    , frame_store_(robot)
    , tag_(tag)
{}

// -- Find Free Box ------------------------------------------------------------

FFBResult LECT::find_free_box(
    const Eigen::VectorXd& seed,
    const float* obs_compact, int n_obs,
    int max_depth, double min_edge) const
{
    // Build grid callbacks if grid union is enabled
    const FFBCallbacks* cb_ptr = nullptr;
    FFBCallbacks grid_cb;
    if (env_config_.use_grid_union && grid_store_ready_) {
        grid_cb = make_grid_callbacks();
        cb_ptr = &grid_cb;
    }

    return tree_.find_free_box(seed, obs_compact, n_obs, max_depth, min_edge,
                               env_config_.safety_check_depth, cb_ptr);
}

// -- Promotion ----------------------------------------------------------------

HierAABBTree::PromotionResult LECT::try_promote(
    const std::vector<int>& path,
    const float* obs_compact, int n_obs,
    int promotion_depth) const
{
    return tree_.try_promote(path, obs_compact, n_obs, promotion_depth);
}

// -- Node intervals -----------------------------------------------------------

std::vector<Interval> LECT::get_node_intervals(int node_idx) const
{
    return tree_.get_node_intervals(node_idx);
}

// -- Hull safety --------------------------------------------------------------

bool LECT::check_hull_safe(
    const std::vector<Interval>& hull,
    const std::vector<Interval>& a_ivs,
    const std::vector<Interval>& b_ivs,
    const float* obs_compact, int n_obs,
    int max_split_depth, double min_edge) const
{
    return tree_.check_hull_safe(
        hull, a_ivs, b_ivs, obs_compact, n_obs, max_split_depth, min_edge);
}

// -- Persistence --------------------------------------------------------------

void LECT::save(const std::string& tree_path)
{
    tree_.save(tree_path);
    save_meta(tree_path, tag_);

    // Persist FrameStore if it has valid nodes
    if (frame_store_.n_valid() > 0)
        frame_store_.save(frames_path(tree_path));

    // Persist GridStore if it has valid nodes
    if (grid_store_ready_ && grid_store_.n_valid() > 0)
        grid_store_.save(grids_path(tree_path));
}

void LECT::save_incremental(const std::string& tree_path)
{
    tree_.save_incremental(tree_path);
    save_meta(tree_path, tag_);

    // Incremental append of newly computed frames
    if (frame_store_.n_dirty() > 0) {
        auto fp = frames_path(tree_path);
        if (frame_store_.save_path().empty()) {
            // First time: full save
            frame_store_.save(fp);
        } else {
            frame_store_.save_incremental(fp);
        }
    }

    // Incremental append of newly computed grids
    if (grid_store_ready_ && grid_store_.n_dirty() > 0) {
        auto gp = grids_path(tree_path);
        if (grid_store_.save_path().empty()) {
            grid_store_.save(gp);
        } else {
            grid_store_.save_incremental(gp);
        }
    }
}

LECT LECT::load(const std::string& tree_path, const Robot& robot)
{
    LECT lect;
    lect.tree_  = HierAABBTree::load(tree_path, robot);
    lect.frame_store_ = FrameStore(robot);
    lect.tag_ = load_meta_tag(tree_path);

    // Load FrameStore if companion .frames file exists
    auto fp = frames_path(tree_path);
    if (std::filesystem::exists(fp))
        lect.frame_store_.load(fp);

    // Load GridStore if companion .grids file exists
    auto gp = grids_path(tree_path);
    if (std::filesystem::exists(gp)) {
        lect.grid_store_.load(gp);
        lect.grid_store_ready_ = true;
    }

    return lect;
}

LECT LECT::load_mmap(const std::string& tree_path, const Robot& robot)
{
    LECT lect;
    lect.tree_  = HierAABBTree::load_mmap(tree_path, robot);
    lect.frame_store_ = FrameStore(robot);
    lect.tag_ = load_meta_tag(tree_path);

    // Load FrameStore via mmap if FRM4 companion .frames file exists;
    // fall back to full load for legacy FRM3 files.
    auto fp = frames_path(tree_path);
    if (std::filesystem::exists(fp)) {
        // Peek magic to decide mmap or legacy load
        FILE* f = std::fopen(fp.c_str(), "rb");
        if (f) {
            uint32_t magic = 0;
            std::fread(&magic, sizeof(uint32_t), 1, f);
            std::fclose(f);
            if (magic == 0x344D5246)  // FRM4
                lect.frame_store_.load_mmap(fp);
            else
                lect.frame_store_.load(fp);
        }
    }

    // Load GridStore via mmap if GRD2 companion .grids file exists;
    // fall back to full load for legacy GRD1 files.
    auto gp = grids_path(tree_path);
    if (std::filesystem::exists(gp)) {
        FILE* f = std::fopen(gp.c_str(), "rb");
        if (f) {
            uint32_t magic = 0;
            std::fread(&magic, sizeof(uint32_t), 1, f);
            std::fclose(f);
            if (magic == 0x32445247)  // GRD2
                lect.grid_store_.load_mmap(gp);
            else
                lect.grid_store_.load(gp);
        }
        lect.grid_store_ready_ = true;
    }

    return lect;
}

void LECT::flush_mmap()
{
    tree_.flush_mmap();
    frame_store_.flush_mmap();
    grid_store_.flush_mmap();
}

// -- Occupation management ----------------------------------------------------

void LECT::mark_occupied(int node_idx, int forest_box_id)
{ tree_.mark_occupied(node_idx, forest_box_id); }

void LECT::unmark_occupied(int node_idx)
{ tree_.unmark_occupied(node_idx); }

void LECT::clear_boxes_occupation(const std::unordered_set<int>& box_ids)
{ tree_.clear_boxes_occupation(box_ids); }

void LECT::clear_all_occupation()
{ tree_.clear_all_occupation(); }

int LECT::find_containing_box_id(const Eigen::VectorXd& config) const
{ return tree_.find_containing_box_id(config); }

bool LECT::is_occupied(const Eigen::VectorXd& config) const
{ return tree_.is_occupied(config); }

bool LECT::sample_unoccupied_seed(
    std::mt19937& rng, Eigen::VectorXd& out, int max_walk_depth) const
{ return tree_.sample_unoccupied_seed(rng, out, max_walk_depth); }

// -- Frame Store operations ---------------------------------------------------

void LECT::store_node_frames(int node_idx, const FKState& fk)
{
    frame_store_.store_from_fk(node_idx, fk);
}

void LECT::union_node_frames(int parent_idx, int child_a, int child_b)
{
    frame_store_.union_frames(parent_idx, child_a, child_b);
}

bool LECT::refine_node_frames(int node_idx, int parent_idx)
{
    return frame_store_.refine_frames(node_idx, parent_idx);
}

// -- Frame-based collision checking -------------------------------------------

bool LECT::check_node_collision_policy(
    int node_idx,
    const float* obs_compact, int n_obs,
    const envelope::EnvelopeConfig& cfg) const
{
    const float* frames = frame_store_.get_frames(node_idx);
    if (!frames) {
        // No frames stored -- fall back to legacy AABB check
        const NodeStore& s = tree_.store();
        if (!s.has_aabb(node_idx)) return true;  // unknown -> assume collision
        return check_collision_aabb_legacy(
            s.aabb(node_idx), s.n_active_links(), obs_compact, n_obs);
    }

    return check_collision(
        cfg.collision_policy,
        frames,
        frame_store_.n_frames(),
        frame_store_.active_link_map(),
        frame_store_.n_active_links(),
        frame_store_.link_radii(),
        frame_store_.base_pos(),
        obs_compact, n_obs,
        cfg.link_subdivision_n,
        cfg.link_subdivision_max,
        cfg.grid_resolution,
        cfg.grid_world_bounds,
        nullptr);
}

bool LECT::check_node_collision_policy(
    int node_idx,
    const float* obs_compact, int n_obs) const
{
    return check_node_collision_policy(node_idx, obs_compact, n_obs, env_config_);
}

// -- Grid Store ---------------------------------------------------------------

void LECT::init_grid_store()
{
    if (grid_store_ready_) return;

    const auto& robot = tree_.robot();
    int n_active = robot.n_active_links();
    const int* link_map = robot.active_link_map();

    // n_frames = max frame_idx + 1
    int n_frames = 0;
    for (int i = 0; i < n_active; ++i)
        n_frames = std::max(n_frames, link_map[i] + 1);

    // Link radii (double → float)
    float radii[MAX_LINKS] = {};
    const double* dr = robot.active_link_radii();
    if (dr) {
        for (int i = 0; i < n_active; ++i)
            radii[i] = static_cast<float>(dr[i]);
    }

    float base_pos[3] = {0.f, 0.f, 0.f};

    grid_store_ = envelope::GridStore(
        n_frames, n_active, link_map, radii, base_pos,
        env_config_.grid_world_bounds, /*initial_capacity=*/1024);

    grid_store_ready_ = true;
}

FFBCallbacks LECT::make_grid_callbacks() const
{
    FFBCallbacks cb;

    // Alias pointers for capture (LECT lifetime outlives callbacks)
    auto* gs_ptr = const_cast<envelope::GridStore*>(&grid_store_);
    auto* fs_ptr = const_cast<envelope::FrameStore*>(&frame_store_);
    int n_sub = env_config_.grid_n_sub_per_link;

    // refine_check: AABB says collision → ask grid for refinement
    cb.refine_check = [gs_ptr](int node_idx, const float* obs_compact, int n_obs) -> bool {
        if (!gs_ptr->has_grid(node_idx))
            return true;  // no grid data → assume collision (conservative)
        return gs_ptr->check_collision(node_idx, obs_compact, n_obs);
    };

    // on_split: derive grid for new children + store frames
    cb.on_split = [gs_ptr, fs_ptr, n_sub](
            int left_idx, int right_idx, int parent_idx,
            const FKState& left_fk, const FKState& right_fk) {
        // Store frames for children
        fs_ptr->store_from_fk(left_idx, left_fk);
        fs_ptr->store_from_fk(right_idx, right_fk);

        // Derive grid for children from their frames
        const float* left_frames = fs_ptr->get_frames(left_idx);
        const float* right_frames = fs_ptr->get_frames(right_idx);
        if (left_frames)
            gs_ptr->derive_from_frames(left_idx, left_frames, n_sub);
        if (right_frames)
            gs_ptr->derive_from_frames(right_idx, right_frames, n_sub);

        // Union → parent grid
        if (gs_ptr->has_grid(left_idx) && gs_ptr->has_grid(right_idx))
            gs_ptr->union_grids(parent_idx, left_idx, right_idx);

        // Union → parent frames
        fs_ptr->union_frames(parent_idx, left_idx, right_idx);
    };

    // propagate_grid: union children grids → parent grid
    cb.propagate_grid = [gs_ptr](int parent_idx, int left_idx, int right_idx) {
        if (gs_ptr->has_grid(left_idx) && gs_ptr->has_grid(right_idx))
            gs_ptr->union_grids(parent_idx, left_idx, right_idx);
    };

    return cb;
}

} // namespace forest
} // namespace sbf
