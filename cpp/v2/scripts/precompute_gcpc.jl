#!/usr/bin/env julia
# ═══════════════════════════════════════════════════════════════════════════
# GCPC — Global Critical Point Cache precomputation
# ═══════════════════════════════════════════════════════════════════════════
#
# Precomputes ALL interior critical points of FK position for each active
# link of a robot, using HomotopyContinuation.jl for algebraic completeness.
#
# Symmetry reductions:
#   - q₀ eliminated analytically (7D → 6D): q₀* = atan2(-B, A)
#   - q₁ period-π: only solve q₁ ∈ [0, π], reflect at query time
#   - q₆ skip: when no tool link and d₆ = 0 (e.g., Panda without tool)
#
# Usage:
#   julia --project=. precompute_gcpc.jl panda        [--no-tool]
#   julia --project=. precompute_gcpc.jl iiwa14       [--no-tool]
#   julia --project=. precompute_gcpc.jl all
#
# Output:
#   {robot}_gcpc.json — critical point cache in JSON format
#
# Dependencies:
#   ] add HomotopyContinuation JSON

using Pkg
# Auto-install deps if missing
for pkg in ["HomotopyContinuation", "JSON"]
    if !haskey(Pkg.project().dependencies, pkg)
        @info "Installing $pkg..."
        Pkg.add(pkg)
    end
end

using HomotopyContinuation
using JSON
using LinearAlgebra

# ═══════════════════════════════════════════════════════════════════════════
#  DH Parameters
# ═══════════════════════════════════════════════════════════════════════════

struct DHParam
    alpha::Float64
    a::Float64
    d::Float64
    theta::Float64
end

struct RobotConfig
    name::String
    dh_params::Vector{DHParam}
    tool_frame::Union{DHParam, Nothing}
    active_links::Vector{Int}   # 0-based frame indices of active links
end

function load_robot(name::String; no_tool::Bool=false)
    configs_dir = joinpath(@__DIR__, "..", "..", "v1", "configs")

    if name == "panda"
        dh = [
            DHParam(0.0,          0.0, 0.333,  0.0),
            DHParam(-π/2,         0.0, 0.0,    0.0),
            DHParam( π/2,         0.0, 0.316,  0.0),
            DHParam( π/2,         0.0, 0.0,    0.0),
            DHParam(-π/2,         0.0, 0.384,  0.0),
            DHParam( π/2,         0.0, 0.0,    0.0),
            DHParam( π/2,         0.0, 0.0,    0.0),
        ]
        tool = no_tool ? nothing : DHParam(0.0, 0.0, 0.107, 0.0)
        # Active links: link 0 forced skip; links with d==0 && a==0 skipped
        # Panda: frame 2 (d=0.316), frame 4 (d=0.384), frame 7 (tool, d=0.107)
        active = no_tool ? [2, 4] : [2, 4, 7]
        return RobotConfig("panda", dh, tool, active)

    elseif name == "iiwa14"
        dh = [
            DHParam(0.0,    0.0, 0.1575, 0.0),
            DHParam(-π/2,   0.0, 0.0,    0.0),
            DHParam( π/2,   0.0, 0.2025, 0.0),
            DHParam( π/2,   0.0, 0.0,    0.0),
            DHParam(-π/2,   0.0, 0.2155, 0.0),
            DHParam(-π/2,   0.0, 0.0,    0.0),
            DHParam( π/2,   0.0, 0.081,  0.0),
        ]
        tool = no_tool ? nothing : DHParam(0.0, 0.0, 0.185, 0.0)
        # iiwa14: frame 2 (d=0.2025), frame 4 (d=0.2155), frame 6 (d=0.081), frame 7 (tool, d=0.185)
        active = no_tool ? [2, 4, 6] : [2, 4, 6, 7]
        return RobotConfig("iiwa14", dh, tool, active)
    else
        error("Unknown robot: $name")
    end
end

# ═══════════════════════════════════════════════════════════════════════════
#  Symbolic FK construction
# ═══════════════════════════════════════════════════════════════════════════
#
# Modified DH transform (Craig convention):
#
#   T(α, a, d, θ) = [ cθ    -sθ     0    a        ]
#                    [ sθ·cα  cθ·cα -sα  -d·sα     ]
#                    [ sθ·sα  cθ·sα  cα   d·cα     ]
#                    [ 0      0      0    1         ]
#
# With Weierstrass substitution t = tan(θ/2):
#   cosθ = (1 - t²)/(1 + t²)
#   sinθ = 2t/(1 + t²)

"""
    build_symbolic_fk(robot, link_idx)

Build the FK position as rational functions in Weierstrass variables t₁..tₖ.
Returns (px, py, pz, denominator) where px,py,pz are polynomials and the true
position is px/den, py/den, pz/den.

Joint 0 (q₀) is analytically eliminated — only joints 1..nj appear.
For the tool link (link_idx == n_joints), chain includes all joints + tool frame.
"""
function build_symbolic_fk(robot::RobotConfig, link_idx::Int)
    # Determine which joints affect this link
    # link_idx is 0-based frame index:
    #   frame 0 = base (identity)
    #   frame i = after joint i-1 (1-based frames)
    #   frame n_joints+1 = tool frame (if present)
    n_joints = length(robot.dh_params)

    if link_idx <= n_joints
        nj = link_idx  # joints 0..link_idx-1
    else
        # tool link
        nj = n_joints
    end

    # Skip joint 0 (analytically eliminated) → effective joints: 1..nj-1
    # These are 0-based joint indices; in Julia we use 1-based
    n_eff = nj - 1  # number of effective joints (q₁..q_{nj-1})
    if n_eff <= 0
        return nothing  # link only depends on q₀ or nothing
    end

    # Check if last joint (q₆, index 6) can be skipped
    # This happens when: link_idx is tool, and we want it included,
    # OR link_idx < 7 so q₆ doesn't matter anyway
    skip_q6 = false
    if !isnothing(robot.tool_frame) && link_idx == n_joints + 1
        # tool link — q₆ needed
    elseif nj >= 7
        # Check if frame 6 is inactive (d₆=0, a₆=0) and link is not tool
        dh6 = robot.dh_params[7]  # 1-based
        if abs(dh6.d) < 1e-10 && abs(dh6.a) < 1e-10 && link_idx <= n_joints
            skip_q6 = true
            n_eff = min(n_eff, 5)  # only q₁..q₅
        end
    end

    @info "Link $link_idx: n_eff=$n_eff joints (skip_q6=$skip_q6)"

    # Create Weierstrass variables t₁..t_{n_eff} (for joints 1..n_eff)
    @var t[1:n_eff]

    # Build rational functions for cos(qⱼ), sin(qⱼ) via Weierstrass
    # t_j = tan(q_j / 2)
    # cos(q_j) = (1 - t_j²) / (1 + t_j²)
    # sin(q_j) = 2t_j / (1 + t_j²)

    # We'll track transform as 4x4 of rational expressions
    # To avoid fractions, we work with numerators and a common denominator

    # Actually, it's cleaner to work with the 4x4 matrix of polynomials
    # divided by a scalar denominator that we accumulate.

    # Start from frame AFTER joint 0 (T₀ is eliminated).
    # In the T₀ frame, the position decomposes as:
    #   p'_x = cos(q₀) · A - sin(q₀) · B     (we want critical of R² = A² + B²)
    #   p'_y = sin(q₀) · A + cos(q₀) · B      (same critical set)
    #   p'_z = C + d₀                          (we want ∇C = 0)
    # where (A, B, C) come from the sub-chain in the T₀ frame.
    #
    # T₀ is a pure z-rotation + z-translation, so after T₀:
    #   sub-chain position (x,y,z) in T₀-frame = T₁ · T₂ ··· T_k · [0,0,0,1]
    # Then A = position_x, B = position_y, C = position_z
    # (No, this isn't right — let me be more careful.)
    #
    # T₀ = Rz(q₀) · Tz(d₀). After T₀, coordinate axes are rotated by q₀.
    # P_world = T₀ · P_local
    # P_local = T₁ · T₂ ··· · [0,0,0,1]
    # P_world_xyz = Rz(q₀) · P_local_xyz + [0, 0, d₀]
    #
    # So A = P_local_x, B = P_local_y, C = P_local_z
    # p_world_x = cos(q₀)·A - sin(q₀)·B
    # p_world_y = sin(q₀)·A + cos(q₀)·B
    # p_world_z = C + d₀

    # Build sub-chain: T₁ · T₂ ··· T_k [· T_tool]
    # Using Expression-based approach with accumulated denominator.
    # Initialize identity as Expression matrix
    T = [Expression(1) Expression(0) Expression(0) Expression(0);
         Expression(0) Expression(1) Expression(0) Expression(0);
         Expression(0) Expression(0) Expression(1) Expression(0);
         Expression(0) Expression(0) Expression(0) Expression(1)]

    den_total = Expression(1)

    for j in 1:nj  # joints 1..nj (0-based: 0..nj-1)
        if j == 1
            # Joint 0 — skip (analytically eliminated)
            continue
        end

        dh = robot.dh_params[j]  # 1-based
        ca = cos(dh.alpha)
        sa = sin(dh.alpha)
        a = dh.a
        d = dh.d

        # t-index for this joint: joint j (1-based DH) = q_{j-1} (0-based)
        # Effective index: joint 1 → t[1], joint 2 → t[2], ... joint n_eff → t[n_eff]
        # But joint 0 is skipped, so joint j (1-based DH) uses t[j-1]
        t_idx = j - 1
        if skip_q6 && t_idx > n_eff
            break
        end

        if t_idx >= 1 && t_idx <= n_eff
            tj = t[t_idx]
            ct_num = 1 - tj^2
            st_num = 2 * tj
            den_j = 1 + tj^2
        else
            ct_num = Expression(1)
            st_num = Expression(0)
            den_j = Expression(1)
        end

        # DH matrix * den_j (to clear fraction)
        Tj = [ct_num        -st_num       Expression(0)    a*den_j;
              st_num*ca      ct_num*ca    -sa*den_j        -d*sa*den_j;
              st_num*sa      ct_num*sa     ca*den_j         d*ca*den_j;
              Expression(0)  Expression(0) Expression(0)    den_j]

        # T_new = T_old * Tj / den_j
        # Accumulate: T = T * Tj, den_total *= den_j
        T_new = T * Tj
        den_total = den_total * den_j
        T = T_new
    end

    # Tool frame (if needed and present)
    if link_idx > length(robot.dh_params) && !isnothing(robot.tool_frame)
        tool = robot.tool_frame
        ca = cos(tool.alpha)
        sa = sin(tool.alpha)
        # Tool frame is constant (no joint variable)
        Tt = [Expression(1)  Expression(0)  Expression(0)   tool.a;
              Expression(0)  Expression(ca)  Expression(-sa) -tool.d*sa;
              Expression(0)  Expression(sa)  Expression(ca)   tool.d*ca;
              Expression(0)  Expression(0)   Expression(0)   Expression(1)]
        T = T * Tt
    end

    # Extract position: P = T * [0,0,0,1]^T = column 4 of T
    # A = T[1,4], B = T[2,4], C = T[3,4]
    # True position: A/den_total, B/den_total, C/den_total
    A_num = T[1,4]
    B_num = T[2,4]
    C_num = T[3,4]

    return (A_num, B_num, C_num, den_total, n_eff, t)
end


# ═══════════════════════════════════════════════════════════════════════════
#  Construct and solve polynomial systems
# ═══════════════════════════════════════════════════════════════════════════

"""
Construct ∇R² = 0 system (for x/y directions).
R² = A² + B² (numerators; we differentiate the numerator form).
"""
function build_R2_system(A_num, B_num, den, t_vars)
    n = length(t_vars)
    # R² ∝ A_num² + B_num² (mod den²)
    # But critical points of A_num/den satisfy:
    #   d/dt_j (A/den) = (A'·den - A·den') / den²  = 0
    #   ⟹ A'·den - A·den' = 0
    #
    # For R² = (A/den)² + (B/den)² = (A² + B²)/den²:
    #   d/dt_j (R²) = [2(A·A' + B·B')·den² - 2(A²+B²)·den·den'] / den⁴
    #               = 2·den · [(A·A' + B·B')·den - (A²+B²)·den'] / den⁴
    #               = 2 · [(A·A' + B·B')·den - (A²+B²)·den'] / den³
    #
    # Setting to zero (den ≠ 0):
    #   (A·A' + B·B')·den - (A²+B²)·den' = 0  for each t_j
    #
    # This is degree ~ 2*(deg_A) + deg_den per equation.
    # That's very high degree. Let me use a simpler formulation.
    #
    # Alternative: separate A and B.
    # For max/min of p_x = cos(q₀)·A/den - sin(q₀)·B/den:
    # At optimal q₀, p_x = √(A² + B²)/den.
    # So ∇(√(A²+B²)/den) = 0  ⟺  ∇R² = 0 (since sqrt is monotone, R²>0)
    # where R² = (A²+B²)/den².
    #
    # Actually for AABB we need max and min of p_x over q₀:
    #   max_{q₀} p_x = +√((A/den)² + (B/den)²) = +R/den (always positive)
    #   min_{q₀} p_x = -R/den
    # So max p_x = max R, min p_x = -max R, and the critical set is the same.
    #
    # Let's work with f = A² + B² and g = den². Then R² = f/g.
    # ∂R²/∂t_j = (f'g - fg')/ g² = 0  ⟹  f'g - fg' = 0
    #
    # This avoids computing √.

    f = A_num^2 + B_num^2
    g = den^2

    eqs = Expression[]
    for j in 1:n
        f_j = differentiate(f, t_vars[j])
        g_j = differentiate(g, t_vars[j])
        eq = f_j * g - f * g_j
        push!(eqs, eq)
    end

    return System(eqs; variables=t_vars)
end

"""
Construct ∇(C/den) = 0 system (for z direction).
"""
function build_z_system(C_num, den, t_vars)
    n = length(t_vars)
    # ∂/∂t_j (C/den) = (C'·den - C·den') / den² = 0
    # ⟹ C'·den - C·den' = 0

    eqs = Expression[]
    for j in 1:n
        C_j = differentiate(C_num, t_vars[j])
        d_j = differentiate(den, t_vars[j])
        eq = C_j * den - C_num * d_j
        push!(eqs, eq)
    end

    return System(eqs; variables=t_vars)
end

"""
Solve a polynomial system and extract real solutions.
Returns array of Float64 vectors.
"""
function solve_and_filter(sys::System; q1_half_range::Bool=true)
    nv = nvariables(sys)
    degs = degrees(sys)
    @info "Solving system: $nv vars, degrees=$degs"
    @info "  Bézout bound = $(prod(degs))"

    result = solve(sys; show_progress=true)

    @info "  Total solutions: $(nresults(result))"
    @info "  Real solutions: $(nreal(result))"

    real_sols = real_solutions(result; tol=1e-8)

    # Convert from Weierstrass t_j back to q_j = 2·atan(t_j)
    q_sols = Vector{Float64}[]
    for sol in real_sols
        q = [2.0 * atan(t) for t in sol]

        # q₁ half-range filter: keep only q₁ ∈ [0, π]
        # In Weierstrass: q₁ ∈ [0, π] ⟺ t₁ ∈ [0, ∞) ⟺ t₁ ≥ 0
        # But also q₁ = 0 (t₁=0) and q₁ = π (t₁ → ∞, singular)
        # We keep t₁ ≥ 0, and separately handle t₁ → ∞ via homogenization
        if q1_half_range && length(q) >= 1
            if q[1] < -1e-10 || q[1] > π + 1e-10
                continue  # discard
            end
            q[1] = clamp(q[1], 0.0, π)
        end

        push!(q_sols, q)
    end

    @info "  After q₁ filter: $(length(q_sols)) solutions"
    return q_sols
end


# ═══════════════════════════════════════════════════════════════════════════
#  kπ/2 background enumeration
# ═══════════════════════════════════════════════════════════════════════════

"""
Enumerate all kπ/2 vertex configurations for n_eff joints.
q₁ restricted to [0, π]: values {0, π/2} only (π maps to 0 via period-π).
Other joints: {-π, -π/2, 0, π/2, π}.
"""
function enumerate_kpi2(n_eff::Int)
    q1_vals = [0.0, π/2]  # q₁ in [0,π], kπ/2 values
    other_vals = [-π, -π/2, 0.0, π/2, π]  # other joints full range

    configs = Vector{Float64}[]

    function recurse(j, partial)
        if j > n_eff
            push!(configs, copy(partial))
            return
        end
        vals = (j == 1) ? q1_vals : other_vals
        for v in vals
            push!(partial, v)
            recurse(j + 1, partial)
            pop!(partial)
        end
    end

    recurse(1, Float64[])
    return configs
end


# ═══════════════════════════════════════════════════════════════════════════
#  Numerical FK verification
# ═══════════════════════════════════════════════════════════════════════════

"""
Evaluate FK position numerically for verification.
Returns (A, B, C) in the T₀-local frame.
q_eff: vector of q₁..q_{n_eff} (0-based: q₁..q_k)
"""
function fk_eval(robot::RobotConfig, link_idx::Int, q_eff::Vector{Float64})
    nj = min(link_idx, length(robot.dh_params))
    n_eff = length(q_eff)

    # Build sub-chain: T₁ · T₂ ··· T_k [· T_tool]
    T = Matrix{Float64}(I, 4, 4)

    for j in 2:nj  # joints 1..nj-1 (1-based DH index), skipping joint 0
        dh = robot.dh_params[j]
        t_idx = j - 1  # effective joint index
        if t_idx >= 1 && t_idx <= n_eff
            theta = q_eff[t_idx]
        else
            theta = 0.0  # shouldn't happen for needed joints
        end

        ct = cos(theta)
        st = sin(theta)
        ca = cos(dh.alpha)
        sa = sin(dh.alpha)

        Tj = [ct      -st      0     dh.a;
              st*ca    ct*ca   -sa   -dh.d*sa;
              st*sa    ct*sa    ca    dh.d*ca;
              0        0        0     1]
        T = T * Tj
    end

    # Tool frame
    if link_idx > length(robot.dh_params) && !isnothing(robot.tool_frame)
        tool = robot.tool_frame
        ca = cos(tool.alpha)
        sa = sin(tool.alpha)
        Tt = [1  0    0     tool.a;
              0  ca  -sa   -tool.d*sa;
              0  sa   ca    tool.d*ca;
              0  0    0     1]
        T = T * Tt
    end

    # Position: column 4
    A = T[1,4]
    B = T[2,4]
    C = T[3,4]
    return (A, B, C)
end

"""
Verify a critical point by checking ∇p = 0 numerically.
"""
function verify_critical(robot::RobotConfig, link_idx::Int,
                         q_eff::Vector{Float64}, direction::Symbol)
    eps = 1e-7
    n = length(q_eff)

    A, B, C = fk_eval(robot, link_idx, q_eff)

    grad = zeros(n)
    for j in 1:n
        q_plus = copy(q_eff)
        q_minus = copy(q_eff)
        q_plus[j] += eps
        q_minus[j] -= eps

        Ap, Bp, Cp = fk_eval(robot, link_idx, q_plus)
        Am, Bm, Cm = fk_eval(robot, link_idx, q_minus)

        if direction == :xy
            R2 = A^2 + B^2
            R2p = Ap^2 + Bp^2
            R2m = Am^2 + Bm^2
            grad[j] = (R2p - R2m) / (2 * eps)
        else  # :z
            grad[j] = (Cp - Cm) / (2 * eps)
        end
    end

    return norm(grad)
end


# ═══════════════════════════════════════════════════════════════════════════
#  Main precomputation pipeline
# ═══════════════════════════════════════════════════════════════════════════

struct CriticalPoint
    q_eff::Vector{Float64}   # q₁..qₖ (q₁ in [0,π])
    link_id::Int             # 0-based frame index
    direction::Symbol        # :xy or :z
    p_critical::Float64      # critical position value
    source::Symbol           # :homotopy or :kpi2
    grad_norm::Float64       # |∇| at this point (for verification)
end

function precompute_robot(robot::RobotConfig)
    @info "═══ Precomputing GCPC for $(robot.name) ═══"
    @info "Active links: $(robot.active_links)"

    all_criticals = CriticalPoint[]

    for link_idx in robot.active_links
        @info "──── Link $link_idx ────"

        # Determine effective joints
        nj = min(link_idx, length(robot.dh_params))
        n_eff = nj - 1  # joints q₁..q_{nj-1}

        # Check q₆ skip
        if n_eff >= 6 && link_idx <= length(robot.dh_params)
            dh6 = robot.dh_params[7]  # joint 6, 1-based
            if abs(dh6.d) < 1e-10 && abs(dh6.a) < 1e-10
                n_eff = 5
                @info "  q₆ skipped (d₆=0, a₆=0, not tool)"
            end
        end

        if n_eff <= 0
            @info "  Skipping (no effective joints)"
            continue
        end

        @info "  Effective joints: q₁..q_$n_eff ($n_eff DOF)"

        # ── Phase 1: kπ/2 enumeration ────────────────────────────────────
        kpi2_configs = enumerate_kpi2(n_eff)
        @info "  kπ/2 configs: $(length(kpi2_configs))"

        for cfg in kpi2_configs
            A, B, C = fk_eval(robot, link_idx, cfg)
            R = sqrt(A^2 + B^2)
            grad_xy = verify_critical(robot, link_idx, cfg, :xy)
            grad_z  = verify_critical(robot, link_idx, cfg, :z)

            push!(all_criticals, CriticalPoint(cfg, link_idx, :xy, R, :kpi2, grad_xy))
            push!(all_criticals, CriticalPoint(cfg, link_idx, :z, C, :kpi2, grad_z))
        end

        # ── Phase 2: Homotopy continuation ────────────────────────────────
        fk_result = build_symbolic_fk(robot, link_idx)
        if isnothing(fk_result)
            @info "  Skipping homotopy (no symbolic FK)"
            continue
        end

        A_num, B_num, C_num, den_total, n_eff_actual, t_vars = fk_result

        if n_eff_actual != n_eff
            @warn "  n_eff mismatch: expected $n_eff, got $n_eff_actual"
            n_eff = n_eff_actual
        end

        # ── R² system (x/y directions) ──
        @info "  Building R² system..."
        try
            R2_sys = build_R2_system(A_num, B_num, den_total, collect(t_vars))
            @info "  Solving R² system..."
            R2_sols = solve_and_filter(R2_sys; q1_half_range=true)

            for sol in R2_sols
                A, B, C = fk_eval(robot, link_idx, sol)
                R = sqrt(A^2 + B^2)
                grad = verify_critical(robot, link_idx, sol, :xy)
                push!(all_criticals, CriticalPoint(sol, link_idx, :xy, R, :homotopy, grad))
            end
            @info "  R² solutions: $(length(R2_sols))"
        catch e
            @warn "  R² system failed: $e"
        end

        # ── z system ──
        @info "  Building z system..."
        try
            z_sys = build_z_system(C_num, den_total, collect(t_vars))
            @info "  Solving z system..."
            z_sols = solve_and_filter(z_sys; q1_half_range=true)

            for sol in z_sols
                A, B, C = fk_eval(robot, link_idx, sol)
                grad = verify_critical(robot, link_idx, sol, :z)
                push!(all_criticals, CriticalPoint(sol, link_idx, :z, C, :homotopy, grad))
            end
            @info "  z solutions: $(length(z_sols))"
        catch e
            @warn "  z system failed: $e"
        end
    end

    # ── Dedup ─────────────────────────────────────────────────────────────
    @info "Total raw critical points: $(length(all_criticals))"

    # Dedup: merge points that are within ε in joint space
    unique_criticals = CriticalPoint[]
    for cp in all_criticals
        is_dup = false
        for ucp in unique_criticals
            if ucp.link_id == cp.link_id && ucp.direction == cp.direction
                if norm(ucp.q_eff - cp.q_eff) < 1e-6
                    is_dup = true
                    break
                end
            end
        end
        if !is_dup
            push!(unique_criticals, cp)
        end
    end
    @info "After dedup: $(length(unique_criticals))"

    # ── Verification stats ────────────────────────────────────────────────
    n_verified = count(cp -> cp.grad_norm < 1e-4, unique_criticals)
    n_approx = count(cp -> cp.grad_norm >= 1e-4 && cp.grad_norm < 1e-2, unique_criticals)
    n_bad = count(cp -> cp.grad_norm >= 1e-2, unique_criticals)
    @info "Verification: $n_verified exact, $n_approx approx, $n_bad suspect"

    return unique_criticals
end


# ═══════════════════════════════════════════════════════════════════════════
#  JSON output
# ═══════════════════════════════════════════════════════════════════════════

function save_gcpc_json(robot::RobotConfig, criticals::Vector{CriticalPoint}, outpath::String)
    data = Dict(
        "robot" => robot.name,
        "has_tool" => !isnothing(robot.tool_frame),
        "active_links" => robot.active_links,
        "n_points" => length(criticals),
        "points" => [Dict(
            "q_eff" => cp.q_eff,
            "link_id" => cp.link_id,
            "direction" => string(cp.direction),
            "p_critical" => cp.p_critical,
            "source" => string(cp.source),
            "grad_norm" => cp.grad_norm,
        ) for cp in criticals]
    )
    open(outpath, "w") do io
        JSON.print(io, data, 2)
    end
    @info "Saved $(length(criticals)) critical points to $outpath"
end


# ═══════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════

function main()
    if length(ARGS) < 1
        println("Usage: julia precompute_gcpc.jl <robot_name> [--no-tool]")
        println("  robot_name: panda, iiwa14, or all")
        return
    end

    robot_name = ARGS[1]
    no_tool = "--no-tool" in ARGS

    outdir = joinpath(@__DIR__, "..", "results", "gcpc")
    mkpath(outdir)

    robots = if robot_name == "all"
        [("panda", false), ("panda", true), ("iiwa14", false), ("iiwa14", true)]
    else
        [(robot_name, no_tool)]
    end

    for (name, skip_tool) in robots
        robot = load_robot(name; no_tool=skip_tool)
        suffix = skip_tool ? "_notool" : ""
        criticals = precompute_robot(robot)
        outpath = joinpath(outdir, "$(name)$(suffix)_gcpc.json")
        save_gcpc_json(robot, criticals, outpath)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
