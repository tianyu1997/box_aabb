#!/usr/bin/env julia
# ═══════════════════════════════════════════════════════════════════════════
# GCPC — Global Critical Point Cache precomputation (v2: c/s parameterization)
# ═══════════════════════════════════════════════════════════════════════════
#
# Uses cos/sin variables with circle constraints instead of Weierstrass
# substitution, keeping polynomial degrees low (multilinear FK).
#
# Weierstrass gives degree ~37/eq for 5 DOF → Bézout 62M (infeasible).
# c/s gives degree ~10/eq + degree-2 circles → mixed volume ~10K (feasible).
#
# Symmetry reductions:
#   - q₀ eliminated analytically (7D → 6D)
#   - q₁ period-π: only solve q₁ ∈ [0, π]
#   - q₆ skip: when no tool link and d₆ = 0
#
# Usage:
#   julia precompute_gcpc.jl panda        [--no-tool]
#   julia precompute_gcpc.jl iiwa14       [--no-tool]
#   julia precompute_gcpc.jl all

using Pkg
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
    active_links::Vector{Int}   # 0-based frame indices
end

function load_robot(name::String; no_tool::Bool=false)
    if name == "panda"
        dh = [
            DHParam(0.0,    0.0, 0.333,  0.0),
            DHParam(-π/2,   0.0, 0.0,    0.0),
            DHParam( π/2,   0.0, 0.316,  0.0),
            DHParam( π/2,   0.0, 0.0,    0.0),
            DHParam(-π/2,   0.0, 0.384,  0.0),
            DHParam( π/2,   0.0, 0.0,    0.0),
            DHParam( π/2,   0.0, 0.0,    0.0),
        ]
        tool = no_tool ? nothing : DHParam(0.0, 0.0, 0.107, 0.0)
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
        active = no_tool ? [2, 4, 6] : [2, 4, 6, 7]
        return RobotConfig("iiwa14", dh, tool, active)
    else
        error("Unknown robot: $name")
    end
end


# ═══════════════════════════════════════════════════════════════════════════
#  Symbolic FK with c/s parameterization
# ═══════════════════════════════════════════════════════════════════════════
#
# Variables: c₁,s₁, c₂,s₂, ..., cₙ,sₙ
# Each DH transform is LINEAR in (cⱼ,sⱼ) → FK position is MULTILINEAR.
# This keeps degrees low: A,B,C have degree ≤ n (vs ~2n with Weierstrass).
# R² = A²+B² ≤ 2n. Gradient equations ≤ 2n.
# Circle constraints cⱼ²+sⱼ² = 1 add degree-2 equations.
#
# Modified DH (Craig):
#   T(α, a, d, q) = [ cq    -sq     0    a        ]
#                    [ sq·cα  cq·cα -sα  -d·sα     ]
#                    [ sq·sα  cq·sα  cα   d·cα     ]
#                    [ 0      0      0    1         ]

"""
Build symbolic FK as multilinear expressions in (c_j, s_j).
Returns (A, B, C, n_eff, c_vars, s_vars) where A,B,C are the position
components in the T₀-local frame.
"""
function build_symbolic_fk(robot::RobotConfig, link_idx::Int)
    n_joints = length(robot.dh_params)

    # Frame k (0-based) needs DH transforms 0..k, i.e., k+1 transforms.
    # Tool frame (link_idx == n_joints) needs all n_joints transforms + tool.
    include_tool = false
    if link_idx == n_joints && !isnothing(robot.tool_frame)
        nj = n_joints
        include_tool = true
    else
        nj = min(link_idx + 1, n_joints)  # k+1 transforms for frame k
    end

    n_eff = nj - 1  # skip joint 0 (analytically eliminated)
    if n_eff <= 0
        return nothing
    end

    # Check q₆ skip: when d₆=a₆=0 and NOT tool link
    skip_q6 = false
    if n_eff >= 6 && !include_tool
        dh6 = robot.dh_params[7]
        if abs(dh6.d) < 1e-10 && abs(dh6.a) < 1e-10
            skip_q6 = true
            n_eff = min(n_eff, 5)
        end
    end

    @info "Link $link_idx: n_eff=$n_eff joints (skip_q6=$skip_q6)"

    # Create c/s variable pairs
    @var c[1:n_eff] s[1:n_eff]

    # Build chain T₁ · T₂ · ... · Tₖ [· T_tool]
    # Start from identity (after T₀ which is eliminated)
    T = [Expression(1) Expression(0) Expression(0) Expression(0);
         Expression(0) Expression(1) Expression(0) Expression(0);
         Expression(0) Expression(0) Expression(1) Expression(0);
         Expression(0) Expression(0) Expression(0) Expression(1)]

    for j in 1:nj
        if j == 1
            continue  # skip joint 0 (analytically eliminated)
        end

        dh = robot.dh_params[j]
        ca = cos(dh.alpha)
        sa = sin(dh.alpha)
        a_val = dh.a
        d_val = dh.d

        t_idx = j - 1  # effective joint index
        if skip_q6 && t_idx > n_eff
            break
        end

        if t_idx >= 1 && t_idx <= n_eff
            cq = c[t_idx]   # symbolic cos(qⱼ)
            sq = s[t_idx]   # symbolic sin(qⱼ)
        else
            cq = Expression(1)
            sq = Expression(0)
        end

        # DH transform matrix — linear in (cq, sq)
        Tj = [cq         -sq          Expression(0)    a_val;
              sq * ca     cq * ca     -sa               -d_val * sa;
              sq * sa     cq * sa      ca                d_val * ca;
              Expression(0) Expression(0) Expression(0)  Expression(1)]

        T = T * Tj
    end

    # Tool frame
    if include_tool && !isnothing(robot.tool_frame)
        tool = robot.tool_frame
        ca = cos(tool.alpha)
        sa = sin(tool.alpha)
        Tt = [Expression(1)  Expression(0)   Expression(0)   tool.a;
              Expression(0)  Expression(ca)  Expression(-sa) -tool.d * sa;
              Expression(0)  Expression(sa)  Expression(ca)   tool.d * ca;
              Expression(0)  Expression(0)   Expression(0)   Expression(1)]
        T = T * Tt
    end

    A = T[1,4]  # x in T₀ frame
    B = T[2,4]  # y in T₀ frame
    C = T[3,4]  # z in T₀ frame

    # Detect which joints actually affect position (auto-prune irrelevant DOF)
    all_vars_used = union(variables(A), variables(B), variables(C))
    active_idx = [j for j in 1:n_eff if c[j] in all_vars_used || s[j] in all_vars_used]
    @info "  Variables in FK: $(length(all_vars_used)), active joints: $active_idx / $n_eff"

    return (A, B, C, n_eff, c, s, active_idx)
end


# ═══════════════════════════════════════════════════════════════════════════
#  Build polynomial systems with circle constraints
# ═══════════════════════════════════════════════════════════════════════════

"""
Build ∇_{circle} R² = 0 system, using only active joints.

R² = A² + B² with constraint cⱼ² + sⱼ² = 1.
Critical points on ∏ S¹: ∂R²/∂θⱼ = 0 for active joints j.
"""
function build_R2_system(A, B, c_vars, s_vars, active_idx)
    R2 = A^2 + B^2
    n_active = length(active_idx)
    if n_active == 0
        return nothing
    end

    eqs = Expression[]

    # Gradient equations for active joints only
    for j in active_idx
        dR2_dc = differentiate(R2, c_vars[j])
        dR2_ds = differentiate(R2, s_vars[j])
        eq = -s_vars[j] * dR2_dc + c_vars[j] * dR2_ds
        push!(eqs, eq)
    end

    # Circle constraints for active joints only
    for j in active_idx
        push!(eqs, c_vars[j]^2 + s_vars[j]^2 - 1)
    end

    # Variable groups: [(cⱼ, sⱼ)] for multi-homogeneous structure
    var_groups = [Variable[c_vars[j], s_vars[j]] for j in active_idx]

    return System(eqs; variable_groups=var_groups)
end

"""
Build ∇_{circle} C = 0 system (z direction), using only active joints.
"""
function build_z_system(C_expr, c_vars, s_vars, active_idx)
    n_active = length(active_idx)
    if n_active == 0
        return nothing
    end

    eqs = Expression[]

    for j in active_idx
        dC_dc = differentiate(C_expr, c_vars[j])
        dC_ds = differentiate(C_expr, s_vars[j])
        eq = -s_vars[j] * dC_dc + c_vars[j] * dC_ds
        push!(eqs, eq)
    end

    for j in active_idx
        push!(eqs, c_vars[j]^2 + s_vars[j]^2 - 1)
    end

    var_groups = [Variable[c_vars[j], s_vars[j]] for j in active_idx]

    return System(eqs; variable_groups=var_groups)
end


# ═══════════════════════════════════════════════════════════════════════════
#  Solver
# ═══════════════════════════════════════════════════════════════════════════

"""
Solve a polynomial system and extract real solutions on the circles.
Converts (cⱼ, sⱼ) back to qⱼ = atan2(sⱼ, cⱼ).
Expands reduced solutions (active joints only) back to full n_eff-dimensional space.
"""
function solve_and_filter(sys::System;
                          q1_half_range::Bool=true,
                          active_idx::Vector{Int}=Int[],
                          n_eff::Int=0)
    nv = nvariables(sys)
    n_active = nv ÷ 2
    degs = degrees(sys)
    bezout = prod(big.(degs))
    @info "System: $nv vars ($n_active active joints), degrees=$degs, Bézout=$bezout"
    flush(stdout)

    # Use total_degree start system (polyhedral hangs on HC.jl 2.x)
    result = solve(sys; show_progress=true)

    @info "  Total: $(nresults(result)), Real: $(nreal(result))"

    real_sols = real_solutions(result; tol=1e-6)

    # Convert (c,s) pairs to angles, expand to full n_eff space
    q_sols = Vector{Float64}[]
    for sol in real_sols
        # sol = [c_{a1}, s_{a1}, c_{a2}, s_{a2}, ...] for active joints
        valid = true
        q_active = Float64[]
        for k in 1:n_active
            cj = sol[2k-1]
            sj = sol[2k]
            if abs(cj^2 + sj^2 - 1.0) > 1e-4
                valid = false
                break
            end
            push!(q_active, atan(sj, cj))
        end
        if !valid
            continue
        end

        # Expand to full n_eff space (inactive joints = 0)
        if n_eff > 0 && !isempty(active_idx)
            q_full = zeros(n_eff)
            for (k, j) in enumerate(active_idx)
                q_full[j] = q_active[k]
            end
        else
            q_full = q_active
        end

        # q₁ half-range: keep q₁ ∈ [0, π]
        if q1_half_range && length(q_full) >= 1
            q_full[1] = mod(q_full[1] + π, 2π) - π
            if q_full[1] < -1e-10
                q_full[1] += π
            end
            q_full[1] = clamp(q_full[1], 0.0, π)
        end

        push!(q_sols, q_full)
    end

    @info "  After filter: $(length(q_sols)) solutions"
    return q_sols
end


# ═══════════════════════════════════════════════════════════════════════════
#  kπ/2 enumeration
# ═══════════════════════════════════════════════════════════════════════════

function enumerate_kpi2(n_eff::Int)
    q1_vals = [0.0, π/2]
    other_vals = [-π, -π/2, 0.0, π/2, π]

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
#  Numerical FK evaluation and verification
# ═══════════════════════════════════════════════════════════════════════════

function fk_eval(robot::RobotConfig, link_idx::Int, q_eff::Vector{Float64})
    n_joints = length(robot.dh_params)
    n_eff = length(q_eff)

    # Same indexing as build_symbolic_fk
    include_tool = (link_idx == n_joints && !isnothing(robot.tool_frame))
    nj = include_tool ? n_joints : min(link_idx + 1, n_joints)

    T = Matrix{Float64}(I, 4, 4)
    for j in 2:nj
        dh = robot.dh_params[j]
        t_idx = j - 1
        theta = (t_idx >= 1 && t_idx <= n_eff) ? q_eff[t_idx] : 0.0

        ct = cos(theta); st = sin(theta)
        ca = cos(dh.alpha); sa = sin(dh.alpha)

        Tj = [ct      -st      0     dh.a;
              st*ca    ct*ca   -sa   -dh.d*sa;
              st*sa    ct*sa    ca    dh.d*ca;
              0        0        0     1]
        T = T * Tj
    end

    if include_tool
        tool = robot.tool_frame
        ca = cos(tool.alpha); sa = sin(tool.alpha)
        Tt = [1  0    0     tool.a;
              0  ca  -sa   -tool.d*sa;
              0  sa   ca    tool.d*ca;
              0  0    0     1]
        T = T * Tt
    end

    return (T[1,4], T[2,4], T[3,4])
end

function verify_critical(robot::RobotConfig, link_idx::Int,
                         q_eff::Vector{Float64}, direction::Symbol)
    eps = 1e-7
    n = length(q_eff)
    A, B, C = fk_eval(robot, link_idx, q_eff)

    grad = zeros(n)
    for j in 1:n
        q_plus = copy(q_eff); q_minus = copy(q_eff)
        q_plus[j] += eps; q_minus[j] -= eps
        Ap, Bp, Cp = fk_eval(robot, link_idx, q_plus)
        Am, Bm, Cm = fk_eval(robot, link_idx, q_minus)

        if direction == :xy
            grad[j] = ((Ap^2 + Bp^2) - (Am^2 + Bm^2)) / (2 * eps)
        else
            grad[j] = (Cp - Cm) / (2 * eps)
        end
    end
    return norm(grad)
end


# ═══════════════════════════════════════════════════════════════════════════
#  Main pipeline
# ═══════════════════════════════════════════════════════════════════════════

struct CriticalPoint
    q_eff::Vector{Float64}
    link_id::Int
    direction::Symbol
    p_critical::Float64
    source::Symbol
    grad_norm::Float64
end

function precompute_robot(robot::RobotConfig)
    @info "═══ Precomputing GCPC for $(robot.name) ═══"
    @info "Active links: $(robot.active_links)"
    flush(stdout)

    all_criticals = CriticalPoint[]

    for link_idx in robot.active_links
        @info "──── Link $link_idx ────"
        flush(stdout)

        n_joints = length(robot.dh_params)
        include_tool = (link_idx == n_joints && !isnothing(robot.tool_frame))
        nj = include_tool ? n_joints : min(link_idx + 1, n_joints)
        n_eff = nj - 1

        # Check q₆ skip
        if n_eff >= 6 && !include_tool
            dh6 = robot.dh_params[7]
            if abs(dh6.d) < 1e-10 && abs(dh6.a) < 1e-10
                n_eff = 5
                @info "  q₆ skipped"
            end
        end

        if n_eff <= 0
            continue
        end

        @info "  $n_eff DOF"

        # ── Phase 1: kπ/2 enumeration ────────────────────────────────
        kpi2_configs = enumerate_kpi2(n_eff)
        @info "  kπ/2: $(length(kpi2_configs)) configs"
        flush(stdout)

        for cfg in kpi2_configs
            A, B, C = fk_eval(robot, link_idx, cfg)
            R = sqrt(A^2 + B^2)
            grad_xy = verify_critical(robot, link_idx, cfg, :xy)
            grad_z  = verify_critical(robot, link_idx, cfg, :z)
            push!(all_criticals, CriticalPoint(cfg, link_idx, :xy, R, :kpi2, grad_xy))
            push!(all_criticals, CriticalPoint(cfg, link_idx, :z, C, :kpi2, grad_z))
        end

        # ── Phase 2: Homotopy continuation ────────────────────────────
        fk_result = build_symbolic_fk(robot, link_idx)
        if isnothing(fk_result)
            continue
        end
        A_sym, B_sym, C_sym, n_eff_actual, c_vars, s_vars, active_idx = fk_result

        if isempty(active_idx)
            @info "  No active joints for homotopy — kπ/2 only"
            continue
        end

        # ── R² system ──
        @info "  Building R² system ($(length(active_idx)) active joints)..."
        flush(stdout)
        try
            R2_sys = build_R2_system(A_sym, B_sym, collect(c_vars), collect(s_vars), active_idx)
            if isnothing(R2_sys)
                @info "  R² system empty, skipping"
            else
                @info "  Solving R² system..."
                flush(stdout)
                R2_raw = solve_and_filter(R2_sys; q1_half_range=true, active_idx=active_idx, n_eff=n_eff)

                for sol in R2_raw
                    A, B, C = fk_eval(robot, link_idx, sol)
                    R = sqrt(A^2 + B^2)
                    grad = verify_critical(robot, link_idx, sol, :xy)
                    push!(all_criticals, CriticalPoint(sol, link_idx, :xy, R, :homotopy, grad))
                end
                @info "  R² found: $(length(R2_raw))"
            end
        catch e
            @warn "  R² failed: $e"
            flush(stdout)
        end

        # ── z system ──
        @info "  Building z system ($(length(active_idx)) active joints)..."
        flush(stdout)
        try
            z_sys = build_z_system(C_sym, collect(c_vars), collect(s_vars), active_idx)
            if isnothing(z_sys)
                @info "  z system empty, skipping"
            else
                @info "  Solving z system..."
                flush(stdout)
                z_raw = solve_and_filter(z_sys; q1_half_range=true, active_idx=active_idx, n_eff=n_eff)

                for sol in z_raw
                    A, B, C = fk_eval(robot, link_idx, sol)
                    grad = verify_critical(robot, link_idx, sol, :z)
                    push!(all_criticals, CriticalPoint(sol, link_idx, :z, C, :homotopy, grad))
                end
                @info "  z found: $(length(z_raw))"
            end
        catch e
            @warn "  z failed: $e"
            flush(stdout)
        end
    end

    # ── Dedup ─────────────────────────────────────────────────────────
    @info "Total raw: $(length(all_criticals))"
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

    n_verified = count(cp -> cp.grad_norm < 1e-4, unique_criticals)
    n_approx = count(cp -> cp.grad_norm >= 1e-4 && cp.grad_norm < 1e-2, unique_criticals)
    n_bad = count(cp -> cp.grad_norm >= 1e-2, unique_criticals)
    @info "Verified: $n_verified exact, $n_approx approx, $n_bad suspect"
    flush(stdout)

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
        println("Usage: julia precompute_gcpc_v2.jl <robot_name> [--no-tool]")
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
        println("\n$(name)$(suffix): $(length(criticals)) critical points saved")
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
