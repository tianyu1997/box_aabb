#!/usr/bin/env julia
# Wrapper to run precompute_gcpc.jl with proper error handling
println("=== GCPC Wrapper starting ===")
println("Julia version: $(VERSION)")
println("ARGS: $ARGS")
println("Working dir: $(pwd())")
flush(stdout)

try
    println("Loading packages...")
    flush(stdout)
    
    using HomotopyContinuation
    using JSON
    using LinearAlgebra
    
    println("Packages loaded successfully")
    flush(stdout)
    
    # Include the v2 script (c/s parameterization, much lower degree)
    script_path = joinpath(@__DIR__, "precompute_gcpc_v2.jl")
    println("Including: $script_path")
    flush(stdout)
    
    include(script_path)
    
    println("Script included, calling main logic...")
    flush(stdout)
    
    # Run the main logic directly
    robot_name = length(ARGS) >= 1 ? ARGS[1] : "panda"
    no_tool = "--no-tool" in ARGS
    
    outdir = joinpath(@__DIR__, "..", "results", "gcpc")
    mkpath(outdir)
    
    robot = load_robot(robot_name; no_tool=no_tool)
    println("Robot loaded: $(robot.name), active links: $(robot.active_links)")
    flush(stdout)
    
    criticals = precompute_robot(robot)
    
    suffix = no_tool ? "_notool" : ""
    outpath = joinpath(outdir, "$(robot_name)$(suffix)_gcpc.json")
    save_gcpc_json(robot, criticals, outpath)
    
    println("=== DONE ===")
catch e
    println("ERROR: $e")
    Base.showerror(stdout, e, catch_backtrace())
    println()
end
flush(stdout)
