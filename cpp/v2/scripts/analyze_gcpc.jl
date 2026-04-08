using JSON

path = joinpath(@__DIR__, "..", "results", "gcpc", "panda_gcpc.json")
d = JSON.parsefile(path)
println("n_points: ", d["n_points"])
println("robot: ", d["robot"])
println("active_links: ", d["active_links"])

pts = d["points"]
src_counts = Dict{String,Int}()
dir_counts = Dict{String,Int}()
link_counts = Dict{Int,Int}()
for p in pts
    k = p["source"]
    src_counts[k] = get(src_counts, k, 0) + 1
    k2 = p["direction"]
    dir_counts[k2] = get(dir_counts, k2, 0) + 1
    k3 = p["link_id"]
    link_counts[k3] = get(link_counts, k3, 0) + 1
end
println("By source: ", src_counts)
println("By direction: ", dir_counts)
println("By link: ", link_counts)

verified = count(p -> p["grad_norm"] < 1e-4, pts)
approx = count(p -> 1e-4 <= p["grad_norm"] < 1e-2, pts)
bad = count(p -> p["grad_norm"] >= 1e-2, pts)
println("Verified (grad<1e-4): ", verified)
println("Approx (1e-4..1e-2): ", approx)
println("Suspect (grad>=1e-2): ", bad)

# Show homotopy-only points
hc_pts = filter(p -> p["source"] == "homotopy", pts)
println("\nHomotopy solutions: $(length(hc_pts))")
for p in hc_pts
    println("  link=$(p["link_id"]) dir=$(p["direction"]) q=$(round.(p["q_eff"], digits=4)) val=$(round(p["p_critical"], digits=6)) grad=$(p["grad_norm"])")
end
