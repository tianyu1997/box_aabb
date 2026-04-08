using HomotopyContinuation

# Test: solve a simple system equivalent to Link 2 R²
# R² = 0.316² · s₁². ∂R²/∂θ₁ = 2·0.316²·s₁·c₁ = 0
# Plus circle: c₁² + s₁² = 1

@var c₁ s₁

eq1 = 2 * 0.316^2 * s₁ * c₁   # gradient
eq2 = c₁^2 + s₁^2 - 1          # circle

println("=== System ===")
sys = System([eq1, eq2]; variables=[c₁, s₁])
println("Variables: ", variables(sys))
println("Degrees: ", degrees(sys))
println()

println("=== Solving with total_degree ===")
result_td = solve(sys; start_system=:total_degree, show_progress=false)
println("Result: ", result_td)
println("Real solutions: ", real_solutions(result_td; tol=1e-6))
println()

println("=== Solving with polyhedral ===")
try
    result_poly = solve(sys; start_system=:polyhedral, show_progress=false)
    println("Result: ", result_poly)
    println("Real solutions: ", real_solutions(result_poly; tol=1e-6))
catch e
    println("Error: ", e)
end
println()

println("=== Solving with default ===")
result_def = solve(sys; show_progress=false)
println("Result: ", result_def)
println("Real solutions: ", real_solutions(result_def; tol=1e-6))
