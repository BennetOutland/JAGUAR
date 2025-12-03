using DifferentialEquations, Plots, LazySets, LinearAlgebra 

include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../DynamicPlanning.jl/src/models.jl")
include("reachability.jl")


# # Test single trajectory from origin
# x0_test = [0.0, 0.0, 0.0]
# u_test = [0.1, 0.0]  # Small forward velocity, no turn
# ctrl_test = [t -> u_test[i] for i in 1:2]

# prob = ODEProblem(unicycle!, x0_test, (0.0, 10.0), ctrl_test)
# sol = solve(prob, Tsit5(), saveat=0.1)

# println("Trajectory from origin with u=[0.1, 0.0]:")
# println("t=0: ", sol[1])
# println("t=5: ", sol(5.0))
# println("t=10: ", sol[end])
# println("Expected final position: [1.0, 0.0, 0.0]")

# # Plot trajectory
# plot(sol, idxs=(1,2), label="Trajectory", aspect_ratio=:equal)
# scatter!([0.0], [0.0], label="Start", markersize=8)


# Test a simple known case
test_outer = VPolygon([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
test_inner = VPolygon([[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]])

println("Test case: outer square minus inner square")
println("Outer area: ", LazySets.area(test_outer))
println("Inner area: ", LazySets.area(test_inner))

# result = polygon_difference_geos(test_outer, test_inner)
# println("Result polygons: ", length(result))
# if !isempty(result)
#     println("Result area: ", sum(LazySets.area, result))
#     println("Expected area: ", LazySets.area(test_outer) - LazySets.area(test_inner), " ≈ 3.0")
# end

# # Plot to visualize
# plot(test_outer, alpha=0.3, label="Outer", fillcolor=:blue)
# plot!(test_inner, alpha=0.3, label="Inner", fillcolor=:red)
# if !isempty(result)
#     plot!(result[1], alpha=0.5, label="Difference", fillcolor=:green)
# end

# result = polygon_difference_geos(test_outer, test_inner)
# println("Fixed result polygons: ", length(result))
# println("Fixed result area: ", sum(LazySets.area, result))

# # Visualize
# plot(test_outer, alpha=0.3, label="Outer", fillcolor=:blue)
# plot!(test_inner, alpha=0.3, label="Inner", fillcolor=:red)
# for (i, poly) in enumerate(result)
#     plot!(poly, alpha=0.5, label="Difference part $i", fillcolor=:green)
# end


# result = polygon_difference_geos(test_outer, test_inner)
# println("Result polygons: ", length(result))
# println("Result total area: ", sum(LazySets.area, result))
# println("Expected area: 3.0")

# # Visualize
# plot(test_outer, alpha=0.3, label="Outer", fillcolor=:blue)
# plot!(test_inner, alpha=0.3, label="Inner", fillcolor=:red)
# for (i, poly) in enumerate(result)
#     plot!(poly, alpha=0.5, label="Difference part $i", fillcolor=:green, linewidth=2)
# end



result = polygon_difference_geos(test_outer, test_inner)
println("Result polygons: ", length(result))
println("Result total area: ", sum(LazySets.area, result))
println("Expected area: 3.0")