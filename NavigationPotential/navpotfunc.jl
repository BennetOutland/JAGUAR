"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../NavigationPotential/navigation_potential.jl")

# Usings 
using LaTeXStrings
using Plots


# Define the workspace
# 𝒲 = workspace(-2, 7, -2, 7)
𝒲 = workspace([0.0, 0.0], 10.0)


# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)

add_obstacles!(𝒲, 𝒪)

d(a, b) = norm(a - b)
NavigationPotentialFunction(𝒲, [0.0, -5.0], [0.0, 5.0], d)




plot_navigation_potential(𝒲, [0.0, 5.0, 0.0], se2_distance_approx, k=3, use_star_space=false)

xt = [0.0, -5.0, 0.0]
NavigationPotentialGradient(𝒲, xt, [0.0, 5.0, 0.0], se2_distance_approx, k=3)

# xt = [0.0, -5.0, 0.0]
# for i in 1:60000
#     xt = xt - 10.0 * NavigationPotentialGradient(𝒲, xt, [0.0, 5.0, 0.0], se2_distance_approx)
# end
# println(xt)


function debug_phi(W, x, goal)
    println("---- DEBUG ψ ----")
    println("at x = ", x)

    # raw ψ
    println("ψ(x): ", psi(x, goal, W, se2_distance_approx, 20))
    println("ψ(x+ε): ", psi(x .+ [1e-6,0,0], goal, W, se2_distance_approx, 20))
    println("ψ(x-ε): ", psi(x .- [1e-6,0,0], goal, W, se2_distance_approx, 20))

    # b_i values
    for (i, obs) in enumerate(W.obstacles)
        println("b_$i(x) = ", b_i(x, obs, se2_distance_approx))
    end
end

debug_phi(𝒲, xt, [0.0,5.0,0.0])


# diagnose_potential(𝒲, [-5.0, 0.0, 0.0], [0.0, 5.0, 0.0], se2_distance_approx)



# plot(𝒲)
# xlabel!(L"x")
# ylabel!(L"y")