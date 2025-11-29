using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   

include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../DynamicPlanning.jl/src/models.jl")
include("reachability.jl")


x0 = [-3.0, -2.5, 0.0]
T = 10.0


# Define the workspace
𝒲 = workspace([0.0, 0.0], 10.0)

# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)
add_obstacles!(𝒲, 𝒪)

# Define a robot
l = 0.2
w = 0.1
box = VPolygon([[-w/2, -l/2], [-w/2, l/2], [w/2, l/2], [w/2, -l/2]])
R = robot(x0, 3, 2, box)
add_dynamics!(R, unicycle!)
add_box_constraint!(R, [-0.25, -pi/3], [0.25, pi/3], :u)
add_box_constraint!(R, [-10, -10, -pi/3], [10, 10, pi/3], :x)


# 1. Compute offline reachable set
reach_set = compute_reachable_set(R, T; n_segments=10, n_samples=20)

# 2. Precompute configuration space obstacles
c_obstacles = precompute_c_space_obstacles(R, 𝒲)

# 3. At runtime: transform reachable set to current pose
current_reach = transform(reach_set, 0.0, [0.0, 0.0])

# 4. Subtract obstacles to get safe region
safe_reach = compute_safe_reachable_set(current_reach, c_obstacles; batch_size=5)

# Plot
p1 = plot()

# for poly in c_obstacles
#     plot!(p1, poly, alpha=0.3, linecolor=:black, color=:grey)
# end

for poly in reach_set.polytopes
    plot!(p1, poly, alpha=0.2, linecolor=:black, color=:yellow)
end

for poly in safe_reach.polytopes
    plot!(p1, poly, alpha=0.3, linecolor=:black, color=:red)
end




display(p1)

