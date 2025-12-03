using DifferentialEquations, Plots, LazySets, LinearAlgebra 

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
# add_obstacles!(𝒲, [𝒪[12], 𝒪[15]])

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
# reach_set = compute_reachable_set(R, T; n_segments=10, n_samples=20)
reach_set = compute_reachable_set(R, T; n_segments=10, n_samples=20)

# 2. Precompute configuration space obstacles
c_obstacles = precompute_c_space_obstacles(R, 𝒲)

# 3. At runtime: transform reachable set to current pose
current_reach = transform(reach_set, 0.0, [0.0, 0.0])

# 4. Subtract obstacles to get safe region
safe_reach = compute_safe_reachable_set(current_reach, c_obstacles, min_area=1e-6)
#safe_reach = compute_safe_reachable_set(reach_set, c_obstacles)


# Summary
summarize(safe_reach)


p1 = plot(aspect_ratio=:equal)

# Plot obstacles
for (i, poly) in enumerate(c_obstacles)
    plot!(p1, poly, 
          fillalpha=0.3, 
          fillcolor=:grey,
          linecolor=:black,
          label=(i == 1 ? "Obstacles" : ""))
end

# Plot safe reachable set with unified color
plot_reachable_set!(p1, safe_reach, 
                   color=:red, 
                   alpha=0.5,
                   use_raster=true,     # Pixel-perfect
                   resolution=300)      # Adjust quality

display(p1)



