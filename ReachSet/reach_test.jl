using DifferentialEquations, Plots, LazySets, LinearAlgebra 
using LaTeXStrings

include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../DynamicPlanning.jl/src/models.jl")
include("reachability.jl")
include("sampling.jl")

# Define initial conditions
x0 = [-3.0, -2.5, 0.0]
T = 20.0

# Define the workspace
𝒲 = workspace([0.0, 0.0], 10.0)

# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)
# add_obstacles!(𝒲, [𝒪[12], 𝒪[14]])

add_obstacles!(𝒲, 𝒪)

# Define a robot
l = 0.5 # 0.2
w = 0.25 # 0.1
box = VPolygon([[-w/2, -l/2], [-w/2, l/2], [w/2, l/2], [w/2, -l/2]])
R = robot(x0, 3, 2, box)
add_dynamics!(R, unicycle!)
add_box_constraint!(R, [-0.25, -pi/3], [0.25, pi/3], :u)
add_box_constraint!(R, [-10, -10, -pi/3], [10, 10, pi/3], :x)


# 1. Compute offline reachable set
# reach_set = compute_reachable_set(R, T; n_segments=50, n_samples=10)
reach_set = compute_reachable_set(R, T; n_segments=40, n_samples=5)

# 2. Precompute configuration space obstacles
c_obstacles = precompute_c_space_obstacles(R, 𝒲)

# 3. At runtime: transform reachable set to current pose
current_reach = transform(reach_set, 0.0, [0.0, 0.0])

# 4. Subtract obstacles to get safe region
safe_reach = compute_safe_reachable_set(current_reach, c_obstacles, min_area=1e-10, merge_adjacent=true)

# Summary
summarize(safe_reach)


# Define objective and gradient
function objective(x)
    # Example: max distance from target point
    target = [-5.0, -5.0]
    return -0.5 * sum((x[i] - target[i])^2 for i in 1:2)
end

# 5. Determine goal location
x_opt = multi_start_optimization(
    objective,
    safe_reach.polytopes;
    n_starts=10,
    strategy=:uniform,
    α=0.01
)

p1 = plot(aspect_ratio=:equal)

# Workspace 
plot!(p1, 𝒲.bounds, c=:white)

# Plot obstacles
for (i, poly) in enumerate(𝒲.obstacles)
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
                   resolution=300, 
                   label="Reachable Set")      # Adjust quality

scatter!(p1, [x0[1]], [x0[2]], label="Initial State", markershape = :star5, markersize = 7, color=:black)

scatter!(p1, [x_opt[1][1]], [x_opt[1][2]], label="Goal State", markershape = :star5, markersize = 7, color=:blue)

box = box_approximation(𝒲.bounds)
l = low(box)
h = high(box)

plot!(p1, xlims=[l[1], h[1]], ylims=[l[2], h[2]])

xlabel!(p1, L"x")
ylabel!(p1, L"y")

display(p1)




