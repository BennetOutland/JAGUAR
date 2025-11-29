using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   

include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../DynamicPlanning.jl/src/models.jl")
include("reachability.jl")


x0 = [-3.0, -2.5, 0.0]
T = 7.0


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


# Compute the reachable set
reach_set = reachable_set(R, T)

# Rotate
reach_set = rotate_reachable_sets(reach_set, π/4, x0[1:2])

# Precompute C-space-like obstacles
new_obstacles = precompute_obstacles(R, 𝒲)


# Remove the obstacles
constrained_reach_set = subtract_obstacles(reach_set, new_obstacles)


# Plot
p1 = plot()

for poly in new_obstacles
    plot!(p1, poly, alpha=0.3, linecolor=:black, color=:grey)
end

for poly in constrained_reach_set
    plot!(p1, poly, alpha=0.3, linecolor=:black, color=:red)
end


display(p1)


# function diagnose_boundary_intersections(convex_sets::Vector, obstacles::Vector)
#     obstacle_boxes = [bounding_box(obs) for obs in obstacles]
    
#     for obs_idx in 1:length(obstacles)
#         obs = obstacles[obs_idx]
        
#         # Find all polytopes that intersect this obstacle
#         intersecting_polys = [i for i in 1:length(convex_sets)
#                              if !isdisjoint(convex_sets[i], obs)]
        
#         if length(intersecting_polys) > 1
#             @info "Obstacle $obs_idx intersects $(length(intersecting_polys)) polytopes"
            
#             # Check for shared boundary regions
#             for i in 1:length(intersecting_polys)-1
#                 for j in i+1:length(intersecting_polys)
#                     p1 = convex_sets[intersecting_polys[i]]
#                     p2 = convex_sets[intersecting_polys[j]]
                    
#                     boundary_intersection = intersection(p1, p2)
#                     if !isempty(boundary_intersection)
#                         obs_on_boundary = intersection(boundary_intersection, obs)
#                         if !isempty(obs_on_boundary)
#                             @warn "Obstacle intersects polytope boundary" obs_idx i j area=LazySets.area(obs_on_boundary)
#                         end
#                     end
#                 end
#             end
#         end
#     end
# end


# diagnose_boundary_intersections(reach_set, new_obstacles)












# convex_sets = compute_reachable_union_of_convex(
#     x0, y0, θ0, v_bounds, ω_bounds, T,
#     n_segments=6, n_samples_per_segment=10
# )

# # Example obstacle polygon
# P = VPolygon([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])

# # Compute differences using Clipper
# reach_set = ReachableDifference(convex_sets, P)

# # Flatten for plotting (may be empty)
# flat = vcat(reach_set...)   # if reach_set contains empty arrays this will still work

# p1 = plot()
# for poly in flat
#     plot!(p1, poly, alpha=0.3, linecolor=:black, color=:red)
# end

# display(p1)


# plot!(P, alpha=0.6, linecolor=:red)
