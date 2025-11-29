using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   

include("reach_utils.jl")
include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")

"""
TODO
"""
function reachable_set(R::Robot, T::Float64)
    # Extract Initial Conditions and Control Bounds 
    x_bounds = bounds_to_tuples(get_bounds(R, :x))
    u_bounds = bounds_to_tuples(get_bounds(R, :u))

    # Compute Base Reachable Set
    reach_set = compute_reachable_union_of_convex(R.x0, u_bounds, x_bounds, T, R.dynamics)

    return reach_set
end


"""
Rotate collection of convex polytopes by angle θ around point x

# Arguments
- `S::Vector{VPolygon}`: collection of convex polytopes
- `θ::Float64`: rotation angle in radians (counterclockwise positive)
- `x::Vector{Float64}`: center of rotation [x, y]

# Returns
- `Vector{VPolygon}`: rotated polytopes
"""
function rotate_reachable_sets(S::Vector, θ::Float64, x::Vector)
    # Rotation matrix
    R = [cos(θ) -sin(θ);
         sin(θ)  cos(θ)]
    
    rotated_sets = VPolygon[]
    
    for poly in S
        # Extract vertices
        vertices = poly.vertices
        
        # Translate to origin, rotate, translate back
        rotated_vertices = [R * (v - x) + x for v in vertices]
        
        # Construct new polytope
        push!(rotated_sets, VPolygon(rotated_vertices))
    end
    
    return rotated_sets
end

"""
Precompute the Minkowski difference with obstacles
"""
function precompute_obstacles(R::Robot, W::Workspace)
    # Apply the Minkowski sum to the obstacles 
    c_obstacles = []
    for obstacle ∈ W.obstacles 
        # Negate robot shape (reflect through origin)
        negated_robot = LinearMap(-I, R.shape)
        
        # Convert to VPolygon for vertex-based computation
        robot_poly = overapproximate(negated_robot, VPolygon)
        
        # Compute Minkowski sum (exact for polygons)
        result = MinkowskiSum(obstacle, robot_poly)

        push!(c_obstacles, overapproximate(result, VPolygon))
    end

    return c_obstacles
end


"""
Optimized reachable set difference computation using spatial indexing.
Assumes obstacles are non-overlapping or minimally overlapping.

# Arguments
- `convex_sets::Vector{VPolygon}`: collection of reachable set polytopes
- `obstacles::Vector{VPolygon}`: non-overlapping configuration space obstacles

# Returns
- `Vector{Vector{VPolygon}}`: nested array of polygon fragments after obstacle subtraction
"""
function subtract_obstacles(convex_sets::Vector, obstacles::Vector; batch_size=5)
    current_safe_region = clipper_union(convex_sets)
    
    # Process obstacles in batches
    for batch_start in 1:batch_size:length(obstacles)
        batch_end = min(batch_start + batch_size - 1, length(obstacles))
        batch = obstacles[batch_start:batch_end]
        
        all_fragments = VPolygon[]
        
        for poly in current_safe_region
            # Subtract all obstacles in batch from this polytope
            result_poly = [poly]
            
            for obs in batch
                next_result = VPolygon[]
                for fragment in result_poly
                    if !isdisjoint(fragment, obs)
                        diff_result = clip_difference(fragment, obs)
                        if diff_result isa Vector
                            append!(next_result, filter(r -> LazySets.area(r) > 1e-12, diff_result))
                        end
                    else
                        push!(next_result, fragment)
                    end
                end
                result_poly = next_result
            end
            
            append!(all_fragments, result_poly)
        end
        
        # Union after each batch
        current_safe_region = clipper_union(all_fragments)
        @info "After batch $(batch_start):$(batch_end), $(length(current_safe_region)) components remain"
    end
    
    return current_safe_region
end




# function subtract_obstacles(convex_sets::Vector, obstacles::Vector)
#     # First unify all reachable polytopes
#     @info "Computing union of $(length(convex_sets)) polytopes..."
#     unified = clipper_union(convex_sets)
#     @info "Union produced $(length(unified)) connected components"
    
#     # Now subtract obstacles from unified set
#     obstacle_boxes = [bounding_box(obs) for obs in obstacles]
#     result = VPolygon[]
    
#     for poly in unified
#         fragments = clip_difference_spatial(poly, obstacles, obstacle_boxes)
#         append!(result, fragments)
#     end
    
#     return result
# end






# function subtract_obstacles(convex_sets::Vector, obstacles::Vector)
#     # Precompute bounding boxes for all obstacles
#     obstacle_boxes = [bounding_box(obs) for obs in obstacles]
    
#     # Process each reachable set and flatten results
#     result = VPolygon[]
#     for poly in convex_sets
#         fragments = clip_difference_spatial(poly, obstacles, obstacle_boxes)
#         append!(result, fragments)
#     end
    
#     return result
# end

