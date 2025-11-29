using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   

include("reach_utils.jl")
include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")

"""
Represents a precomputed reachable set that can be transformed and intersected with obstacles

# Fields
- `polytopes`: Collection of convex polygons representing the reachable set
- `is_unified`: Whether polytopes have been unioned (for efficiency)
"""
struct ReachableSet
    polytopes::Vector{VPolygon}
    is_unified::Bool
end

"""
Compute reachable set for robot over time horizon T

# Arguments
- `robot`: Robot object with dynamics, initial state, constraints, and shape
- `T`: Time horizon
- `n_segments`: Control space discretization (higher = more accurate, slower)
- `n_samples`: Samples per segment (higher = smoother polytopes)

# Returns
- `ReachableSet`: Precomputed reachable set
"""
function compute_reachable_set(robot, T::Float64; n_segments::Int=8, n_samples::Int=15)
    # Extract bounds
    x_bounds = bounds_to_tuples(get_bounds(robot, :x))
    u_bounds = bounds_to_tuples(get_bounds(robot, :u))
    
    # Compute polytopes
    @info "Computing reachable set (T=$T, segments=$n_segments, samples=$n_samples)..."
    polytopes = compute_reachable_polytopes(
        robot.x0, u_bounds, x_bounds, T, robot.dynamics;
        n_segments=n_segments, n_samples=n_samples
    )
    @info "  → Generated $(length(polytopes)) convex polytopes"
    
    return ReachableSet(polytopes, false)
end

"""
Transform reachable set by rotation and translation

# Arguments
- `reach_set`: ReachableSet to transform
- `θ`: Rotation angle in radians (around origin)
- `displacement`: Translation vector [dx, dy]

# Returns
- `ReachableSet`: Transformed reachable set
"""
function transform(reach_set::ReachableSet, θ::Float64, displacement::Vector{Float64})
    transformed = transform_polygons(reach_set.polytopes, θ, displacement)
    return ReachableSet(transformed, reach_set.is_unified)
end

"""
Precompute configuration space obstacles from workspace obstacles

# Arguments
- `robot`: Robot with shape geometry
- `workspace`: Workspace with obstacles

# Returns
- `Vector{VPolygon}`: Configuration space obstacles (Minkowski sums)
"""
function precompute_c_space_obstacles(robot, workspace)
    @info "Computing configuration space obstacles..."
    
    # Extract robot shape as polygon
    robot_poly = overapproximate(robot.shape, VPolygon)
    
    c_obstacles = VPolygon[]
    for obs in workspace.obstacles
        obs_poly = overapproximate(obs, VPolygon)
        c_obs = minkowski_sum_obstacle(obs_poly, robot_poly)
        push!(c_obstacles, c_obs)
    end
    
    @info "  → Processed $(length(c_obstacles)) obstacles"
    return c_obstacles
end

"""
Compute safe (obstacle-free) reachable set

# Arguments
- `reach_set`: ReachableSet to filter
- `c_space_obstacles`: Configuration space obstacles
- `batch_size`: Tuning parameter for union frequency (default: 10)

# Returns
- `ReachableSet`: Safe reachable set with obstacles removed
"""
function compute_safe_reachable_set(reach_set::ReachableSet, 
                                   c_space_obstacles::Vector{VPolygon};
                                   batch_size::Int=10)
    
    # safe_polytopes = subtract_obstacles(reach_set.polytopes, c_space_obstacles; 
    #                                    batch_size=batch_size)
    safe_polytopes = subtract_obstacles(reach_set.polytopes, c_space_obstacles)
    
    return ReachableSet(safe_polytopes, true)  # Mark as unified after subtraction
end

"""
Check if a point is in the reachable set

# Arguments
- `reach_set`: ReachableSet to query
- `point`: Query point [x, y]

# Returns
- `Bool`: True if point is reachable
"""
function contains(reach_set::ReachableSet, point::Vector{Float64})
    return any(point ∈ poly for poly in reach_set.polytopes)
end

"""
Get total area of reachable set
"""
function area(reach_set::ReachableSet)
    if reach_set.is_unified
        # Polytopes are disjoint after union
        return sum(LazySets.area, reach_set.polytopes)
    else
        # May have overlaps - union first for accurate area
        unified = polygon_union(reach_set.polytopes)
        return sum(LazySets.area, unified)
    end
end


