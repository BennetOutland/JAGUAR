using DifferentialEquations, LazySets, LinearAlgebra
using Clipper
include("../DynamicPlanning.jl/src/robot.jl")

#===================================================#
#                CLIPPER INTERFACE
#===================================================#

const SCALE = 1_000_000_000  # Higher precision for better accuracy   1_000_000_000 

"""
Convert VPolygon to Clipper integer coordinates
"""
function to_clipper(poly::VPolygon)
    return [IntPoint(round(Int, v[1] * SCALE), round(Int, v[2] * SCALE)) 
            for v in poly.vertices]
end

# """
# Convert Clipper integer coordinates back to VPolygon
# """
# function from_clipper(paths::Vector)
#     polygons = VPolygon[]
#     for path in paths
#         if length(path) >= 3
#             pts = [[p.X / SCALE, p.Y / SCALE] for p in path]
#             push!(polygons, VPolygon(pts))
#         end
#     end
#     return polygons
# end

function from_clipper(paths::Vector)
    polygons = VPolygon[]
    for path in paths
        if length(path) >= 3
            pts = [[p.X / SCALE, p.Y / SCALE] for p in path]
            poly = VPolygon(pts)
            
            # Ensure positive area (outer boundary, not hole)
            if LazySets.area(poly) > 0
                push!(polygons, poly)
            else
                # Reverse if negative area
                push!(polygons, VPolygon(reverse(pts)))
            end
        end
    end
    return polygons
end

"""
Union operation on polygons using Clipper
"""
function polygon_union(polys::Vector{VPolygon})
    isempty(polys) && return VPolygon[]
    
    c = Clip()
    for poly in polys
        add_path!(c, to_clipper(poly), PolyTypeSubject, true)
    end
    
    success, result = execute(c, ClipTypeUnion, PolyFillTypeNonZero, PolyFillTypeNonZero)
    
    return success ? from_clipper(result) : VPolygon[]
end

"""
Difference operation: A \\ B using Clipper
"""
function polygon_difference(A::VPolygon, B::VPolygon)
    c = Clip()
    add_path!(c, to_clipper(A), PolyTypeSubject, true)
    add_path!(c, to_clipper(B), PolyTypeClip, true)
    
    success, result = execute(c, ClipTypeDifference, PolyFillTypeEvenOdd, PolyFillTypeEvenOdd)

    if !success
        @info "  → difference failed"
    end
    
    return success ? from_clipper(result) : VPolygon[]
end

"""
Batch difference: subtract multiple obstacles from a polygon
Returns the remaining fragments after all subtractions
"""
function polygon_difference_batch(poly::VPolygon, obstacles::Vector)
    current = [poly]
    
    for obs in obstacles
        next = VPolygon[]
        for fragment in current
            # Only process if intersection exists
            if !isdisjoint(fragment, obs)
                result = polygon_difference(fragment, obs)
                append!(next, result)
            else
                push!(next, fragment)
            end
        end
        current = next
        isempty(current) && break
    end
    
    return current
end

#===================================================#
#             REACHABLE SET COMPUTATION
#===================================================#

"""
Compute reachable set as collection of convex polytopes via control space sampling

# Arguments
- `x0`: Initial state vector
- `u_bounds`: Control bounds [(u1_min, u1_max), ...]
- `x_bounds`: State bounds [(x1_min, x1_max), ...]
- `T`: Time horizon
- `dynamics!`: ODE right-hand side function
- `n_segments`: Discretization of control space per dimension
- `n_samples`: Samples per segment for convex hull
- `dt`: Integration timestep

# Returns
- `Vector{VPolygon}`: Convex polytopes covering reachable set
"""
function compute_reachable_polytopes(x0::Vector, 
                                     u_bounds::Vector{Tuple{Float64,Float64}},
                                     x_bounds::Vector{Tuple{Float64,Float64}},
                                     T::Float64,
                                     dynamics!;
                                     n_segments::Int=8,
                                     n_samples::Int=15,
                                     dt::Float64=0.01)
    
    n_controls = length(u_bounds)
    n_states = length(x0)
    
    # State constraint callback
    cb = DiscreteCallback(
        (u,t,integrator) -> true,
        integrator -> begin
            for i in 1:length(integrator.u)
                integrator.u[i] = clamp(integrator.u[i], x_bounds[i][1], x_bounds[i][2])
            end
        end,
        save_positions=(false, false)
    )
    
    # Partition control space into segments
    partitions = [range(u_bounds[i][1], u_bounds[i][2], length=n_segments+1) 
                  for i in 1:n_controls]
    
    polytopes = VPolygon[]
    
    # For each segment combination
    for segment_idx in Iterators.product([1:n_segments for _ in 1:n_controls]...)
        points = Vector{Float64}[]
        
        # Sample within this segment
        sample_ranges = [range(partitions[k][segment_idx[k]], 
                              partitions[k][segment_idx[k]+1],
                              length=n_samples)
                        for k in 1:n_controls]
        
        for u_sample in Iterators.product(sample_ranges...)
            # Constant control functions
            ctrl = [t -> u_sample[i] for i in 1:n_controls]
            
            # Solve ODE
            prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
            sol = solve(prob, Tsit5(), callback=cb, saveat=dt)
            
            # Extract 2D position (assumes first 2 states are spatial)
            push!(points, [sol[end][1], sol[end][2]])
        end
        
        # Create convex hull of sampled points
        if length(points) >= 3
            push!(polytopes, VPolygon(points))
        end
    end
    
    return polytopes
end

#===================================================#
#           GEOMETRIC TRANSFORMATIONS
#===================================================#

"""
Rotate polygon by angle θ around center point
"""
function rotate_polygon(poly::VPolygon, θ::Float64, center::Vector{Float64})
    R = [cos(θ) -sin(θ); sin(θ) cos(θ)]
    new_vertices = [R * (v - center) + center for v in poly.vertices]
    return VPolygon(new_vertices)
end

"""
Translate polygon by displacement vector
"""
function translate_polygon(poly::VPolygon, displacement::Vector{Float64})
    new_vertices = [v + displacement for v in poly.vertices]
    return VPolygon(new_vertices)
end

"""
Apply affine transformation to collection of polygons
"""
function transform_polygons(polys::Vector{VPolygon}, θ::Float64, displacement::Vector{Float64})
    center = [0.0, 0.0]  # Rotation around origin, adjust if needed
    return [translate_polygon(rotate_polygon(p, θ, center), displacement) for p in polys]
end

#===================================================#
#         CONFIGURATION SPACE OBSTACLES
#===================================================#

"""
Compute configuration space obstacle via Minkowski sum
"""
function minkowski_sum_obstacle(obstacle::VPolygon, robot_shape::VPolygon)
    # Negate robot shape (reflection through origin)
    negated_vertices = [-v for v in robot_shape.vertices]
    negated_robot = VPolygon(negated_vertices)
    
    # Compute Minkowski sum
    result = MinkowskiSum(obstacle, negated_robot)
    
    # Convert to VPolygon
    return overapproximate(result, VPolygon)
end

#===================================================#
#            OBSTACLE SUBTRACTION
#===================================================#

# """
# Subtract obstacles from reachable set with smart batching

# # Algorithm
# 1. Union all reachable polytopes into unified region(s)
# 2. Process obstacles in batches to balance union cost vs. fragment explosion
# 3. Union fragments after each batch to merge adjacent pieces

# # Arguments
# - `reachable_polytopes`: Collection of convex polytopes from reachability computation
# - `c_space_obstacles`: Configuration space obstacles (after Minkowski sum)
# - `batch_size`: Number of obstacles to process before re-unioning (tune for performance)

# # Returns
# - `Vector{VPolygon}`: Safe reachable region after obstacle subtraction
# """
# function subtract_obstacles(reachable_polytopes::Vector{VPolygon}, 
#                            c_space_obstacles::Vector{VPolygon};
#                            batch_size::Int=10)
    
#     # Step 1: Union all reachable polytopes
#     @info "Unifying $(length(reachable_polytopes)) reachable polytopes..."
#     current_region = polygon_union(reachable_polytopes)
#     @info "  → $(length(current_region)) connected component(s)"
    
#     #isempty(current_region) && return VPolygon[]    # TODO: Silently failing?
    
#     # Step 2: Process obstacles in batches
#     n_obstacles = length(c_space_obstacles)
    
#     for batch_start in 1:batch_size:n_obstacles
#         batch_end = min(batch_start + batch_size - 1, n_obstacles)
#         batch = c_space_obstacles[batch_start:batch_end]
        
#         @info "Processing obstacles $(batch_start):$(batch_end)..."
        
#         # Subtract batch from each component of current region
#         all_fragments = VPolygon[]
        
#         for component in current_region
#             fragments = polygon_difference_batch(component, batch)
#             append!(all_fragments, fragments)
#         end
        
#         # Re-union to merge adjacent fragments
#         if !isempty(all_fragments)
#             current_region = polygon_union(all_fragments)
#             @info "  → $(length(current_region)) component(s) remain"
#         else
#             @warn "Reachable set completely eliminated by obstacles!"
#             return VPolygon[]
#         end
#     end
    
#     return current_region
# end



"""
Subtract obstacles from reachable set (non-batched version)

Algorithm:
1. Union all reachable polytopes into unified region(s)
2. For each obstacle:
       - Subtract it from every connected component
       - Collect resulting fragments
3. Return remaining safe regions

Arguments:
- `reachable_polytopes`: Collection of convex polytopes from reachability computation
- `c_space_obstacles`: Configuration space obstacles (after Minkowski sum)

Returns:
- `Vector{VPolygon}`: Safe reachable region after obstacle subtraction
"""
function subtract_obstacles(
    reachable_polytopes::Vector{VPolygon},
    c_space_obstacles::Vector{VPolygon}
)

    # Step 1: Union all reachable polytopes
    @info "Unifying $(length(reachable_polytopes)) reachable polytopes..."
    current_region = polygon_union(reachable_polytopes)
    @info "  → $(length(current_region)) connected component(s)"

    isempty(current_region) && return VPolygon[]

    # Step 2: Process each obstacle individually
    for (i, obs) in enumerate(c_space_obstacles)
        @info "Subtracting obstacle $i..."

        fragments = VPolygon[]

        for comp in current_region
            result = polygon_difference(comp, obs)
            append!(fragments, result)
        end

        if isempty(fragments)
            @warn "Reachable set completely eliminated after obstacle $i!"
            return VPolygon[]
        end

        # *Optionally* union fragments to merge touching components
        # Comment this out if you want raw fragments.
        current_region = polygon_union(fragments)
        @info "  → $(length(current_region)) component(s) remain"
    end

    return current_region
end






#===================================================#
#                  UTILITIES
#===================================================#

"""
Extract box constraint bounds from robot constraints
"""
function get_bounds(R::Robot, type::Symbol)
    for con in R.constraints
        if con isa BoxConstraint && con.type == type
            return con.lower, con.upper
        end
    end
    error("No BoxConstraint found for type :$type")
end


"""
Convert bounds to tuple format
"""
function bounds_to_tuples(bounds)
    return [(bounds[1][i], bounds[2][i]) for i in eachindex(bounds[1])]
end

"""
Validate that obstacles are fully subtracted (diagnostic)
"""
function validate_subtraction(safe_region::Vector{VPolygon}, obstacles::Vector{VPolygon})
    for (i, obs) in enumerate(obstacles)
        total_intersection = sum(area(intersection(poly, obs)) for poly in safe_region; init=0.0)
        
        if total_intersection > 1e-8
            @warn "Incomplete subtraction detected" obstacle=i intersection_area=total_intersection
        end
    end
end