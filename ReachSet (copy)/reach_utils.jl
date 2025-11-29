using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   
using Interpolations


#===================================================#
#                     LOOPUP
#===================================================#

"""
Get the robot box constraints
"""
function get_bounds(R::Robot, type::Symbol)
    contraints = R.constraints 

    for con in contraints 
        if con isa BoxConstraint && con.type == type
            return con.lower, con.upper
        end
    end
end


#===================================================#
#                   CONVERSION
#===================================================#

# Scale used to convert floating coordinates to integer Clipper coordinates.
const SCALE = 1_000_000  # adjust if you need more/less precision

"""
Convert LazySets.VPolygon -> Clipper Path (Vector{IntPoint})
"""
function to_intpoly(poly::VPolygon; scale::Real = SCALE)
    # poly.vertices is a Vector{Vector{Float64}} or similar: each v is [x,y]
    return [IntPoint(round(Int, v[1] * scale), round(Int, v[2] * scale)) for v in poly.vertices]
end


"""
Convert Clipper output (Vector{Vector{IntPoint}} or Vector{IntPoint}) -> Vector{VPolygon}
"""
function to_vpolygons(clipper_polys; scale=SCALE)
    out = VPolygon[]
    for path in clipper_polys
        # Convert IntPoint → Vector{Float64}
        pts = [[p.X / scale, p.Y / scale] for p in path]

        if length(pts) >= 3
            push!(out, VPolygon(pts))   # this now matches LazySets constructor
        end
    end
    return out
end

"""
Converts the standard bounds format to a vector of tuples
"""
function bounds_to_tuples(bounds)
    tuple_vec = []
    for i ∈ eachindex(bounds[1])
        push!(tuple_vec, (bounds[1][i], bounds[2][i]))
    end

    return tuple_vec
end


#===================================================#
#             DIFFERENCE OPERATORS
#===================================================#

"""
Boolean set difference using Clipper.jl:
    A \\ B
Returns possibly multiple VPolygons.
"""
function clip_difference(A::VPolygon, B::VPolygon)
    pathA = to_intpoly(A)
    pathB = to_intpoly(B)

    c = Clip()

    # Add subject and clip paths (closed polygons)
    add_path!(c, pathA, PolyTypeSubject, true)
    add_path!(c, pathB, PolyTypeClip, true)

    # Execute difference
    #result, polys = execute(c, ClipTypeDifference, PolyFillTypeEvenOdd, PolyFillTypeEvenOdd)
    result, polys = execute(c, ClipTypeDifference, PolyFillTypeNonZero, PolyFillTypeNonZero)

    if !result
        return VPolygon[]  # empty result
    end

    return to_vpolygons(polys)
end



"""
Compute union of convex polytopes using Clipper
"""
function clipper_union(polys::Vector{VPolygon})
    isempty(polys) && return VPolygon[]
    
    int_paths = [to_intpoly(p) for p in polys]
    
    c = Clip()
    for path in int_paths
        add_path!(c, path, PolyTypeSubject, true)
    end
    
    result, union_polys = execute(c, ClipTypeUnion, 
                                   PolyFillTypeNonZero, 
                                   PolyFillTypeNonZero)
    
    if !result || isempty(union_polys)
        return VPolygon[]
    end
    
    return to_vpolygons(union_polys)
end


#===================================================#
#                    REACH SET
#===================================================#

"""
Compute reachable set as union of convex polytopes

# Arguments
- `x0::AbstractVector`: n-dimensional initial state
- `u_bounds::Vector{Tuple{Float64,Float64}}`: control bounds [(u1_min,u1_max), (u2_min,u2_max), ...]
- `x_bounds::Vector{Tuple{Float64,Float64}}`: state bounds [(x1_min,x1_max), (x2_min,x2_max), ...]
- `T::Float64`: time horizon
- `dynamics!`: ODE function with signature f!(dx, x, p, t)
- `n_segments::Int`: number of segments per control dimension
- `n_samples_per_segment::Int`: samples per segment
- `dt::Float64`: integration step size

# Returns
- `Vector{VPolygon}`: collection of convex polytopes representing reachable set
"""
function compute_reachable_union_of_convex(x0::AbstractVector, 
                                           u_bounds::Vector,
                                           x_bounds::Vector,
                                           T, dynamics!;
                                           n_segments=8, 
                                           n_samples_per_segment=15,
                                           dt=0.01)
    n_controls = length(u_bounds)
    n_states = length(x0)
    
    # Validate dimensions
    @assert length(x_bounds) == n_states "x_bounds must match state dimension"
    
    # Create partitions for each control dimension
    partitions = [range(u_bounds[i][1], u_bounds[i][2], length=n_segments+1) 
                  for i in 1:n_controls]
    
    # State constraint callback
    function affect!(integrator)
        for i in 1:length(integrator.u)
            integrator.u[i] = clamp(integrator.u[i], x_bounds[i][1], x_bounds[i][2])
        end
    end
    
    cb = DiscreteCallback((u,t,integrator) -> true, affect!, 
                          save_positions=(false,false))
    
    convex_sets = VPolygon[]
    
    # Generate all segment combinations using Cartesian product
    segment_ranges = [1:n_segments for _ in 1:n_controls]
    
    for segment_indices in Iterators.product(segment_ranges...)
        segment_points = Vector{Float64}[]
        
        # Create sampling ranges for this segment
        sample_ranges = [range(partitions[k][segment_indices[k]], 
                              partitions[k][segment_indices[k]+1],
                              length=n_samples_per_segment)
                        for k in 1:n_controls]
        
        # Sample control space 
        for u_sample in Iterators.product(sample_ranges...)
            # Create constant control functions
            ctrl = [t -> u_sample[i] for i in 1:n_controls]

            # Define and solve the problem
            prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
            sol = solve(prob, Tsit5(), callback=cb, saveat=dt)
            
            final_state = sol[end]

            # Extract position coordinates (assuming first two dimensions are spatial)
            push!(segment_points, [final_state[1], final_state[2]])
        end
        
        if length(segment_points) >= 3
            poly = VPolygon(segment_points)
            push!(convex_sets, poly)
        end
    end
    
    return convex_sets
end



#===================================================#
#                OBSTACLE REMOVAL
#===================================================#


"""
Enhanced difference computation with robustness checks
"""
function clip_difference_spatial(poly::VPolygon, 
                                obstacles::Vector,
                                obstacle_boxes::Vector)
    poly_box = bounding_box(poly)
    
    # Bounding box test
    candidate_indices = [i for i in 1:length(obstacles) 
                        if boxes_intersect(poly_box, obstacle_boxes[i])]
    
    isempty(candidate_indices) && return [poly]
    
    current_fragments = [poly]
    
    for idx in candidate_indices
        next_fragments = VPolygon[]
        
        for fragment in current_fragments
            # Replace isdisjoint with explicit intersection check
            intersection_area = LazySets.area(intersection(fragment, obstacles[idx]))
            
            if intersection_area > 1e-12  # Numerical tolerance
                result = clip_difference(fragment, obstacles[idx])
                
                if result isa Vector
                    for r in result
                        if !isempty(vertices_list(r)) && LazySets.area(r) > 1e-12
                            push!(next_fragments, r)
                        end
                    end
                else
                    # Should never happen with current implementation
                    @error "Unexpected return type from clip_difference" typeof(result)
                    push!(next_fragments, fragment)  # Preserve fragment
                end
            else
                # Truly disjoint - preserve fragment
                push!(next_fragments, fragment)
            end
        end
        
        current_fragments = next_fragments
        isempty(current_fragments) && break
    end
    
    return current_fragments
end

# function clip_difference_spatial(poly::VPolygon, 
#                                 obstacles::Vector,
#                                 obstacle_boxes::Vector)
#     # Get bounding box of input polygon
#     poly_box = bounding_box(poly)
    
#     # Filter to potentially intersecting obstacles using bounding box test
#     candidate_indices = [i for i in 1:length(obstacles) 
#                         if boxes_intersect(poly_box, obstacle_boxes[i])]
    
#     # Early return if no potential intersections
#     isempty(candidate_indices) && return [poly]
    
#     # Apply successive differences only with candidate obstacles
#     current_fragments = [poly]
    
#     for idx in candidate_indices
#         next_fragments = VPolygon[]
        
#         for fragment in current_fragments
#             # Check for actual intersection before clipping
#             if !isdisjoint(fragment, obstacles[idx])
#                 result = clip_difference(fragment, obstacles[idx])
                
#                 # Handle both single polygon and vector returns
#                 if result isa VPolygon
#                     # Additional check: ensure result is non-empty
#                     if !isempty(vertices_list(result))
#                         push!(next_fragments, result)
#                     end
#                 elseif result isa Vector
#                     for r in result
#                         if !isempty(vertices_list(r))
#                             push!(next_fragments, r)
#                         end
#                     end
#                 end
#             else
#                 # No intersection - keep fragment unchanged
#                 push!(next_fragments, fragment)
#             end
#         end
        
#         current_fragments = next_fragments
#         isempty(current_fragments) && break
#     end
    
#     return current_fragments
# end

"""
Fast axis-aligned bounding box intersection test
"""
function boxes_intersect(box1::Hyperrectangle, box2::Hyperrectangle)
    c1, r1 = LazySets.center(box1), radius_hyperrectangle(box1)
    c2, r2 = LazySets.center(box2), radius_hyperrectangle(box2)
    
    return abs(c1[1] - c2[1]) <= (r1[1] + r2[1]) &&
           abs(c1[2] - c2[2]) <= (r1[2] + r2[2])
end

"""
Compute axis-aligned bounding box for polygon
"""
function bounding_box(poly::VPolygon)
    verts = poly.vertices
    x_coords = [v[1] for v in verts]
    y_coords = [v[2] for v in verts]
    
    x_min, x_max = extrema(x_coords)
    y_min, y_max = extrema(y_coords)
    
    center_pt = [(x_min + x_max)/2, (y_min + y_max)/2]
    radii = [(x_max - x_min)/2, (y_max - y_min)/2]
    
    return Hyperrectangle(center_pt, radii)
end