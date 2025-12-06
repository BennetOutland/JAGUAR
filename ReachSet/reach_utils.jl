using DifferentialEquations, LazySets, LinearAlgebra
using LibGEOS
using GeoInterface
include("../DynamicPlanning.jl/src/robot.jl")



#===================================================#
#                 BASE STRUCT
#===================================================#


"""
Represents a precomputed reachable set that can be transformed and intersected with obstacles

# Fields
- `polytopes`: Collection of convex polygons representing the reachable set
- `is_unified`: Whether polytopes have been unioned (convexifying - use with caution!)
- `is_safe`: Whether obstacles have been subtracted (nonconvex representation preserved)

# Notes
- When `is_safe=true`, polytopes may represent a nonconvex set (e.g., with holes)
- The polytopes are kept as separate convex pieces to preserve nonconvexity
- Use `to_union_set()` to convert to LazySets.UnionSetArray when needed
"""
struct ReachableSet
    polytopes::Vector{VPolygon}
    is_unified::Bool
    is_safe::Bool  # NEW: tracks if obstacles have been subtracted
end

# Constructor for initial reachable set (not yet safe)
ReachableSet(polytopes::Vector{VPolygon}, is_unified::Bool) = 
    ReachableSet(polytopes, is_unified, false)

# Empty constructor
ReachableSet() = ReachableSet(VPolygon[], false, false)


#===================================================#
#                 BASE UTILS 
#===================================================#

"""
Efficient indicator function: check if a point is in the reachable set

# Arguments
- `reach_set`: ReachableSet to query
- `point`: Query point [x, y]

# Returns
- `Bool`: True if point is in any polytope

# Performance
- O(n*m) where n = number of polytopes, m = vertices per polytope
- Early termination on first match
- Bounding box pre-check could be added for further speedup
"""
function Base.in(point::Vector{Float64}, reach_set::ReachableSet)
    # Early termination - return true as soon as we find containing polytope
    for poly in reach_set.polytopes
        if point ∈ poly
            return true
        end
    end
    return false
end

# Convenience method
function contains(reach_set::ReachableSet, point::Vector{Float64})
    return point ∈ reach_set
end

"""
Get total area of reachable set

# Notes
- For safe sets (is_safe=true), sums individual polytope areas
- May slightly overcount if polytopes overlap (rare after subtraction)
- For non-safe sets, considers potential overlaps
"""
function area(reach_set::ReachableSet)
    if reach_set.is_safe
        # After subtraction, polytopes are typically non-overlapping
        # Sum areas directly (fast)
        return sum(LazySets.area, reach_set.polytopes)
    elseif reach_set.is_unified
        # Polytopes are disjoint after union
        return sum(LazySets.area, reach_set.polytopes)
    else
        # May have overlaps - this is approximate
        # For exact area, would need to union first (expensive)
        @warn "Area computation for non-unified, non-safe set may overcount overlaps"
        return sum(LazySets.area, reach_set.polytopes)
    end
end

"""
Count number of polytopes in the set
"""
function count_polytopes(reach_set::ReachableSet)
    return length(reach_set.polytopes)
end

"""
Get bounding box of reachable set

# Returns
- Named tuple with (x_min, x_max, y_min, y_max)
"""
function bounding_box(reach_set::ReachableSet)
    isempty(reach_set.polytopes) && return nothing
    
    all_vertices = vcat([poly.vertices for poly in reach_set.polytopes]...)
    x_coords = [v[1] for v in all_vertices]
    y_coords = [v[2] for v in all_vertices]
    
    return (
        x_min = minimum(x_coords),
        x_max = maximum(x_coords),
        y_min = minimum(y_coords),
        y_max = maximum(y_coords)
    )
end


#===================================================#
#              LAZYSETS INTERFACE
#===================================================#


"""
Convert to nested UnionSet (recommended for LazySets compatibility)

# Returns
- Nested `UnionSet` representing the union of all polytopes

# Note
- This is the most compatible approach with LazySets operations
- Works with ∈ operator, support functions, etc.
- UnionSet is the binary union type in LazySets
"""
function to_union_set(reach_set::ReachableSet)
    isempty(reach_set.polytopes) && error("Cannot create UnionSet from empty set")
    
    # Build nested union: (poly1 ∪ poly2) ∪ poly3 ∪ ...
    result = reach_set.polytopes[1]
    for i in 2:length(reach_set.polytopes)
        result = UnionSet(result, reach_set.polytopes[i])
    end
    
    return result
end


"""
Simplify reachable set by removing tiny polytopes

# Arguments
- `reach_set`: ReachableSet to simplify
- `min_area`: Minimum area threshold (default: 1e-8)

# Returns
- `ReachableSet`: Simplified set
"""
function simplify(reach_set::ReachableSet; min_area::Float64=1e-8)
    valid = filter(reach_set.polytopes) do poly
        length(poly.vertices) >= 3 && LazySets.area(poly) > min_area
    end
    return ReachableSet(valid, reach_set.is_unified, reach_set.is_safe)
end



#===================================================#
#           VISUALIZATION HELPERS
#===================================================#

"""
Indicator function on a grid (useful for visualization/sampling)

# Arguments
- `reach_set`: ReachableSet to query
- `x_range`: Range or vector of x coordinates
- `y_range`: Range or vector of y coordinates

# Returns
- `Matrix{Bool}`: Grid of indicators (true = inside set)
"""
function indicator_grid(reach_set::ReachableSet, x_range, y_range)
    nx, ny = length(x_range), length(y_range)
    result = zeros(Bool, ny, nx)  # Note: row-major for plotting
    
    for (j, x) in enumerate(x_range)
        for (i, y) in enumerate(y_range)
            result[i, j] = [x, y] ∈ reach_set
        end
    end
    
    return result
end


"""
Get plotting data for the reachable set

# Returns
- Vector of vertex lists, one per polytope
"""
function get_plot_data(reach_set::ReachableSet)
    return [poly.vertices for poly in reach_set.polytopes]
end

"""
Plot reachable set with unified color (no layering artifacts)

# Arguments
- `reach_set`: ReachableSet to plot
- `color`: Fill color (default: :red)
- `alpha`: Fill transparency (default: 0.3)
- `show_boundaries`: Whether to show polygon boundaries (default: false)
- `use_raster`: Use rasterized approach for truly uniform color (default: false)
- `linecolor`: Boundary line color (default: matches fill color)
- `linewidth`: Boundary line width (default: 0)
- `label`: Legend label (default: based on safety status)

# Note on overlaps
- Overlapping polytopes will create darker shading due to alpha blending
- This is unavoidable without losing nonconvex geometry (holes)
- Set alpha=1.0 for opaque fill to hide overlaps (but loses see-through effect)
- Or set use_raster=true for a pixel-based approach (slower but uniform)

# Example
```julia
p = plot(aspect_ratio=:equal)
plot_reachable_set!(p, safe_reach, color=:green, alpha=1.0)  # Opaque
```
"""
function plot_reachable_set!(p, reach_set::ReachableSet; 
                            color=:red, 
                            alpha=0.3,
                            show_boundaries=false,
                            use_raster=false,
                            linecolor=nothing,
                            linewidth=nothing,
                            label=nothing,
                            kwargs...)
    
    # Default label based on status
    if isnothing(label)
        label = reach_set.is_safe ? "Safe Reachable Set" : "Reachable Set"
    end
    
    # Set line properties to hide boundaries by default
    if isnothing(linecolor)
        linecolor = show_boundaries ? :black : color
    end
    if isnothing(linewidth)
        linewidth = show_boundaries ? 1 : 0
    end
    
    if use_raster
        # Rasterized approach - slow but perfectly uniform
        return plot_reachable_set_raster!(p, reach_set; 
                                         color=color, 
                                         alpha=alpha, 
                                         label=label,
                                         kwargs...)
    end
    
    # Standard approach - plot each polytope
    # Note: Overlaps will show as darker regions with alpha < 1
    for (i, poly) in enumerate(reach_set.polytopes)
        plot!(p, poly; 
              fillcolor=color,
              fillalpha=alpha, 
              linecolor=linecolor,
              linewidth=linewidth,
              linealpha=show_boundaries ? 1.0 : 0.0,
              label=(i == 1 ? label : ""),
              kwargs...)
    end
    
    return p
end

"""
Plot using rasterization for truly uniform color (slower but exact)

This approach:
1. Creates a fine grid over the reachable set
2. Tests each pixel with the indicator function
3. Plots as a contour/heatmap

Pro: Perfectly uniform color, preserves holes
Con: Slower, resolution-dependent
"""
function plot_reachable_set_raster!(p, reach_set::ReachableSet;
                                   color=:red,
                                   alpha=0.3,
                                   label=nothing,
                                   resolution=200,
                                   kwargs...)
    
    if isnothing(label)
        label = reach_set.is_safe ? "Safe Reachable Set" : "Reachable Set"
    end
    
    # Get bounding box
    bbox = bounding_box(reach_set)
    isnothing(bbox) && return p
    
    # Add padding
    padding = 0.1
    x_range = range(bbox.x_min - padding, bbox.x_max + padding, length=resolution)
    y_range = range(bbox.y_min - padding, bbox.y_max + padding, length=resolution)
    
    # Compute indicator grid
    @info "Computing indicator grid ($(resolution)x$(resolution))..."
    grid = indicator_grid(reach_set, x_range, y_range)
    
    # Convert boolean to float for contour
    grid_float = Float64.(grid)
    
    # Plot as filled contour at level 0.5
    contourf!(p, collect(x_range), collect(y_range), grid_float;
              levels=[0.5, 1.5],  # Only show the "inside" region
              color=color,
              fillalpha=alpha,
              linewidth=0,
              colorbar=false,
              label=label,
              kwargs...)

    scatter!(p, [NaN], [NaN], 
        markersize=10, 
        markershape=:square,
        markercolor=color,
        markeralpha=alpha,
        label=label,
        markerstrokewidth=0)
    
    return p
end

"""
Plot reachable set using vertex data (alternative method)

This plots polygons directly from vertices, giving more control over rendering.
"""
function plot_reachable_set_vertices!(p, reach_set::ReachableSet;
                                     color=:red,
                                     alpha=0.3,
                                     show_boundaries=false,
                                     merge_overlaps=true,
                                     linecolor=nothing,
                                     linewidth=nothing,
                                     label=nothing,
                                     kwargs...)
    
    if isnothing(label)
        label = reach_set.is_safe ? "Safe Reachable Set" : "Reachable Set"
    end
    
    # Set line properties to hide boundaries by default
    if isnothing(linecolor)
        linecolor = show_boundaries ? :black : color
    end
    if isnothing(linewidth)
        linewidth = show_boundaries ? 1 : 0
    end
    
    # Determine which polytopes to plot
    polytopes_to_plot = reach_set.polytopes
    
    # If merging overlaps, union the polytopes first
    if merge_overlaps && length(reach_set.polytopes) > 1
        @info "Merging overlapping polytopes for uniform color..."
        polytopes_to_plot = polygon_union_geos(reach_set.polytopes)
        @info "  → Reduced from $(length(reach_set.polytopes)) to $(length(polytopes_to_plot)) polytopes"
    end
    
    for (i, poly) in enumerate(polytopes_to_plot)
        verts = poly.vertices
        xs = [v[1] for v in verts]
        ys = [v[2] for v in verts]
        push!(xs, xs[1])  # Close polygon
        push!(ys, ys[1])
        
        plot!(p, xs, ys;
              fillcolor=color,
              fillalpha=alpha,
              linecolor=linecolor,
              linewidth=linewidth,
              linealpha=show_boundaries ? 1.0 : 0.0,
              seriestype=:shape,
              label=(i == 1 ? label : ""),
              kwargs...)
    end
    
    return p
end

# Standalone plotting function
function plot_reachable_set(reach_set::ReachableSet; kwargs...)
    p = plot(legend=:topright, aspect_ratio=:equal)
    plot_reachable_set!(p, reach_set; kwargs...)
    return p
end

"""
Complete plotting function: obstacles + safe reachable set

# Example
```julia
plot_workspace(safe_reach, c_obstacles)
```
"""
function plot_workspace(reach_set::ReachableSet, 
                       obstacles::Vector{VPolygon};
                       reach_color=:red,
                       reach_alpha=0.3,
                       obs_color=:grey,
                       obs_alpha=0.3,
                       show_reach_boundaries=false,
                       show_obs_boundaries=true,
                       merge_overlaps=true,
                       title="Reachable Set with Obstacles",
                       kwargs...)
    
    p = plot(aspect_ratio=:equal, title=title, legend=:topright)
    
    # Plot obstacles first (background)
    for (i, obs) in enumerate(obstacles)
        plot!(p, obs;
              fillcolor=obs_color,
              fillalpha=obs_alpha,
              linecolor=:black,
              linewidth=show_obs_boundaries ? 1 : 0,
              linealpha=show_obs_boundaries ? 1.0 : 0.0,
              label=(i == 1 ? "Obstacles" : ""))
    end
    
    # Plot reachable set on top
    plot_reachable_set!(p, reach_set;
                       color=reach_color,
                       alpha=reach_alpha,
                       show_boundaries=show_reach_boundaries,
                       merge_overlaps=merge_overlaps)
    
    return p
end



#===================================================#
#           DIAGNOSTIC UTILITIES
#===================================================#

"""
Validate that obstacles are properly subtracted from safe reachable set
"""
function validate_safety(reach_set::ReachableSet, c_space_obstacles::Vector{VPolygon})
    !reach_set.is_safe && @warn "ReachableSet is not marked as safe"
    
    total_intersection = 0.0
    for obs in c_space_obstacles
        for poly in reach_set.polytopes
            inter = intersection(poly, obs)
            if !isempty(inter)
                total_intersection += LazySets.area(inter)
            end
        end
    end
    
    if total_intersection > 1e-6
        @warn "Safety violation detected!" intersection_area=total_intersection
        return false
    end
    
    @info "Safety validation passed" intersection_area=total_intersection
    return true
end

"""
Print summary statistics about the reachable set
"""
function summarize(reach_set::ReachableSet)
    println("ReachableSet Summary:")
    println("  Polytopes: ", count_polytopes(reach_set))
    println("  Total area: ", round(area(reach_set), digits=4))
    println("  Is unified: ", reach_set.is_unified)
    println("  Is safe: ", reach_set.is_safe)
    
    bbox = bounding_box(reach_set)
    if !isnothing(bbox)
        println("  Bounding box:")
        println("    x ∈ [$(round(bbox.x_min, digits=2)), $(round(bbox.x_max, digits=2))]")
        println("    y ∈ [$(round(bbox.y_min, digits=2)), $(round(bbox.y_max, digits=2))]")
    end
end


#===================================================#
#              LibGEOS INTERFACE
#===================================================#

"""
Convert VPolygon to LibGEOS Polygon
"""
function to_geos(poly::VPolygon)
    # Validate polygon has vertices
    if length(poly.vertices) < 3
        @warn "Cannot convert VPolygon with < 3 vertices to GEOS polygon. Has $(length(poly.vertices)) vertices."
    end
    
    # GEOS requires closed rings (first point == last point)
    coords = [v for v in poly.vertices]
    push!(coords, coords[1])  # Close the ring
    
    return LibGEOS.Polygon([coords])
end

# """
# Convert LibGEOS geometry back to VPolygon(s)
# """
# function from_geos(geom)
#     polygons = VPolygon[]
    
#     # Handle different geometry types
#     geom_type = LibGEOS.geomTypeId(geom)
    
#     if geom_type == LibGEOS.GEOS_POLYGON
#         ring = LibGEOS.exteriorRing(geom)
#         coords = GeoInterface.coordinates(ring)
#         # Remove closing point (last == first)
#         pts = [[c[1], c[2]] for c in coords[1:end-1]]
#         push!(polygons, VPolygon(pts))
        
#     elseif geom_type == LibGEOS.GEOS_MULTIPOLYGON
#         n = LibGEOS.numGeometries(geom)
#         for i in 1:n
#             subgeom = LibGEOS.getGeometry(geom, i)
#             ring = LibGEOS.exteriorRing(subgeom)
#             coords = GeoInterface.coordinates(ring)
#             pts = [[c[1], c[2]] for c in coords[1:end-1]]
#             push!(polygons, VPolygon(pts))
#         end
#     end
    
#     return polygons
# end



"""
Convert LibGEOS geometry back to VPolygon(s)
Uses Delaunay triangulation to decompose polygons with holes
"""
function from_geos(geom)
    polygons = VPolygon[]
    
    geom_type = LibGEOS.geomTypeId(geom)
    
    if geom_type == LibGEOS.GEOS_POLYGON
        n_interior = LibGEOS.numInteriorRings(geom)
        
        if n_interior > 0
            # Use delaunay triangulation then filter triangles inside the polygon
            all_points = LibGEOS.Point[]
            
            # Exterior vertices
            exterior_ring = LibGEOS.exteriorRing(geom)
            ext_coords = GeoInterface.coordinates(exterior_ring)
            for c in ext_coords[1:end-1]
                push!(all_points, LibGEOS.Point(c[1], c[2]))
            end
            
            # Interior vertices
            for i in 1:n_interior
                interior_ring = LibGEOS.interiorRing(geom, i)
                int_coords = GeoInterface.coordinates(interior_ring)
                for c in int_coords[1:end-1]
                    push!(all_points, LibGEOS.Point(c[1], c[2]))
                end
            end
            
            # Create multipoint for triangulation
            mp = LibGEOS.MultiPoint(all_points)
            triangulated = LibGEOS.delaunayTriangulation(mp)
            
            # Filter triangles
            tri_type = LibGEOS.geomTypeId(triangulated)
            
            if tri_type == LibGEOS.GEOS_GEOMETRYCOLLECTION
                n_tris = LibGEOS.numGeometries(triangulated)
                for i in 1:n_tris
                    tri = LibGEOS.getGeometry(triangulated, i)
                    centroid = LibGEOS.centroid(tri)
                    
                    # Check if centroid is in original polygon (not in holes)
                    if LibGEOS.contains(geom, centroid)
                        # Convert triangle to VPolygon
                        tri_ring = LibGEOS.exteriorRing(tri)
                        tri_coords = GeoInterface.coordinates(tri_ring)
                        tri_pts = [[c[1], c[2]] for c in tri_coords[1:end-1]]
                        
                        # CRITICAL: Filter degenerate triangles
                        if length(tri_pts) >= 3 && LazySets.area(VPolygon(tri_pts)) > 1e-10
                            push!(polygons, VPolygon(tri_pts))
                        end
                    end
                end
            end
            
            return polygons
        else
            # Simple polygon without holes
            ring = LibGEOS.exteriorRing(geom)
            coords = GeoInterface.coordinates(ring)
            pts = [[c[1], c[2]] for c in coords[1:end-1]]
            
            # Validate before creating polygon
            if length(pts) >= 3
                push!(polygons, VPolygon(pts))
            end
        end
        
    elseif geom_type == LibGEOS.GEOS_MULTIPOLYGON
        n = LibGEOS.numGeometries(geom)
        for i in 1:n
            subgeom = LibGEOS.getGeometry(geom, i)
            append!(polygons, from_geos(subgeom))
        end
        
    elseif geom_type == LibGEOS.GEOS_GEOMETRYCOLLECTION
        n = LibGEOS.numGeometries(geom)
        for i in 1:n
            subgeom = LibGEOS.getGeometry(geom, i)
            append!(polygons, from_geos(subgeom))
        end
    end
    
    return polygons
end




function polygon_difference_geos(A::VPolygon, B::VPolygon)
    geom_A = to_geos(A)
    geom_B = to_geos(B)
    
    # println("A valid: ", LibGEOS.isValid(geom_A))
    # println("B valid: ", LibGEOS.isValid(geom_B))
    
    result_geom = LibGEOS.difference(geom_A, geom_B)
    
    # println("Result valid: ", LibGEOS.isValid(result_geom))
    # println("Result is empty: ", LibGEOS.isEmpty(result_geom))
    
    return from_geos(result_geom)
end




function polygon_intersection_geos(A::VPolygon, B::VPolygon)
    geom_A = to_geos(A)
    geom_B = to_geos(B)
    
    result_geom = LibGEOS.intersection(geom_A, geom_B)
    
    return from_geos(result_geom)
end



"""
Union multiple VPolygons using LibGEOS
"""
function polygon_union_geos(polygons::Vector{VPolygon})
    isempty(polygons) && return VPolygon[]
    
    # Convert all to GEOS geometries
    geoms = [to_geos(p) for p in polygons]
    
    # Iteratively union
    result_geom = geoms[1]
    for i in 2:length(geoms)
        result_geom = LibGEOS.union(result_geom, geoms[i])
    end
    
    return from_geos(result_geom)
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
            sol = DifferentialEquations.solve(prob, Tsit5(), callback=cb, saveat=dt)
            
            # Extract 2D position (assumes first 2 states are spatial)
            push!(points, [sol[end][1], sol[end][2]])
        end
        
        # Create convex hull of sampled points
        if length(points) >= 3
            push!(polytopes, VPolygon(points))
        end


        # After the first segment is computed, add:
        if length(polytopes) == 1
            println("First polytope sample points:")
            for (i, pt) in enumerate(points[1:min(5, end)])
                println("  Point $i: ", pt)
            end
            println("First polytope center: ", sum(points) / length(points))
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

"""
Subtract obstacles from reachable set (nonconvex-preserving)

This version:
1. Preserves holes by NOT performing early union
2. Returns Vector{VPolygon} that correctly represents nonconvex geometry
3. Optionally simplifies by merging adjacent polytopes at the end

# Arguments
- `reachable_polytopes`: Convex polytopes from reachability computation
- `c_space_obstacles`: Configuration-space obstacles (after Minkowski sum)
- `merge_adjacent`: If true, merge adjacent fragments to reduce count (default: false)

# Returns
- `Vector{VPolygon}`: Remaining safe regions as separate convex pieces
"""
function subtract_obstacles(
    reachable_polytopes::Vector{VPolygon},
    c_space_obstacles::Vector{VPolygon};
    merge_adjacent::Bool = false
)
    @info "Beginning obstacle subtraction (nonconvex-preserving)..."
    fragments = VPolygon[]

    # Process each reachable polytope independently
    for (j, poly) in enumerate(reachable_polytopes)
        #@info "Processing reachable polytope $j/$(length(reachable_polytopes))..."
        current_parts = [poly]

        # Subtract each obstacle sequentially
        for (i, obs) in enumerate(c_space_obstacles)
            new_parts = VPolygon[]
            for part in current_parts
                result = polygon_difference_geos(part, obs)
                append!(new_parts, result)
            end

            if isempty(new_parts)
                @info "  → polytope $j completely removed by obstacle $i"
                current_parts = VPolygon[]
                break
            end

            current_parts = new_parts
        end

        append!(fragments, current_parts)
    end

    @info "  → Generated $(length(fragments)) fragments before merging"

    # Optional: merge adjacent fragments to reduce count
    if merge_adjacent && !isempty(fragments)
        @info "Merging adjacent fragments..."
        fragments = polygon_union_geos(fragments)
        @info "  → Reduced to $(length(fragments)) polytopes after merging"
    end

    return fragments
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