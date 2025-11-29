using DifferentialEquations, LazySets, LinearAlgebra
using LibGEOS
using GeoInterface
include("../DynamicPlanning.jl/src/robot.jl")

#===================================================#
#              LibGEOS INTERFACE
#===================================================#

# """
# Convert VPolygon to LibGEOS Polygon
# """
# function to_geos(poly::VPolygon)
#     # GEOS requires closed rings (first point == last point)
#     coords = [v for v in poly.vertices]
#     push!(coords, coords[1])  # Close the ring
    
#     return LibGEOS.Polygon([coords])
# end

"""
Convert VPolygon to LibGEOS Polygon
"""
function to_geos(poly::VPolygon)
    # Validate polygon has vertices
    if length(poly.vertices) < 3
        error("Cannot convert VPolygon with < 3 vertices to GEOS polygon. Has $(length(poly.vertices)) vertices.")
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

# """
# Convert LibGEOS geometry back to VPolygon(s)
# Simplified version: ignores holes (conservative approximation)
# """
# function from_geos(geom)
#     polygons = VPolygon[]
    
#     geom_type = LibGEOS.geomTypeId(geom)
    
#     if geom_type == LibGEOS.GEOS_POLYGON
#         # Always use exterior ring only
#         # For polygons with holes, this is a conservative over-approximation
#         ring = LibGEOS.exteriorRing(geom)
#         coords = GeoInterface.coordinates(ring)
#         pts = [[c[1], c[2]] for c in coords[1:end-1]]
        
#         n_interior = LibGEOS.numInteriorRings(geom)
#         if n_interior > 0
#             @warn "Polygon has $n_interior hole(s) - using exterior only (conservative approximation)"
#         end
        
#         push!(polygons, VPolygon(pts))
        
#     elseif geom_type == LibGEOS.GEOS_MULTIPOLYGON
#         n = LibGEOS.numGeometries(geom)
#         for i in 1:n
#             subgeom = LibGEOS.getGeometry(geom, i)
#             append!(polygons, from_geos(subgeom))
#         end
        
#     elseif geom_type == LibGEOS.GEOS_GEOMETRYCOLLECTION
#         n = LibGEOS.numGeometries(geom)
#         for i in 1:n
#             subgeom = LibGEOS.getGeometry(geom, i)
#             append!(polygons, from_geos(subgeom))
#         end
#     end
    
#     return polygons
# end




# """
# Convert LibGEOS geometry back to VPolygon(s)
# Uses Delaunay triangulation to decompose polygons with holes
# """
# function from_geos(geom)
#     polygons = VPolygon[]
    
#     geom_type = LibGEOS.geomTypeId(geom)
    
#     if geom_type == LibGEOS.GEOS_POLYGON
#         n_interior = LibGEOS.numInteriorRings(geom)
        
#         if n_interior > 0
#             # Use delaunay triangulation then filter triangles inside the polygon
#             # Get all vertices (exterior + interior)
#             all_points = LibGEOS.Point[]
            
#             # Exterior vertices
#             exterior_ring = LibGEOS.exteriorRing(geom)
#             ext_coords = GeoInterface.coordinates(exterior_ring)
#             for c in ext_coords[1:end-1]  # Skip duplicate closing point
#                 push!(all_points, LibGEOS.Point(c[1], c[2]))
#             end
            
#             # Interior vertices
#             for i in 1:n_interior
#                 interior_ring = LibGEOS.interiorRing(geom, i)
#                 int_coords = GeoInterface.coordinates(interior_ring)
#                 for c in int_coords[1:end-1]
#                     push!(all_points, LibGEOS.Point(c[1], c[2]))
#                 end
#             end
            
#             # Create multipoint for triangulation
#             mp = LibGEOS.MultiPoint(all_points)
#             triangulated = LibGEOS.delaunayTriangulation(mp)
            
#             # Filter triangles: keep only those whose centroids are in original polygon
#             tri_type = LibGEOS.geomTypeId(triangulated)
            
#             if tri_type == LibGEOS.GEOS_GEOMETRYCOLLECTION
#                 n_tris = LibGEOS.numGeometries(triangulated)
#                 for i in 1:n_tris
#                     tri = LibGEOS.getGeometry(triangulated, i)
#                     centroid = LibGEOS.centroid(tri)
                    
#                     # Check if centroid is in original polygon (not in holes)
#                     if LibGEOS.contains(geom, centroid)
#                         # Convert triangle to VPolygon
#                         tri_ring = LibGEOS.exteriorRing(tri)
#                         tri_coords = GeoInterface.coordinates(tri_ring)
#                         tri_pts = [[c[1], c[2]] for c in tri_coords[1:end-1]]
#                         push!(polygons, VPolygon(tri_pts))
#                     end
#                 end
#             end
            
#             return polygons
#         else
#             # Simple polygon without holes
#             ring = LibGEOS.exteriorRing(geom)
#             coords = GeoInterface.coordinates(ring)
#             pts = [[c[1], c[2]] for c in coords[1:end-1]]
#             push!(polygons, VPolygon(pts))
#         end
        
#     elseif geom_type == LibGEOS.GEOS_MULTIPOLYGON
#         n = LibGEOS.numGeometries(geom)
#         for i in 1:n
#             subgeom = LibGEOS.getGeometry(geom, i)
#             append!(polygons, from_geos(subgeom))
#         end
        
#     elseif geom_type == LibGEOS.GEOS_GEOMETRYCOLLECTION
#         n = LibGEOS.numGeometries(geom)
#         for i in 1:n
#             subgeom = LibGEOS.getGeometry(geom, i)
#             append!(polygons, from_geos(subgeom))
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
    
    println("A valid: ", LibGEOS.isValid(geom_A))
    println("B valid: ", LibGEOS.isValid(geom_B))
    
    result_geom = LibGEOS.difference(geom_A, geom_B)
    
    println("Result valid: ", LibGEOS.isValid(result_geom))
    println("Result is empty: ", LibGEOS.isEmpty(result_geom))
    
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
            sol = solve(prob, Tsit5(), callback=cb, saveat=dt)
            
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
Subtract obstacles from reachable set using LibGEOS

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
    # Union all reachable polytopes once
    @info "Unifying $(length(reachable_polytopes)) reachable polytopes..."
    current_region = polygon_union_geos(reachable_polytopes)
    @info "  → $(length(current_region)) connected component(s)"

    isempty(current_region) && return VPolygon[]

    # Subtract all obstacles sequentially WITHOUT intermediate unions
    for (i, obs) in enumerate(c_space_obstacles)
        @info "Subtracting obstacle $i..."
        
        new_fragments = VPolygon[]
        for comp in current_region
            result = polygon_difference_geos(comp, obs)
            append!(new_fragments, result)
        end
        
        if isempty(new_fragments)
            @warn "Reachable set completely eliminated after obstacle $i!"
            return VPolygon[]
        end
        
        current_region = new_fragments
        @info "  → $(length(current_region)) fragment(s)"
    end

    # Final union to merge adjacent fragments
    @info "Final union of fragments..."
    return polygon_union_geos(current_region)
end



# function subtract_obstacles(
#     reachable_polytopes::Vector{VPolygon},
#     c_space_obstacles::Vector{VPolygon}
# )

#     # Step 1: Union all reachable polytopes
#     @info "Unifying $(length(reachable_polytopes)) reachable polytopes..."
#     current_region = polygon_union_geos(reachable_polytopes)
#     @info "  → $(length(current_region)) connected component(s)"

#     isempty(current_region) && return VPolygon[]

#     # Step 2: Process each obstacle individually
#     for (i, obs) in enumerate(c_space_obstacles)
#         @info "Subtracting obstacle $i..."

#         fragments = VPolygon[]

#         for comp in current_region
#             result = polygon_difference_geos(comp, obs)
#             append!(fragments, result)
#         end

#         if isempty(fragments)
#             @warn "Reachable set completely eliminated after obstacle $i!"
#             return VPolygon[]
#         end

#         # Union fragments to merge touching components
#         current_region = polygon_union_geos(fragments)
#         @info "  → $(length(current_region)) component(s) remain"
#     end

#     return current_region
# end






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