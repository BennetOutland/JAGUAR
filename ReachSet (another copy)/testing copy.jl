using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   
include("reachability.jl")

using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   

include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../DynamicPlanning.jl/src/models.jl")
include("reachability.jl")


# Params
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
#safe_reach = compute_safe_reachable_set(current_reach, c_obstacles; batch_size=5)


# res = subtract_obstacles(reach_set.polytopes, c_obstacles)
# res[1]


current_region = polygon_union(reach_set.polytopes)

# p1 = plot(current_region[1])
# for (i, obs) ∈ enumerate(c_obstacles)
#     plot!(p1, obs, label=i)
# end

# display(p1)

# current_region = polygon_union(reach_set.polytopes)
# plot(current_region[1])
# plot!(c_obstacles[8])
# plot!(c_obstacles[12])

obs = c_obstacles[8]
comp = current_region[1]

function polygon_intersection(A::VPolygon, B::VPolygon)
    c = Clip()

    add_path!(c, to_clipper(A), PolyTypeSubject, true)
    add_path!(c, to_clipper(B), PolyTypeClip, true)

    success, result =
        execute(c, ClipTypeIntersection, PolyFillTypeEvenOdd, PolyFillTypeNonZero)

    return success ? from_clipper(result) : VPolygon[]
end


# function polygon_difference(A::VPolygon, B::VPolygon)
#     c = Clip()

#     add_path!(c, to_clipper(A), PolyTypeSubject, true)
#     add_path!(c, to_clipper(B), PolyTypeClip, true)

#     success, result =
#         execute(c, ClipTypeDifference, PolyFillTypeEvenOdd, PolyFillTypeNonZero)

#     return success ? from_clipper(result) : VPolygon[]
# end


function polygon_difference_robust(A::VPolygon, B::VPolygon, universe::VPolygon)
    # Compute complement of B relative to universe
    Bc = polygon_complement(B, universe)
    
    # Intersect A with B^c
    return polygon_intersection(A, Bc[1])
end


function ellipsoid_to_vpolygon(E, n::Int=64)
    # Center and shape matrix
    c = E.center
    Q = E.shape_matrix  
    
    θ = range(0, 2π, length=n+1)[1:end-1]  # avoid duplicate point
    points = [c .+ sqrt(Q) * [cos(t); sin(t)] for t in θ]  # sqrt(Q) = Cholesky
    
    return VPolygon(points)
end

universe = ellipsoid_to_vpolygon(𝒲.bounds, 12)



function ensure_ccw(poly::VPolygon)
    verts = poly.vertices
    area = 0.0
    for i in 1:length(verts)
        x1, y1 = verts[i]
        x2, y2 = verts[mod1(i+1,length(verts))]
        area += (x1*y2 - x2*y1)
    end
    area < 0 ? VPolygon(reverse(verts)) : poly
end

function ensure_cw(poly::VPolygon)
    verts = poly.vertices
    area = 0.0
    for i in 1:length(verts)
        x1, y1 = verts[i]
        x2, y2 = verts[mod1(i+1,length(verts))]
        area += (x1*y2 - x2*y1)
    end
    area > 0 ? VPolygon(reverse(verts)) : poly
end

function polygon_complement(B::VPolygon, universe::VPolygon)
    c = Clip()

    # Ensure CCW outer rings
    U = ensure_ccw(universe)
    Bb = ensure_ccw(B)

    # Convert to clipper paths
    Up = to_clipper(U)
    Bp = to_clipper(Bb)

    # Subject = universe, Clip = B (not both as Subject)
    add_path!(c, Up, PolyTypeSubject, true)
    add_path!(c, Bp, PolyTypeClip, true)  # Changed from PolyTypeSubject

    success, result = execute(
        c,
        ClipTypeDifference,
        PolyFillTypeEvenOdd,
        PolyFillTypeEvenOdd,
    )

    return success ? from_clipper(result) : VPolygon[]
end


# function polygon_difference(A::VPolygon, B::VPolygon)
#     c = Clip()
    
#     # Ensure consistent orientation
#     A_ccw = ensure_ccw(A)
#     B_ccw = ensure_ccw(B)

#     add_path!(c, to_clipper(A_ccw), PolyTypeSubject, true)
#     add_path!(c, to_clipper(B_ccw), PolyTypeClip, true)

#     success, result =
#         execute(c, ClipTypeDifference, PolyFillTypeNonZero, PolyFillTypeNonZero)

#     return success ? from_clipper(result) : VPolygon[]
# end

# function polygon_difference(A::VPolygon, B::VPolygon)
#     c = Clip()

#     add_path!(c, to_clipper(A), PolyTypeSubject, true)
#     add_path!(c, to_clipper(B), PolyTypeClip, true)

#     success, result =
#         execute(c, ClipTypeDifference, PolyFillTypePositive, PolyFillTypePositive)

#     return success ? from_clipper(result) : VPolygon[]
# end

function polygon_difference(A::VPolygon, B::VPolygon)
    c = Clip()
    
    # Subject should be CCW (outer boundary)
    A_ccw = ensure_ccw(A)
    
    # Clip should be CW (to represent "subtract this region")
    B_cw = ensure_cw(B)

    add_path!(c, to_clipper(A_ccw), PolyTypeSubject, true)
    add_path!(c, to_clipper(B_cw), PolyTypeClip, true)

    success, result =
        execute(c, ClipTypeDifference, PolyFillTypeNonZero, PolyFillTypeNonZero)

    return success ? from_clipper(result) : VPolygon[]
end



res = polygon_difference(comp, obs)
# res = polygon_complement(obs, universe)


# println("Obstacle vertices: ", length(obs.vertices))
# println("Obstacle area: ", LazySets.area(obs))  # Should be positive
# println("Reachable set area: ", LazySets.area(comp))

# # Check if obstacle is actually inside/intersecting
# intersect_test = polygon_intersection(comp, obs)
# println("Intersection returned $(length(intersect_test)) polygon(s)")
# if !isempty(intersect_test)
#     println("Intersection area: ", LazySets.area(intersect_test[1]))
# end


# result = polygon_difference(comp, obs)
# diff_poly = result[1]

println("Difference polygon vertices: ", length(diff_poly.vertices))
println("Difference polygon area: ", LazySets.area(diff_poly))
println("Expected area (approx): ", LazySets.area(comp) - LazySets.area(intersect_test[1]))

# Verify the vertices are reasonable
println("Sample vertices: ", diff_poly.vertices[1:min(5, length(diff_poly.vertices))])


result = polygon_difference(comp, obs)
diff_poly = result[1]

# Plot each component separately with explicit bounds
plot(comp, alpha=0.3, label="Reachable", xlims=(-8,8), ylims=(-8,8))
plot!(obs, alpha=0.3, label="Obstacle")
plot!(diff_poly, alpha=0.5, label="Difference", linewidth=2)

# Manually verify a point that should be in difference
test_point = [-5.0, 0.0]  # Visually inside comp but outside obs
println("Test point in comp: ", test_point ∈ comp)
println("Test point in obs: ", test_point ∈ obs)
println("Test point in result: ", test_point ∈ diff_poly)



plot(obs, label="Obstacle") 
plot!(comp, label="Reach Set")
plot!(res[1], label="Difference")


# """
# Issue:
# - Difference computation is not working as expected
# - Does not capture the whole obstacle
# """


# *Optionally* union fragments to merge touching components
# Comment this out if you want raw fragments.
# new_region = polygon_union(fragments)
# @info "  → $(length(new_region)) component(s) remain"



# """
# Difference operation: A \\ B using Clipper
# """
# function polygon_difference(A::VPolygon, B::VPolygon)
#     c = Clip()
#     add_path!(c, to_clipper(A), PolyTypeSubject, true)
#     add_path!(c, to_clipper(B), PolyTypeClip, true)
    
#     success, result = execute(c, ClipTypeDifference, PolyFillTypeEvenOdd, PolyFillTypeEvenOdd)

#     if !success
#         @info "  → difference failed"
#     end
    
#     return success ? from_clipper(result) : VPolygon[]
# end





# plot(new_region[1])
# plot!(c_obstacles[8])