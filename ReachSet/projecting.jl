using NearestNeighbors
using JuMP, OSQP
using LinearAlgebra
using LazySets  # Assuming you're using LazySets for VPolygon/VPolytope

struct FastProjector
    tree::KDTree
    polytopes::Vector
    centroids::Matrix{Float64}
    radii::Vector{Float64}  # Precomputed bounding radii
    active_idx::Ref{Int}
end

function FastProjector(polytopes)
    centroids = hcat([centroid(p) for p in polytopes]...)
    tree = KDTree(centroids)
    radii = [radius_bound(p, centroids[:, i]) for (i, p) in enumerate(polytopes)]
    FastProjector(tree, polytopes, centroids, radii, Ref(1))
end

function project!(proj::FastProjector, x::Vector{Float64})
    # 1. Check cached polytope first (O(1) amortized)
    if in_polytope(proj.polytopes[proj.active_idx[]], x)
        return x
    end
    
    p_active = project_to_polytope(proj.polytopes[proj.active_idx[]], x)
    dist_active = norm(x - p_active)
    
    # 2. Query k nearest centroids (O(log n))
    idxs, dists = knn(proj.tree, x, 5)
    
    # 3. Project to nearest candidates only (O(k))
    best_proj = p_active
    best_dist = dist_active
    
    for (i, idx) in enumerate(idxs)
        # Early termination: if centroid farther than current best, skip
        if dists[i] - proj.radii[idx] > best_dist
            continue
        end
        
        p = project_to_polytope(proj.polytopes[idx], x)
        d = norm(x - p)
        if d < best_dist
            best_dist = d
            best_proj = p
            proj.active_idx[] = idx
        end
    end
    
    return best_proj
end

function project_to_polytope(poly, x)
    # If already inside, return immediately
    in_polytope(poly, x) && return x
    
    # Convert to H-representation
    A, b = hrep(poly)
    n = length(x)
    
    # QP: min ||z - x||² s.t. A*z ≤ b
    model = Model(OSQP.Optimizer)
    set_silent(model)
    set_attribute(model, "polish", true)  # Improved accuracy
    set_attribute(model, "eps_abs", 1e-6)
    set_attribute(model, "eps_rel", 1e-6)
    
    @variable(model, z[1:n])
    @objective(model, Min, sum((z[i] - x[i])^2 for i in 1:n))
    @constraint(model, A * z .<= b)
    
    JuMP.optimize!(model)
    
    return termination_status(model) == MOI.OPTIMAL ? value.(z) : x
end

# Helper functions
function centroid(poly)
    verts = vertices_list(poly)
    return vec(mean(hcat(verts...), dims=2))
end

function radius_bound(poly, center)
    verts = vertices_list(poly)
    return maximum(norm(v - center) for v in verts)
end

function in_polytope(poly, x)
    A, b = hrep(poly)
    return all(A * x .<= b .+ 1e-8)  # Small tolerance for numerical stability
end

function hrep(poly)
    # Convert V-representation to H-representation
    # This assumes LazySets.jl; adjust if using different library
    hpoly = tohrep(poly)
    A = hpoly.constraints
    b = [c.b for c in A]
    A = vcat([c.a' for c in A]...)
    return A, b
end