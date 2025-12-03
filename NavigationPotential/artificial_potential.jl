using LazySets
using LinearAlgebra
using LazySets: ρ, σ

#############################
#  Point in polygon (LazySets)
#############################

pip(vertices::Vector{Vector{Float64}}, x::Vector) = (x ∈ VPolygon(vertices))


using LinearAlgebra
using LazySets

# Compute polygon centroid manually
function polygon_centroid(poly::VPolygon)
    verts = vertices(poly)
    n = length(verts)
    signed_area = 0.0
    Cx, Cy = 0.0, 0.0

    for i in 1:n
        p0 = verts[i]
        p1 = verts[mod1(i+1, n)]
        a = p0[1]*p1[2] - p1[1]*p0[2]
        signed_area += a
        Cx += (p0[1]+p1[1])*a
        Cy += (p0[2]+p1[2])*a
    end

    signed_area *= 0.5
    Cx /= (6 * signed_area)
    Cy /= (6 * signed_area)
    return [Cx, Cy]
end

# Line-intersection distance to polygon
function dist_to_obstacle(x::AbstractVector, obs::VPolygon)
    verts = vertices(obs)
    c = polygon_centroid(obs)
    d = x .- c

    if norm(d) < 1e-9
        return 0.0
    end

    closest_t = Inf

    for i in 1:length(verts)
        p1 = verts[i]
        p2 = verts[mod1(i+1,length(verts))]
        e = p2 .- p1

        A = [d -e]   # t*d - u*e = p1 - x
        b = p1 .- x

        if abs(det(A)) < 1e-12
            continue
        end

        sol = A \ b
        t, u = sol[1], sol[2]

        if t ≥ 0 && 0 ≤ u ≤ 1
            closest_t = min(closest_t, t)
        end
    end

    if closest_t < Inf
        boundary_dist = closest_t * norm(d)
        return max(norm(d) - boundary_dist, 0.0)
    else
        return norm(d)
    end
end

"""
Distance outside a circular workspace (Ball2)
Returns:
  >0 if outside
  0 if inside
"""
function dist_outside_workspace(x::AbstractVector, ws::Ball2)
    r = ws.radius
    d = norm(x - ws.center)
    if d <= r
        return 0.0        # inside → no penalty
    else
        return d - r      # outside → distance from boundary
    end
end



#############################
# Polygon centroid
#############################

function polygon_centroid(vertices::Vector{Vector{Float64}})
    signed_area = 0.0
    Cx, Cy = 0.0, 0.0
    n = length(vertices)

    for i in 1:n
        p0 = vertices[i]
        p1 = vertices[mod1(i+1,n)]
        a = p0[1]*p1[2] - p1[1]*p0[2]
        signed_area += a
        Cx += (p0[1] + p1[1]) * a
        Cy += (p0[2] + p1[2]) * a
    end

    signed_area *= 0.5
    Cx /= (6signed_area)
    Cy /= (6signed_area)
    return [Cx, Cy]
end


#############################
# Potential Function Struct
#############################

struct PotentialFunction
    d_star::Float64
    zeta::Float64
    Q_star::Float64
    eta::Float64
    q_init::Vector
    q_goal::Vector
    obstacles::Vector{VPolygon}
end


#############################
# Potential Value
#############################

function (P::PotentialFunction)(q::Vector, ws::Ball2)
    # Attractive term
    dist = norm(q - P.q_goal)
    potential = dist ≤ P.d_star ?
        0.5 * P.zeta * dist^2 :
        P.d_star * P.zeta * dist - 0.5 * P.zeta * P.d_star^2

    # Repulsive obstacles
    for obs in P.obstacles
        d_obs = dist_to_obstacle(q, obs)
        if d_obs < P.Q_star && d_obs > 1e-6
            potential += 0.5 * P.eta * (1/d_obs - 1/P.Q_star)^2
        end
    end

    # Repulsive workspace (outside)
    d_ws = dist_outside_workspace(q, ws)
    if d_ws > 1e-6
        # can reuse eta or define separate weight
        potential += 0.5 * P.eta * (1/d_ws)^2
    end

    if potential > 50.0
        return 50.0
    end

    return potential
end


to_Ball2(E::Ellipsoid) = Ball2(E.center, sqrt(maximum(eigvals(E.shape_matrix))))

