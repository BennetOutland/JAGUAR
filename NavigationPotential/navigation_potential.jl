"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- 
"""

# Includes
include("../DynamicPlanning.jl/src/workspace.jl")


# Usings 
using LinearAlgebra
using LazySets
import LazySets: overapproximate



# Define a custom overapproximation method
function LazySets.overapproximate(S::VPolygon, ::Type{Ball2})
    verts = vertices_list(S)
    c = mean(verts)
    r = maximum(norm(v - c) for v in verts)
    return Ball2(c, r)
end

"""
Compute the beta function for obstacle i: b_i(x) = d²(x, x_i) - r_i²
Returns ≤ 0 inside the spherical set.
"""
function b_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    dist = d(x, sphere_set.center)
    return dist^2 - sphere_set.radius^2
end



"""
TODO
"""
function b_product(x::Vector{Float64}, W::Workspace, d::Function)
    prod = 1.0
    for obs in W.obstacles
        prod *= b_i(x, obs, d)
    end
    return prod
end




"""
TODO
"""
function NavigationPotentialFunction(W::Workspace, x0::Vector{Float64}, xf::Vector{Float64}, d::Function; k=3)

    # Compute the potential in sphere space 
    

    # Convert to star space

    #return b_i(x0, W.obstacles[1], d) 
    return b_product(x0, W, d)
end 