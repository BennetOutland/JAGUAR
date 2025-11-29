"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Navigation Potential Field Implementation
Based on Rimon & Koditschek (1992) with sphere-space formulation
"""

# Includes
include("../DynamicPlanning.jl/src/workspace.jl")

# Usings 
using LinearAlgebra
using LazySets
import LazySets: overapproximate
using Plots
using Statistics

# Define a custom overapproximation method
function LazySets.overapproximate(S::VPolygon, ::Type{Ball2})
    verts = vertices_list(S)
    c = mean(verts)
    r = maximum(norm(v - c) for v in verts)
    return Ball2(c, r)
end

"""
Wrap angle to [-π, π].
"""
function wrap_angle(θ::Float64)
    return mod(θ + π, 2π) - π
end

"""
Compute the approximate SE(2) distance using weighted Euclidean metric.

d(x₁, x₂) ≈ √(c₁||t₁ - t₂||² + c₂·wrap(θ₂ - θ₁)²)
"""
function se2_distance_approx(x1::Vector{Float64}, x2::Vector{Float64};
                            c1::Float64=1.0, c2::Float64=1.0)
    # Extract positions
    t1 = x1[1:2]
    t2 = x2[1:2]
    
    # Translational component
    trans_dist_sq = norm(t1 - t2)^2
    
    # Rotational component (if orientation is provided)
    if length(x1) >= 3 && length(x2) >= 3
        θ1 = x1[3]
        θ2 = x2[3]
        Δθ = wrap_angle(θ2 - θ1)
        rot_dist_sq = Δθ^2
    else
        rot_dist_sq = 0.0
    end
    
    # Weighted distance
    return sqrt(c1 * trans_dist_sq + c2 * rot_dist_sq)
end

"""
Compute b_i(x) = d²(x, x_i) - r_i²

Returns ≤ 0 inside obstacle, > 0 outside obstacle.
Uses Euclidean distance to obstacle center.
"""
function b_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    
    # Extract 2D position and use Euclidean distance
    x_pos = x[1:2]
    center_pos = sphere_set.center
    
    dist = norm(x_pos - center_pos)
    return dist^2 - sphere_set.radius^2
end

"""
Compute sphere-space navigation potential function using geometric mean.

ψ(x) = d²(x, x_goal) / [b_geom(x) + d^(2k)(x, x_goal)]^(1/k)

where b_geom(x) = (∏ᵢ bᵢ(x))^(1/n) is the geometric mean of obstacle functions.

The geometric mean prevents numerical overflow when there are many obstacles.
"""
function psi(x::Vector{Float64}, x_goal::Vector{Float64}, W::Workspace, 
            d::Function, k::Int)
    
    d_goal = d(x, x_goal)
    
    # At goal
    if d_goal < 1e-10
        return 0.0
    end
    
    d_goal_sq = d_goal^2
    d_goal_2k = d_goal^(2*k)
    
    n_obs = length(W.obstacles)
    
    # No obstacles - simple potential
    if n_obs == 0
        return d_goal_sq
    end
    
    # Compute geometric mean of b_i values using log-space arithmetic
    log_b_sum = 0.0
    for obs in W.obstacles
        bi = b_i(x, obs, d)
        
        if bi <= 0.0
            # Inside or touching obstacle - return large penalty
            return 1e6
        end
        
        log_b_sum += log(bi)
    end
    
    # Geometric mean: b_geom = exp(mean(log(b_i)))
    b_geom = exp(log_b_sum / n_obs)
    
    # Compute potential
    sum_term = b_geom + d_goal_2k
    denominator = sum_term^(1.0/k)
    potential = d_goal_sq / denominator
    
    # Clamp to reasonable range
    return clamp(potential, 0.0, 1e6)
end

"""
Navigation potential function φ(x) = ψ(x) in sphere space.

This formulation guarantees no local minima in sphere-space workspaces.

Parameters:
- W: Workspace containing obstacles
- x0: Current position [x, y] or [x, y, θ]
- xf: Goal position [x, y] or [x, y, θ]
- d: Distance metric function
- k: Sharpening parameter (default: 3, higher = steeper gradients near obstacles)

Returns: Scalar potential value at x0
"""
function NavigationPotentialFunction(W::Workspace, x0::Vector{Float64}, 
                                    xf::Vector{Float64}, d::Function; 
                                    k::Int=3, mu::Float64=1.0)
    return psi(x0, xf, W, d, k)
end

"""
Compute gradient of navigation potential function using numerical differentiation.

Parameters:
- W: Workspace
- x: Current position
- xf: Goal position
- d: Distance metric
- k: Sharpening parameter
- epsilon: Step size for numerical differentiation

Returns: Gradient vector ∇φ(x)
"""
function NavigationPotentialGradient(W::Workspace, x::Vector{Float64},
                                    xf::Vector{Float64}, d::Function;
                                    k::Int=3, mu::Float64=1.0, 
                                    epsilon::Float64=1e-3)
    grad = zeros(length(x))
    
    for i in 1:length(x)
        x_plus = copy(x)
        x_minus = copy(x)
        x_plus[i] += epsilon
        x_minus[i] -= epsilon
        
        phi_plus = NavigationPotentialFunction(W, x_plus, xf, d; k=k)
        phi_minus = NavigationPotentialFunction(W, x_minus, xf, d; k=k)
        
        grad[i] = (phi_plus - phi_minus) / (2 * epsilon)
    end
    
    return grad
end

"""
Plot navigation potential field as a heatmap.

Parameters:
- W: Workspace
- xf: Goal position
- d: Distance function
- k: Sharpening parameter (default: 3)
- resolution: Grid resolution (default: 100)
- max_potential: Maximum potential value to display (default: 50.0)
"""
function plot_navigation_potential(W::Workspace, xf::Vector{Float64}, 
                                  d::Function;
                                  k::Int=3,
                                  resolution::Int=100,
                                  max_potential::Float64=50.0)
    
    # Compute workspace bounds
    all_x = Float64[]
    all_y = Float64[]
    
    for obs in W.obstacles
        verts = vertices_list(obs)
        append!(all_x, [v[1] for v in verts])
        append!(all_y, [v[2] for v in verts])
    end
    
    push!(all_x, xf[1])
    push!(all_y, xf[2])
    
    margin = 0.2 * max(maximum(all_x) - minimum(all_x), 
                      maximum(all_y) - minimum(all_y))
    
    xlims = (minimum(all_x) - margin, maximum(all_x) + margin)
    ylims = (minimum(all_y) - margin, maximum(all_y) + margin)
    
    # Create grid
    x_range = range(xlims[1], xlims[2], length=resolution)
    y_range = range(ylims[1], ylims[2], length=resolution)
    
    # Compute potential at each grid point
    Z = zeros(resolution, resolution)
    valid_vals = Float64[]
    
    println("Computing potential field over $(resolution)×$(resolution) grid...")
    
    for (i, y) in enumerate(y_range)
        for (j, x) in enumerate(x_range)
            point = length(xf) >= 3 ? [x, y, 0.0] : [x, y]
            
            # Check if inside any obstacle
            inside = any(point[1:2] ∈ obs for obs in W.obstacles)
            
            if inside
                Z[i, j] = NaN
            else
                phi = log10(NavigationPotentialFunction(W, point, xf, d; k=k))
                Z[i, j] = min(phi, log10(max_potential))
                push!(valid_vals, phi)
            end
        end
    end
    
    # Print statistics
    println("\nPotential field statistics:")
    println("  Min: ", minimum(valid_vals))
    println("  Max: ", maximum(valid_vals))
    println("  Mean: ", mean(valid_vals))
    println("  Median: ", median(valid_vals))
    println("  Values > $(max_potential): ", count(v -> v > max_potential, valid_vals))
    
    # Create heatmap
    p = heatmap(x_range, y_range, Z,
                xlabel="x", ylabel="y",
                title="Navigation Potential Field (k=$k)",
                colorbar_title=L"\log \phi",
                aspect_ratio=:equal,
                color=:viridis)
    
    # Overlay obstacles
    for obs in W.obstacles
        verts = vertices_list(obs)
        verts_closed = vcat(verts, [verts[1]])
        xs = [v[1] for v in verts_closed]
        ys = [v[2] for v in verts_closed]
        plot!(p, xs, ys, linewidth=2, color=:red, label="", 
              fillalpha=0.3, fill=true, fillcolor=:red)
    end
    
    # Mark goal
    scatter!(p, [xf[1]], [xf[2]], marker=:star, markersize=10, 
             color=:yellow, markerstrokecolor=:black, markerstrokewidth=2,
             label="Goal")
    
    return p
end