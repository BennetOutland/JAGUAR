"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Navigation Potential Field Implementation
Based on Rimon & Koditschek (1992) - Exact Formulation
"""

# Includes
include("../DynamicPlanning.jl/src/workspace.jl")

# Usings 
using LinearAlgebra
using LazySets
import LazySets: overapproximate
using Plots
using Statistics

# Define a custom overapproximation method with tighter fit
function LazySets.overapproximate(S::VPolygon, ::Type{Ball2})
    verts = vertices_list(S)
    c = mean(verts)
    # Use minimum enclosing circle radius (tighter fit)
    r = maximum(norm(v - c) for v in verts)
    # Scale down to get tighter approximation
    r_tight = 0.7 * r  # Adjust this factor as needed (0.5-0.9)
    return Ball2(c, r_tight)
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
Compute b(x) = ∏ᵢ bᵢ(x)
Product of all obstacle functions.
"""
function b_product(x::Vector{Float64}, W::Workspace, d::Function)
    if isempty(W.obstacles)
        return 1.0
    end
    
    # Use log-space to prevent overflow
    log_b_sum = 0.0
    for obs in W.obstacles
        bi = b_i(x, obs, d)
        if bi <= 0.0
            return -Inf  # Inside obstacle
        end
        log_b_sum += log(bi)
    end
    
    return exp(log_b_sum)
end

"""
Compute b̄ᵢ(x) = ∏_{j≠i} bⱼ(x)
Product of all obstacle functions except obstacle i.
"""
function b_bar_i(x::Vector{Float64}, W::Workspace, i::Int, d::Function)
    if isempty(W.obstacles)
        return 1.0
    end
    
    log_b_sum = 0.0
    for (j, obs) in enumerate(W.obstacles)
        if j != i
            bi = b_i(x, obs, d)
            if bi <= 0.0
                return -Inf
            end
            log_b_sum += log(bi)
        end
    end
    
    return exp(log_b_sum)
end

"""
Compute ηᵢ(x) for obstacle i.
Default implementation: ηᵢ(x) = 1.0
"""
function eta_i(x::Vector{Float64}, obs, d::Function)
    return 1.0
end

"""
Compute νᵢ(x) = (1 + bᵢ(x))^(1/2) * rᵢ / d(x, xᵢ)
Scaling factor for transformation Tᵢ.
"""
function nu_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    bi = b_i(x, obs, d)
    
    # Use Euclidean distance to obstacle center (not d function)
    x_pos = x[1:2]
    dist = norm(x_pos - sphere_set.center)
    
    return sqrt(max(1.0 + bi, 0.0)) * sphere_set.radius / max(dist, 1e-10)
end

"""
Compute switching function σᵢ(x, μ) = ηᵢ(x)b̄ᵢ(x) / [μ·bᵢ(x) + ηᵢ(x)b̄ᵢ(x)]
Determines blending between obstacle transformations.
"""
function sigma_i(x::Vector{Float64}, W::Workspace, i::Int, d::Function, mu::Float64)
    obs = W.obstacles[i]
    bi = b_i(x, obs, d)
    b_bar = b_bar_i(x, W, i, d)
    eta = eta_i(x, obs, d)
    
    numerator = eta * b_bar
    denominator = mu * bi + eta * b_bar
    
    return numerator / denominator
end

"""
Compute transformation Tᵢ(x) = νᵢ(x)(x - xᵢ) + pᵢ
where xᵢ is the obstacle center and pᵢ = xᵢ.
Returns same dimensionality as input x.
"""
function T_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    nu = nu_i(x, obs, d)
    
    # Extract 2D position
    x_pos = x[1:2]
    center_pos = sphere_set.center
    
    # Tᵢ(x) = νᵢ(x)(x - xᵢ) + pᵢ, with pᵢ = xᵢ
    transformed_pos = nu * (x_pos - center_pos) + center_pos
    
    # Return with same dimensionality as input
    if length(x) >= 3
        return [transformed_pos[1], transformed_pos[2], x[3]]
    else
        return transformed_pos
    end
end

"""
Compute the mapping h(x) from sphere space to star space:
h(x) = σ_goal(x, μ)T_goal(x) + ∑ᵢ σᵢ(x, μ)Tᵢ(x)
where T_goal(x) = x.
"""
function h_mapping(x::Vector{Float64}, W::Workspace, d::Function, mu::Float64)
    n_dim = length(x)
    n_obs = length(W.obstacles)
    
    if n_obs == 0
        return x  # No obstacles, identity mapping
    end
    
    # Compute all switching functions
    sigmas = zeros(n_obs)
    sigma_sum = 0.0
    
    for (i, obs) in enumerate(W.obstacles)
        sigmas[i] = sigma_i(x, W, i, d, mu)
        sigma_sum += sigmas[i]
    end
    
    # σ_goal = 1 - ∑ᵢ σᵢ
    sigma_goal = 1.0 - sigma_sum
    
    # Apply transformations
    transformed = zeros(n_dim)
    for (i, obs) in enumerate(W.obstacles)
        transformed += sigmas[i] * T_i(x, obs, d)
    end
    
    # T_goal(x) = x
    return sigma_goal * x + transformed
end

"""
Compute sphere-space potential function:
ψ(x) = d²(x, x_goal) / [b(x) + d^(2k)(x, x_goal)]^(1/k)
where b(x) = ∏ᵢ bᵢ(x).
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
    
    # Compute b(x) = ∏ᵢ bᵢ(x)
    b = b_product(x, W, d)
    
    if b == -Inf || b <= 0.0
        # Inside or touching obstacle
        return 1e6
    end
    
    # ψ(x) = d²(x, x_goal) / [b(x) + d^(2k)(x, x_goal)]^(1/k)
    sum_term = b + d_goal_2k
    denominator = sum_term^(1.0/k)
    potential = d_goal_sq / denominator
    
    return clamp(potential, 0.0, 1e6)
end

"""
Navigation potential function φ(x) = ψ(h(x)).

This is the complete navigation potential field with guaranteed convergence
to the goal without local minima in star-space workspaces.

Parameters:
- W: Workspace containing obstacles
- x0: Current position [x, y] or [x, y, θ]
- xf: Goal position [x, y] or [x, y, θ]
- d: Distance metric function
- k: Sharpening parameter (default: 3)
- mu: Tuning parameter for switching functions (default: 1.0)
- use_star_space: If true, use star-space mapping h(x); if false, use sphere-space only

Returns: Scalar potential value at x0
"""
function NavigationPotentialFunction(W::Workspace, x0::Vector{Float64}, 
                                    xf::Vector{Float64}, d::Function; 
                                    k::Int=3, mu::Float64=1.0,
                                    use_star_space::Bool=true)
    if use_star_space && !isempty(W.obstacles)
        # Star-space formulation: φ(x) = ψ(h(x))
        h_x = h_mapping(x0, W, d, mu)
        return psi(h_x, xf, W, d, k)
    else
        # Sphere-space formulation: φ(x) = ψ(x)
        return psi(x0, xf, W, d, k)
    end
end

"""
Compute gradient of navigation potential function using numerical differentiation.

Parameters:
- W: Workspace
- x: Current position
- xf: Goal position
- d: Distance metric
- k: Sharpening parameter
- mu: Tuning parameter
- epsilon: Step size for numerical differentiation
- use_star_space: Whether to use star-space mapping

Returns: Gradient vector ∇φ(x)
"""
function NavigationPotentialGradient(W::Workspace, x::Vector{Float64},
                                    xf::Vector{Float64}, d::Function;
                                    k::Int=3, mu::Float64=1.0, 
                                    epsilon::Float64=1e-3,
                                    use_star_space::Bool=true)
    grad = zeros(length(x))
    
    for i in 1:length(x)
        x_plus = copy(x)
        x_minus = copy(x)
        x_plus[i] += epsilon
        x_minus[i] -= epsilon
        
        phi_plus = NavigationPotentialFunction(W, x_plus, xf, d; k=k, mu=mu, use_star_space=use_star_space)
        phi_minus = NavigationPotentialFunction(W, x_minus, xf, d; k=k, mu=mu, use_star_space=use_star_space)
        
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
- mu: Tuning parameter (default: 1.0)
- resolution: Grid resolution (default: 100)
- max_potential: Maximum potential value to display (default: 50.0)
- use_star_space: Whether to use star-space mapping (default: true)
"""
function plot_navigation_potential(W::Workspace, xf::Vector{Float64}, 
                                  d::Function;
                                  k::Int=3,
                                  mu::Float64=1.0,
                                  resolution::Int=100,
                                  max_potential::Float64=50.0,
                                  use_star_space::Bool=true)
    
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
    
    space_type = use_star_space ? "Star-Space" : "Sphere-Space"
    println("Computing $(space_type) potential field over $(resolution)×$(resolution) grid...")
    
    for (i, y) in enumerate(y_range)
        for (j, x) in enumerate(x_range)
            point = length(xf) >= 3 ? [x, y, 0.0] : [x, y]
            
            # Check if inside any obstacle
            inside = any(point[1:2] ∈ obs for obs in W.obstacles)
            
            if inside
                Z[i, j] = NaN
            else
                phi = NavigationPotentialFunction(W, point, xf, d; k=k, mu=mu, use_star_space=use_star_space)
                Z[i, j] = min(phi, max_potential)
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
    title_str = "Navigation Potential Field ($(space_type), k=$k, μ=$mu)"
    
    p = heatmap(x_range, y_range, Z,
                xlabel="x", ylabel="y",
                title=title_str,
                colorbar_title="φ",
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