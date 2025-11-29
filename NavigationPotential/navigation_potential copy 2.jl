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
Compute b_i(x) = d²(x, x_i) - r_i²
Handles mixed dimensionality (3D poses, 2D obstacle centers).

Parameters:
- x: Current pose [x, y] or [x, y, θ]
- obs: Obstacle (polygon)
- d: Distance function

Returns: b_i value (≤ 0 inside obstacle)
"""
function b_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    
    # Extract 2D position and use Euclidean distance
    x_pos = x[1:2]
    center_pos = sphere_set.center
    
    dist = norm(x_pos - center_pos)
    result = dist^2 - sphere_set.radius^2
    
    # Clamp very negative values
    if result < -1e10
        return -1e10
    end
    
    return result
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
Compute b̄ᵢ(x) = ∏_{j≠i} bⱼ(x)
Product of all obstacle functions except obstacle i.
"""
function b_bar_i(x::Vector{Float64}, W::Workspace, i::Int, d::Function)
    prod = 1.0
    for (j, obs) in enumerate(W.obstacles)
        if j != i
            prod *= b_i(x, obs, d)
        end
    end
    return prod
end

"""
Compute ηᵢ(x) for obstacle i.
Default implementation: ηᵢ(x) = 1.0 (can be modified for tuning)
"""
function eta_i(x::Vector{Float64}, obs, d::Function)
    return 1.0
end

# """
# Compute νᵢ(x) = (1 + bᵢ(x))^(1/2) * rᵢ / d(x, xᵢ)
# Scaling factor for transformation Tᵢ.
# """
# function nu_i(x::Vector{Float64}, obs, d::Function)
#     sphere_set = overapproximate(obs, Ball2)
#     bi = b_i(x, obs, d)
#     dist = d(x, sphere_set.center)
#     return sqrt(1.0 + bi) * sphere_set.radius / dist
# end

"""
Compute νᵢ(x) = (1 + bᵢ(x))^(1/2) * rᵢ / d(x, xᵢ)
Scaling factor for transformation Tᵢ.
Fixed to use Euclidean distance for obstacle proximity.
"""
function nu_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    bi = b_i(x, obs, d)
    
    # Use Euclidean distance to obstacle center (2D positions only)
    x_pos = x[1:2]
    dist = norm(x_pos - sphere_set.center)
    
    return sqrt(max(1.0 + bi, 0.0)) * sphere_set.radius / max(dist, 1e-10)
end

"""
Compute switching function σᵢ(x, μ) = ηᵢ(x)b̄ᵢ(x) / [μ(x)bᵢ(x) + ηᵢ(x)b̄ᵢ(x)]
Determines blending between obstacle transformations.
"""
function sigma_i(x::Vector{Float64}, W::Workspace, i::Int, d::Function, mu::Float64)
    obs = W.obstacles[i]
    bi = b_i(x, obs, d)
    b_bar = b_bar_i(x, W, i, d)
    eta = eta_i(x, obs, d)
    
    numerator = eta * b_bar
    denominator = mu * bi + eta * b_bar
    
    # Clamp to avoid numerical issues
    return clamp(numerator / denominator, 0.0, 1.0)
end

"""
Compute transformation T_i(x) with proper dimension handling.
Returns same dimensionality as input x.
"""
function T_i(x::Vector{Float64}, obs, d::Function)
    sphere_set = overapproximate(obs, Ball2)
    nu = nu_i(x, obs, d)
    
    # Extract 2D position
    x_pos = x[1:2]
    center_pos = sphere_set.center
    
    # Compute transformed 2D position
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
    for (i, obs) in enumerate(W.obstacles)
        sigmas[i] = sigma_i(x, W, i, d, mu)
    end
    
    # Normalize switching functions to sum to at most 1
    sigma_sum = sum(sigmas)
    
    if sigma_sum > 1.0
        # Normalize so they sum to 1
        sigmas ./= sigma_sum
        sigma_sum = 1.0
    end
    
    sigma_goal = 1.0 - sigma_sum
    
    # Apply transformations
    transformed = zeros(n_dim)
    for (i, obs) in enumerate(W.obstacles)
        transformed += sigmas[i] * T_i(x, obs, d)
    end
    
    # Final mapping
    return sigma_goal * x + transformed
end

"""
Compute sphere-space potential function:
ψ(x) = d²(x, x_goal) / [b(x) + d^(2k)(x, x_goal)]^(1/k)
where b(x) = ∏ᵢ bᵢ(x).
"""
function psi(x::Vector{Float64}, x_goal::Vector{Float64}, W::Workspace, 
                      d::Function, k::Int; epsilon::Float64=1e-10)
    d_goal = d(x, x_goal)
    
    # Handle case where we're at the goal
    if d_goal < epsilon
        return 0.0
    end
    
    d_goal_sq = d_goal^2
    d_goal_2k = d_goal^(2*k)
    
    # Compute b_product in log space to avoid overflow
    log_b_sum = 0.0
    for obs in W.obstacles
        bi = b_i(x, obs, d)
        if bi > 0
            log_b_sum += log(bi)
        else
            # Point is inside or very close to an obstacle
            return 1e10  # Large penalty
        end
    end
    b_prod = exp(log_b_sum)
    
    # If b_product is huge compared to d_goal_2k, use approximation
    # When b >> d^(2k): ψ ≈ d²/b^(1/k)
    if b_prod > 1e6 * d_goal_2k
        # Use log-space arithmetic
        log_potential = 2*log(d_goal) - log_b_sum / k
        potential = exp(log_potential)
        return clamp(potential, 0.0, 1e6)
    end
    
    # Standard computation when values are reasonable
    sum_term = b_prod + d_goal_2k
    denominator = sum_term^(1.0/k)
    potential = d_goal_sq / denominator
    
    return clamp(potential, 0.0, 1e6)
end

"""
Navigation potential function φ(x) = ψ(h(x)).
This is the complete navigation potential field with guaranteed convergence
to the goal without local minima.

Parameters:
- W: Workspace containing obstacles
- x0: Current position
- xf: Goal position  
- d: Distance metric function
- k: Sharpening parameter (default: 3)
- mu: Tuning parameter for switching functions (default: 1.0)

Returns: Scalar potential value at x0
"""
function NavigationPotentialFunction(W::Workspace, x0::Vector{Float64}, 
                                    xf::Vector{Float64}, d::Function; 
                                    k::Int=3, mu::Float64=1.0)
    # Map from sphere space to star space
    h_x = h_mapping(x0, W, d, mu)
    
    # Compute potential in star space
    return psi(h_x, xf, W, d, k)
end

"""
Compute gradient of navigation potential function.
Uses numerical differentiation with central differences.

Parameters:
- W: Workspace
- x: Current position
- xf: Goal position
- d: Distance metric
- k: Sharpening parameter
- mu: Tuning parameter
- epsilon: Step size for numerical differentiation

Returns: Gradient vector ∇φ(x)
"""
function NavigationPotentialGradient(W::Workspace, x::Vector{Float64},
                                     xf::Vector{Float64}, d::Function;
                                     k::Int=3, mu::Float64=1.0, 
                                     epsilon::Float64=1e-6)
    grad = zeros(length(x))
    
    for i in 1:length(x)
        x_plus = copy(x)
        x_minus = copy(x)
        x_plus[i] += epsilon
        x_minus[i] -= epsilon
        
        phi_plus = NavigationPotentialFunction(W, x_plus, xf, d; k=k, mu=mu)
        phi_minus = NavigationPotentialFunction(W, x_minus, xf, d; k=k, mu=mu)
        
        grad[i] = (phi_plus - phi_minus) / (2 * epsilon)
    end
    
    return grad
end



"""
Wrap angle to [-π, π].

Parameters:
- θ: Angle in radians

Returns: Wrapped angle in [-π, π]
"""
function wrap_angle(θ::Float64)
    return mod(θ + π, 2π) - π
end


"""
Compute the approximate SE(2) distance using weighted Euclidean metric.

d(x₁, x₂) ≈ √(c₁||t₁ - t₂||² + c₂·wrap(θ₂ - θ₁)²)

This approximation is computationally efficient and works well when
rotational differences are small or when exact geodesic distances 
are not critical.

Parameters:
- x1: First pose as [x, y, θ] or [x, y]
- x2: Second pose as [x, y, θ] or [x, y]  
- c1: Weight for translational component (default: 1.0)
- c2: Weight for rotational component (default: 1.0)

Returns: Scalar distance value
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