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
using Plots
using Statistics

# ============================================================================
# Distance Functions
# ============================================================================

distance(x1, x2) = norm(x1 - x2)

# ============================================================================
# Boundary Functions (b_i)
# ============================================================================

function b_sphere(x, ball::Ball2)
    """Returns b_i(x) = ||x - x_i||² - r_i²"""
    return norm(x - ball.center)^2 - ball.radius^2
end

function b_workspace(x, ball::Ball2)
    """Returns b_0(x) = r_0² - ||x||²"""
    return ball.radius^2 - norm(x)^2
end

function compute_b_product(x, obstacles, workspace)
    """Returns b(x) = b_0(x) * ∏ b_i(x)"""
    b_prod = b_workspace(x, workspace)
    for obs in obstacles
        b_prod *= b_sphere(x, obs)
    end
    return b_prod
end

# ============================================================================
# Sphere Space Navigation Function (ψ)
# ============================================================================

function psi(x, x_goal, obstacles, workspace, k)
    """
    ψ(x) = d²(x, x_goal) / [b(x) + d^(2k)(x, x_goal)]^(1/k)
    """
    d_goal = distance(x, x_goal)
    b_val = compute_b_product(x, obstacles, workspace)
    
    if b_val <= 0
        return NaN  # Invalid region
    end
    
    return d_goal^2 / (b_val + d_goal^(2*k))^(1/k)
end

# ============================================================================
# Star World Transformation Components
# ============================================================================

function compute_nu_i(x, obs, r_i)
    """
    ν_i(x) = √(1 + b_i(x)) * r_i / d(x, x_i)
    """
    b_i = b_sphere(x, obs)
    d_val = distance(x, obs.center)
    
    # Handle singularities
    if d_val < 1e-10
        d_val = 1e-10
    end
    
    if 1 + b_i < 0
        return 1.0  # Fallback for points inside obstacle
    end
    
    return sqrt(1 + b_i) * r_i / d_val
end

function compute_T_i(x, obs, p_i, r_i)
    """
    T_i(x) = ν_i(x)(x - x_i) + p_i
    """
    x_i = obs.center
    nu_i = compute_nu_i(x, obs, r_i)
    return nu_i * (x - x_i) + p_i
end

# ============================================================================
# Switching Functions (σ)
# ============================================================================

# ============================================================================
# Switching Functions (σ) - FIXED VERSION
# ============================================================================

function compute_b_bar_i(x, i, obstacles, workspace)
    """
    Compute b̄_i(x) = b_0(x) * ∏_{j≠i} b_j(x)
    Returns NaN if any component is ≤ 0 (invalid region)
    """
    b_bar = b_workspace(x, workspace)
    
    if b_bar <= 0
        return NaN
    end
    
    for (j, obs) in enumerate(obstacles)
        if j != i
            b_j = b_sphere(x, obs)
            if b_j <= 0
                return NaN
            end
            b_bar *= b_j
        end
    end
    
    return b_bar
end

function compute_sigma_i(x, i, obstacles, workspace, μ)
    """
    σ_i(x, μ) = (η_i * b̄_i) / (μ * b_i + η_i * b̄_i)
    
    Returns 0 if point is not in free space
    """
    b_i = b_sphere(x, obstacles[i])
    
    # Point inside obstacle i
    if b_i <= 0
        return 0.0
    end
    
    b_bar_i = compute_b_bar_i(x, i, obstacles, workspace)
    
    # Point in invalid region (inside another obstacle or outside workspace)
    if isnan(b_bar_i) || b_bar_i <= 0
        return 0.0
    end
    
    η_i = 1.0
    numerator = η_i * b_bar_i
    denominator = μ * b_i + η_i * b_bar_i
    
    return numerator / denominator
end

function compute_sigma_goal(x, obstacles, workspace, μ)
    """
    σ_goal = 1 - ∑ σ_i
    
    Only defined in free space
    """
    # First check if we're in free space
    if compute_b_product(x, obstacles, workspace) <= 0
        return 0.0
    end
    
    sum_sigma = 0.0
    for i in 1:length(obstacles)
        sum_sigma += compute_sigma_i(x, i, obstacles, workspace, μ)
    end
    
    return 1.0 - sum_sigma
end

# ============================================================================
# Diffeomorphism h: Star World → Sphere World
# ============================================================================

function h_mapping(x, obstacles, workspace, μ, p_vals, r_vals)
    """
    h(x) = σ_goal(x, μ) * T_goal(x) + ∑ σ_i(x, μ) * T_i(x)
    
    Only valid in free space - returns x for invalid regions
    """
    # Check free space
    if compute_b_product(x, obstacles, workspace) <= 0
        return x  # Or return NaN if you prefer
    end
    
    σ_goal = compute_sigma_goal(x, obstacles, workspace, μ)
    result = σ_goal * x  # T_goal is identity
    
    for (i, obs) in enumerate(obstacles)
        σ_i = compute_sigma_i(x, i, obstacles, workspace, μ)
        T_i = compute_T_i(x, obs, p_vals[i], r_vals[i])
        result += σ_i * T_i
    end
    
    return result
end

# ============================================================================
# Star World Navigation Function (φ)
# ============================================================================

function phi(x, x_goal, obstacles, workspace, k, μ)
    """
    φ(x) = ψ(h(x))
    
    Navigation function for star worlds
    """
    # Use identity mapping: p_i = obstacle centers, r_i = obstacle radii
    p_vals = [obs.center for obs in obstacles]
    r_vals = [obs.radius for obs in obstacles]
    
    h_x = h_mapping(x, obstacles, workspace, μ, p_vals, r_vals)
    
    return psi(h_x, x_goal, obstacles, workspace, k)
end

# ============================================================================
# Conversion Utilities
# ============================================================================

to_Ball2(E::Ellipsoid) = Ball2(E.center, sqrt(maximum(eigvals(E.shape_matrix))))

function to_Ball2(P::VPolygon)
    c = sum(P.vertices) / length(P.vertices)
    r = maximum(norm(v - c) for v in P.vertices)
    return Ball2(c, r)
end

to_Ball2(B::Ball2) = B



