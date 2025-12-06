"""
Author: Bennet Outland
Organization: CU Boulder
License/Control: MIT
"""

# Includes 
include("../DynamicPlanning.jl/src/constraints.jl")

# Imports
import iLQGames: dx

# Usings
using iLQGames
using Plots
using LinearAlgebra
using StaticArrays
using LaTeXStrings


#+=========================================================================+
#                              DYNAMICS
#+=========================================================================+

# # Create the struct
nx = 6
nxi = 3
nu = 4
nui = 2
Δt = 0.1

struct CBR <: ControlSystem{Δt,nx,nu} end


"""
Dynamics for a multiple unicycles from a stacked state and control vector. Assumes pursuer; evader
"""
function f(x, u)
    # Derivatives
    dx = []

    # Loop through agents
    for i ∈ 1:2

        # Variables 
        xi = x[nxi*(i-1)+1:nxi*i]
        ui = u[nui*(i-1)+1:nui*i]

        # Push
        push!(dx, [ui[1] * cos(xi[3]); ui[1] * sin(xi[3]); ui[2]])

    end

    # Concatenate and return
    return SVector{nx}(vcat(dx...)) 
end


"""
x ∈ ℝ^3xN
u ∈ ℝ^2xN

These are the dynamics for a set of unicycles. Assumes pursuer; evader
"""
dx(cs::CBR, x, u, t) = f(x, u)
dynamics = CBR()


#+=========================================================================+
#                          HELPER FUNCTIONS
#+=========================================================================+


function dummy_strategy(g)
    nx = n_states(g)
    nu = n_controls(g)
    h  = horizon(g)

    # P: nu × nx feedback gain matrix
    # α: nu-vector feedforward
    zero_strategy = AffineStrategy(zeros(SMatrix{nu,nx}), zeros(SVector{nu}))

    return SizedVector{h}(fill(zero_strategy, h))
end


"""
Log barrier penalty function with element-wise upper/lower constraints.
lcon and ucon are vectors with bounds for each element.
"""
function log_barrier(q, lcon::Vector, ucon::Vector, μ)
    barrier = 0.0
    for (i, qi) ∈ enumerate(q)
        barrier -= μ * (log(ucon[i] - qi) + log(qi - lcon[i]))
    end
    return barrier
end


"""
Softer log barrier with quadratic transition near bounds
"""
function soft_log_barrier(x, lcon, ucon, μ, δ=0.1)
    # Handle scalar bounds
    if isa(lcon, Real)
        lcon = fill(lcon, length(x))
    end
    if isa(ucon, Real)
        ucon = fill(ucon, length(x))
    end
    
    barrier = 0.0
    for (i, xi) ∈ enumerate(x)
        # Distance to bounds
        d_upper = ucon[i] - xi
        d_lower = xi - lcon[i]
        
        # Soft barrier: quadratic near bounds, log in interior
        if d_upper < δ
            barrier += μ * (d_upper - δ)^2 / (2*δ)
        else
            barrier -= μ * log(d_upper)
        end
        
        if d_lower < δ
            barrier += μ * (d_lower - δ)^2 / (2*δ)
        else
            barrier -= μ * log(d_lower)
        end
    end
    return barrier
end

"""
Penalty function for upper/lower constraints on a variable.
Supports both scalar and vector bounds with NaN safety checks.
"""
function pf(x, lcon, ucon, μ)
    # Handle scalar bounds (broadcast to all elements)
    if isa(lcon, Real)
        lcon = fill(lcon, length(x))
    end
    if isa(ucon, Real)
        ucon = fill(ucon, length(x))
    end
    
    p = 0.0
    for (i, xi) ∈ enumerate(x)
        # Check for NaN in input
        if isnan(xi) || isnan(lcon[i]) || isnan(ucon[i])
            @warn "NaN detected in penalty function at index $i: xi=$xi, lcon=$lcon[i], ucon=$ucon[i]"
            return Inf  # Return large penalty for NaN
        end
        
        # Upper violation: xi > ucon[i]
        upper_violation = max(0.0, xi - ucon[i])
        # Lower violation: xi < lcon[i]
        lower_violation = max(0.0, lcon[i] - xi)
        
        p += μ * (upper_violation^2.0 + lower_violation^2.0)
    end
    
    return p
end

#+=========================================================================+
#                              PARAMETERS
#+=========================================================================+
"""
WIP
"""
function SolvePDGNEP(N, x0_stack, X_stack, U_stack, T, dt, t0, constraints, potentials; ρ=1.0)
    # Extract constraints 
    p_ul = constraints[1][1].lower
    p_uu = constraints[1][1].upper
    p_xl = constraints[1][2].lower
    p_xu = constraints[1][2].upper

    e_ul = constraints[2][1].lower
    e_uu = constraints[2][1].upper
    e_xl = constraints[2][2].lower
    e_xu = constraints[2][2].upper
     
    # Player Costs
    pursuer_cost = FunctionPlayerCost((g, x, u, t) -> norm(x[1:2] - x[4:5]) + potentials[1](x[1:2]) +
    soft_log_barrier(u[1:2], p_ul, p_uu, ρ) + soft_log_barrier(x[1:3], p_xl, p_xu, ρ))
    evader_cost = FunctionPlayerCost((g, x, u, t) -> -norm(x[1:2] - x[4:5]) + potentials[2](x[4:5]) +
    soft_log_barrier(u[3:4], e_ul, e_uu, ρ) + soft_log_barrier(x[4:6], e_xl, e_xu, ρ))
    costs = Tuple([pursuer_cost, evader_cost])


    # Defining the game 
    player_inputs = (SVector(1, 2), SVector(3, 4))
    g = GeneralGame(T, player_inputs, dynamics, costs)

    # Type Conversion/Setup for Warm Starts 
    # init_traj = SystemTrajectory{dt}(X_stack, U_stack, t0)
    # γ0 = dummy_strategy(g)

    # Solve 
    print("Solving...")
    solver = iLQSolver(g)
    converged, trajectory, strategies = iLQGames.solve(g, solver, SVector{6}(x0_stack))
    println("Complete. Convergence: ", converged)
    # converged, trajectory, strategies = solve!(init_traj, γ0, g, solver, SVector{nx}(x0))

    return trajectory
end 

