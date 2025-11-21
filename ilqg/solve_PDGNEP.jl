"""
Author: Bennet Outland
Organization: CU Boulder
License/Control: MIT
"""

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

# Create the struct
struct CBR <: ControlSystem{Δt,nx,nu} end


"""
Dynamics for a multiple unicycles from a stacked state and control vector. Assumes pursuer; evader
"""
function f(x, u)
    # Derivatives
    dx = []

    # Loop through agents
    for i ∈ eachindex(m_vec)

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
Compute log barrier penalty for inequality constraints g(x, u) ≤ 0.
Returns -μ * sum(log(-g_i)) for all constraints.
Assumes g_i < 0 for feasible points (strict interior required).
"""
function log_barriers(g::AbstractVector, μ::Real)
    barrier = zero(promote_type(eltype(g), typeof(μ)))
    @inbounds @simd for i in eachindex(g)
        barrier -= μ * log(-g[i])
    end
    return barrier
end


#+=========================================================================+
#                              PARAMETERS
#+=========================================================================+
"""
WIP
"""
function SolvePDGNEP(N, x0_stack, X_stack, U_stack, T, dt, t0, constraints, potentials; ρ=1.0)
    # Extract agent and stacked diemsnions 
    nx = length(x0_stack)
    nxi = nx / N
    nu = length(U_stack[1])
    nui = nu / N

    # Player Costs NOTE: hardcoded for 2 players atm
    pursuer_cost = FunctionPlayerCost((g, x, u, t) -> (potentials[1] + log_barriers(constraints[1], ρ)))
    evader_cost = FunctionPlayerCost((g, x, u, t) -> (potentials[2] + log_barriers(constraints[2], ρ)))
    costs = Tuple([pursuer_cost, evader_cost])

    # Defining the game 
    player_inputs = (SVector(1, 2), SVector(3, 4))
    g = GeneralGame(T, player_inputs, dynamics, costs)

    # Type Conversion/Setup for Warm Starts 
    init_traj = SystemTrajectory{dt}(X_stack, U_stack, t0)
    γ0 = dummy_strategy(g)

    # Solve 
    solver = iLQSolver(g)
    converged, trajectory, strategies = solve!(init_traj, γ0, g, solver, SVector{nx}(x0))

    return trajectory
end 

