"""
Author: Bennet Outland
Organization: CU Boulder
License/Control: MIT
"""

# Includes:
# include("../DynamicPlanning.jl/src/models.jl")


# Usings: 
using ReachabilityAnalysis, LazySets, Plots

# Control input sets: v ∈ [-0.5, 0.5], ω ∈ [-π/3, π/3]
U = Hyperrectangle(low=[-0.5, -pi/3], high=[0.5, pi/3])
U_P = Hyperrectangle(low=[-1.0, -pi/6], high=[1.0, pi/6])

# Initial configuration uncertainty (x, y, θ)
X0 = Hyperrectangle(low=[-1e-4, -1e-4, -1e-4], high=[1e-4, 1e-4, 1e-4])

# Augmented dynamics: state = [x, y, θ, v, ω]
# Controls v and ω are modeled as constant inputs via zero derivatives
@taylorize function unicycle!(dx, x, p, t)
    dx[1] = x[4] * cos(x[3])  # ẋ = v cos(θ)
    dx[2] = x[4] * sin(x[3])  # ẏ = v sin(θ)
    dx[3] = x[5]              # θ̇ = ω
    dx[4] = zero(x[4])        # v̇ = 0 (constant control)
    dx[5] = zero(x[5])        # ω̇ = 0 (constant control)
end

# Construct augmented initial sets: configuration × controls
X0_full = X0 × U      # Evader
X0_full_P = X0 × U_P  # Pursuer

# Define initial value problems
sys = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_full)
sys_P = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_full_P)

# Compute reachable sets over time horizon
T = 0.05
sol = solve(sys, tspan=(0.0, T), alg=TMJets(orderT=10, orderQ=2, abstol=1e-12, maxsteps=1000))

plot(sol, vars=(1, 2), alpha=0.3, lw=0.5)


# sol_P = solve(sys_P, tspan=(0.0, T), alg=TMJets(orderT=10, orderQ=2, abstol=1e-12))

# Visualize projection onto workspace (x, y)
# plot(sol, vars=(1, 2), color=:lightblue, alpha=0.5, lw=0.0, label="Evader")
# # plot!(sol_P, vars=(1, 2), color=:red, alpha=0.5, lw=0.0, label="Pursuer")
# xlabel!("x")
# ylabel!("y")
# title!("Kinematic Unicycle Reachable Set")
