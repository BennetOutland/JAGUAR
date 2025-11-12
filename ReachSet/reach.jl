"""
Author: Bennet Outland
Organization: CU Boulder
License/Control: MIT
"""

# Includes:


# Usings: 
using ReachabilityAnalysis, LazySets, Plots

# Control bounds: v ∈ [0, 1], ω ∈ [-1, 1]
U = Hyperrectangle(low=[-0.5, -pi/3], high=[0.5, pi/3])
U_P = Hyperrectangle(low=[-1.0, -pi/6], high=[1.0, pi/6])

# Initial set
X0 = Hyperrectangle(low=[0.0, 0.0, 0.0], high=[0.001, 0.001, 0.001])

# Kinematic unicycle dynamics: state is [x, y, theta, v, ω]
@taylorize function unicycle!(dx, x, p, t)
    dx[1] = x[4] * cos(x[3])  # x' = v*cos(theta)
    dx[2] = x[4] * sin(x[3])  # y' = v*sin(theta)
    dx[3] = x[5]              # theta' = ω
    dx[4] = zero(x[4])        # v' = 0 (constant velocity input)
    dx[5] = zero(x[5])        # ω' = 0 (constant angular velocity input)
end

# Define the initial set (now 5-dimensional: [x, y, theta, v, ω])
X0_full = X0 × U
X0_full_P = X0 × U_P

# Define the system
sys = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_full)

# Compute reachable set
sol = solve(sys, tspan=(0.0, 0.25), alg=TMJets())


sys_P = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_full_P)
sol_P = solve(sys_P, tspan=(0.0, 0.25), alg=TMJets())


# Plot projection onto (x, y)
plot(sol, vars=(1, 2), color=:lightblue, alpha=0.5, lw=0.0, label="Evader")
plot!(sol_P, vars=(1, 2), color=:red, alpha=0.5, lw=0.0, label="Pursuer")
xlabel!("x")
ylabel!("y")
title!("Kinematic Unicycle Reachable Set")