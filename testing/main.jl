"""
Author: Bennet Outland
Organization: CU Boulder
License/Control: MIT
"""

# Usings 
using LinearAlgebra
using DifferentialEquations 
using LazySets
using Plots

# Includes 
include("reduced_jaguar.jl")


# ============================================================================================================= #
#                                           CONFIGURATION
# ============================================================================================================= #

# Define initial conditions
x0_pursuer = [0.0, -5.0, 0.0]
# x0_evader = [1.5, -2.5, 0.0]
x0_evader = [-2.0, 7.0, 0.0]

# Timing
T = 100.0
dt = 0.5

# Define the workspace
𝒲 = workspace([0.0, 0.0], 10.0)

# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)
add_obstacles!(𝒲, 𝒪)

# Define the chassis
l = 0.5 # 0.2
w = 0.25 # 0.1
chassis = VPolygon([[-w/2, -l/2], [-w/2, l/2], [w/2, l/2], [w/2, -l/2]])

# P-E constraints 
α_PE = 1.25
β_PE = 0.75

# Define the pursuer 
pursuer = robot(x0_pursuer, 3, 2, chassis)
add_dynamics!(pursuer, unicycle!)
add_box_constraint!(pursuer, [-0.25*α_PE, (-pi/3)*β_PE], [0.25*α_PE, (pi/3)*β_PE], :u)
add_box_constraint!(pursuer, [-10, -10, -pi/3], [10, 10, pi/3], :x)

# Define the evader
evader = robot(x0_evader, 3, 2, chassis)
add_dynamics!(evader, unicycle!)
add_box_constraint!(evader, [-0.25/α_PE, (-pi/3)/β_PE], [0.25/α_PE, (pi/3)/β_PE], :u)
add_box_constraint!(evader, [-10, -10, -pi/3], [10, 10, pi/3], :x)

# Add players to the workspace
add_robots!(𝒲, [pursuer, evader])
# ============================================================================================================= #
#                                      SOLVE WITH iLQGames
# ============================================================================================================= #

# Set tuning parameters
d_star = 2.0 
zeta   = 1.0
Q_star = 2.0
eta    = 60.0

# Workspace Ball 
W_ball = to_Ball2(𝒲.bounds)

# Make the potentials
Φ_pursuer = PotentialFunction(d_star, zeta, Q_star, eta, pursuer.X[end], zeros(2), 𝒲.obstacles, W_ball)
Φ_evader = PotentialFunction(d_star, zeta, Q_star, eta, evader.X[end], zeros(2), 𝒲.obstacles, W_ball)

traj = SolvePDGNEP(2, vcat(x0_pursuer, x0_evader), 0.0, 0.0, Int(T/dt), 0.0, 0.0, [pursuer.constraints, evader.constraints], [Φ_pursuer, Φ_evader], ρ=1.0)

# ============================================================================================================= #
#                                           PLOTTING
# ============================================================================================================= #

# Get states and controls into the correct shape s
sol_X = [[traj.x[i][1:3] for i ∈ eachindex(traj.x)], [traj.x[i][4:6] for i ∈ eachindex(traj.x)]]
sol_U = [[traj.u[i][1:2] for i ∈ eachindex(traj.u)], [traj.u[i][3:4] for i ∈ eachindex(traj.u)]]

# Make a solution object 
sol = Solution([true], sol_X, sol_U, [[]])

# Plot it
plot(𝒲, sol)