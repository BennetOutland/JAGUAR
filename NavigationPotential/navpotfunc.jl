"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../DynamicPlanning.jl/src/workspace.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../NavigationPotential/artificial_potential.jl")

# Usings 
using LaTeXStrings
using Plots

# Define the workspace
𝒲 = workspace([0.0, 0.0], 10.0)

# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)
add_obstacles!(𝒲, 𝒪)

# Define and add obstacles 
# 𝒪 = [
#     circle([2.0, 2.0], 1.0),
#     circle([-2.0, -2.0], 1.0),
# ] 
# add_obstacles!(𝒲, 𝒪)


# Convert to balls
𝒲_ball = to_Ball2(𝒲.bounds)
# obstacles_ball = [to_Ball2(obs) for obs in 𝒲.obstacles]

# Define goal
x_goal = [0.0, -5.0]
x_init = [0.0, -5.1]

# Set tuning parameters
d_star = 2.0 # 2.0
zeta   = 1.0
Q_star = 2.0
eta    = 60.0

P = PotentialFunction(d_star, zeta, Q_star, eta, x_init, x_goal, 𝒲.obstacles, 𝒲_ball)


# Define your domain and grid
X_domain = [(-10, 10), (-10, 10)]
h = 0.05
x_vals = (X_domain[1][1] - h):h:(X_domain[1][2] + h) 
y_vals = (X_domain[2][1] - h):h:(X_domain[2][2] + h) 
xs = collect(x_vals)
ys = collect(y_vals)

# Compute the field
field = zeros(length(ys), length(xs))

for (i, yi) in enumerate(ys)
    for (j, xj) in enumerate(xs)
        pos = [xj, yi] 
        field[i, j] = P(pos) # Compute potential here
    end
end



# Plot
p = heatmap(xs, ys, field, 
        xlabel=L"x", ylabel=L"y", colorbar_title = "Potential",
        aspect_ratio=:equal,
        color=:coolwarm)

# Plot obstacles
for (i, poly) in enumerate(𝒲.obstacles)
    plot!(p, poly, 
          fillalpha=0.3, 
          fillcolor=:grey,
          linecolor=:black,
          label=(i == 1 ? "Obstacles" : ""))
end
# Add goal
# scatter!(p, [xg[1]], [xg[2]], 
#          marker=:star, markersize=10, 
#          markercolor=:white, label="Goal")

box = box_approximation(𝒲.bounds)
l = low(box)
h = high(box)

plot!(p, xlims=[l[1], h[1]], ylims=[l[2], h[2]])

display(p)

