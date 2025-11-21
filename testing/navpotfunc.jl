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
include("../DynamicPlanning.jl/src/plotting.jl")
include("../DynamicPlanning.jl/src/robot.jl")
include("../DynamicPlanning.jl/src/utils.jl")
include("../NavigationPotential/navigation_potential.jl")

# Usings 
using LaTeXStrings


# Define the workspace
# 𝒲 = workspace(-2, 7, -2, 7)
𝒲 = workspace([0.0, 0.0], 10.0)


# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)

add_obstacles!(𝒲, 𝒪)

d(a, b) = norm(a - b)
NavigationPotentialFunction(𝒲, [0.0, -5.0], [0.0, 5.0], d)


# plot(𝒲)
# xlabel!(L"x")
# ylabel!(L"y")