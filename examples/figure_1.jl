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
include("../DynamicPlanning.jl/src/tasks/navigation.jl")
include("../DynamicPlanning.jl/src/sensors/gps.jl")
include("../DynamicPlanning.jl/src/sensors/touch.jl")
include("../DynamicPlanning.jl/src/models.jl")
include("../DynamicPlanning.jl/src/solve.jl")
include("../DynamicPlanning.jl/src/planning_algorithms/KinoFMTStar/KinoFMTStar.jl")

# Usings 


# Define the workspace
𝒲 = workspace([0.0, 0.0], 10.0)

# Define and add obstacles 
𝒪 = create_pursuit_evasion_obstacles(𝒲)
add_obstacles!(𝒲, 𝒪)

# Define the robot with a navigation task
x0_pursuer = [0.0, -5.0, 0.0]
x0_evader = [4.0, 3.0, 0.0]
xg1 = [4.0, 3.0, 0.0]
xg2 = [-2.0, 4.0, 0.0]

# Define robot and it's constraints
R1 = robot(x0_pursuer, 3, 2, circle([0.0, 0.0], 0.025))
add_dynamics!(R1, unicycle!)
add_box_constraint!(R1, [-0.25, -pi/3], [0.25, pi/3], :u)
add_box_constraint!(R1, [-10, -10, -pi/3], [10, 10, pi/3], :x)

R2 = robot(x0_evader, 3, 2, circle([0.0, 0.0], 0.025))
add_dynamics!(R2, unicycle!)
add_box_constraint!(R2, [-0.25, -pi/3], [0.25, pi/3], :u)
add_box_constraint!(R2, [-10, -10, -pi/3], [10, 10, pi/3], :x)

# Define robot tasks
nav_task_1 = navigation_task(xg1)
add_tasks!(R1, [nav_task_1])

nav_task_2 = navigation_task(xg2)
add_tasks!(R2, [nav_task_2])

# Configure the planning algorithm
fmt = KinoFMTStar()
add_planner!(R1, fmt)
add_planner!(R2, fmt)

# Add robot(s) to the simulation
add_robots!(𝒲, [R1, R2])

# Define the planning problem 
prob = PlanningProblem(𝒲, (0.0, 1.0), 0.01)

# Solve the problem
sol = solve!(prob)

# Plot 
plot(prob, goals=[xg1, xg2])