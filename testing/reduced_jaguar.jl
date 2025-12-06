"""
Author: Bennet Outland
Organization: CU Boulder
License/Control: MIT
"""

# Usings 
using LinearAlgebra
using DifferentialEquations 
using LazySets

# Includes 
include("DynamicPlanning.jl/src/workspace.jl")
include("DynamicPlanning.jl/src/plotting.jl")
include("DynamicPlanning.jl/src/robot.jl")
include("DynamicPlanning.jl/src/utils.jl")
include("DynamicPlanning.jl/src/tasks/navigation.jl")
include("DynamicPlanning.jl/src/sensors/gps.jl")
include("DynamicPlanning.jl/src/sensors/touch.jl")
include("DynamicPlanning.jl/src/models.jl")
include("DynamicPlanning.jl/src/solve.jl")
include("DynamicPlanning.jl/src/planning_algorithms/KinoFMTStar/KinoFMTStar.jl")
include("ReachSet/reachability.jl")
include("ReachSet/sampling.jl")
include("NavigationPotential/artificial_potential.jl")
include("ilqg/solve_PDGNEP.jl")

"""
TODO
"""
function solve_jaguar(W::Workspace, players::Vector{Robot}, T)
    # ============================================================================================================= #
    #                                             STAGE 0: Setup
    # ============================================================================================================= #

    # Configure the planning algorithm
    fmt = KinoFMTStar()
    for player in players
        add_planner!(player, fmt)
    end

    # Add players to the workspace
    add_robots!(W, players)

    # ============================================================================================================= #
    #                                          STAGE 0.I: Reachability
    # ============================================================================================================= #

    # Store the pre-computed sets
    reach_sets = []
    obstacle_set = []

    for player ∈ players
        # 1. Compute offline reachable set
        push!(reach_sets, compute_reachable_set(player, T; n_segments=40, n_samples=5))

        # 2. Precompute configuration space obstacles
        push!(obstacle_set, precompute_c_space_obstacles(player, W))
    end

    # ============================================================================================================= #
    #                                          STAGE 0.II: Potential Field
    # ============================================================================================================= #

    # Set tuning parameters
    d_star = 2.0 
    zeta   = 1.0
    Q_star = 2.0
    eta    = 60.0

    # Workspace Ball 
    W_ball = to_Ball2(W.bounds)

    # ============================================================================================================= #
    #                                                MAIN LOOP
    # ============================================================================================================= #
    safety = 0
    while (safety < 1)
        # ============================================================================================================= #
        #                                      STAGE I: Reachability
        # ============================================================================================================= #

        # # Determine the uncoupled goals
        # player_goals = []

        # for (i, player) ∈ enumerate(players)
        #     # 3. At runtime: transform reachable set to current pose
        #     current_reach = transform(reach_sets[i], player.X[end][3], player.X[end][1:2])

        #     # 4. Subtract obstacles to get safe region
        #     safe_reach = compute_safe_reachable_set(current_reach, obstacle_set[i], min_area=1e-10, merge_adjacent=true)

        #     # Define the objectives
        #     sign = i == 1 ? 1.0 : -1.0
        #     id = i == 1 ? 2 : 1
        #     objective(x) = sign * norm(x - players[id].X[end][1:2])

        #     # 5. Determine goal location
        #     x_opt = multi_start_optimization(
        #         objective,
        #         safe_reach.polytopes;
        #         n_starts=10,
        #         strategy=:uniform,
        #         α=0.01
        #     )

        #     push!(player_goals, x_opt[1])
        # end

        # ============================================================================================================= #
        #                                STAGE II: Independent Motion Planning
        # ============================================================================================================= #

        # # WARNING: Hardcoding for 1v1 PE 
        # println(player_goals[2])
        # nav_task_pursuer = navigation_task(player_goals[2])
        # nav_task_evader = navigation_task(player_goals[2])

        # # Add new navigation tasks
        # add_tasks!(pursuer, [nav_task_pursuer])
        # add_tasks!(evader, [nav_task_evader])

        # # Define the planning problem 
        # prob = PlanningProblem(W, (0.0, T), 0.1)

        # # TODO: Might be some timing issues with this setup
        # #sol = solve!(prob)


        # ============================================================================================================= #
        #                                STAGE III: Variational Biasing [WIP]
        # ============================================================================================================= #
        # Not implemented yet



        # ============================================================================================================= #
        #                                         STAGE IV: iLQGames
        # ============================================================================================================= #

        # Make the struct
        Φ_pursuer = PotentialFunction(d_star, zeta, Q_star, eta, players[1].X[end], zeros(2), W.obstacles, to_Ball2(W.bounds))
        Φ_evader = PotentialFunction(d_star, zeta, Q_star, eta, players[2].X[end], zeros(2), W.obstacles, to_Ball2(W.bounds))
        
        traj = SolvePDGNEP(2, vcat(players[1].X[end], players[2].X[end]), 0.0, 0.0, T, 0.0, 0.0, [players[1].constraints, players[2].constraints], [Φ_pursuer, Φ_evader])


        # Safety mechanism 
        safety += 1
        
    end

    return 42
end