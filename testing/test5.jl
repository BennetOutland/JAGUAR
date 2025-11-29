using IntervalArithmetic
using ReachabilityAnalysis, LazySets, Plots

# Split initial control set into smaller pieces
function solve_with_splitting(X0, U, n_splits=5)
    v_range = interval(low(U)[1], high(U)[1])
    ω_range = interval(low(U)[2], high(U)[2])
    
    v_splits = mince(v_range, n_splits)
    ω_splits = mince(ω_range, n_splits)
    
    all_sols = []
    
    for v_int in v_splits, ω_int in ω_splits
        U_split = Hyperrectangle(low=[inf(v_int), inf(ω_int)], 
                                 high=[sup(v_int), sup(ω_int)])
        X0_aug = X0 × U_split
        sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
        sol_temp = solve(sys_temp, tspan=(0.0, 1.0), 
                        alg=TMJets(orderT=10, orderQ=2))
        push!(all_sols, sol_temp)
    end
    
    return all_sols
end

X0 = Hyperrectangle(low=[-1e-4, -1e-4, -1e-4], high=[1e-4, 1e-4, 1e-4])
U = Hyperrectangle(low=[-0.5, -pi/3], high=[0.5, pi/3])
solutions = solve_with_splitting(X0, U, 10)

p = plot(xlabel="x", ylabel="y", title="Kinematic Unicycle Reachable Set",
         aspect_ratio=:equal, legend=false)
for sol in solutions
    plot!(p, sol, vars=(1, 2), color=:lightblue, alpha=0.4, lw=0.0)
end
display(p)