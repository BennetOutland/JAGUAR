using ReachabilityAnalysis, LazySets, Plots

X0 = Hyperrectangle(low=[-1e-4, -1e-4, -1e-4], high=[1e-4, 1e-4, 1e-4])

@taylorize function unicycle!(dx, x, p, t)
    dx[1] = x[4] * cos(x[3])
    dx[2] = x[4] * sin(x[3])
    dx[3] = x[5]
    dx[4] = zero(x[4])
    dx[5] = zero(x[5])
end

# Sample control space
v_samples = range(-0.5, 0.5, length=5)
ω_samples = range(-pi/3, pi/3, length=5)

# Collect all reachable sets
all_reach_sets = []

for v in v_samples, ω in ω_samples
    U_point = Singleton([v, ω])
    X0_aug = X0 × U_point
    sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
    sol_temp = solve(sys_temp, tspan=(0.0, 1.0), alg=TMJets(orderT=10, orderQ=2))
    
    # Extract final reachable set, overapproximate, then project to (x, y)
    final_reach = sol_temp[end]
    final_set = overapproximate(final_reach, Zonotope)
    projected = project(final_set, [1, 2])
    push!(all_reach_sets, projected)
end


# Create union 
reach_union = all_reach_sets[1].X
for i in 2:length(all_reach_sets)
    reach_union = UnionSet(reach_union, all_reach_sets[i].X)
end


# Plot the union
plot(reach_union, color=:lightblue, alpha=0.5, lw=0.0, label="Reachable Set Union")
xlabel!("x")
ylabel!("y")
title!("Kinematic Unicycle Reachable Set (Union)")