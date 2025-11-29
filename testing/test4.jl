using ReachabilityAnalysis, LazySets, Plots

X0 = Hyperrectangle(low=[-1e-2, -1e-2, -1e-2], high=[1e-2, 1e-2, 1e-2])

@taylorize function unicycle!(dx, x, p, t)
    dx[1] = x[4] * cos(x[3])
    dx[2] = x[4] * sin(x[3])
    dx[3] = x[5]
    dx[4] = zero(x[4])
    dx[5] = zero(x[5])
end


function compute_true_union(X0, v_samples, ω_samples)
    all_polygons = []
    
    for v in v_samples, ω in ω_samples
        U_point = Singleton([v, ω])
        X0_aug = X0 × U_point
        sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
        sol_temp = solve(sys_temp, tspan=(0.0, 0.25), 
                        alg=TMJets(orderT=7, orderQ=2))
        
        final_reach = sol_temp[end]
        final_set = overapproximate(final_reach, Zonotope)
        projected = project(final_set, [1, 2])
        
        # Convert to HPolygon for better union operations
        poly = tohrep(projected.X)
        push!(all_polygons, poly)
    end
    
    # Iteratively union pairs to build complete set
    result = all_polygons[1]
    for i in 2:length(all_polygons)
        result = UnionSet(result, all_polygons[i])
    end
    
    return result
end

v_samples = range(-0.5, 0.5, length=30)
ω_samples = range(-pi/3, pi/3, length=30)

reach_set = compute_true_union(X0, v_samples, ω_samples)
plot(reach_set, color=:lightblue, alpha=0.5, lw=1.0,
     xlabel="x", ylabel="y", title="Kinematic Unicycle Reachable Set",
     aspect_ratio=:equal)