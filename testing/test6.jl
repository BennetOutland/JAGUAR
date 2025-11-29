using ReachabilityAnalysis, LazySets, Plots

X0 = Hyperrectangle(low=[-1e-4, -1e-4, -1e-4], high=[1e-4, 1e-4, 1e-4])

@taylorize function unicycle!(dx, x, p, t)
    dx[1] = x[4] * cos(x[3])
    dx[2] = x[4] * sin(x[3])
    dx[3] = x[5]
    dx[4] = zero(x[4])
    dx[5] = zero(x[5])
end

function compute_reachable_with_boundaries(X0, v_min, v_max, ω_min, ω_max, n_interior)
    all_polygons = []
    
    # Interior grid
    v_interior = range(v_min, v_max, length=n_interior)
    ω_interior = range(ω_min, ω_max, length=n_interior)

    # Time Horizon
    T = 1.0
    
    for v in v_interior, ω in ω_interior
        U_point = Singleton([v, ω])
        X0_aug = X0 × U_point
        sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
        sol_temp = solve(sys_temp, tspan=(0.0, T), 
                        alg=TMJets(orderT=7, orderQ=2))
        
        final_reach = sol_temp[end]
        final_set = overapproximate(final_reach, Zonotope)
        projected = project(final_set, [1, 2])
        verts = vertices_list(projected.X)
        poly = VPolygon(verts)
        push!(all_polygons, poly)
    end
    
    # Add extra boundary samples for accurate envelope
    # Top and bottom edges (ω = ±ω_max)
    v_boundary = range(v_min, v_max, length=n_interior*2)
    for ω in [ω_min, ω_max]
        for v in v_boundary
            U_point = Singleton([v, ω])
            X0_aug = X0 × U_point
            sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
            sol_temp = solve(sys_temp, tspan=(0.0, T), 
                            alg=TMJets(orderT=7, orderQ=2))
            
            final_reach = sol_temp[end]
            final_set = overapproximate(final_reach, Zonotope)
            projected = project(final_set, [1, 2])
            verts = vertices_list(projected.X)
            poly = VPolygon(verts)
            push!(all_polygons, poly)
        end
    end
    
    # Left and right edges (v = v_min, v_max)
    ω_boundary = range(ω_min, ω_max, length=n_interior*2)
    for v in [v_min, v_max]
        for ω in ω_boundary
            U_point = Singleton([v, ω])
            X0_aug = X0 × U_point
            sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
            sol_temp = solve(sys_temp, tspan=(0.0, T), 
                            alg=TMJets(orderT=7, orderQ=2))
            
            final_reach = sol_temp[end]
            final_set = overapproximate(final_reach, Zonotope)
            projected = project(final_set, [1, 2])
            verts = vertices_list(projected.X)
            poly = VPolygon(verts)
            push!(all_polygons, poly)
        end
    end
    
    return all_polygons
end

# Compute with boundary emphasis
all_polys = compute_reachable_with_boundaries(X0, -0.5, 0.5, -pi/3, pi/3, 15)

# Plot
p = plot(xlabel="x", ylabel="y", title="Kinematic Unicycle Reachable Set", 
         legend=false, aspect_ratio=:equal)

for poly in all_polys
    plot!(p, poly, color=:lightblue, alpha=0.3, lw=0.0, fillalpha=0.3)
end
display(p)