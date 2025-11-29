using ReachabilityAnalysis, LazySets, Plots

X0 = Hyperrectangle(low=[-1e-4, -1e-4, -1e-4], high=[1e-4, 1e-4, 1e-4])

@taylorize function unicycle!(dx, x, p, t)
    dx[1] = x[4] * cos(x[3])
    dx[2] = x[4] * sin(x[3])
    dx[3] = x[5]
    dx[4] = zero(x[4])
    dx[5] = zero(x[5])
end


# function compute_reachable_union(X0, U, v_samples, ω_samples)
#     all_reach_sets = []
    
#     for v in v_samples, ω in ω_samples
#         U_point = Singleton([v, ω])
#         X0_aug = X0 × U_point
#         sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
        
#         # Reduce orderT and orderQ for faster computation
#         sol_temp = solve(sys_temp, tspan=(0.0, 0.25), 
#                         alg=TMJets(orderT=5, orderQ=1, abstol=1e-10))
        
#         final_reach = sol_temp[end]
#         final_set = overapproximate(final_reach, Zonotope)
#         projected = project(final_set, [1, 2])
#         push!(all_reach_sets, projected)
#     end
    
#     # reach_union = all_reach_sets[1].X
#     # for i in 2:length(all_reach_sets)
#     #     reach_union = UnionSet(reach_union, all_reach_sets[i].X)
#     # end
    
#     return all_reach_sets
# end


function compute_reachable_tube(X0, v_samples, ω_samples)
    all_polygons = []
    
    for v in v_samples, ω in ω_samples
        U_point = Singleton([v, ω])
        X0_aug = X0 × U_point
        sys_temp = @ivp(x' = unicycle!(x), dim=5, x(0) ∈ X0_aug)
        sol_temp = solve(sys_temp, tspan=(0.0, 0.25), 
                        alg=TMJets(orderT=7, orderQ=2))
        
        # Process all time steps
        for reach_set in sol_temp
            set_approx = overapproximate(reach_set, Zonotope)
            projected = project(set_approx, [1, 2])
            verts = vertices_list(projected.X)
            poly = VPolygon(verts)
            push!(all_polygons, poly)
        end
    end
    
    return all_polygons
end

v_samples = range(-0.5, 0.5, length=15)
ω_samples = range(-pi/3, pi/3, length=15)

all_polys = compute_reachable_tube(X0, v_samples, ω_samples)

p = plot(xlabel="x", ylabel="y", title="Kinematic Unicycle Reachable Tube", 
         legend=false, aspect_ratio=:equal)
for poly in all_polys
    plot!(p, poly, color=:lightblue, alpha=0.05, lw=0.0)
end
display(p)

# Adaptive grid: dense at boundaries, sparse in interior
v_interior = range(-0.3, 0.3, length=2)
v_boundary = range(-0.5, 0.5, length=5)
v_samples = unique(sort(vcat(v_interior, v_boundary)))

ω_interior = range(-pi/6, pi/6, length=2)
ω_boundary = range(-pi/3, pi/3, length=5)
ω_samples = unique(sort(vcat(ω_interior, ω_boundary)))

U = Hyperrectangle(low=[-0.5, -pi/3], high=[0.5, pi/3])

# Compute union
# reach_union = compute_reachable_union(X0, v_samples, ω_samples)
compute_reachable_tube

# usa = UnionSetArray(2, Float64)  # 2D sets, Float64 numeric type
# for s in reach_union
#     push!(array(usa), s)
# end


# plot(usa)

# Plot
# plot(reach_union, color=:lightblue, alpha=0.5, lw=0.0, label="Reachable Set Union")
# xlabel!("x")
# ylabel!("y")
# title!("Kinematic Unicycle Reachable Set")