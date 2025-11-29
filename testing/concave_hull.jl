using DifferentialEquations, ConcaveHull, Plots, LinearAlgebra

# Define unicycle dynamics
function unicycle!(du, u, p, t)
    v, ω = p  # Control parameters
    x, y, θ = u
    
    du[1] = v * cos(θ)  # ẋ
    du[2] = v * sin(θ)  # ẏ
    du[3] = ω           # θ̇
end

"""
Compute reachable set for kinematic unicycle using boundary sampling
"""
function compute_reachable_set(x0, y0, θ0, v_bounds, ω_bounds, T; 
                               n_samples=50, k=3, dt=0.01)
    """
    Parameters:
    - x0, y0, θ0: Initial state
    - v_bounds: (v_min, v_max) velocity bounds
    - ω_bounds: (ω_min, ω_max) angular velocity bounds
    - T: Time horizon
    - n_samples: Number of samples per control boundary edge
    - k: Concavity parameter for concave hull (higher = more concave)
    - dt: Integration time step
    """
    
    v_min, v_max = v_bounds
    ω_min, ω_max = ω_bounds
    
    # Generate control samples along boundary
    v_samples = range(v_min, v_max, length=n_samples)
    ω_samples = range(ω_min, ω_max, length=n_samples)
    
    # Traverse control boundary in order
    control_sequence = []
    
    # Bottom edge: ω = ω_min, v varies
    for v in v_samples
        push!(control_sequence, (v, ω_min))
    end
    
    # Right edge: v = v_max, ω varies (skip first corner)
    for ω in ω_samples[2:end]
        push!(control_sequence, (v_max, ω))
    end
    
    # Top edge: ω = ω_max, v varies in reverse (skip first corner)
    for v in reverse(v_samples[1:end-1])
        push!(control_sequence, (v, ω_max))
    end
    
    # Left edge: v = v_min, ω varies in reverse (skip both corners)
    for ω in reverse(ω_samples[2:end-1])
        push!(control_sequence, (v_min, ω))
    end
    
    # Compute reachable points for each control
    reachable_points = []
    
    for (v, ω) in control_sequence
        u0 = [x0, y0, θ0]
        tspan = (0.0, T)
        prob = ODEProblem(unicycle!, u0, tspan, (v, ω))
        sol = solve(prob, Tsit5(), saveat=dt)
        
        # Take final position
        final_state = sol[end]
        push!(reachable_points, [final_state[1], final_state[2]])
    end
    
    # Compute concave hull
    hull_obj = concave_hull(reachable_points, k)
    
    # Extract hull vertices from Hull object
    hull_vertices = collect(hull_obj.vertices)
    
    # Convert to matrix for plotting (extract x, y coordinates)
    xs = [v[1] for v in hull_vertices]
    ys = [v[2] for v in hull_vertices]
    hull_matrix = hcat(xs, ys)
    
    return hull_matrix, reachable_points
end

"""
Compute reachable tube (all intermediate times)
"""
function compute_reachable_tube(x0, y0, θ0, v_bounds, ω_bounds, T;
                                n_samples=50, k=3, dt=0.01, n_time_slices=10)
    
    time_points = range(0, T, length=n_time_slices)
    hulls = []
    
    for t in time_points[2:end]  # Skip t=0
        hull, _ = compute_reachable_set(x0, y0, θ0, v_bounds, ω_bounds, t,
                                       n_samples=n_samples, k=k, dt=dt)
        push!(hulls, hull)
    end
    
    return hulls, time_points[2:end]
end

# Example usage
x0, y0, θ0 = 0.0, 0.0, 0.0
v_bounds = (-0.5, 0.5)
ω_bounds = (-π/3, π/3)
T = 2.0

# Compute reachable set at final time
hull, points = compute_reachable_set(x0, y0, θ0, v_bounds, ω_bounds, T, 
                                     n_samples=60, k=5)

# Plot
p = plot(hull[:, 1], hull[:, 2], 
         seriestype=:shape, fillalpha=0.5, fillcolor=:lightblue,
         linecolor=:blue, linewidth=2,
         xlabel="x", ylabel="y", 
         title="Kinematic Unicycle Reachable Set (T=$T)",
         legend=false, aspect_ratio=:equal)

# Optionally plot sample points
scatter!(p, [pt[1] for pt in points], [pt[2] for pt in points],
         markersize=2, color=:red, alpha=0.5, label="Boundary samples")

display(p)