using DifferentialEquations, Plots, LazySets, LinearAlgebra


"""
Use Clipper.jl

"""


# Define unicycle dynamics
function unicycle!(du, u, p, t)
    v, ω = p  # Control parameters
    x, y, θ = u
    
    du[1] = v * cos(θ)  # ẋ
    du[2] = v * sin(θ)  # ẏ
    du[3] = ω           # θ̇
end

"""
Compute reachable set as union of convex polytopes
"""
function compute_reachable_union_of_convex(x0, y0, θ0, v_bounds, ω_bounds, T;
                                           n_segments=8, n_samples_per_segment=15,
                                           dt=0.01)
    """
    Parameters:
    - x0, y0, θ0: Initial state
    - v_bounds: (v_min, v_max) velocity bounds
    - ω_bounds: (ω_min, ω_max) angular velocity bounds
    - T: Time horizon
    - n_segments: Number of segments to partition control space (more = better approximation)
    - n_samples_per_segment: Samples per segment
    - dt: Integration time step
    """
    
    v_min, v_max = v_bounds
    ω_min, ω_max = ω_bounds
    
    # Partition control space into segments
    v_partitions = range(v_min, v_max, length=n_segments+1)
    ω_partitions = range(ω_min, ω_max, length=n_segments+1)
    
    convex_sets = []
    
    # For each rectangular partition in control space
    for i in 1:n_segments, j in 1:n_segments
        segment_points = Vector{Float64}[]  # Proper typing
        
        # Sample within this control segment
        v_seg = range(v_partitions[i], v_partitions[i+1], length=n_samples_per_segment)
        ω_seg = range(ω_partitions[j], ω_partitions[j+1], length=n_samples_per_segment)
        
        for v in v_seg, ω in ω_seg
            u0 = [x0, y0, θ0]
            prob = ODEProblem(unicycle!, u0, (0.0, T), (v, ω))
            sol = solve(prob, Tsit5(), saveat=dt)
            
            final_state = sol[end]
            push!(segment_points, [final_state[1], final_state[2]])
        end
        
        # Create convex hull for this segment
        if length(segment_points) >= 3
            poly = VPolygon(segment_points)  # Automatically computes convex hull
            push!(convex_sets, poly)
        end
    end
    
    return convex_sets
end

"""
Alternative: Partition along control boundary
"""
function compute_reachable_boundary_convex(x0, y0, θ0, v_bounds, ω_bounds, T;
                                          n_boundary_segments=12, n_samples=20,
                                          dt=0.01)
    """
    Partition the control boundary into segments and create convex hulls
    This often gives better approximation for nonholonomic systems
    """
    
    v_min, v_max = v_bounds
    ω_min, ω_max = ω_bounds
    
    # Sample entire control boundary
    v_samples = range(v_min, v_max, length=n_samples)
    ω_samples = range(ω_min, ω_max, length=n_samples)
    
    # Get all boundary points
    boundary_controls = Tuple{Float64, Float64}[]
    
    # Bottom edge
    for v in v_samples
        push!(boundary_controls, (v, ω_min))
    end
    # Right edge (skip corner)
    for ω in ω_samples[2:end]
        push!(boundary_controls, (v_max, ω))
    end
    # Top edge (skip corner)
    for v in reverse(v_samples[1:end-1])
        push!(boundary_controls, (v, ω_max))
    end
    # Left edge (skip both corners)
    for ω in reverse(ω_samples[2:end-1])
        push!(boundary_controls, (v_min, ω))
    end
    
    # Compute reachable points
    all_points = Vector{Float64}[]
    for (v, ω) in boundary_controls
        u0 = [x0, y0, θ0]
        prob = ODEProblem(unicycle!, u0, (0.0, T), (v, ω))
        sol = solve(prob, Tsit5(), saveat=dt)
        final_state = sol[end]
        push!(all_points, [final_state[1], final_state[2]])
    end
    
    # Partition into segments along the boundary
    segment_size = div(length(all_points), n_boundary_segments)
    convex_sets = []
    
    for i in 1:n_boundary_segments
        start_idx = (i-1) * segment_size + 1
        end_idx = min(i * segment_size + segment_size÷2, length(all_points))  # Overlap
        
        segment_points = all_points[start_idx:end_idx]
        
        if length(segment_points) >= 3
            # Add origin to make sure we cover interior
            segment_with_origin = copy(segment_points)
            push!(segment_with_origin, [x0, y0])
            poly = VPolygon(segment_with_origin)
            push!(convex_sets, poly)
        end
    end
    
    return convex_sets, all_points
end

"""
Create UnionSetArray from convex sets
"""
function create_union_set(convex_sets)
    # Build union iteratively
    if length(convex_sets) == 0
        return nothing
    end
    
    union_set = convex_sets[1]
    for i in 2:length(convex_sets)
        union_set = UnionSet(union_set, convex_sets[i])
    end
    
    return union_set
end

# Example 1: Grid partition approach
x0, y0, θ0 = 0.0, 0.0, 0.0
v_bounds = (-0.5, 0.5)
ω_bounds = (-π/3, π/3)
T = 2.0

convex_sets = compute_reachable_union_of_convex(x0, y0, θ0, v_bounds, ω_bounds, T,
                                                n_segments=6, n_samples_per_segment=10)

# # Plot
# p = plot(xlabel="x", ylabel="y", title="Reachable Set (Union of Convex Sets)",
#          legend=false, aspect_ratio=:equal)

# for (i, poly) in enumerate(convex_sets)
#     plot!(p, poly, color=:lightblue, alpha=0.3, lw=0.5, linecolor=:blue)
# end

# display(p)



"""
Method 2: Lazy union (doesn't compute anything upfront)
"""
function create_lazy_union(convex_sets)
    if length(convex_sets) == 0
        return EmptySet{Float64}(2)
    elseif length(convex_sets) == 1
        return convex_sets[1]
    end
    
    # Build balanced binary tree of unions for better operation performance
    return build_balanced_union(convex_sets)
end

function build_balanced_union(sets)
    if length(sets) == 1
        return sets[1]
    elseif length(sets) == 2
        return UnionSet(sets[1], sets[2])
    end
    
    # Split in half and recursively build
    mid = div(length(sets), 2)
    left = build_balanced_union(sets[1:mid])
    right = build_balanced_union(sets[mid+1:end])
    
    return UnionSet(left, right)
end



reach_set = create_lazy_union(convex_sets)
P = VPolygon([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]);


plot(reach_set)


# convex_sets

# convex_sets[1] ∪ convex_sets[2]
# reach_set_test = UnionSetArray(convex_sets)

# diff_sets = [difference(S, P) for S in convex_sets]

c_test = Complement(convex_sets[2])

# intersection(convex_sets[1], c_test)

Complement(union(Complement(convex_sets[1]), convex_sets[2]))

# reach_set[2]

# diff_set = difference(reach_set, P)