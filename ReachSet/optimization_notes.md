# OPTIMIZATION STRATEGIES FOR REACHABILITY COMPUTATION

#===================================================#
# 1. PARALLEL TRAJECTORY COMPUTATION
#===================================================#

using Distributed  # Add this to your imports

"""
Parallel version of reachable polytope computation
Distributes trajectory computation across available workers
"""
function compute_reachable_polytopes_parallel(x0::Vector, 
                                              u_bounds::Vector{Tuple{Float64,Float64}},
                                              x_bounds::Vector{Tuple{Float64,Float64}},
                                              T::Float64,
                                              dynamics!;
                                              n_segments::Int=8,
                                              n_samples::Int=15,
                                              dt::Float64=0.01)
    
    n_controls = length(u_bounds)
    
    # State constraint callback
    cb = DiscreteCallback(
        (u,t,integrator) -> true,
        integrator -> begin
            for i in 1:length(integrator.u)
                integrator.u[i] = clamp(integrator.u[i], x_bounds[i][1], x_bounds[i][2])
            end
        end,
        save_positions=(false, false)
    )
    
    # Partition control space
    partitions = [range(u_bounds[i][1], u_bounds[i][2], length=n_segments+1) 
                  for i in 1:n_controls]
    
    # Generate all segment indices
    all_segments = collect(Iterators.product([1:n_segments for _ in 1:n_controls]...))
    
    # Parallel computation of polytopes
    polytopes = @distributed (vcat) for segment_idx in all_segments
        points = Vector{Float64}[]
        
        sample_ranges = [range(partitions[k][segment_idx[k]], 
                              partitions[k][segment_idx[k]+1],
                              length=n_samples)
                        for k in 1:n_controls]
        
        for u_sample in Iterators.product(sample_ranges...)
            ctrl = [t -> u_sample[i] for i in 1:n_controls]
            prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
            sol = solve(prob, Tsit5(), callback=cb, saveat=dt, dense=false)
            
            push!(points, [sol[end][1], sol[end][2]])
        end
        
        # Return polytope if valid
        length(points) >= 3 ? [VPolygon(points)] : VPolygon[]
    end
    
    return polytopes
end


#===================================================#
# 2. ADAPTIVE SAMPLING WITH EARLY TERMINATION
#===================================================#

"""
Adaptive sampling that stops early when trajectories converge
Reduces redundant computation in stable regions
"""
function compute_reachable_polytopes_adaptive(x0::Vector, 
                                              u_bounds::Vector{Tuple{Float64,Float64}},
                                              x_bounds::Vector{Tuple{Float64,Float64}},
                                              T::Float64,
                                              dynamics!;
                                              n_segments::Int=8,
                                              min_samples::Int=5,
                                              max_samples::Int=15,
                                              convergence_tol::Float64=1e-3,
                                              dt::Float64=0.01)
    
    n_controls = length(u_bounds)
    
    cb = DiscreteCallback(
        (u,t,integrator) -> true,
        integrator -> begin
            for i in 1:length(integrator.u)
                integrator.u[i] = clamp(integrator.u[i], x_bounds[i][1], x_bounds[i][2])
            end
        end,
        save_positions=(false, false)
    )
    
    partitions = [range(u_bounds[i][1], u_bounds[i][2], length=n_segments+1) 
                  for i in 1:n_controls]
    
    polytopes = VPolygon[]
    
    for segment_idx in Iterators.product([1:n_segments for _ in 1:n_controls]...)
        points = Vector{Float64}[]
        
        # Start with min_samples
        sample_ranges = [range(partitions[k][segment_idx[k]], 
                              partitions[k][segment_idx[k]+1],
                              length=min_samples)
                        for k in 1:n_controls]
        
        # Compute initial samples
        for u_sample in Iterators.product(sample_ranges...)
            ctrl = [t -> u_sample[i] for i in 1:n_controls]
            prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
            sol = solve(prob, Tsit5(), callback=cb, saveat=dt, dense=false)
            push!(points, [sol[end][1], sol[end][2]])
        end
        
        # Adaptively add samples if variation is high
        if length(points) >= 3
            initial_area = LazySets.area(VPolygon(points))
            
            for n_extra in 1:(max_samples - min_samples)
                # Add one more sample
                sample_ranges = [range(partitions[k][segment_idx[k]], 
                                      partitions[k][segment_idx[k]+1],
                                      length=min_samples + n_extra)
                                for k in 1:n_controls]
                
                u_sample = [sr[end] for sr in sample_ranges]  # Take last sample
                ctrl = [t -> u_sample[i] for i in 1:n_controls]
                prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
                sol = solve(prob, Tsit5(), callback=cb, saveat=dt, dense=false)
                push!(points, [sol[end][1], sol[end][2]])
                
                # Check convergence
                new_area = LazySets.area(VPolygon(points))
                if abs(new_area - initial_area) / initial_area < convergence_tol
                    break
                end
                initial_area = new_area
            end
        end
        
        if length(points) >= 3
            push!(polytopes, VPolygon(points))
        end
    end
    
    return polytopes
end


#===================================================#
# 3. SPATIAL INDEXING FOR OBSTACLE SUBTRACTION
#===================================================#

using NearestNeighbors  # Add to imports

"""
Bounding box for quick intersection tests
"""
struct BoundingBox
    x_min::Float64
    x_max::Float64
    y_min::Float64
    y_max::Float64
end

function get_bbox(poly::VPolygon)
    xs = [v[1] for v in poly.vertices]
    ys = [v[2] for v in poly.vertices]
    return BoundingBox(minimum(xs), maximum(xs), minimum(ys), maximum(ys))
end

function boxes_intersect(b1::BoundingBox, b2::BoundingBox)
    return !(b1.x_max < b2.x_min || b2.x_max < b1.x_min ||
             b1.y_max < b2.y_min || b2.y_max < b1.y_min)
end

"""
Optimized obstacle subtraction with spatial indexing
Only tests polytopes that could possibly intersect
"""
function subtract_obstacles_optimized(
    reachable_polytopes::Vector{VPolygon},
    c_space_obstacles::Vector{VPolygon};
    merge_adjacent::Bool = false
)
    @info "Beginning optimized obstacle subtraction..."
    
    # Precompute bounding boxes
    poly_bboxes = [get_bbox(p) for p in reachable_polytopes]
    obs_bboxes = [get_bbox(o) for o in c_space_obstacles]
    
    fragments = VPolygon[]
    
    for (j, poly) in enumerate(reachable_polytopes)
        current_parts = [poly]
        poly_bbox = poly_bboxes[j]
        
        # Only test obstacles that could intersect
        relevant_obstacles = Int[]
        for (i, obs_bbox) in enumerate(obs_bboxes)
            if boxes_intersect(poly_bbox, obs_bbox)
                push!(relevant_obstacles, i)
            end
        end
        
        if isempty(relevant_obstacles)
            # No possible intersections - keep entire polytope
            push!(fragments, poly)
            continue
        end
        
        @info "Polytope $j: testing $(length(relevant_obstacles))/$(length(c_space_obstacles)) obstacles"
        
        # Subtract only relevant obstacles
        for i in relevant_obstacles
            new_parts = VPolygon[]
            for part in current_parts
                result = polygon_difference_geos(part, c_space_obstacles[i])
                append!(new_parts, result)
            end
            
            if isempty(new_parts)
                current_parts = VPolygon[]
                break
            end
            
            current_parts = new_parts
        end
        
        append!(fragments, current_parts)
    end
    
    @info "  → Generated $(length(fragments)) fragments"
    
    if merge_adjacent && !isempty(fragments)
        @info "Merging adjacent fragments..."
        fragments = polygon_union_geos(fragments)
        @info "  → Reduced to $(length(fragments)) polytopes"
    end
    
    return fragments
end


#===================================================#
# 4. CACHING AND INCREMENTAL COMPUTATION
#===================================================#

"""
Cache structure for reachability computations
Stores results keyed by control inputs
"""
mutable struct ReachabilityCache
    trajectory_cache::Dict{Vector{Float64}, Vector{Float64}}
    polytope_cache::Dict{Tuple, VPolygon}
end

ReachabilityCache() = ReachabilityCache(Dict(), Dict())

"""
Compute with caching - useful for repeated queries
"""
function compute_reachable_polytopes_cached(x0::Vector, 
                                           u_bounds::Vector{Tuple{Float64,Float64}},
                                           x_bounds::Vector{Tuple{Float64,Float64}},
                                           T::Float64,
                                           dynamics!;
                                           cache::Union{ReachabilityCache,Nothing}=nothing,
                                           n_segments::Int=8,
                                           n_samples::Int=15,
                                           dt::Float64=0.01)
    
    if isnothing(cache)
        cache = ReachabilityCache()
    end
    
    n_controls = length(u_bounds)
    
    cb = DiscreteCallback(
        (u,t,integrator) -> true,
        integrator -> begin
            for i in 1:length(integrator.u)
                integrator.u[i] = clamp(integrator.u[i], x_bounds[i][1], x_bounds[i][2])
            end
        end,
        save_positions=(false, false)
    )
    
    partitions = [range(u_bounds[i][1], u_bounds[i][2], length=n_segments+1) 
                  for i in 1:n_controls]
    
    polytopes = VPolygon[]
    
    for segment_idx in Iterators.product([1:n_segments for _ in 1:n_controls]...)
        # Check cache first
        cache_key = (segment_idx..., T, n_samples)
        if haskey(cache.polytope_cache, cache_key)
            push!(polytopes, cache.polytope_cache[cache_key])
            continue
        end
        
        points = Vector{Float64}[]
        sample_ranges = [range(partitions[k][segment_idx[k]], 
                              partitions[k][segment_idx[k]+1],
                              length=n_samples)
                        for k in 1:n_controls]
        
        for u_sample in Iterators.product(sample_ranges...)
            u_vec = [u_sample...]
            
            # Check trajectory cache
            if haskey(cache.trajectory_cache, u_vec)
                push!(points, cache.trajectory_cache[u_vec])
            else
                ctrl = [t -> u_sample[i] for i in 1:n_controls]
                prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
                sol = solve(prob, Tsit5(), callback=cb, saveat=dt, dense=false)
                endpoint = [sol[end][1], sol[end][2]]
                
                cache.trajectory_cache[u_vec] = endpoint
                push!(points, endpoint)
            end
        end
        
        if length(points) >= 3
            poly = VPolygon(points)
            cache.polytope_cache[cache_key] = poly
            push!(polytopes, poly)
        end
    end
    
    return polytopes
end


#===================================================#
# 5. OPTIMIZED ODE SOLVING
#===================================================#

"""
Use faster ODE solver with relaxed tolerances for reachability
Accuracy vs speed tradeoff
"""
function compute_reachable_polytopes_fast(x0::Vector, 
                                         u_bounds::Vector{Tuple{Float64,Float64}},
                                         x_bounds::Vector{Tuple{Float64,Float64}},
                                         T::Float64,
                                         dynamics!;
                                         n_segments::Int=8,
                                         n_samples::Int=15,
                                         dt::Float64=0.01,
                                         reltol::Float64=1e-3,
                                         abstol::Float64=1e-4)
    
    n_controls = length(u_bounds)
    
    cb = DiscreteCallback(
        (u,t,integrator) -> true,
        integrator -> begin
            for i in 1:length(integrator.u)
                integrator.u[i] = clamp(integrator.u[i], x_bounds[i][1], x_bounds[i][2])
            end
        end,
        save_positions=(false, false)
    )
    
    partitions = [range(u_bounds[i][1], u_bounds[i][2], length=n_segments+1) 
                  for i in 1:n_controls]
    
    polytopes = VPolygon[]
    
    for segment_idx in Iterators.product([1:n_segments for _ in 1:n_controls]...)
        points = Vector{Float64}[]
        sample_ranges = [range(partitions[k][segment_idx[k]], 
                              partitions[k][segment_idx[k]+1],
                              length=n_samples)
                        for k in 1:n_controls]
        
        for u_sample in Iterators.product(sample_ranges...)
            ctrl = [t -> u_sample[i] for i in 1:n_controls]
            prob = ODEProblem(dynamics!, x0, (0.0, T), ctrl)
            
            # Use Euler method for speed (or BS3 for better accuracy/speed balance)
            sol = solve(prob, BS3(), 
                       callback=cb, 
                       save_everystep=false,  # Only save endpoint
                       reltol=reltol, 
                       abstol=abstol,
                       dense=false)
            
            push!(points, [sol[end][1], sol[end][2]])
        end
        
        if length(points) >= 3
            push!(polytopes, VPolygon(points))
        end
    end
    
    return polytopes
end


#===================================================#
# 6. PERFORMANCE RECOMMENDATIONS
#===================================================#

"""
Recommended optimization strategy based on problem size

Usage:
```julia
strategy = recommend_strategy(robot, T, workspace)
println("Recommended: ", strategy)
```
"""
function recommend_strategy(robot, T::Float64, workspace)
    n_controls = length(get_bounds(robot, :u)[1])
    n_obstacles = length(workspace.obstacles)
    n_segments = 8  # default
    
    control_space_size = n_segments^n_controls
    
    if control_space_size > 100
        return :parallel
    elseif n_obstacles > 10
        return :spatial_indexing
    elseif control_space_size > 50
        return :adaptive_sampling
    else
        return :standard
    end
end

# Example usage function
function compute_optimized(robot, T::Float64; strategy=:auto, kwargs...)
    if strategy == :auto
        strategy = recommend_strategy(robot, T, nothing)
        @info "Auto-selected strategy: $strategy"
    end
    
    x_bounds = bounds_to_tuples(get_bounds(robot, :x))
    u_bounds = bounds_to_tuples(get_bounds(robot, :u))
    
    if strategy == :parallel
        return compute_reachable_polytopes_parallel(
            robot.x0, u_bounds, x_bounds, T, robot.dynamics; kwargs...
        )
    elseif strategy == :adaptive_sampling
        return compute_reachable_polytopes_adaptive(
            robot.x0, u_bounds, x_bounds, T, robot.dynamics; kwargs...
        )
    elseif strategy == :fast
        return compute_reachable_polytopes_fast(
            robot.x0, u_bounds, x_bounds, T, robot.dynamics; kwargs...
        )
    else
        return compute_reachable_polytopes(
            robot.x0, u_bounds, x_bounds, T, robot.dynamics; kwargs...
        )
    end
end