using LinearAlgebra
using JuMP
using ForwardDiff
using Random
using StatsBase
include("projecting.jl")

"""
    ProjectedGradientDescent optimizer for constrained optimization over union of polytopes.
"""
struct ProjectedGradientDescent{F, G}
    objective::F          # f(x) -> scalar
    gradient::G           # ∇f(x) -> vector
    projector::FastProjector
    α::Float64           # Step size
    max_iter::Int
    tol::Float64         # Convergence tolerance
    line_search::Bool    # Enable backtracking line search
end

function ProjectedGradientDescent(
    objective, 
    # gradient, 
    polytopes;
    α::Float64=0.01,
    max_iter::Int=1000,
    tol::Float64=1e-6,
    line_search::Bool=true
)
    projector = FastProjector(polytopes)
    gradient(x) = ForwardDiff.gradient(objective, x)
    ProjectedGradientDescent(objective, gradient, projector, α, max_iter, tol, line_search)
end

"""
    optimize!(pgd, x0) -> (x_opt, f_opt, history)

Perform projected gradient descent starting from x0.

Returns:
- x_opt: Optimal point found
- f_opt: Objective value at x_opt
- history: NamedTuple with convergence info
"""
function opt!(pgd::ProjectedGradientDescent, x0::Vector{Float64})
    x = copy(x0)
    x = project!(pgd.projector, x)  # Ensure feasibility
    
    # History tracking
    f_hist = Float64[]
    grad_norm_hist = Float64[]
    step_size_hist = Float64[]
    
    f_curr = pgd.objective(x)
    push!(f_hist, f_curr)
    
    for iter in 1:pgd.max_iter
        # Compute gradient
        g = pgd.gradient(x)
        grad_norm = norm(g)
        push!(grad_norm_hist, grad_norm)
        
        # Check convergence via projected gradient norm
        x_trial = x - pgd.α * g
        x_proj = project!(pgd.projector, x_trial)
        proj_grad_norm = norm(x_proj - x) / pgd.α
        
        if proj_grad_norm < pgd.tol
            @info "Converged at iteration $iter: ||∇_P f|| = $proj_grad_norm"
            break
        end
        
        # Step size selection
        if pgd.line_search
            α_k = backtracking_line_search(pgd, x, g, f_curr)
        else
            α_k = pgd.α
        end
        push!(step_size_hist, α_k)
        
        # Gradient step + projection
        x_new = x - α_k * g
        x_new = project!(pgd.projector, x_new)
        
        # Update
        f_new = pgd.objective(x_new)
        
        # Monitoring
        if iter % 50 == 0
            @info "Iter $iter: f = $f_new, ||∇_P f|| = $proj_grad_norm, α = $α_k"
        end
        
        x = x_new
        f_curr = f_new
        push!(f_hist, f_curr)
    end
    
    history = (
        objective = f_hist,
        grad_norm = grad_norm_hist,
        step_size = step_size_hist,
        iterations = length(f_hist) - 1
    )
    
    return x, f_curr, history
end

"""
    backtracking_line_search(pgd, x, g, f_curr) -> α

Armijo backtracking line search with projection.
"""
function backtracking_line_search(
    pgd::ProjectedGradientDescent,
    x::Vector{Float64},
    g::Vector{Float64},
    f_curr::Float64;
    β::Float64=0.5,      # Backtracking factor
    c::Float64=1e-4,     # Armijo constant
    α_max::Float64=1.0,  # Maximum step size
    max_bt::Int=20       # Maximum backtracking iterations
)
    α = α_max
    
    for _ in 1:max_bt
        x_trial = x - α * g
        x_proj = project!(pgd.projector, x_trial)
        f_trial = pgd.objective(x_proj)
        
        # Armijo condition: f(x_new) ≤ f(x) - c*α*||∇f||²
        # Modified for projection: use actual decrease
        if f_trial <= f_curr - c * α * dot(g, x - x_proj)
            return α
        end
        
        α *= β
    end
    
    return α  # Return last α even if condition not met
end

"""
    optimize_with_restarts(pgd, x0, n_restarts) -> (x_best, f_best)

Run PGD with random restarts for global optimization.
"""
function optimize_with_restarts(
    pgd::ProjectedGradientDescent,
    x0::Vector{Float64},
    n_restarts::Int=5
)
    x_best = x0
    f_best = Inf
    
    for i in 1:n_restarts
        # First iteration uses x0, rest use random feasible points
        if i == 1
            x_init = x0
        else
            x_init = random_feasible_point(pgd.projector)
        end
        
        x_opt, f_opt, _ = optimize!(pgd, x_init)
        
        if f_opt < f_best
            f_best = f_opt
            x_best = x_opt
            @info "Restart $i: New best f = $f_best"
        end
    end
    
    return x_best, f_best
end

"""
    random_feasible_point(projector) -> x

Sample random feasible point from union of polytopes.
"""
function random_feasible_point(projector::FastProjector)
    # Sample random polytope weighted by volume (approximate by radius³)
    weights = projector.radii.^3
    weights ./= sum(weights)
    idx = rand(1:length(projector.polytopes))
    
    # Sample from selected polytope (convex combination of vertices)
    poly = projector.polytopes[idx]
    verts = vertices_list(poly)
    λ = rand(length(verts))
    λ ./= sum(λ)
    
    return sum(λ[i] * verts[i] for i in 1:length(verts))
end







using Random
using StatsBase

"""
    ReachableSetSampler for efficient sampling from union of polytopes.
"""
struct ReachableSetSampler
    polytopes::Vector
    centroids::Matrix{Float64}
    volumes::Vector{Float64}      # Approximate volumes
    cumulative_weights::Vector{Float64}  # For weighted sampling
    bboxes::Vector{Tuple{Vector{Float64}, Vector{Float64}}}  # Bounding boxes
end

function ReachableSetSampler(polytopes)
    centroids = hcat([centroid(p) for p in polytopes]...)
    
    # Compute approximate volumes (use bounding box volume as proxy)
    volumes = zeros(length(polytopes))
    bboxes = Vector{Tuple{Vector{Float64}, Vector{Float64}}}(undef, length(polytopes))
    
    for (i, poly) in enumerate(polytopes)
        verts = vertices_list(poly)
        lb = minimum(hcat(verts...), dims=2) |> vec
        ub = maximum(hcat(verts...), dims=2) |> vec
        bboxes[i] = (lb, ub)
        volumes[i] = prod(ub - lb)  # Bounding box volume
    end
    
    # Normalize to create probability distribution
    weights = volumes ./ sum(volumes)
    cumulative_weights = cumsum(weights)
    
    ReachableSetSampler(polytopes, centroids, volumes, cumulative_weights, bboxes)
end

"""
    sample(sampler, n) -> Vector{Vector{Float64}}

Generate n uniform samples from the reachable set union.
"""
function sample(sampler::ReachableSetSampler, n::Int)
    samples = Vector{Vector{Float64}}(undef, n)
    
    for i in 1:n
        samples[i] = sample_single(sampler)
    end
    
    return samples
end

"""
    sample_single(sampler) -> Vector{Float64}

Generate one sample using rejection sampling within selected polytope.
"""
function sample_single(sampler::ReachableSetSampler)
    # 1. Select polytope proportional to volume
    r = rand()
    idx = searchsortedfirst(sampler.cumulative_weights, r)
    idx = clamp(idx, 1, length(sampler.polytopes))
    
    poly = sampler.polytopes[idx]
    lb, ub = sampler.bboxes[idx]
    
    # 2. Rejection sampling within bounding box
    max_attempts = 1000
    for _ in 1:max_attempts
        x = lb .+ rand(length(lb)) .* (ub .- lb)
        if in_polytope(poly, x)
            return x
        end
    end
    
    # Fallback: return centroid if rejection fails
    @warn "Rejection sampling failed for polytope $idx, returning centroid"
    return sampler.centroids[:, idx]
end

"""
    sample_convex_combination(sampler) -> Vector{Float64}

Alternative: sample via random convex combination of vertices (always feasible).
"""
function sample_convex_combination(sampler::ReachableSetSampler)
    # Select polytope proportional to volume
    r = rand()
    idx = searchsortedfirst(sampler.cumulative_weights, r)
    idx = clamp(idx, 1, length(sampler.polytopes))
    
    poly = sampler.polytopes[idx]
    verts = vertices_list(poly)
    
    # Random convex combination
    λ = rand(length(verts))
    λ ./= sum(λ)
    
    return sum(λ[i] * verts[i] for i in 1:length(verts))
end

"""
    stratified_sample(sampler, n) -> Vector{Vector{Float64}}

Stratified sampling: ensure representation from all polytopes.
"""
function stratified_sample(sampler::ReachableSetSampler, n::Int)
    samples = Vector{Vector{Float64}}()
    n_polys = length(sampler.polytopes)
    
    # Allocate samples proportional to volume
    n_per_poly = max.(1, round.(Int, n .* sampler.volumes ./ sum(sampler.volumes)))
    
    # Adjust to ensure exactly n samples
    while sum(n_per_poly) < n
        idx = argmax(sampler.volumes)
        n_per_poly[idx] += 1
    end
    while sum(n_per_poly) > n
        idx = argmax(n_per_poly)
        n_per_poly[idx] -= 1
    end
    
    # Sample from each polytope
    for (i, poly) in enumerate(sampler.polytopes)
        for _ in 1:n_per_poly[i]
            push!(samples, sample_from_polytope(poly, sampler.bboxes[i]))
        end
    end
    
    return samples
end

"""
    sample_from_polytope(poly, bbox) -> Vector{Float64}

Sample single point from specified polytope.
"""
function sample_from_polytope(poly, bbox)
    lb, ub = bbox
    max_attempts = 1000
    
    for _ in 1:max_attempts
        x = lb .+ rand(length(lb)) .* (ub .- lb)
        if in_polytope(poly, x)
            return x
        end
    end
    
    # Fallback: convex combination
    verts = vertices_list(poly)
    λ = rand(length(verts))
    λ ./= sum(λ)
    return sum(λ[i] * verts[i] for i in 1:length(verts))
end

"""
    multi_start_optimization(objective, gradient, polytopes, n_starts; kwargs...)

Perform multi-start projected gradient descent with efficient sampling.
"""
function multi_start_optimization(
    objective,
    polytopes;
    n_starts::Int=20,
    strategy::Symbol=:uniform,  # :stratified, :uniform, :convex
    α::Float64=0.1,
    max_iter::Int=500,
    tol::Float64=1e-5,
    verbose::Bool=true,
    parallel::Bool=false
)
    # Create optimizer
    pgd = ProjectedGradientDescent(
        objective,
        polytopes;
        α = α,
        max_iter = max_iter,
        tol = tol,
        line_search = true
    )
    
    # Create sampler
    sampler = ReachableSetSampler(polytopes)
    
    # Generate starting points
    if strategy == :stratified
        x_starts = stratified_sample(sampler, n_starts)
    elseif strategy == :convex
        x_starts = [sample_convex_combination(sampler) for _ in 1:n_starts]
    else  # :uniform
        x_starts = sample(sampler, n_starts)
    end
    
    verbose && @info "Generated $n_starts starting points using $strategy sampling"
    
    # Run optimizations
    results = Vector{NamedTuple}(undef, n_starts)
    
    if parallel
        # Parallel execution (requires Threads or Distributed)
        Threads.@threads for i in 1:n_starts
            x_opt, f_opt, history = opt!(pgd, x_starts[i])
            results[i] = (x = x_opt, f = f_opt, history = history, x0 = x_starts[i])
        end
    else
        # Sequential execution
        for i in 1:n_starts
            if verbose && i % 5 == 0
                @info "Running optimization $i / $n_starts"
            end
            
            x_opt, f_opt, history = opt!(pgd, x_starts[i])
            results[i] = (x = x_opt, f = f_opt, history = history, x0 = x_starts[i])
        end
    end
    
    # Find best result
    f_values = [r.f for r in results]
    best_idx = argmin(f_values)
    
    if verbose
        @info "Multi-start optimization completed"
        @info "Best objective: $(results[best_idx].f)"
        @info "Worst objective: $(maximum(f_values))"
        @info "Mean objective: $(mean(f_values))"
        @info "Std objective: $(std(f_values))"
    end
    
    # return (
    #     best = results[best_idx],
    #     all_results = results,
    #     statistics = (
    #         best_f = results[best_idx].f,
    #         worst_f = maximum(f_values),
    #         mean_f = mean(f_values),
    #         std_f = std(f_values),
    #         n_starts = n_starts
    #     )
    # )
    return results[best_idx]
end

