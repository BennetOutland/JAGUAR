using ReachabilityAnalysis, LazySets, Plots

# Control bounds
U = Hyperrectangle(low=[-0.5, -pi/3], high=[0.5, pi/3])

# Single initial configuration (point)
x0 = [0.0, 0.0, 0.0]  # [x, y, θ]

# Generate sample control trajectories
n_samples = 100
v_samples = range(-0.5, 0.5, length=Int(sqrt(n_samples)))
ω_samples = range(-pi/3, pi/3, length=Int(sqrt(n_samples)))

# Integrate trajectories
T = 1.0
dt = 0.001
t_vec = 0:dt:T

trajectories = []
for v in v_samples, ω in ω_samples
    x, y, θ = x0
    traj_x = [x]
    traj_y = [y]
    
    for t in t_vec[1:end-1]
        x += v * cos(θ) * dt
        y += v * sin(θ) * dt
        θ += ω * dt
        push!(traj_x, x)
        push!(traj_y, y)
    end
    push!(trajectories, (traj_x, traj_y))
end

# Plot
plot(legend=:topright)
for (tx, ty) in trajectories
    plot!(tx, ty, color=:lightblue, alpha=0.7, lw=1.0, label="")
end
xlabel!("x")
ylabel!("y")
title!("Kinematic Unicycle Reachable Set")