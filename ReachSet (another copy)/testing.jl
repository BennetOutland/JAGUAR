using DifferentialEquations, Plots, LazySets, LinearAlgebra
using Clipper   
include("reachability.jl")

# Simple test 1: square minus overlapping square
A = VPolygon([[0.0,0.0],[3.0,0.0],[3.0,3.0],[0.0,3.0]])
B = VPolygon([[2.0, -1.0],[4.0,-1.0],[4.0,4.0],[2.0,4.0]])

println("A area = ", LazySets.area(A))
println("B area = ", LazySets.area(B))

res = polygon_difference(A, B)
println("difference returned $(length(res)) polygons; areas = ", [LazySets.area(r) for r in res])


# Simple test 2: a reachable region and two obstacles, one overlapping and one far away.
reach = VPolygon([[0.0,0.0],[5.0,0.0],[5.0,5.0],[0.0,5.0]])
O1 = VPolygon([[1.0,1.0],[2.0,1.0],[2.0,2.0],[1.0,2.0]])  # inside R
O2 = VPolygon([[10.0,10.0],[12.0,10.0],[12.0,12.0],[10.0,12.0]]) # far
res_batch = polygon_difference_batch(reach, [O1, O2])
println("batch difference returned $(length(res_batch)) polygons; areas = ", [LazySets.area(r) for r in res_batch])





res_batch[2]

p1 = plot()

for poly in res_batch
    plot!(p1, poly, alpha=0.3, linecolor=:black, color=:red)
end

# for poly in [O1, O2]
#     plot!(p1, poly, alpha=0.3, linecolor=:black, color=:grey)
# end


display(p1)

# res[1]

# p1 = plot()
# plot!(p1, A)
# plot!(p1, B)
# plot!(res[1], label="Result")
# display(p1)