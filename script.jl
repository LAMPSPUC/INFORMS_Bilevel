using BilevelJuMP
using Cbc
using Ipopt
using JuMP
using Test

# Simple example

# TODO

# Power systems example
model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.SOS1Mode())

@variable(Upper(model), q1, start = 15)
@variable(Lower(model), g[i=1:4], start = [20, 40, 40, 0][i])

@constraint(Upper(model), q1 >= 0)
@constraint(Upper(model), q1 <= 100)

@objective(Lower(model), Min, 50g[2] + 100g[3] + 1000g[4])
@constraint(Lower(model), b, g[1] + g[2] + g[3] + g[4] == 100)
@constraint(Lower(model), g[1] <= q1)
@constraint(Lower(model), g[2] <= 40)
@constraint(Lower(model), g[3] <= 40)
@constraint(Lower(model), g[4] <= 100)
@constraint(Lower(model), lb[i=1:4], g[i] >= 0)

@variable(Upper(model), lambda, DualOf(b), start = 1_000)

@objective(Upper(model), Max, lambda*g[1])

optimize!(model)

@test objective_value(model) ≈ 20_000  atol=1e-1
@test BilevelJuMP.lower_objective_value(model) ≈ 6_000  atol=1e-1
@test value(q1) ≈ 20 atol=1e-3
@test value.(g) ≈ [20, 40, 40, 0] atol=1e-3
@test value(lambda) ≈ 1_000 atol=1e-3