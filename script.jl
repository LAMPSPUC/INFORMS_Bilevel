using BilevelJuMP
using Cbc
using Ipopt
using JuMP
using Test

# Example slide 9

model = BilevelModel(Cbc.Optimizer, mode = BilevelJuMP.SOS1Mode())

@variable(Upper(model), x)
@variable(Lower(model), y)

@objective(Upper(model), Min, -4x -3y)

@objective(Lower(model), Min, y)

@constraints(Lower(model), begin
    c1, 2x+y <= 4
    c2, x+2y <= 4
    c3, x >= 0
    c4, y >= 0
end)

optimize!(model)

@test value(x) ≈ 2
@test value(y) ≈ 0



# Power systems example

model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-5))

@variable(Upper(model), qS, start = 15)

@constraint(Upper(model), qS >= 0)
@constraint(Upper(model), qS <= 100)

@variable(Lower(model), gS, start = 20)
@variable(Lower(model), gR1, start = 40)
@variable(Lower(model), gR2, start = 40)
@variable(Lower(model), gD, start = 0)

@objective(Lower(model), Min, 50gR1 + 100gR2 + 1000gD)
@constraint(Lower(model), b, gS + gR1 + gR2 + gD == 100)
@constraint(Lower(model), gS <= qS)
@constraint(Lower(model), gR1 <= 40)
@constraint(Lower(model), gR2 <= 40)
@constraint(Lower(model), gD <= 100)

@constraint(Lower(model), gS >= 0)
@constraint(Lower(model), gR1 >= 0)
@constraint(Lower(model), gR2 >= 0)
@constraint(Lower(model), gD >= 0)

@variable(Upper(model), lambda, DualOf(b), start = 1_000)
@objective(Upper(model), Max, lambda*gS)

optimize!(model)

@test objective_value(model) ≈ 20_000  atol=1e-1
@test BilevelJuMP.lower_objective_value(model) ≈ 6_000  atol=1e-1
@test value(qS) ≈ 20 atol=1e-3
@test value.(gS) ≈ 20 atol=1e-3
@test value(lambda) ≈ 1_000 atol=1e-3