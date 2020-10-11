using BilevelJuMP
using Cbc
using Ipopt
using JuMP
using QuadraticToBinary
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

# Power systems example NLP

model = BilevelModel(Ipopt.Optimizer, 
            mode = BilevelJuMP.ProductMode(1e-5))

@variable(Upper(model), 0 <= qS <= 100, start = 15)

@variable(Lower(model), 0 <= gS, start = 20)
@variable(Lower(model), 0 <= gR1 <= 40, start = 40)
@variable(Lower(model), 0 <= gR2 <= 40, start = 40)
@variable(Lower(model), 0 <= gD <= 100, start = 0)

@objective(Lower(model), Min, 50gR1 + 100gR2 + 1000gD)
@constraint(Lower(model), b, gS + gR1 + gR2 + gD == 100)
@constraint(Lower(model), gS <= qS)

@variable(Upper(model), lambda, DualOf(b), start = 1_000)
@objective(Upper(model), Max, lambda*gS)

optimize!(model)

@test objective_value(model) ≈ 20_000  atol=1e-1
@test BilevelJuMP.lower_objective_value(model) ≈ 6_000  atol=1e-1
@test value(qS) ≈ 20 atol=1e-3
@test value.(gS) ≈ 20 atol=1e-3
@test value(lambda) ≈ 1_000 atol=1e-3


# Power systems example MIP

const CBC_OPTIMIZER = Cbc.Optimizer()
MOI.set(CBC_OPTIMIZER, MOI.Silent(), true)
const CBC_CACHE = MOIU.UniversalFallback(MOIU.Model{Float64}())
const CBC_CACHED = MOIU.CachingOptimizer(CBC_CACHE, CBC_OPTIMIZER)
const CBC_BRIDGED = MOI.Bridges.full_bridge_optimizer(CBC_CACHED, Float64)
            MOI.Bridges.add_bridge(CBC_BRIDGED, MOI.Bridges.Constraint.SOCtoNonConvexQuadBridge{Float64})

opt = QuadraticToBinary.Optimizer{Float64}(CBC_BRIDGED)
model = BilevelModel(() -> opt, 
            mode = BilevelJuMP.SOS1Mode())

@variable(Upper(model), 0 <= qS <= 100)

@variable(Lower(model), 0 <= gS <= 1000)
@variable(Lower(model), 0 <= gR1 <= 40)
@variable(Lower(model), 0 <= gR2 <= 40)
@variable(Lower(model), 0 <= gD <= 100)

@objective(Lower(model), Min, 50gR1 + 100gR2 + 1000gD)
@constraint(Lower(model), b, gS + gR1 + gR2 + gD == 100)
@constraint(Lower(model), gS <= qS)

@variable(Upper(model), -10_000 <= lambda <= 10_000, DualOf(b))
@objective(Upper(model), Max, lambda*gS)

optimize!(model)

@test objective_value(model) ≈ 20_000  atol=20
@test BilevelJuMP.lower_objective_value(model) ≈ 6_000  atol=1e-1
@test value(qS) ≈ 20 atol=1e-3
@test value.(gS) ≈ 20 atol=1e-3
@test value(lambda) ≈ 1_000 atol=1e-3