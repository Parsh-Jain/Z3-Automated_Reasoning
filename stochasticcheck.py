from z3 import *
import random

# CLI INPUT

ROWS = int(input("Enter number of rows: "))
COLS = int(input("Enter number of columns: "))

sx, sy = map(int, input("Enter start (x y): ").split())
gx, gy = map(int, input("Enter goal (x y): ").split())

T = int(input("Enter max steps (battery limit): "))

num_obs = int(input("Enter number of obstacles: "))
obstacles = set()
for i in range(num_obs):
    ox, oy = map(int, input(f"Obstacle {i+1} (x y): ").split())
    obstacles.add((ox, oy))


# OPTIMIZER (STATIC PART)

opt = Optimize()

x = [Int(f"x_{t}") for t in range(T + 1)]
y = [Int(f"y_{t}") for t in range(T + 1)]

t_goal = Int("t_goal")
opt.add(t_goal >= 0, t_goal <= T)

# Initial state
opt.add(x[0] == sx, y[0] == sy)

# Bounds
for t in range(T + 1):
    opt.add(x[t] >= 0, x[t] < ROWS)
    opt.add(y[t] >= 0, y[t] < COLS)

# Obstacles
for t in range(T + 1):
    for (ox, oy) in obstacles:
        opt.add(Or(x[t] != ox, y[t] != oy))

# Goal condition (no symbolic indexing)
opt.add(
    Or([
        And(t_goal == t, x[t] == gx, y[t] == gy)
        for t in range(T + 1)
    ])
)

# Motion + Stop(in 4 direction)
for t in range(T):
    opt.add(
        If(
            t < t_goal,
            Or(
                And(x[t + 1] == x[t] + 1, y[t + 1] == y[t]),
                And(x[t + 1] == x[t] - 1, y[t + 1] == y[t]),
                And(x[t + 1] == x[t], y[t + 1] == y[t] + 1),
                And(x[t + 1] == x[t], y[t + 1] == y[t] - 1)
            ),
            And(x[t + 1] == x[t], y[t + 1] == y[t])
        )
    )


# STOCHASTIC COST (ONE RUN)

opt.push()  # optional, but keeps "incremental" idea

cost = [Real(f"cost_{t}") for t in range(T)]
total_cost = Real("total_cost")

rand_vals = [random.uniform(0.0, 0.5) for _ in range(T)]

for t in range(T):
    opt.add(cost[t] == If(t < t_goal, 1.0 + rand_vals[t], 0.0))

opt.add(total_cost == Sum(cost))

# minimizing the path
opt.minimize(total_cost)


# CHECK SAT / UNSAT

result = opt.check()

if result == sat:
    print(" SAT: A path exists under this random-cost scenario.")
    m = opt.model()
    print("\nRandom step costs (first t_goal):", [round(v, 3) for v in rand_vals[:m[t_goal].as_long()]])
    path = [(m[x[t]].as_long(), m[y[t]].as_long()) for t in range(T + 1)]
    print(" Path found:", path)
    sum = 0
    for i in range(m[t_goal].as_long()):
        step_cost = 1.0 + rand_vals[i]
        sum += step_cost 
        print(f" Step {i}: cost = {step_cost:.3f}, cumulative cost = {sum:.3f}")

else:
    print(" UNSAT: No path exists under this random-cost scenario.")

opt.pop()
