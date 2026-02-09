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

TRIALS = int(input("Enter number of stochastic trials: "))


# BASE OPTIMIZER (STATIC PART)

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

# Motion + Stop (correct)
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


# STOCHASTIC TRIALS

for trial in range(1, TRIALS + 1):

    # Push a new "random-cost world"
    opt.push()

    # cost[t] is the cost of step t -> t+1
    cost = [Real(f"cost_{trial}_{t}") for t in range(T)]
    total_cost = Real(f"total_cost_{trial}")

    # Random values per step
    rand_vals = [random.uniform(0.0, 0.5) for _ in range(T)]

    # Apply cost constraints
    for t in range(T):
        # Only count cost if robot is still moving (t < t_goal)
        opt.add(
            cost[t] ==
            If(t < t_goal, 1.0 + rand_vals[t], 0.0)
        )

    opt.add(total_cost == Sum(cost))

    # Minimize stochastic total cost
    opt.minimize(total_cost)

    # Solve this trial
    result = opt.check()

    print(f"\n================ TRIAL {trial} ================")
    print("Random step costs (first 10):", [round(v, 3) for v in rand_vals[:10]])

    if result == sat:
        m = opt.model()
        tg = m[t_goal].as_long()

        path = [(m[x[t]].as_long(), m[y[t]].as_long()) for t in range(tg + 1)]

        # total_cost is Real → convert safely
        tc = m.evaluate(total_cost)
        tc_float = float(tc.as_decimal(10).replace("?", ""))

        print("SAT: Stochastic optimal path found")
        print("Goal reached at step:", tg)
        print("Path:", path)
        print("Stochastic Total Cost:", round(tc_float, 4))

    else:
        print(" UNSAT in this trial")

    # Pop trial constraints
    opt.pop()
