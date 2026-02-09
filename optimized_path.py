from z3 import *


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


# OPTIMIZER

opt = Optimize()

# STATE VARIABLES

x = [Int(f"x_{t}") for t in range(T + 1)]
y = [Int(f"y_{t}") for t in range(T + 1)]

# Time when goal is reached
t_goal = Int("t_goal")
opt.add(t_goal >= 0, t_goal <= T)


# INITIAL STATE

opt.add(x[0] == sx, y[0] == sy)


# GRID BOUNDS

for t in range(T + 1):
    opt.add(x[t] >= 0, x[t] < ROWS)
    opt.add(y[t] >= 0, y[t] < COLS)

# OBSTACLES

for t in range(T + 1):
    for (ox, oy) in obstacles:
        opt.add(Or(x[t] != ox, y[t] != oy))


# MOTION CONSTRAINTS (4-dir)

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



# GOAL CONDITION

opt.add(
    Or([
        And(t_goal == t, x[t] == gx, y[t] == gy)
        for t in range(T + 1)
    ])
)


# STOP AFTER GOAL

for t in range(T):
    opt.add(
        Implies(
            t >= t_goal,
            And(x[t + 1] == x[t], y[t + 1] == y[t])
        )
    )


# MINIMIZE COST
# Cost = number of steps = t_goal

opt.minimize(t_goal)


# SOLVE

if opt.check() == sat:
    m = opt.model()
    tg = m[t_goal].as_long()

    path = [(m[x[t]].as_long(), m[y[t]].as_long()) for t in range(tg + 1)]

    print("\n SAT: Optimal Path Found")
    print("Path:", path)
    print("Minimum Cost (Battery Used):", tg)

else:
    print("\n UNSAT: No path exists")
