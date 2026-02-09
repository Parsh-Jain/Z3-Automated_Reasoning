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


# Solver

s = Solver()


# State variables

x = [Int(f"x_{t}") for t in range(T+1)]
y = [Int(f"y_{t}") for t in range(T+1)]


# Initial state

s.add(x[0] == sx, y[0] == sy)


# Bounds

for t in range(T+1):
    s.add(x[t] >= 0, x[t] < ROWS)
    s.add(y[t] >= 0, y[t] < COLS)


# Obstacles

for t in range(T+1):
    for (ox, oy) in obstacles:
        s.add(Or(x[t] != ox, y[t] != oy))


# Motion constraints (4-direction)

for t in range(T):
    s.add(
        Or(
            And(x[t+1] == x[t] + 1, y[t+1] == y[t]),
            And(x[t+1] == x[t] - 1, y[t+1] == y[t]),
            And(x[t+1] == x[t], y[t+1] == y[t] + 1),
            And(x[t+1] == x[t], y[t+1] == y[t] - 1)
        )
    )


# Reach goal at some time

s.add(
    Or([And(x[t] == gx, y[t] == gy) for t in range(T+1)])
)


# Solve

if s.check() == sat:
    m = s.model()
    path = [(m[x[t]].as_long(), m[y[t]].as_long()) for t in range(T+1)]
    print("\nSAT: Path found")
    print(path)
else:
    print("\nUNSAT: No path exists")
