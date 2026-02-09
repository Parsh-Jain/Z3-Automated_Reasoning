from z3 import *


# Parameters

ROWS, COLS = 6, 7
T = 20                     # max steps (or  battery limit)
start = (0, 0)
goal = (5, 6)

obstacles = {
    (2, 3),
    (3, 3),
    (4, 1)
}


# Solver

s = Solver()


# State variables
# x[t], y[t] = robot position at time t

x = [Int(f"x_{t}") for t in range(T+1)]
y = [Int(f"y_{t}") for t in range(T+1)]


# Initial position

s.add(x[0] == start[0])
s.add(y[0] == start[1])


# Grid bounds

for t in range(T+1):
    s.add(x[t] >= 0, x[t] < ROWS)
    s.add(y[t] >= 0, y[t] < COLS)


# Obstacles

for t in range(T+1):
    for (ox, oy) in obstacles:
        s.add(Or(x[t] != ox, y[t] != oy))


# Motion constraints (4-directional)

for t in range(T):
    s.add(
        Or(
            And(x[t+1] == x[t] + 1, y[t+1] == y[t]),     # down
            And(x[t+1] == x[t] - 1, y[t+1] == y[t]),     # up
            And(x[t+1] == x[t], y[t+1] == y[t] + 1),     # right
            And(x[t+1] == x[t], y[t+1] == y[t] - 1)      # left
        )
    )


# Goal must be reached at some time

s.add(
    Or([And(x[t] == goal[0], y[t] == goal[1]) for t in range(T+1)])
)


# Check SAT

if s.check() == sat:
    m = s.model()
    path = [(m[x[t]].as_long(), m[y[t]].as_long()) for t in range(T+1)]
    print("Found path:")
    print(path)
else:
    print("No path exists")
