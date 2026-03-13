# microbit_wavefront_5x5_symbols.py
# Shows a 5x5 grid with barriers (1), open (0), start (S), end (X),
# runs a wavefront (BFS) shortest path, and displays the path.

from microbit import display, sleep
from microbit import *
import radio
import robotbit_library as r
# Define Ports for the bank of high current output
M1A = 0x1
M1B = 0x2
M2A = 0x3
M2B = 0x4

chnl = 13   #define channel to your team number

r.setup()

radio.config(channel=chnl)
radio.on()


# -------------------------------------------------------------------
# EDIT THIS GRID for your scenarios:
# Use exactly one 'S' (start) and one 'X' (end).
# '1' = barrier, '0' = open space.
# Each string must be length 5, and there must be 5 rows.
# -------------------------------------------------------------------

# Brightness map (0..9)
BRIGHT_OPEN   = 1
BRIGHT_WALL   = 5
BRIGHT_START  = 6
BRIGHT_END    = 8
BRIGHT_PATH   = 9

# Parse the text grid -> (grid: 0/1), start, goal
def receive_map():
    display.show(Image.ARROW_S)  # indicate waiting
    while True:
        msg = radio.receive()
        if msg and len(msg) == 25:
            # Split the flat string into rows of length 5
            return [msg[i*5:(i+1)*5] for i in range(5)]
        sleep(50)
grid_text = receive_map()
grid = [[0] * W for _ in range(H)]
start = None
goal = None
for y, row in enumerate(grid_text):
    if len(row) != W:
        raise ValueError("Each row in grid_text must be length 5")
    for x, ch in enumerate(row):
        if ch == '1':
            grid[y][x] = 1
        elif ch == '0':
            grid[y][x] = 0
        elif ch == 'S':
            start = (x, y)
            grid[y][x] = 0  # start sits on open cell
        elif ch == 'X':
            goal = (x, y)
            grid[y][x] = 0  # goal sits on open cell
        else:
            raise ValueError("Invalid character in grid_text: use '0','1','S','X' only")

if start is None or goal is None:
    raise ValueError("Grid must contain exactly one 'S' and one 'X'")

def show_grid():
    """Render the base grid with barriers, open cells, S, and X."""
    for y in range(H):
        for x in range(W):
            if (x, y) == start:
                b = BRIGHT_START
            elif (x, y) == goal:
                b = BRIGHT_END
            elif grid[y][x] == 1:
                b = BRIGHT_WALL
            else:
                b = BRIGHT_OPEN
            display.set_pixel(x, y, b)

def wavefront(g, start, goal):
    """BFS on 4-neighborhood. Returns (dist, path).
       dist[y][x] = steps from start; -1 if unreached.
       path: list of (x,y) from start -> goal (inclusive), or [] if none."""
    sx, sy = start
    gx, gy = goal
    if g[sy][sx] == 1 or g[gy][gx] == 1:
        return None, []

    dist = [[-1] * W for _ in range(H)]
    dist[sy][sx] = 0

    # Lightweight queue (two arrays + head index)
    qx, qy = [sx], [sy]
    head = 0

    while head < len(qx):
        x = qx[head]
        y = qy[head]
        head += 1

        if x == gx and y == gy:
            break

        # Explore 4-connected neighbors
        for nx, ny in ((x+1, y), (x-1, y), (x, y+1), (x, y-1)):
            if 0 <= nx < W and 0 <= ny < H:
                if g[ny][nx] == 0 and dist[ny][nx] == -1:
                    dist[ny][nx] = dist[y][x] + 1
                    qx.append(nx); qy.append(ny)

    # No path?
    if dist[gy][gx] == -1:
        return dist, []

    # Backtrack shortest path
    path = [(gx, gy)]
    x, y = gx, gy
    d = dist[y][x]
    while d > 0:
        # choose neighbor with d-1
        for nx, ny in ((x+1, y), (x-1, y), (x, y+1), (x, y-1)):
            if 0 <= nx < W and 0 <= ny < H and dist[ny][nx] == d - 1:
                path.append((nx, ny))
                x, y = nx, ny
                d -= 1
                break
    path.reverse()
    return dist, path

def animate_path_step(path):
    """Draw the path once, step-by-step, leaving S and X as-is."""
    if not path:
        return
    # Skip the first and last entries (start and goal) for drawing
    for i, (x, y) in enumerate(path):
        if (x, y) != start and (x, y) != goal:
            display.set_pixel(x, y, BRIGHT_PATH)
            sleep(130)

def blink_path(path, times=2):
    """Blink the final path over the base grid, for visibility."""
    if not path:
        return
    
    for _ in range(times):
        # Turn on path pixels
        show_path(path)
        sleep(350)
        # Restore base grid
        show_grid()
        sleep(250)

def show_path(path):
    for (x, y) in path:
        if (x, y) != start and (x, y) != goal:
            display.set_pixel(x, y, BRIGHT_PATH)

def perform(path):
    for i in range(len(path)-1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]

        if dx == 1:      # move right
            right()

        elif dx == -1:   # move left
            left()

        elif dy == -1:    # move down
            backward()

        elif dy == 1:   # move up
            forward()

        sleep(200)
        
        
def forward():
    Drive(60, 60)
    display.show(Image.ARROW_N)
    sleep(950)
    Drive(0,0)

def backward():
    Drive(-60, -60)
    display.show(Image.ARROW_S)
    sleep(950)
    Drive(0,0)

def right():
    Drive(-60,60)
    display.show(Image.ARROW_E)
    sleep(350)
    Drive(60,60)
    sleep(870)
    Drive(60, -60)
    sleep(410)
    Drive(0,0)

def left():
    Drive(60, -60)
    display.show(Image.ARROW_W)
    sleep(410)
    Drive(60,60)
    sleep(870)
    Drive(-60, 60)
    sleep(350)
    Drive(0,0)

def Drive(lft,rgt):
# Receive the percent power to drive each motor in a specific direction
    r.motor(M2B, lft * 1)
    r.motor(M1A, -rgt)


# ------------------------ Main flow ------------------------
show_grid()
sleep(700)  # Show initial map with barriers, S, and X

dist, path = wavefront(grid, start, goal)

if not path:
    display.scroll("NO PATH")
    show_grid()
else:
    # Draw the path on the device
    animate_path_step(path)
    blink_path(path, times=2)
    show_path(path)
    # End by leaving the full map visible
#    show_grid()

perform(path)
