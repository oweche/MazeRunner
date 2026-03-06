# microbit_wavefront_5x5_symbols.py
# Shows a 5x5 grid with barriers (1), open (0), start (S), end (X),
# runs a wavefront (BFS) shortest path, and displays the path.

from microbit import display, sleep
import robotbit_library as r

W, H = 5, 5
M1A = 0x1
M1B = 0x2
M2A = 0x3
M2B = 0x4
LEFT_POWR = 68
RIGHT_POWR = 100


# -------------------------------------------------------------------
# EDIT THIS GRID for your scenarios:
# Use exactly one 'S' (start) and one 'X' (end).
# '1' = barrier, '0' = open space.
# Each string must be length 5, and there must be 5 rows.
# -------------------------------------------------------------------
grid_text = [
    "S0100",
    "00100",
    "01110",
    "00010",
    "1000X",
]

# Brightness map (0..9)
BRIGHT_OPEN   = 1
BRIGHT_WALL   = 5
BRIGHT_START  = 6
BRIGHT_END    = 8
BRIGHT_PATH   = 9
FWD_BLOCK_TIME = 1500 #time for robot to go a block
TURN_LEFT_TIME = 1000 #Time to tun left 90 deg
TURN_RIGHT_TIME = 1000 #Time to turn right 90 deg

# Parse the text grid -> (grid: 0/1), start, goal
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
def forward_a_block():
    r.motor(M1A,-LEFT_POWR)
    r.motor(M2B,RIGHT_POWR)
    sleep(FWD_BLOCK_TIME)

def left_90deg():
    r.motor(M1A,-100)
    r.motor(M2B,-100)
    sleep(TURN_LEFT_TIME)

def right_90deg():
    r.motor(M1A,100)
    r.motor(M2B,100)
    sleep(TURN_RIGHT_TIME)

def go(Currentfacing, Target): 
    if(Currentfacing == Target):#facing where we want to go
        forward_a_block() 
    else:            
        if(Currentfacing == '+y'):#facing up
            if(Target == '+x'):
                right_90deg()
            elif(Target == '-x'):
                left_90deg()
            elif(Target == '-y'):
                right_90deg()
                right_90deg()
        if(Currentfacing == '+x'):#facing right
            if(Target == '-y'):
                right_90deg()
            elif(Target == '+y'):
                left_90deg()
            elif(Target == '-x'):
                right_90deg()
                right_90deg()
        if(Currentfacing == '-x'):#facing left
            if(Target == '+y'):
                right_90deg()
            elif(Target == '-y'):
                left_90deg()
            elif(Target == '+x'):
                right_90deg()
                right_90deg()
        if(Currentfacing == '-y'):#facing right
            if(Target == '-x'):
                right_90deg()
            elif(Target == '+x'):
                left_90deg()
            elif(Target == '+y'):
                right_90deg()
                right_90deg()
        forward_a_block() 


        




# ------------------------ Main flow ------------------------
show_grid()
sleep(700)  # Show initial map with barriers, S, and X

dist, path = wavefront(grid, start, goal)



if not path:
    display.scroll("NO PATH")
    show_grid()
else:
    # Main
    '''animate_path_step(path)
    blink_path(path, times=2)
    show_path(path)'''
    facing = '+y' #Front
    for step in range(len(dist)):
        current = path[step]
        if(step<len(dist)-1): #next stuff if your not at goal yet to prevent outofbond
            dx = current[step][0] - current[step+1][0]
            dy = current[step][1] - current[step+1][1]
        if (dx==1):#need to go +x
            facing = go(facing,'+x')
        elif (dx==-1):#need to go -x
            facing = go(facing,'-x')
        elif (dy == 1):#need to go +y
            facing = go(facing, '+y')
        else:
            facing = go(facing, '-y') #need to go -y
            

##TODO Make function update facing
            
            

