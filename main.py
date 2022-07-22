
from pyamaze import maze, agent, textLabel, COLOR
from queue import PriorityQueue
import time

print('welcome to the AI Maze choose your search method:')
print('1)DFS')
print('2)BFS')
print('3)UCS')
print('4)A*')
print('5)GBFS')
choice = input('choice:')


# Define a depth first search function:
def DFS(m):
    start = (m.rows, m.cols)
    explored = [start]
    frontier = [start]
    dfsPath = {}
    while len(frontier) > 0:
        currCell = frontier.pop()
        if currCell == (1, 1):
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d] == True: #checking law el tare2 mftuh
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                if childCell in explored:
                    continue
                explored.append(childCell)
                frontier.append(childCell)
                dfsPath[childCell] = currCell #reverse path
    fwdPath = {}
    cell = (1, 1)
    while cell != start:
        fwdPath[dfsPath[cell]] = cell
        cell = dfsPath[cell]
    return fwdPath


# Define a breadth first search function:
def BFS(m):
    start = (m.rows, m.cols)
    frontier = [start]
    explored = [start]
    bfsPath = {}
    while len(frontier) > 0:
        currCell = frontier.pop(0)
        if currCell == (1, 1):
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d] == True:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                if childCell in explored:
                    continue
                frontier.append(childCell)
                explored.append(childCell)
                bfsPath[childCell] = currCell
    fwdPath = {}
    cell = (1, 1)
    while cell != start:
        fwdPath[bfsPath[cell]] = cell
        cell = bfsPath[cell]
    return fwdPath


# Define a Uniform Cost search function:
def UCS(m, *h, start=None):
    if start is None:
        start = (m.rows, m.cols)

    hurdles = [(i.position, i.cost) for i in h]

    unvisited = {n: float('inf') for n in m.grid}
    unvisited[start] = 0
    visited = {}
    revPath = {}
    while unvisited:
        currCell = min(unvisited, key=unvisited.get)
        visited[currCell] = unvisited[currCell]
        if currCell == m._goal:
            break
        for d in 'EWNS':
            if m.maze_map[currCell][d] == True:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                if childCell in visited:
                    continue
                tempDist = unvisited[currCell] + 1
                for hurdle in hurdles:
                    if hurdle[0] == currCell:
                        tempDist += hurdle[1]

                if tempDist < unvisited[childCell]:
                    unvisited[childCell] = tempDist
                    revPath[childCell] = currCell
        unvisited.pop(currCell)

    fwdPath = {}
    cell = m._goal
    while cell != start:
        fwdPath[revPath[cell]] = cell
        cell = revPath[cell]

    return fwdPath, visited[m._goal]


#Define the heuristiccost:
def h(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)


# Define the ASTAR search function:
def ASTAR(m):
    start = (m.rows, m.cols)
    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0
    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = h(start, (1, 1))

    open = PriorityQueue()
    open.put((h(start, (1, 1)), h(start, (1, 1)), start))
    aPath = {}
    while not open.empty():
        currentcell = open.get()[2]
        if currentcell == (1, 1):
            break
        for i in 'ESNW':
            if m.maze_map[currentcell][i] == True:
                if i == 'E':
                    childcell = (currentcell[0], currentcell[1] + 1)
                if i == 'W':
                    childcell = (currentcell[0], currentcell[1] - 1)
                if i == 'N':
                    childcell = (currentcell[0] - 1, currentcell[1])
                if i == 'S':
                    childcell = (currentcell[0] + 1, currentcell[1])

                temp_g_score = g_score[currentcell] + 1
                temp_f_score = temp_g_score + h(childcell, (1, 1))

                if temp_f_score < f_score[childcell]:
                    g_score[childcell] = temp_g_score
                    f_score[childcell] = temp_f_score
                    open.put((temp_f_score, h(childcell, (1, 1)), childcell))
                    aPath[childcell] = currentcell
    fwdPath = {}
    cell = (1, 1)
    while cell != start:
        fwdPath[aPath[cell]] = cell
        cell = aPath[cell]
    return fwdPath


# Define the GBFS search function:
def greedy(inputMaze, startCell, goalCell):
    h_score = {cell: float("inf") for cell in m.grid}
    h_score[startCell] = h(startCell, goalCell)

    cellPriorityQueue = PriorityQueue()
    cellPriorityQueue.put((h(startCell, goalCell), startCell))
    pathWithReservedArrows = {}
    searchPath = [startCell]
    while not cellPriorityQueue.empty():
        currCell = cellPriorityQueue.get()[1]
        searchPath.append(currCell)
        if currCell == goalCell:
            break
        for direction in 'ESNW':
            if m.maze_map[currCell][direction] == True:
                if direction == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif direction == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                elif direction == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif direction == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                tempHScore = h(childCell, goalCell)

                if tempHScore < h_score[childCell]:
                    h_score[childCell] = tempHScore
                    cellPriorityQueue.put((tempHScore, childCell))
                    pathWithReservedArrows[childCell] = currCell

    path = {}
    cell = goalCell
    while cell != startCell:
        path[pathWithReservedArrows[cell]] = cell
        cell = pathWithReservedArrows[cell]
    return searchPath, path


# create the maze and choose the searching algo:
if __name__ == '__main__':
    m = maze(10, 15)
    m.CreateMaze(loadMaze='maze--2022-04-25--21-15-31.csv')
    a = agent(m, footprints=True,filled=True)
    t1 = time.time()
    # call the DFS function:
    if choice == '1':
        path = DFS(m)
        m.tracePath({a: path}, delay=70)
        l = textLabel(m, 'DFS path length', len(path) + 1)
        t2 = time.time()
        l2= textLabel(m, "Time taken", (t2 - t1))
    # call the BFS function:
    if choice == '2':
        path = BFS(m)
        m.tracePath({a: path}, delay=70)
        l = textLabel(m, 'BFS path length', len(path) + 1)
        t2 = time.time()
        l2 = textLabel(m, "Time taken", (t2 - t1))
    # call the UCS function:
    if choice == '3':
        h1 = agent(m, 4, 4, color=COLOR.red)
        h2 = agent(m, 4, 6, color=COLOR.red)
        h3 = agent(m, 4, 1, color=COLOR.red)
        h4 = agent(m, 4, 2, color=COLOR.red)
        h5 = agent(m, 4, 3, color=COLOR.red)

        h1.cost = 100
        h2.cost = 100
        h3.cost = 100
        h4.cost = 100
        h5.cost = 100

        path, c = UCS(m, h1, h2, h3, h4, h5, start=(10, 15))
        l = textLabel(m, 'Total Cost', c)
        l2 = textLabel(m, 'UCS path length ', len(path) + 1)
        a = agent(m, 10, 15, color=COLOR.cyan, footprints=True)
        m.tracePath({a: path}, delay=70)
        t2 = time.time()
        l3 = textLabel(m, "Time taken", (t2 - t1))

# call the ASTAR function:
if choice == '4':
    path = ASTAR(m)
    m.tracePath({a: path}, delay=70)
    l = textLabel(m, 'A star path length', len(path) + 1)
    t2 = time.time()
    l2 = textLabel(m, "Time taken", (t2 - t1))
# call the GBFS function:
if choice == '5':
   a = agent(m, 10, 15, color=COLOR.cyan, footprints=True)
#  b = agent(m, footprints=True, color=COLOR.green)
   searchPath, path = greedy(m, startCell=(10, 15), goalCell=(1, 1))
   m.tracePath({a: path}, delay=70)
# m.tracePath({b: searchPath}, delay=70)
   l2 = textLabel(m, 'GBFS path length', len(path) + 1)
   t2 = time.time()
   l = textLabel(m, "Time taken", (t2 - t1))
m.run()
