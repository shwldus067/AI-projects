###### Write Your Library Here ###########
from collections import deque
import heapq
import copy

#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)

# -------------------- Make Path ------------------------ #
def make_path(start, last, parent):
    path=[last[0]]

    cur=parent[last]
    while cur!=start:
        path.append(cur[0])
        cur=parent[cur]
    
    path.append(start[0])
    return path[::-1]

# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """
    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################
    
    objs=tuple(maze.circlePoints())
    start=(start_point, objs)

    parent={}
    fringe=deque([start])
    closed=set(start)

    while fringe:
        node=fringe.popleft()
        actions=maze.neighborPoints(node[0][0], node[0][1])
        for ny, nx in actions:
            nb_objs=list(copy.deepcopy(node[1]))
            if maze.isObjective(ny, nx) and (ny, nx) in nb_objs:
                nb_objs.remove((ny, nx))
            nb=((ny, nx), tuple(nb_objs))
            if nb not in closed:
                closed.add(nb)
                fringe.append(nb)
                parent[nb]=node
                if(len(nb_objs)==0):
                    return make_path(start, nb, parent)
    return path

    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[]

        # F = G+H
        self.f=0
        self.g=0
        self.h=0

    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def find_path(start, last):
    path=[]
    cur=last
    while cur:
        path.append(cur.location)
        cur=cur.parent

    return path[::-1]

def astar(maze):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]

    path=[]
    
    ####################### Write Your Code Here ################################

    start=Node(None, start_point)
    start.obj=maze.circlePoints()
    start.h=manhatten_dist(start_point, end_point)

    frontier=[]
    cost={}
    
    heapq.heappush(frontier, start)
    cost[(start.location, tuple(start.obj))]=0

    while frontier:
        node=heapq.heappop(frontier)
        if cost[(node.location, tuple(node.obj))]!=node.g:
            continue
        if(len(node.obj)==0):
            return find_path(start, node)
        actions=maze.neighborPoints(node.location[0], node.location[1])
        for ny, nx in actions:
            nb=Node(node, (ny, nx))
            nb.obj=copy.deepcopy(node.obj)
            nb.g=node.g+1
            if maze.isObjective(ny, nx) and (ny, nx) in nb.obj:
                nb.obj.remove((ny, nx))
            nb_val=(nb.location, tuple(nb.obj))
            if nb_val not in cost or nb.g<cost[nb_val]:
                cost[nb_val]=nb.g
                if len(nb.obj)!=0:
                    nb.h=manhatten_dist(nb.location, nb.obj[0])
                heapq.heappush(frontier, nb)

    return path

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #

def stage2_heuristic(node):
    point=node.location
    dist=max([manhatten_dist(point, x) for x in node.obj])
    return dist


def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]
    
    ####################### Write Your Code Here ################################
    
    start_point=maze.startPoint()
    start=Node(None, start_point)
    start.obj=copy.deepcopy(end_points)
    start.h=stage2_heuristic(start)
    
    frontier=[]
    cost={}
    
    heapq.heappush(frontier, start)
    cost[(start.location, tuple(start.obj))]=0
    
    while frontier:
        node=heapq.heappop(frontier)
        if cost[(node.location, tuple(node.obj))]!=node.g:
            continue
        if(len(node.obj)==0):
            return find_path(start, node)
        actions=maze.neighborPoints(node.location[0], node.location[1])
        for ny, nx in actions:
            nb=Node(node, (ny, nx))
            nb.obj=copy.deepcopy(node.obj)
            nb.g=node.g+1
            if maze.isObjective(ny, nx) and (ny, nx) in nb.obj:
                nb.obj.remove((ny, nx))
            nb_val=(nb.location, tuple(nb.obj))
            if nb_val not in cost or nb.g<cost[nb_val]:
                cost[nb_val]=nb.g
                if len(nb.obj)!=0:
                    nb.h=stage2_heuristic(nb)
                heapq.heappush(frontier, nb)
    
    return path

    ############################################################################


# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def mst(objectives, edges, pows):

    cost_sum=0
    ####################### Write Your Code Here ################################

    # edges=[((v, u), dist), ...]
    edges.sort()
    total_obj=objectives
    for edge in edges:
        x=pows[edge[1][0]]
        y=pows[edge[1][1]]
        if (x & total_obj) or (y & total_obj):
            cost_sum+=edge[0]
            if x & total_obj:
                total_obj=total_obj&~(x)
            if y & total_obj:
                total_obj=total_obj&~(y)
        if total_obj==0:
            break
    
    return cost_sum

    ############################################################################

def stage3_heuristic(node, alledges, pows):
    objs=node.objs
    return mst(objs, alledges, pows)

def find_edges(objs, alledges, pows):
    edges=[]
    num_objs=len(pows)
    for x in range(num_objs):
        if pows[x] & objs!=0:
            for y in range(x+1, num_objs):
                if pows[y] & objs!=0:
                    edges.append((alledges[(x, y)], (x, y)))
    return edges

def getneighbors(maze, row, col):
    possibleNeighbors = [
        (row + 1, col),
        (row - 1, col),
        (row, col + 1),
        (row, col - 1)
    ]
    neighbors = []
    for r, c in possibleNeighbors:
        if maze.choose_move(r, c):
            neighbors.append((r,c))
    return neighbors

def shortestPath(maze, start):  #start부터 모든 점까지의 최단 거리
    queue=deque([start])
    visit=[[-1 for i in range(maze.cols)] for j in range(maze.rows)]
    visit[start[0]][start[1]]=0
    dist=1
    while queue:
        size=len(queue)
        while size>0:
            cur=queue.popleft()
            neighbors=getneighbors(maze, cur[0], cur[1])
            for ny, nx in neighbors:
                if visit[ny][nx]==-1:
                    visit[ny][nx]=dist
                    queue.append((ny, nx))
            size-=1
        dist+=1
    return visit

class many_node(Node):
    def __init__(self, parent, location):
        super().__init__(parent, location)
        self.objs=0
        self.cost_val=0

def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################
    
    frontier=[]
    cost={}
    alledges={}

    num_circles=len(end_points)

    obj_idx={}
    pows=[]

    all_obj=(1<<num_circles)-1
    for i in range(num_circles):
        x=end_points[i]
        obj_idx[x]=i
        pows.append(1<<i)

    heuristics=[-1]*(all_obj+1)
    circle_edges=[-1]*(all_obj+1)

    cols=maze.cols
    rows=maze.rows
    maze_size=rows*cols

    minToCircle=[[] for k in range(num_circles)]  #circle마다 shortest path
    for i in range(num_circles):
        minToCircle[i]=shortestPath(maze, end_points[i])
        for j in range(i+1, num_circles):
            y, x=end_points[j]
            alledges[(i, j)]=minToCircle[i][y][x]

    start_point=maze.startPoint()
    start=many_node(None, start_point)
    start.objs=all_obj
    
    circle_edges[all_obj]=find_edges(all_obj, alledges, pows)
    start.h=stage3_heuristic(start, circle_edges[all_obj], pows)
    heuristics[all_obj]=start.h
    min_cost=min([minToCircle[x][start_point[0]][start_point[1]] for x in range(num_circles)])
    start.h+=min_cost

    heapq.heappush(frontier, start)
    row, col=start.location
    start.cost_val=all_obj*maze_size+row*cols+col
    cost[start.cost_val]=0
    
    while frontier:
        node=heapq.heappop(frontier)
        all_obj=node.objs

        if cost[node.cost_val]!=node.g:
            continue

        if all_obj==0:
            return find_path(start, node)

        actions=maze.neighborPoints(node.location[0], node.location[1])
        for ny, nx in actions:
            new_objs=all_obj
            if (ny, nx) in obj_idx and (pows[obj_idx[(ny, nx)]] & new_objs):
                new_objs=new_objs&~(pows[obj_idx[(ny, nx)]])
            nb_val=new_objs*maze_size+ny*cols+nx
            
            if nb_val not in cost or node.g+1<cost[nb_val]:
                nb=many_node(node, (ny, nx))
                nb.objs=new_objs
                nb.g=node.g+1
                nb.cost_val=nb_val
                cost[nb_val]=nb.g
                if new_objs!=0:
                    min_cost=min([minToCircle[x][ny][nx] for x in range(num_circles) if pows[x]&new_objs])
                    if circle_edges[new_objs]==-1:
                        circle_edges[new_objs]=find_edges(new_objs, alledges, pows)
                    if heuristics[new_objs]!=-1:
                        nb.h=heuristics[new_objs]+min_cost
                    else:
                        nb.h=stage3_heuristic(nb, circle_edges[new_objs], pows)
                        heuristics[new_objs]=nb.h
                        nb.h+=min_cost
                heapq.heappush(frontier, nb)
    
    return path
