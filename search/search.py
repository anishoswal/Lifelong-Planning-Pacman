# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions
from graphicsDisplay import PacmanGraphics
from datetime import datetime

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def dbfSearch(problem, datatype):
    start = problem.getStartState()

    visitedList = []  # List of the visited nodes
    node = (start, [])  # starting node and actions
    datatype.push(node)
    emptyStatus = datatype.isEmpty()

    # The loop works till the particular stack/queue is empty where it checks each node if its already visited and then
    # if its not already visited then we check all the successors and keep pushing them into the datatype chosen one by one.

    while not emptyStatus:
        temp = datatype.pop()
        current = temp[0]
        actionList = temp[1]

        if current not in visitedList:
            visitedList.append(current)

            if problem.isGoalState(current) == True:
                return actionList

            allSuccessors = problem.getSuccessors(current)  # gets all successors
            for successor, step, cost in allSuccessors:
                node = (successor, actionList + [step])  # new node to be added to the stack/queue
                datatype.push(node)

        emptyStatus = datatype.isEmpty()

    return 0


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    stack = util.Stack()
    return dbfSearch(problem, stack)
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    return dbfSearch(problem, queue)
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()

    pqueue = util.PriorityQueue()
    visitedList = []  # List of the visited nodes
    priority = 0  # initial probability
    node = (start, [], priority)  # starting node and actions
    pqueue.push(node, priority)
    emptyStatus = pqueue.isEmpty()

    # The loop works till the priority queue is empty where it checks each node if its already visited and then
    # if its not already visited then we check all the successors and keep pushing them into the datatype chosen one by one with a priority.

    while not emptyStatus:
        temp = pqueue.pop()
        current = temp[0]
        actionList = temp[1]
        pastCost = temp[2]

        if current not in visitedList:
            visitedList.append(current)

            if problem.isGoalState(current) == True:
                return actionList

            allSuccessors = problem.getSuccessors(current)
            for successor, step, newCost in allSuccessors:
                priority = pastCost + newCost  # update priority
                node = (successor, actionList + [step], priority)
                pqueue.push(node, priority)

        emptyStatus = pqueue.isEmpty()

    return 0
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def astarRoute(problem, start, goal, heuristic=nullHeuristic):
    visitedList = []
    pqueue = util.PriorityQueue()
    node = (start, "Start", 0)
    pqueue.push([node], 0)
    emptyStatus = pqueue.isEmpty()
    #looping over all nodes in priority queue
    while not emptyStatus:
        #popping out the node to explore
        route = pqueue.pop()
        current = route[-1]
        if current[0] == goal:
            Froute = []
            for state in route:
                Froute.append(state[0])
            return Froute
        if current[0] not in visitedList:
            #add explored node to visited list.
            visitedList.append(current[0])
            for successor in problem.getSuccessors(current[0]):
                if successor not in visitedList or pqueue:
                    successorPath = route + []
                    successorPath.append((successor[0], 'dir', successor[1] + current[2]))
                    #pushing the node in queue
                    pqueue.push(successorPath, successor[1] + current[2] + util.manhattanDistance(successor[0], goal))


#implemented A* baseline
def aStarBase(problem, heuristic=nullHeuristic):
    start_time = datetime.now()
    nodes, finalRoute = [], []
    start, goal = problem.getStartState(), problem.getGoalState()
    flagSet = True
    #iterating from start node to end node
    while start != goal and flagSet:
        #calling Astarroute function to get the route
        route = astarRoute(problem, start, goal)
        state = route[0]
        nodes.append(problem._expanded)
        if len(route) is 1 and state is goal:
            #breaking loop as path contains single node
            break
        #length of all nodes
        routeRange = range(len(route) - 1)
        for i in routeRange:
            currentState = route[i]
            nextState = route[i+1]
            finalRoute.append(currentState)
            if problem.isthereWall(nextState):
                start = currentState
                problem.addWall(nextState)
                break
            elif nextState == goal:
                flagSet = False
                break
    finalRoute.append(goal)
    directions = getDirections(finalRoute)
    stop_time = datetime.now()
    #execution time
    elapsed_time = stop_time - start_time
    print("Execution Time: {} seconds".format(elapsed_time))
    return directions

#implementation of lifelong Astar search
def aStarSearch(problem, heuristic=nullHeuristic):
    start_time = datetime.now()
    nodes = []
    problem._expanded
    #initiated priority queue to store inconsistent node
    Que = util.PriorityQueue()
    rhs, gVal = {}, {}
    #initiating start state and goal state
    goal, start = problem.getGoalState(), problem.getStartState()
    endpath = []
    fval = 1
    #calculating key modifier
    def calculateKey(s):
        #we are using manhattenDistance as heuristic
        return min(gVal[s],rhs[s])+util.manhattanDistance(s,goal),min(gVal[s],rhs[s])

    #intitializing the all rhs and gvalues to infynity ans assigning start state rhs value to 0
    def initilize(allStates):
        for state in allStates:
            #initiating rhs and gval of all nodes to infinite
            rhs[state] = gVal[state] = 9999999
        rhs[start] = 0
        #pussing start node in priority queue as it is inconsistent
        Que.push(start,calculateKey(start))

    #updating the gval and rhs value at each vertex with iterations
    def updateVertex(next_state):
        if next_state != start:
            next_states = problem.getSuccessors(next_state)
            for s,c  in next_states:
                new_gval=gVal[s]+c
                rhs[next_state] = min(rhs[next_state], new_gval)
        #removing the consistent nodes
        Que.remove(next_state)
        if gVal[next_state]==rhs[next_state]:
            pass
        else:
            #pushing inconsistent nodes
            Que.push(next_state,calculateKey(next_state))

    #finding the shortest path
    def computeShortestPath():
        while Que.topKey() < calculateKey(goal) or rhs[goal] != gVal[goal]:
            #fetching the inconsistent node
            q  = Que.pop()
            if gVal[q]>rhs[q]:
                gVal[q] = rhs[q]
                #looping over all successor nodes
                next_states = problem.getSuccessors(q)
                for successor ,_ in next_states:
                    #initiating updateVertx function
                    updateVertex(successor)
            else:
                gVal[q] = 9999999
                next_states = problem.getSuccessors(q)
                next_states.append((q,1))
                for successor,_ in next_states:
                    #initiating updateVertx function
                    updateVertex(successor)

    #finalising the goal path
    def createPath(goal):
        way = []
        while goal != start:
            temp = 9999999
            minState = None
            way.append(goal)
            # looping over all successor nodes
            next_states = problem.getSuccessors(goal)
            for successor , _ in next_states:
                if temp > gVal[successor]:
                    temp = gVal[successor]
                    minState =successor
            #updating goal state value
            goal=minState

        way.append(goal)
        return way[::-1]

    initilize(problem.getAllStates())
    #performing all operations in sequence
    while start != goal and fval == True:
        initilize(problem.getAllStates())
        computeShortestPath()
        way = createPath(goal)
        nodes.append(problem._expanded)
        if len(way) == True and way[0] == goal:
            #breaking because path contains only single node
            break
        rangeRoute = range(len(way)-1)
        for i in rangeRoute:
            endpath.append(way[i])
            currentState = way[i]
            nextState = way[i + 1]
            if problem.isthereWall(nextState) == True:
                problem.addWall(nextState)
                updateVertex(nextState)
                start = currentState
                break
            elif nextState is goal:
                fval = 0
                break

    endpath.append(goal)
    directions = getDirections(endpath)
    stop_time = datetime.now()
    #execution time
    elapsed_time = stop_time - start_time
    print("Execution Time: {} seconds".format(elapsed_time))
    return directions

def getDirections(way):
    direct = []
    x,y = way[0]
    way_range = range(1,len(way))
    for i in way_range:
        xi,yi = way[i]
        #X val compare
        if x > xi:
            direct.append(Directions.WEST)
        elif x<xi:
            direct.append(Directions.EAST)
        #y val compare
        if y<yi:
            direct.append(Directions.NORTH)
        elif y>yi:
            direct.append(Directions.SOUTH)
        x = xi
        y = yi
    return direct



def dStarSearch(problem, heuristic=nullHeuristic):
    start_time = datetime.now()
    nodes = []
    Que = util.PriorityQueue()
    rhs, gVal = {}, {}
    keymod = 0
    #initiating start and goal state
    start, goal = problem.getStartState(), problem.getGoalState()
    endpath = []

    #assigning kmod to zero when agent is on start node
    def keymod0():
        keymod=0

    #updating the key as per gval, rhs and mahhatten distance
    def calculateKey(s):
        #we are using manhatten distance as heuristic
        distance_travelled = util.manhattanDistance(s, start)
        direct = (min(gVal[s], rhs[s]) + distance_travelled+ keymod, min(gVal[s], rhs[s]))
        keymod0()
        return direct

    #ititializing all nodes with gval and rhs value to infinity and final node with rhs value zero
    def initilize(allStates):
        for state in allStates:
            rhs[state] = gVal[state] = 9999999
        rhs[goal] = 0
        #as goal state is inconsistent, pushing goal state in priority queue
        Que.push(goal,calculateKey(goal))

    #updating the gval and rhs value at each vertex with iterations
    def updateVertex(next_state):
        if not(next_state == goal):
            temp = 9999999
            next_states = problem.getSuccessors(next_state)
            for successor, cost in next_states:
                new_gval = gVal[successor] + cost
                temp = min(temp, new_gval)
            rhs[next_state] = temp
        #removing consistent node from queue
        Que.remove(next_state)
        if gVal[next_state] != rhs[next_state]:
            #pushing inconsistent node in queue
            Que.push(next_state, calculateKey(next_state))

    #finding the shortest path
    def computeShortestPath():
        while Que.topKey() < calculateKey(start) or rhs[start] != gVal[start]:
            kold = Que.topKey()
            u = Que.pop()
            if kold < calculateKey(u):
                Que.push(u,calculateKey(u))
            elif gVal[u] > rhs[u]:
                gVal[u] = rhs[u]
                next_states = problem.getSuccessors(u)
                for successor, _ in next_states:
                    updateVertex(successor)
            else:  # underestimate
                gVal[u] = 9999999999
                next_states = problem.getSuccessors(u)
                next_states.append((u, 1))
                for successor, _ in next_states:
                    updateVertex(successor)

    slast = start
    initilize(problem.getAllStates())
    computeShortestPath()
    nodes.append(problem._expanded)
    fluctuatingpath = []
    #performing all operations in sequence
    while start != goal:
        minimum = 9999999
        minimumState = None

        fluctuatingpath.append(start)
        next_states = problem.getSuccessors(start)
        for successor, cost in next_states:
            temp = gVal[successor]+cost
            if temp<minimum:
                minimum = temp
                minimumState = successor
        #checking wall at state or not
        if problem.isthereWall(minimumState) == True:
            problem.addWall(minimumState)
            keymod =  keymod + util.manhattanDistance(slast,start)
            slast = start
            updateVertex(minimumState)
            fluctuatingpath=[]
            computeShortestPath()
            nodes.append(problem._expanded)
        else:
            endpath.append(start)
            start = minimumState
            fluctuatingpath.append(start)

    endpath.append(goal)
    directions = getDirections(endpath)
    stop_time = datetime.now()
    #execution time
    elapsed_time = stop_time - start_time
    print("Execution Time: {} seconds".format(elapsed_time))
    return directions

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
#use this abbrevation to run life long A algorithm
astar = aStarSearch
#use this abbrevation to run A* baseline algorithm
abase = aStarBase
ucs = uniformCostSearch
#use this abbrevation to run d* lite algorithm
dstar = dStarSearch
