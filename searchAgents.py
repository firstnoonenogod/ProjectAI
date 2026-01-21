# searchAgents.py
# ---------------
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
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

from typing import List, Tuple, Any
from game import Directions
from game import Agent
from game import Actions
import util
import time
import search
import pacman

class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP

class SearchAgent(Agent):
    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        if fn not in dir(search):
            raise AttributeError(fn+' is not a search function in search.py.')
        func=getattr(search, fn)
        if 'heuristic' not in func.__code__.co_varnames:
            print('[SearchAgent] using function '+fn)
            self.searchFunction=func
        else:
            if heuristic in globals().keys():
                heur=globals()[heuristic]
            elif heuristic in dir(search):
                heur=getattr(search, heuristic)
            else:
                raise AttributeError(heuristic+' is not a function in searchAgents.py or search.py.')
            print('[SearchAgent] using function %s and heuristic %s'%(fn, heuristic))
            self.searchFunction=lambda x: func(x, heuristic=heur)

        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError(prob+' is not a search problem type in SearchAgents.py.')
        self.searchType=globals()[prob]
        print('[SearchAgent] using problem type '+prob)

    def registerInitialState(self, state):
        if self.searchFunction==None: raise Exception("No search function provided for SearchAgent")
        starttime=time.time()
        problem=self.searchType(state)
        self.actions=self.searchFunction(problem)
        if self.actions==None:
            self.actions=[]
        totalCost=problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds'%(totalCost, time.time()-starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d'%problem._expanded)

    def getAction(self, state):
        if 'actionIndex' not in dir(self): self.actionIndex=0
        i=self.actionIndex
        self.actionIndex+=1
        if i<len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP

class PositionSearchProblem(search.SearchProblem):
    def __init__(self, gameState, costFn=lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
        self.walls=gameState.getWalls()
        self.startState=gameState.getPacmanPosition()
        if start!=None: self.startState=start
        self.goal=goal
        self.costFn=costFn
        self.visualize=visualize
        if warn and (gameState.getNumFood()!=1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')
        self._visited, self._visitedlist, self._expanded={}, [], 0

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal=state==self.goal
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): 
                    __main__._display.drawExpandedCells(self._visitedlist) 
        return isGoal

    def getSuccessors(self, state):
        successors=[]
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y=state
            dx,dy=Actions.directionToVector(action)
            nextx,nexty=int(x+dx), int(y+dy)
            if not self.walls[nextx][nexty]:
                nextState=(nextx, nexty)
                cost=self.costFn(nextState)
                successors.append((nextState, action, cost))

        self._expanded+=1
        if state not in self._visited:
            self._visited[state]=True
            self._visitedlist.append(state)
        return successors

    def getCostOfActions(self, actions):
        if actions==None: return 999999
        x,y=self.getStartState()
        cost=0
        for action in actions:
            dx,dy=Actions.directionToVector(action)
            x,y=int(x+dx), int(y+dy)
            if self.walls[x][y]: return 999999
            cost+=self.costFn((x,y))
        return cost

class StayEastSearchAgent(SearchAgent):
    def __init__(self):
        self.searchFunction=search.uniformCostSearch
        costFn=lambda pos: .5 ** pos[0]
        self.searchType=lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)

class StayWestSearchAgent(SearchAgent):
    def __init__(self):
        self.searchFunction=search.uniformCostSearch
        costFn=lambda pos: 2 ** pos[0]
        self.searchType=lambda state: PositionSearchProblem(state, costFn)

def manhattanHeuristic(position, problem, info={}):
    xy1=position
    xy2=problem.goal
    return abs(xy1[0]-xy2[0])+abs(xy1[1]-xy2[1])

def euclideanHeuristic(position, problem, info={}):
    xy1=position
    xy2=problem.goal
    return ((xy1[0]-xy2[0])**2 + (xy1[1]-xy2[1])**2)**0.5

class CornersProblem(search.SearchProblem):
    def __init__(self, startingGameState: pacman.GameState):
        self.walls=startingGameState.getWalls()
        self.startingPosition=startingGameState.getPacmanPosition()
        top,right=self.walls.height-2, self.walls.width-2
        self.corners=((1,1), (1,top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print('Warning: no food in corner '+str(corner))
        self._expanded=0

    def getStartState(self):
        return (self.startingPosition, (False, False, False, False))

    def isGoalState(self, state: Any):
        return all(state[1])

    def getSuccessors(self, state: Any):
        successors=[]
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            curr=state[0]
            chk=state[1]
            dx,dy=Actions.directionToVector(action)
            a,b=int(curr[0]+dx), int(curr[1]+dy)
            if not self.walls[a][b]:
                tmp=list(chk)
                if (a, b) in self.corners:
                    ind=self.corners.index((a, b))
                    tmp[ind]=True
                successors.append((((a, b), tuple(tmp)), action, 1))

        self._expanded+=1
        return successors

    def getCostOfActions(self, actions):
        if actions==None: return 999999
        x,y=self.startingPosition
        for action in actions:
            dx,dy=Actions.directionToVector(action)
            x,y=int(x+dx), int(y+dy)
            if self.walls[x][y]: return 999999
        return len(actions)

def cornersHeuristic(state: Any, problem: CornersProblem):
    loc=state[0]
    found=state[1]
    corners=problem.corners
    
    m=0
    for i in range(4):
        if not found[i]:
            c_pos=corners[i]
            d=abs(loc[0]-c_pos[0])+abs(loc[1]-c_pos[1])
            if d>m: m=d
    return m

class AStarCornersAgent(SearchAgent):
    def __init__(self):
        self.searchFunction=lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType=CornersProblem

class FoodSearchProblem:
    def __init__(self, startingGameState: pacman.GameState):
        self.start=(startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls=startingGameState.getWalls()
        self.startingGameState=startingGameState
        self._expanded=0
        self.heuristicInfo={}

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count()==0

    def getSuccessors(self, state):
        successors=[]
        self._expanded+=1
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y=state[0]
            dx,dy=Actions.directionToVector(direction)
            nextx,nexty=int(x+dx), int(y+dy)
            if not self.walls[nextx][nexty]:
                nextFood=state[1].copy()
                nextFood[nextx][nexty]=False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
        x,y=self.getStartState()[0]
        cost=0
        for action in actions:
            dx,dy=Actions.directionToVector(action)
            x,y=int(x+dx), int(y+dy)
            if self.walls[x][y]:
                return 999999
            cost+=1
        return cost

class AStarFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchFunction=lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType=FoodSearchProblem

def foodHeuristic(state: Tuple[Tuple, List[List]], problem: FoodSearchProblem):
    xy,grid=state
    aa=grid.asList()
    if not aa: return 0
    res=0
    for f in aa:
        d=mazeDistance(xy, f, problem.startingGameState)
        if d>res: res=d
    return res

class ClosestDotSearchAgent(SearchAgent):
    def registerInitialState(self, state):
        self.actions=[]
        currentState=state
        while(currentState.getFood().count()>0):
            nextPathSegment=self.findPathToClosestDot(currentState)
            self.actions+=nextPathSegment
            for action in nextPathSegment:
                legal=currentState.getLegalActions()
                if action not in legal:
                    t=(str(action), str(currentState))
                    raise Exception('findPathToClosestDot returned an illegal move: %s!\n%s'%t)
                currentState=currentState.generateSuccessor(0, action)
        self.actionIndex=0
        print('Path found with cost %d.'%len(self.actions))

    def findPathToClosestDot(self, gameState: pacman.GameState):
        prob=AnyFoodSearchProblem(gameState)
        return search.bfs(prob)

class AnyFoodSearchProblem(PositionSearchProblem):
    def __init__(self, gameState):
        self.food=gameState.getFood()
        self.walls=gameState.getWalls()
        self.startState=gameState.getPacmanPosition()
        self.costFn=lambda x: 1
        self._visited, self._visitedlist, self._expanded={}, [], 0

    def isGoalState(self, state: Tuple[int, int]):
        x,y=state
        return self.food[x][y]

def mazeDistance(point1: Tuple[int, int], point2: Tuple[int, int], gameState: pacman.GameState) -> int:
    x1,y1=point1
    x2,y2=point2
    walls=gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: '+str(point1)
    assert not walls[x2][y2], 'point2 is a wall: '+str(point2)
    prob=PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))