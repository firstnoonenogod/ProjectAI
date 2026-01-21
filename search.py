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

class SearchProblem:
    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    from game import Directions
    s=Directions.SOUTH
    w=Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    S=util.Stack()
    start_node=problem.getStartState()
    S.push((start_node, []))
    visited=[]

    while not S.isEmpty():
        A=S.pop()
        curr_state=A[0]
        path=A[1]

        if problem.isGoalState(curr_state):
            return path

        if curr_state not in visited:
            visited.append(curr_state)
            succs=problem.getSuccessors(curr_state)
            for B in succs:
                next_xy=B[0]
                direction=B[1]
                new_path=path+[direction]
                S.push((next_xy, new_path))
    return []

def breadthFirstSearch(problem: SearchProblem):
    Q=util.Queue()
    start=problem.getStartState()
    Q.push((start, []))
    seen=[]

    while not Q.isEmpty():
        tmp=Q.pop()
        node=tmp[0]
        moves=tmp[1]

        if problem.isGoalState(node):
            return moves

        if node not in seen:
            seen.append(node)
            children=problem.getSuccessors(node)
            for child in children:
                child_pos=child[0]
                child_dir=child[1]
                next_moves=moves+[child_dir]
                Q.push((child_pos, next_moves))
    return []

def uniformCostSearch(problem: SearchProblem):
    PQ=util.PriorityQueue()
    start=problem.getStartState()
    PQ.push((start, [], 0), 0)
    done=[]

    while not PQ.isEmpty():
        X=PQ.pop()
        loc=X[0]
        path=X[1]
        cost=X[2]

        if problem.isGoalState(loc):
            return path

        if loc not in done:
            done.append(loc)
            for output in problem.getSuccessors(loc):
                nxt=output[0]
                action=output[1]
                step_cost=output[2]
                new_cost=cost+step_cost
                new_path=path+[action]
                PQ.push((nxt, new_path, new_cost), new_cost)
    return []

def nullHeuristic(state, problem=None):
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    my_pq=util.PriorityQueue()
    s=problem.getStartState()
    h_start=heuristic(s, problem)
    my_pq.push((s, [], 0), 0+h_start)
    closed_set=[]

    while not my_pq.isEmpty():
        curr=my_pq.pop()
        state=curr[0]
        actions=curr[1]
        g=curr[2]

        if problem.isGoalState(state):
            return actions

        if state not in closed_set:
            closed_set.append(state)
            for item in problem.getSuccessors(state):
                succ_state=item[0]
                succ_action=item[1]
                succ_cost=item[2]
                
                new_g=g+succ_cost
                new_actions=actions+[succ_action]
                h=heuristic(succ_state, problem)
                f_score=new_g+h
                
                my_pq.push((succ_state, new_actions, new_g), f_score)
    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch