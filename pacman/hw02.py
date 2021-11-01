from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

## Example Agent
class ReflexAgent(Agent):

  def Action(self, gameState):

    move_candidate = gameState.getLegalActions()

    scores = [self.reflex_agent_evaluationFunc(gameState, action) for action in move_candidate]
    bestScore = max(scores)
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def reflex_agent_evaluationFunc(self, currentGameState, action):

    successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = successorGameState.getPacmanPosition()
    oldFood = currentGameState.getFood()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    return successorGameState.getScore()



def scoreEvalFunc(currentGameState):

  return currentGameState.getScore()

class AdversialSearchAgent(Agent):

  def __init__(self, getFunc ='scoreEvalFunc', depth ='2'):
    self.index = 0
    self.evaluationFunction = util.lookup(getFunc, globals())

    self.depth = int(depth)

######################################################################################

class MinimaxAgent(AdversialSearchAgent):
  """
    [문제 01] MiniMax의 Action을 구현하시오. (20점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    NumAgent=gameState.getNumAgents();
    return self.MiniMax(gameState, self.depth*NumAgent, NumAgent)[1]

  def MiniMax(self, gameState, RemainDepth, NumAgent):
    if RemainDepth==0 or gameState.isWin() or gameState.isLose():
      return self.evaluationFunction(gameState), None
    agent=RemainDepth%NumAgent  #0:Pacman else:Ghost
    move_candidate=gameState.getLegalActions(agent)
    scores=[]
    if agent==0:
      bestScore=float("-inf")
    else:
      bestScore=float("inf")
    for action in move_candidate:
      successor=gameState.generateSuccessor(agent, action)
      score=self.MiniMax(successor, RemainDepth-1, NumAgent)[0]
      scores.append(score)
      if agent==0:
        if bestScore<score:
          bestScore=score
      else:
        if bestScore>score:
          bestScore=score
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)
    return bestScore, move_candidate[get_index]

    ############################################################################



class AlphaBetaAgent(AdversialSearchAgent):
  """
    [문제 02] AlphaBeta의 Action을 구현하시오. (25점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    NumAgent=gameState.getNumAgents();
    return self.AlphaBeta(gameState, self.depth*NumAgent, NumAgent, float("-inf"), float("inf"))[1]

  def AlphaBeta(self, gameState, RemainDepth, NumAgent, alpha, beta):
    if RemainDepth==0 or gameState.isWin() or gameState.isLose():
      return self.evaluationFunction(gameState), None
    agent=RemainDepth%NumAgent  #0:Pacman else:Ghost
    move_candidate=gameState.getLegalActions(agent)
    bestScore=0
    scores=[]
    if agent==0:  # Pacman
      bestScore=float("-inf")
      for action in move_candidate:
        successor=gameState.generateSuccessor(agent, action)
        value2=self.AlphaBeta(successor, RemainDepth-1, NumAgent, alpha, beta)[0]
        scores.append(value2)
        if value2>bestScore:
          bestScore=value2
          alpha=max(alpha, bestScore)
        if bestScore>beta:
          Index = [index for index in range(len(scores)) if scores[index] == bestScore]
          get_index = random.choice(Index)
          return bestScore, move_candidate[get_index]
    else:   # Ghost
      bestScore=float("inf")
      for action in move_candidate:
        successor=gameState.generateSuccessor(agent, action)
        value2=self.AlphaBeta(successor, RemainDepth-1, NumAgent, alpha, beta)[0]
        scores.append(value2)
        if value2<bestScore:
          bestScore=value2
          beta=min(beta, bestScore)
        if bestScore<alpha:
          Index = [index for index in range(len(scores)) if scores[index] == bestScore]
          get_index = random.choice(Index)
          return bestScore, move_candidate[get_index]
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)
    return bestScore, move_candidate[get_index]


    ############################################################################



class ExpectimaxAgent(AdversialSearchAgent):
  """
    [문제 03] Expectimax의 Action을 구현하시오. (25점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    NumAgent=gameState.getNumAgents();
    return self.ExpectiMax(gameState, self.depth*NumAgent, NumAgent)[1]

  def ExpectiMax(self, gameState, RemainDepth, NumAgent):
    if RemainDepth==0 or gameState.isWin() or gameState.isLose():
      return self.evaluationFunction(gameState), None
    agent=RemainDepth%NumAgent  #0:Pacman else:Ghost
    move_candidate=gameState.getLegalActions(agent)
    bestAction=Directions.STOP
    if agent==0:
      bestScore=float("-inf")
    else:
      bestScore=0
    scores=[]
    for action in move_candidate:
      successor=gameState.generateSuccessor(agent, action)
      score=self.ExpectiMax(successor, RemainDepth-1, NumAgent)[0]
      if agent==0:
        if bestScore<score:
          bestScore=score
          scores.append(score)
      else:
        bestScore+=score
    if agent!=0:
      bestScore/=len(move_candidate)
    else:
      Index = [index for index in range(len(scores)) if scores[index] == bestScore]
      get_index = random.choice(Index)
      bestAction=move_candidate[get_index]
    return bestScore, bestAction

    ############################################################################