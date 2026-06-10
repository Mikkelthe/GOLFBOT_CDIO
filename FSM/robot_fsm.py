from .state import *

# States
def initialStateHandler(golfBot: GolfBot):
    return None

def retrieveBallFromCornerStateHandler(golfBot: GolfBot):
    return None

def approachCornerStateHandler(golfBot: GolfBot):
    return None

def approachClosestBallStateHandler(golfBot: GolfBot):
    return None

def avoidObstacleStateHandler(golfBot: GolfBot):
    return None

def approachNarrowGoalStateHandler(golfBot: GolfBot):
    return None

def approachWideGoalStateHandler(golfBot: GolfBot):
    return None

def submitBallsStateHandler(golfBot: GolfBot):
    return None

# Transitions
def ballFoundTransitionHandler(golfBot: GolfBot):
    return False

def lastBallInCornerTransitionHandler(golfBot: GolfBot):
    return False

def obstacleInPathTransitionHandler(golfBot: GolfBot):
    return False


def createRobotFSM():
    initialState = State()
    initialState.setHandler(initialStateHandler)

    retrieveBallFromCornerState = State()
    retrieveBallFromCornerState.setHandler(retrieveBallFromCornerStateHandler)

    approachCornerState = State()
    approachCornerState.setHandler(approachCornerStateHandler)

    approachClosestBallState = State()
    approachClosestBallState.setHandler(approachClosestBallStateHandler)

    avoidObstacleState = State()
    avoidObstacleState.setHandler(avoidObstacleStateHandler)

    approachNarrowGoalState = State()
    approachNarrowGoalState.setHandler(approachNarrowGoalStateHandler)

    approachWideGoalState = State()
    approachWideGoalState.setHandler(approachWideGoalStateHandler)

    submitBallsState = State()
    submitBallsState.setHandler(submitBallsStateHandler)