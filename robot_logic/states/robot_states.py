from enum import Enum


class RobotState(Enum):
    SCAN_FIELD = "SCAN_FIELD"
    PLAN_ROUTE = "PLAN_ROUTE"
    MOVE_TO_PICKUP = "MOVE_TO_PICKUP"
    ALIGN_TO_BALL = "ALIGN_TO_BALL"
    VACUUM_COLLECT = "VACUUM_COLLECT"
    MOVE_TO_GOAL = "MOVE_TO_GOAL"
    DELIVER = "DELIVER"
    DONE = "DONE"
    RECOVERY = "RECOVERY"


STATE_FLOW = [
    RobotState.SCAN_FIELD,
    RobotState.PLAN_ROUTE,
    RobotState.MOVE_TO_PICKUP,
    RobotState.ALIGN_TO_BALL,
    RobotState.VACUUM_COLLECT,
    RobotState.MOVE_TO_GOAL,
    RobotState.DELIVER,
    RobotState.DONE,
]


def print_state_flow():
    print("Robot state flow:")
    for state in STATE_FLOW:
        print(f"- {state.value}")


if __name__ == "__main__":
    print_state_flow()
