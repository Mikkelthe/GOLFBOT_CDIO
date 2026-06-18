from FSM.robot_fsm import FSMFactory

if __name__ == "__main__":
    fsm = FSMFactory().createRobotFSM()
    fsm.run()