from golfbot.fsm import FSMFactory

if __name__ == "__main__":
    fsm = FSMFactory().create_robot_fsm()
    fsm.run(debug=True)