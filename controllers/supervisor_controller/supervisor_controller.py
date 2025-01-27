from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()

pioneer = robot.getFromDef('pioneer')

while robot.step(TIME_STEP) != -1:
    position = pioneer.getPosition()
    x = position[0]
    y = position[1]
    # print(f"supervisor: x {x} y {y}")