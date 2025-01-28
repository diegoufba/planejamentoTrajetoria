from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()

wall_node = robot.getFromDef('WALL')
translation_field = wall_node.getField('translation')

while robot.step(TIME_STEP) != -1:
    
    position = wall_node.getPosition()
    print(position[2])
    if (position[2] >= 0):
        new_value = [-8.43 , 3.92, position[2] - 0.004]
        translation_field.setSFVec3f(new_value)