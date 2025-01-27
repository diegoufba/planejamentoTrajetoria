from controller import Robot, Motor, DistanceSensor, LED
import math

max_speed = 6.28

# Inicialização do robô
robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Motores
wheels = {
    "left_front": robot.getDevice("front left wheel"),
    "left_rear": robot.getDevice("back left wheel"),
    "right_front": robot.getDevice("front right wheel"),
    "right_rear": robot.getDevice("back right wheel")
}
for wheel in wheels.values():
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)


# Sensores de distância
prox_sensors = []
for i in range(16):
    sensor_name = f"so{i}"
    prox_sensors.append(robot.getDevice(sensor_name))
    prox_sensors[i].enable(time_step)


# Função para definir velocidades das rodas
def set_wheel_speeds(left_speed, right_speed):
    wheels["left_front"].setVelocity(left_speed)
    wheels["left_rear"].setVelocity(left_speed)
    wheels["right_front"].setVelocity(right_speed)
    wheels["right_rear"].setVelocity(right_speed)

# Loop de controle
while robot.step(time_step) != -1:
    # for i in range(16):
        # print(f"i: {i}, val: {prox_sensors[i].getValue()}")
    
    sensor_threshold = 640    
    left_wall = prox_sensors[0].getValue() > sensor_threshold
    front_wall = prox_sensors[3].getValue() > sensor_threshold
    
    right_wall = prox_sensors[7].getValue() > sensor_threshold
    
    left_speed = max_speed
    right_speed = max_speed
    
    if front_wall:
        print("Turn right in place")
        left_speed = +max_speed
        right_speed = -max_speed
    else:
        if right_wall:
            print("Drive forward")
            left_speed = max_speed
            right_speed = max_speed
        else:
            print("Turn Left")
            left_speed = max_speed
            right_speed = max_speed/4
    
    # if front_wall:
        # print("Turn right in place")
        # left_speed = max_speed
        # right_speed = -max_speed
    # else:
        # if left_wall:
            # print("Drive forward")
            # left_speed = max_speed
            # right_speed = max_speed
        # else:
            # print("Turn Left")
            # left_speed = max_speed/4
            # right_speed = max_speed
    
    set_wheel_speeds(left_speed,right_speed)
    # print(f"right: {prox_sensors[0].getValue()}")
    # print(f"front: {prox_sensors[3].getValue()}")
    
    for i in range(16):
        print(f"i: {i}, val: {prox_sensors[i].getValue()}")
    
    
    