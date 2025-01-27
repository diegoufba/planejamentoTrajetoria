from controller import Robot, Motor, DistanceSensor, LED
import math

# Configurações do robô
MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
MAX_SENSOR_VALUE = 1024
MIN_DISTANCE = 1.0
WHEEL_WEIGHT_THRESHOLD = 100
DELAY = 70

# Dados dos sensores
sensors = [
    {"wheel_weight": [150, 0]}, {"wheel_weight": [200, 0]}, {"wheel_weight": [300, 0]}, {"wheel_weight": [600, 0]},
    {"wheel_weight": [0, 600]}, {"wheel_weight": [0, 300]}, {"wheel_weight": [0, 200]}, {"wheel_weight": [0, 150]},
    {"wheel_weight": [0, 0]},   {"wheel_weight": [0, 0]},   {"wheel_weight": [0, 0]},   {"wheel_weight": [0, 0]},
    {"wheel_weight": [0, 0]},   {"wheel_weight": [0, 0]},   {"wheel_weight": [0, 0]},   {"wheel_weight": [0, 0]}
]

# Estados do robô
FORWARD, LEFT, RIGHT = 0, 1, 2

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
for i in range(MAX_SENSOR_NUMBER):
    sensor_name = f"so{i}"
    sensors[i]["device"] = robot.getDevice(sensor_name)
    sensors[i]["device"].enable(time_step)

# Variáveis de controle
delay = 0
led_number = 0
state = FORWARD

# Função para definir velocidades das rodas
def set_wheel_speeds(left_speed, right_speed):
    wheels["left_front"].setVelocity(left_speed)
    wheels["left_rear"].setVelocity(left_speed)
    wheels["right_front"].setVelocity(right_speed)
    wheels["right_rear"].setVelocity(right_speed)

# Loop de controle
while robot.step(time_step) != -1:
    speed = [0.0, 0.0]
    wheel_weight_total = [0.0, 0.0]

    # Processa os sensores
    for i in range(MAX_SENSOR_NUMBER):
        sensor_value = sensors[i]["device"].getValue()

        if sensor_value == 0.0:
            speed_modifier = 0.0
        else:
            distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE))
            speed_modifier = 1 - (distance / MIN_DISTANCE) if distance < MIN_DISTANCE else 0.0

        for j in range(2):
            wheel_weight_total[j] += sensors[i]["wheel_weight"][j] * speed_modifier

    # Lógica de estado
    if state == FORWARD:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
            speed = [0.7 * MAX_SPEED, -0.7 * MAX_SPEED]
            state = LEFT
        elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed = [-0.7 * MAX_SPEED, 0.7 * MAX_SPEED]
            state = RIGHT
        else:
            speed = [MAX_SPEED, MAX_SPEED]
    elif state == LEFT:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed = [0.7 * MAX_SPEED, -0.7 * MAX_SPEED]
        else:
            speed = [MAX_SPEED, MAX_SPEED]
            state = FORWARD
    elif state == RIGHT:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed = [-0.7 * MAX_SPEED, 0.7 * MAX_SPEED]
        else:
            speed = [MAX_SPEED, MAX_SPEED]
            state = FORWARD

    # Define as velocidades
    set_wheel_speeds(speed[0], speed[1])
