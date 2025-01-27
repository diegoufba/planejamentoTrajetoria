from controller import Robot
import numpy as np
import matplotlib.pyplot as plt

time_step = 32
max_speed = 1.0

def rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot):
    front_left_wheel.setVelocity(max_speed * 4)
    front_right_wheel.setVelocity(-max_speed * 4)
    back_left_wheel.setVelocity(max_speed * 4)
    back_right_wheel.setVelocity(-max_speed * 4)
    robot.step(time_step * 4)

# Função para calcular a equação da reta (reta ótima) entre o ponto de partida e chegada

def calcular_reta(x_start, y_start, x_goal, y_goal):
    m = (y_goal - y_start) / (x_goal - x_start) if x_goal != x_start else float('inf')
    b = y_start - m * x_start
    return m, b

# Função para calcular a distância perpendicular até a reta ótima

def distancia_ate_reta(x, y, m, b):
    return abs(m * x - y + b) / np.sqrt(m**2 + 1)

def run_robot(robot):
    n_sonar_sensors = 16
    
    # Posições de partida e chegada
    x_start, y_start = -4.27, 9.36  # Ponto de partida
    x_goal, y_goal = 4.32, 5.7      # Ponto de chegada
    
    # Calcular a equação da reta ótima
    m, b = calcular_reta(x_start, y_start, x_goal, y_goal)
    
    # Obter rodas
    front_left_wheel = robot.getDevice("front left wheel")
    front_right_wheel = robot.getDevice("front right wheel")
    back_left_wheel = robot.getDevice("back left wheel")
    back_right_wheel = robot.getDevice("back right wheel")
    
    # Inicializar as rodas
    front_left_wheel.setPosition(float('inf'))
    front_right_wheel.setPosition(float('inf'))
    back_left_wheel.setPosition(float('inf'))
    back_right_wheel.setPosition(float('inf'))
    front_left_wheel.setVelocity(max_speed)
    front_right_wheel.setVelocity(max_speed)
    back_left_wheel.setVelocity(max_speed)
    back_right_wheel.setVelocity(max_speed)
    
    # Inicializar Sensores Sonar
    s0_left = robot.getDevice("so0")
    s0_left.enable(time_step)
    s15_left = robot.getDevice("so15")
    s15_left.enable(time_step)
    s01_front = robot.getDevice("so1")
    s01_front.enable(time_step)
    s02_front = robot.getDevice("so2")
    s02_front.enable(time_step)
    s03_front = robot.getDevice("so3")
    s03_front.enable(time_step)
    s04_front = robot.getDevice("so4")
    s04_front.enable(time_step)
    s05_front = robot.getDevice("so5")
    s05_front.enable(time_step)
    s06_front = robot.getDevice("so6")
    s06_front.enable(time_step)
    
    # Inicializar GPS
    gps = robot.getDevice("gps")
    gps.enable(time_step)
    left_speed = max_speed
    right_speed = max_speed
    erro_anterior = 0
    
    # CONSTANTES DOS CONTROLADORES
    kd_PD = 0.5
    kp_PD = 0.6
    
    # Loop da simulação
    while robot.step(time_step) != -1:
        # Distâncias dos sensores
        left_distance = s0_left.getValue() if s0_left.getValue() > 0.0 else s15_left.getValue()
        front_distance = max(s01_front.getValue(), s02_front.getValue(), s03_front.getValue(), s04_front.getValue(), s05_front.getValue(), s06_front.getValue())
        front_wall = front_distance > 980
        
        # Obter a posição do robô
        pos = gps.getValues()
        x_pos, y_pos = round(pos[0], 2), round(pos[1], 2)
        distancia_reta = distancia_ate_reta(x_pos, y_pos, m, b)
        print(f"Front reading: {front_distance}, Left reading: {left_distance}")
        print(f"X: {x_pos}, Y: {y_pos}, Distance to line: {distancia_reta}")
        
        # Priorizar seguir a linha ótima se não houver obstáculos
        if not front_wall and distancia_reta < 1.0:
            print("Following optimal line")
            left_speed = max_speed
            right_speed = max_speed
        
        elif front_wall:  # Turn right if obstacle is ahead
            print("---------------------Turning right---------------------")
            rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot)
        
        else:
            # Se o sensor esquerdo detectou uma parede, siga a parede
            if left_distance > 500: 
                print("Follow the wall")
                
                # Movimentação guiada por PD controller
                erro = left_distance / 1000 - 0.97  # Ajuste para manter distância de 970 da parede
                derivada_erro = (erro - erro_anterior) / time_step
                controle = erro * kp_PD + derivada_erro * kd_PD
                if controle > 0.01:
                    left_speed = max_speed
                    right_speed = max_speed * controle
                    print("--Muito próximo da parede")
                elif controle < -0.01:
                    left_speed = max_speed * controle
                    right_speed = max_speed
                    print("--Muito distante da parede")
                else:
                    left_speed = max_speed
                    right_speed = max_speed
                    print("--Distância correta")
                erro_anterior = erro
            else:  # Passed left wall, turn left to follow the wall again
                print("Turn Left")
                left_speed = max_speed / 8
                right_speed = max_speed
        
        # Atualizar as velocidades das rodas
        front_left_wheel.setVelocity(left_speed)
        back_left_wheel.setVelocity(left_speed)
        front_right_wheel.setVelocity(right_speed)
        back_right_wheel.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
