from controller import Robot
import numpy as np
import matplotlib.pyplot as plt

time_step = 32
max_speed = 1.0

# Definir o ponto de chegada (goal) em uma posição fixa ou acessível
goal = (4.3, 5.63)  # Você pode ajustar isso conforme o cenário desejado

# Função para calcular a equação da reta
def calcular_reta(start, goal):
    m = (goal[1] - start[1]) / (goal[0] - start[0])  # Inclinação
    b = start[1] - m * start[0]  # Interseção com o eixo Y
    return m, b

# Função para calcular a distância do ponto (x, y) até a reta
def distancia_ate_reta(x, y, m, b):
    return abs(m * x - y + b) / np.sqrt(m**2 + 1)

# Função para o movimento de rotação
def rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot):
    front_left_wheel.setVelocity(max_speed*4)
    front_right_wheel.setVelocity(-max_speed*4)
    back_left_wheel.setVelocity(max_speed*4)
    back_right_wheel.setVelocity(-max_speed*4)
    robot.step(time_step * 4)

# Função principal para rodar o robô
def run_robot(robot):
    n_sonar_sensors = 16

    # Get wheels
    front_left_wheel = robot.getDevice("front left wheel")
    front_right_wheel = robot.getDevice("front right wheel")
    back_left_wheel = robot.getDevice("back left wheel")
    back_right_wheel = robot.getDevice("back right wheel")

    # Initialize wheels
    front_left_wheel.setPosition(float('inf'))
    front_right_wheel.setPosition(float('inf'))
    back_left_wheel.setPosition(float('inf'))
    back_right_wheel.setPosition(float('inf'))

    front_left_wheel.setVelocity(max_speed)
    front_right_wheel.setVelocity(max_speed)
    back_left_wheel.setVelocity(max_speed)
    back_right_wheel.setVelocity(max_speed)

    # Initialize Sonar Sensors
    s0_left = robot.getDevice("so0")
    s0_left.enable(time_step)

    s15_left2 = robot.getDevice("so15")
    s15_left2.enable(time_step)
    
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

    # Initialize GPS
    gps = robot.getDevice("gps")
    gps.enable(time_step)
    
    compass = robot.getDevice("compass")
    compass.enable(time_step)
    
    left_speed = max_speed
    right_speed = max_speed
    
    erro_anterior = 0
    
    # CONSTANTES DOS CONTROLADORES
    kd_PD = 0.5
    kp_PD = 0.5

    # Obter as coordenadas do ponto de partida (start) usando o GPS
    start = gps.getValues()
    start = [round(start[0], 2), round(start[1], 2), round(start[2], 2)]
    
    # Obter os parâmetros da reta
    m, b = calcular_reta(start, goal)

    # Posição anterior
    last_position = gps.getValues()

    # Step simulation
    while robot.step(time_step) != -1:
        # Ler os sensores de distância
        left_distance = s0_left.getValue() if s0_left.getValue() > 0.0 else s15_left2.getValue()
        front_distance = max(s01_front.getValue(), s02_front.getValue(), s03_front.getValue(), s04_front.getValue(), s05_front.getValue(), s06_front.getValue())

        left_wall = left_distance > 500
        front_wall = front_distance > 980
        
        # Obter posição atual
        pos = gps.getValues()
        pos = [round(pos[0], 2), round(pos[1], 2), round(pos[2], 2)]

        # Calcular a distância até a reta
        dist_to_line = distancia_ate_reta(pos[0], pos[1], m, b)
        
        erro = left_distance / 1000 - 0.97  # Queremos sempre manter a 970 de proximidade da parede

        if front_wall:  # Se houver obstáculo à frente, vira à direita
            print("---------------------Turning right---------------------")
            rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot)
        
        else:
            if left_wall:  # Seguir a parede
                print("Follow the wall")
                
                # PD Controller
                derivada_erro = (erro - erro_anterior) / time_step
                controle = erro * kp_PD + derivada_erro * kd_PD
                
                if controle > 0.01:  # Se erro maior que zero, ajusta para a direita
                    left_speed = max_speed
                    right_speed = max_speed * controle
                    print("--Muito proximo da parede")
                elif controle < -0.01:  # Se erro menor que zero, ajusta para a esquerda
                    left_speed = max_speed * controle
                    right_speed = max_speed
                    print("--Muito distante da parede")
                else:
                    left_speed = max_speed
                    right_speed = max_speed
                    print("--Distância correta")

                erro_anterior = erro

            else:  # Se passou da parede esquerda, virar à esquerda
                print("Turn Left")
                left_speed = max_speed / 8
                right_speed = max_speed

        # Verificar se o robô ainda segue a reta ou está mais avançado
        if dist_to_line < 0.05 and (pos[0] > last_position[0]):  # Proximidade à reta e avanço
            print("Seguindo a reta")
            left_speed = max_speed
            right_speed = max_speed
        else:
            print("Desviando da reta, seguindo parede")
            left_speed = max_speed / 8
            right_speed = max_speed

        # Se o robô está muito perto do objetivo (goal), pare
        if abs(pos[0] - goal[0]) < 0.1 and abs(pos[1] - goal[1]) < 0.1:
            print("Chegou ao destino!")
            front_left_wheel.setVelocity(0)
            back_left_wheel.setVelocity(0)
            front_right_wheel.setVelocity(0)
            back_right_wheel.setVelocity(0)
            break  # Para a execução do robô quando atingir o destino

        # Atualiza os motores
        front_left_wheel.setVelocity(left_speed)
        back_left_wheel.setVelocity(left_speed)
        front_right_wheel.setVelocity(right_speed)
        back_right_wheel.setVelocity(right_speed)

        # Atualizar a posição anterior
        last_position = pos
        pos = gps.getValues()
        pos = [round(pos[0], 2), round(pos[1], 2), round(pos[2], 2)]
        #print("----- Ping position -----")
        #print("X:", pos[0], " Y:", pos[1], " Z:", pos[2])
        
        angle = compass.getValues()
        # print(angle)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
