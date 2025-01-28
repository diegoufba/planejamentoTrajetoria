from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math

time_step = 32
max_speed = 1.0

def calculate_line_coefficients(x1, y1, x2, y2):
    """
    Calcula os coeficientes a  e b  da reta que passa pelos pontos (x1, y1) e (x2, y2).
    """

    a = (y2 - y1) / (x2 - x1) 
    b = y1 - a * x1
    return a, b

def is_circle_touching_line(cx, cy, radius, a, b):
    """
    Verifica se um círculo com centro (cx, cy) e raio 'radius' toca a reta y = ax + b.
    """
    # Fórmula da distância de um ponto (cx, cy) à reta y = ax + b
    distance = abs(a * cx - cy + b) / math.sqrt(a**2 + 1)
    return distance <= radius
    
def calcular_heading(Hx, Hy):
    heading_rad = math.atan2(-Hy, Hx)  # Calcula o ângulo em radianos
    heading_deg = math.degrees(heading_rad)  # Converte para graus
    heading_deg = (heading_deg + 360) % 360  # Normaliza para [0, 360]
    return heading_deg
    
def calcular_heading_reta(a):
    """
    Calcula o heading (ângulo em relação ao norte) para uma reta y = ax + b.
    
    Parâmetros:
    a (float): Coeficiente angular da reta.
    
    Retorna:
    float: Heading em graus no intervalo [0, 360].
    """
    theta_x = np.degrees(np.arctan(a))  # Inclinação em relação ao eixo X
    heading = 270 + theta_x  # Conversão para referência ao norte
    heading = (heading + 360) % 360 # Normaliza para [0, 360]
    return heading

def rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot) :
    front_left_wheel.setVelocity(max_speed*4)
    front_right_wheel.setVelocity(-max_speed*4)
    back_left_wheel.setVelocity(max_speed*4)
    back_right_wheel.setVelocity(-max_speed*4)
    robot.step(time_step * 4)


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

    # Initialize Sonar Sensors - Use only needed for left wall following
    
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
    
    # Initialize Compass
    compass = robot.getDevice("compass")
    compass.enable(time_step)
    
    left_speed = max_speed
    right_speed = max_speed
    
    erro_anterior = 0
    
    #CONSTANTES DOS CONTROLADORES
    kd_PD = 0.5
    kp_PD = 0.5
    
    #Obs: Q_START fixado somente para vizualizar a reta m_line no mapa, para que seja variante ao start basta dispor x_start e y_start como  position = gps.getValues(), x = position[0], y = position[1]
    # Posições de partida e chegada
    x_start, y_start = -8.223819998184288, 3.6639342262967203  # Ponto de partida
    x_goal, y_goal = 8.546180001815712, -4.35606577370328      # Ponto de chegada
    
    # Calcula os coeficientes da reta
    a, b = calculate_line_coefficients(x_start, y_start, x_goal, y_goal) #Bug 2, m_line (calculada uma vez)
    
    # Considerando o robo como um circulo de raio 0,5
    r = 0.4
    
    i = 0
    
    heading_reta = calcular_heading_reta(a)
    print(heading_reta)
    # Step simulation
    while robot.step(time_step) != -1:
    
        position = gps.getValues()
        x = position[0]
        y = position[1]
        
        # Verifica se o círculo(robo) toca a reta
        if is_circle_touching_line(x, y, r, a, b):
            i = i + 1
            print(f"O robo toca a reta.({i})")

        left_distance = s0_left.getValue() if s0_left.getValue()> 0.0 else s15_left2.getValue()
        front_distance = max(s01_front.getValue(), s02_front.getValue(), s03_front.getValue(), s04_front.getValue(), s05_front.getValue(), s06_front.getValue())

        left_wall = left_distance > 500
        front_wall = front_distance > 980
        
        # print(f"Front reading: {front_distance}, Left reading: {left_distance}")
        
        erro = left_distance/1000 - 0.97 #Quero que sempre se mantenha a 970 de proximidade da parede

        #print(f"----DISTANCES - Front: {front_distance} Left: {left_distance}----")

        if front_wall: #Turn right
            # print("---------------------Turning right---------------------")   
            rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot)
        
        else:
            
            if left_wall: #Follow the wall
                # print("Follow the wall")
                
                #Movimento de ir pra frente guiado pelo controller
                #Controla o erro quando não realiza outros movimentos
                
                #PD
                derivada_erro = (erro - erro_anterior)/time_step
                controle = erro*kp_PD + derivada_erro*kd_PD
            
                if controle > 0.01: #Se erro maior que zero precisa ajustar para a direita, está muito proximo da parede
                    left_speed = max_speed
                    right_speed = max_speed*controle
                    # print("--Muito proximo da parede")
                elif controle < -0.01: #Se erro menor que zero precisa ajustar para a esquerda, está muito distante da parede
                    left_speed = max_speed*controle
                    right_speed = max_speed
                    # print("--Muito distante da parede")
                else:
                    left_speed = max_speed
                    right_speed = max_speed
                    # print("--Distância correta")

                #print(f"Left distance: {left_distance} Erro: {erro} Erro P: {erro*kp_PD} Erro D: {derivada_erro*kd_PD}")
                #print(f"New left speed: {left_speed} New right speed: {right_speed}")

                erro_anterior = erro   
                
            else: #Passed left wall
                # print("Turn Left")
                left_speed = max_speed/8
                right_speed = max_speed

                
        front_left_wheel.setVelocity(left_speed)
        back_left_wheel.setVelocity(left_speed)
        front_right_wheel.setVelocity(right_speed)
        back_right_wheel.setVelocity(right_speed)
        
        pos = gps.getValues()
        pos = [round(pos[0],2), round(pos[1],2), round(pos[2],2)]
        #print("----- Ping position -----")
        #print("X:", pos[0], " Y:", pos[1], " Z:", pos[2])
        
        angle = compass.getValues()
        print(angle)
        
        heading = calcular_heading(angle[0], angle[1])
        print(heading)
                

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
