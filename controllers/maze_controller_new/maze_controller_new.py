from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math

time_step = 32
max_speed = 1.0
dregrees_per_step_at_max_speed = 0.7

#Obs: Q_START fixado somente para vizualizar a reta m_line no mapa, para que seja variante ao start basta dispor x_start e y_start como  position = gps.getValues(), x = position[0], y = position[1]
# Posições de partida e chegada
x_start, y_start = -8.223819998184288, 3.6639342262967203  # Ponto de partida
x_goal, y_goal = 8.546180001815712, -4.35606577370328      # Ponto de chegada

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
    
def alinhar_com_reta(heading_reta, heading_robo, front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot):
    turn = heading_robo - heading_reta
    
    turn_direction = 1 if turn >= 0 else -1 #Vira para direita se + e esquerda se - 
    
    needed_steps_at_ms = abs(turn/dregrees_per_step_at_max_speed)
    complete_steps = int(needed_steps_at_ms)
    partial_step = abs(needed_steps_at_ms-complete_steps)
    
    #COMPLETE STEPS
    if turn_direction == 1: #Virar para direita
        front_left_wheel.setVelocity(max_speed)
        front_right_wheel.setVelocity(-max_speed)
        back_left_wheel.setVelocity(max_speed)
        back_right_wheel.setVelocity(-max_speed)
        robot.step(time_step * complete_steps)
    else: #Virar para a esquerda
        front_left_wheel.setVelocity(-max_speed)
        front_right_wheel.setVelocity(max_speed)
        back_left_wheel.setVelocity(-max_speed)
        back_right_wheel.setVelocity(max_speed)
        robot.step(time_step * complete_steps)
   
    #PARTIAL STEP   
    if turn_direction == 1: #Virar para direita
        front_left_wheel.setVelocity(max_speed*partial_step)
        front_right_wheel.setVelocity(-max_speed*partial_step)
        back_left_wheel.setVelocity(max_speed*partial_step)
        back_right_wheel.setVelocity(-max_speed*partial_step)
        robot.step(time_step * 1)
    else: #Virar para a esquerda
        front_left_wheel.setVelocity(-max_speed*partial_step)
        front_right_wheel.setVelocity(max_speed*partial_step)
        back_left_wheel.setVelocity(-max_speed*partial_step)
        back_right_wheel.setVelocity(max_speed*partial_step)
        robot.step(time_step * 1)
    
def projetar_ponto_na_reta(x0, y0, a, b):
    # Calcular a coordenada x da projeção
    xp = (x0 + a * (y0 - b)) / (a**2 + 1)
    
    # Calcular a coordenada y da projeção
    yp = a * xp + b
    
    return [xp, yp]

def distance_to_goal(x_point, y_point):
    return math.sqrt((x_goal - x_point)**2 + (y_goal - y_point)**2)


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
    
    # Calcula os coeficientes da reta
    a, b = calculate_line_coefficients(x_start, y_start, x_goal, y_goal) #Bug 2, m_line (calculada uma vez)
    
    # Considerando o robo como um circulo de raio 0,5
    r = 0.1

    i = 0
    segue_reta = False
    dist_ultimo_ponto_reta = distance_to_goal(x_start, y_start)
    
    heading_reta = calcular_heading_reta(a)
    print(heading_reta)
    
    
    # Step simulation
    while robot.step(time_step) != -1:
         
        position = gps.getValues()
        x = position[0]
        y = position[1]
        
        if distance_to_goal(x, y) < 0.2:
            print("Labirinto Completo")
            front_left_wheel.setVelocity(0.0)
            back_left_wheel.setVelocity(0.0)
            front_right_wheel.setVelocity(0.0)
            back_right_wheel.setVelocity(0.0)     
            break     
        
        angle = compass.getValues()
        #print(angle)
                
        heading = calcular_heading(angle[0], angle[1])
        #print(heading)     
        
        # Verifica se o círculo(robo) toca a reta
        if is_circle_touching_line(x, y, r, a, b):
            i = i + 1
            #print(f"O robo toca a reta.({i})")

            pontos_projetados = projetar_ponto_na_reta(x, y, a, b)

            dist_ponto_atual_reta = distance_to_goal(pontos_projetados[0], pontos_projetados[1])

            if (dist_ponto_atual_reta < dist_ultimo_ponto_reta):
                #print("Ponto mais a frente")
                dist_ultimo_ponto_reta = dist_ponto_atual_reta
                segue_reta = True
        
        left_distance = s0_left.getValue() if s0_left.getValue()> 0.0 else s15_left2.getValue()
        front_distance = max(s01_front.getValue(), s02_front.getValue(), s03_front.getValue(), s04_front.getValue(), s05_front.getValue(), s06_front.getValue())
    
        left_wall = left_distance > 500
        front_wall = front_distance > 970

        erro = left_distance/1000 - 0.95 #Quero que sempre se mantenha a 920 de proximidade da parede

        #print(f"----DISTANCES - Front: {front_distance} Left: {left_distance}----")

        if segue_reta and not front_wall:
        
            #print("SEGUIR RETA")
            if abs(heading - heading_reta) > 0.5:
                #print("Alinhando com a reta")
                alinhar_com_reta(heading_reta, heading, front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot)
            
            #Alinhado com a reta basta seguir para frente
            left_speed = max_speed
            right_speed = max_speed

        else:
            
            #print("Para de seguir reta, obstáculo")
            segue_reta = False

            if front_wall: #Turn right
                #print("---------------------Turning right---------------------")   
                rotate_90(front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, robot)
            
            else:
                
                if left_wall: #Follow the wall
                    #print("Follow the wall")
                    
                    #Movimento de ir pra frente guiado pelo controller
                    #Controla o erro quando não realiza outros movimentos
                    
                    #PD
                    derivada_erro = (erro - erro_anterior)/time_step
                    controle = erro*kp_PD + derivada_erro*kd_PD
                
                    if controle > 0.01: #Se erro maior que zero precisa ajustar para a direita, está muito proximo da parede
                        left_speed = max_speed
                        right_speed = max_speed*controle
                        #print("--Muito proximo da parede")
                    elif controle < -0.01: #Se erro menor que zero precisa ajustar para a esquerda, está muito distante da parede
                        left_speed = max_speed*controle
                        right_speed = max_speed
                        #print("--Muito distante da parede")
                    else:
                        left_speed = max_speed
                        right_speed = max_speed
                        #print("--Distância correta")

                    #print(f"Left distance: {left_distance} Erro: {erro} Erro P: {erro*kp_PD} Erro D: {derivada_erro*kd_PD}")
                    #print(f"New left speed: {left_speed} New right speed: {right_speed}")

                    erro_anterior = erro   
                    
                else: #Passed left wall
                    #print("Turn Left")
                    left_speed = max_speed/8
                    right_speed = max_speed

        front_left_wheel.setVelocity(left_speed)
        back_left_wheel.setVelocity(left_speed)
        front_right_wheel.setVelocity(right_speed)
        back_right_wheel.setVelocity(right_speed)         
        
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
