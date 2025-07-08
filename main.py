import cv2 
import numpy as np
import math
import time
import serial
import simple_pid as sp
from PIL import Image

#============================
# Conexion Bluetooth
print("Iniciando bluetooth...")
ser = serial.Serial("COM14", baudrate=38400, timeout=1)
time.sleep(5)
print("Conectado exitosamente!")

def send_command(command):
    command += ";"
    ser.write(command.encode())


#============================
# Funciones Auxiliares

def get_vector(a:tuple, b:tuple):
    '''Obtiene vector desde a -> b'''
    return (b[0]-a[0], b[1]-a[1])


def get_angle(vec1:tuple, vec2:tuple)->float:
    '''Obtiene angulo entre vec1 y vec2'''
    vec1 = np.array(vec1)
    vec2 = np.array(vec2)

    #Normalizar
    try:
        vec1 = vec1/np.linalg.norm(vec1)
        vec2 = vec2/np.linalg.norm(vec2)
    except:
        print("Alguno de los vectores fallo en get_angle :(")
        return 0

    dot_prod = np.dot(vec1, vec2)
    angle = np.degrees(np.arccos(dot_prod))

    #Obtener signo
    cross = vec1[0]*vec2[1] - vec1[1]*vec2[0]
    if cross < 0:
        angle = -angle

    return angle


def get_center_coords(box_coords)->tuple:
    '''Obtiene coordenadas centrales de una bounding box'''
    if box_coords != None:
        x1, y1, x2, y2 = box_coords
        return (x1 + int((x2-x1)/2), y1 + int((y2-y1)/2))

    return (0, 0)


def update_target(new_target:tuple)->None:
    global OBJECTIVE_POS
    '''Actualiza el target actual'''
    with data_lock:
        OBJECTIVE_POS = new_target


#============================
#Threading xd
from threading import Thread, Lock
data_lock = Lock()
executing_order = False

#Constantes calculos
TEAM_SIDE = 0 # 0=Left | 1=Right
SCREEN_WIDTH = 0
SCREEN_HEIGHT = 0

#Indices
FRONT, BACK = 1, 0
LEFT, RIGHT = 0, 1

#Posiciones
BALL_POS = (0, 0)
LEFT_COURT_POS = (0, 0)
RIGHT_COURT_POS = (0, 0)
LEFT_QUARTER_POS = (0, 0)
RIGHT_QUARTER_POS = (0, 0)
CENTER_POS = (0, 0)
OBJECTIVE_POS = (0, 0)


class Robot:
    def __init__(self):
        #Stats
        self.positions = [(0, 0), (0, 0)]
        self.angle = 0.0
        self.to_target_vector = (0, 0)
        self.circles_vector = (0, 0)

        #PIDs
        #TODO: Calibrar los PID cercanos y lejanos xd
        self.farAnglePid = sp.PID(Kp=2, Ki=0, Kd=0, setpoint=0)
        self.farPosPid = sp.PID(Kp=2, Ki=0, Kd=1, setpoint=0)

        self.closeAnglePid = sp.PID(Kp=2, Ki=0, Kd=0, setpoint=0)
        self.closePosPid = sp.PID(Kp=1, Ki=0.01, Kd=0, setpoint=0)

        #Error Margins
        #TODO: Obtener bien margenes de error
        self.TOUCH_MARGIN = 135
        self.POS_MARGIN = 180
        self.THETA_MARGIN = 5
        self.FAR_POS_THRESHOLD = 200
        self.FAR_ANGLE_THRESHOLD = 20
    
    def update(self, new_positions:list[tuple])->None:
        '''Actualizar robot'''
        global OBJECTIVE_POS
        self.positions = new_positions
        self.circles_vector = get_vector(self.positions[BACK], self.positions[FRONT])
        self.to_target_vector = get_vector(self.positions[BACK], OBJECTIVE_POS)
        self.angle = get_angle(self.to_target_vector, self.circles_vector)
    
    def send_speeds(self, left, rigth)->None:
        command = f"V{left} {rigth}"
        send_command(command)
        print(f"Sent --> Left:{left}\t\tRight:{rigth}")


    def get_theta_powers(self, angle:float)->list:
        '''Obtener potencias para las ruedas segun angulo'''
        if(angle < self.FAR_ANGLE_THRESHOLD):
            power = self.closeAnglePid(angle)
        else:
            power = self.farAnglePid(angle)
        return [power, -power]


    def get_pos_powers(self, dist:int)->list:
        '''Obtener potencias para las ruedas segun distancia'''
        if(dist < self.FAR_POS_THRESHOLD):
            power = self.closePosPid(dist)
        else:
            power = self.farPosPid(dist)
        return [-power, -power]


    def align(self)->None:
        '''Alinearse con la posicion deseada'''
        with data_lock:
            angle = self.angle
        
        while(abs(angle) > self.THETA_MARGIN):
            with data_lock:
                self.send_speeds(*self.get_theta_powers(angle))
                angle = self.angle
            time.sleep(0.01)


    def go_to(self, touch:bool=True)->None:
        '''Avanzar controladito hacia posicion deseada'''
        global OBJECTIVE_POS
        with data_lock:
            print(f"Avanzando a: {OBJECTIVE_POS}...")

        while True:
            with data_lock:
                vec1 = self.to_target_vector
                angle = self.angle

            #Distancia
            dist = np.linalg.norm(vec1)

            #Llego?
            if((dist < self.TOUCH_MARGIN and touch) or (dist < self.POS_MARGIN and not touch)) and abs(angle) < self.THETA_MARGIN:
                break

            if abs(angle) > self.FAR_ANGLE_THRESHOLD:
                #Alinear de nuevo
                speeds = self.get_theta_powers(angle)
            else:
                #Avanzar
                speeds = self.get_pos_powers(dist)

            self.send_speeds(*speeds)
        
        self.send_speeds(0, 0)
        time.sleep(0.5)



ronaldinho = Robot()

#=====================
# COLORES

#Colores fijos
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

# Rangos de color
                       
LOW_RED = np.array([0, 140, 190])
HIGH_RED = np.array([10, 190, 255])

LOW_LIGHT_BLUE = np.array([90, 100, 130])
HIGH_LIGHT_BLUE = np.array([130, 255, 255])

LOW_BLUE = np.array([100, 150, 0])
HIGH_BLUE = np.array([140, 255, 255])

LOW_PURPLE = np.array([130, 100, 20])
HIGH_PURPLE = np.array([160, 255, 255])

LOW_YELLOW = np.array([20, 150, 150])
HIGH_YELLOW = np.array([35, 255, 255])

LOW_GREEN = np.array([40, 100, 100])
HIGH_GREEN = np.array([80, 255, 255])

LOW_PINK = np.array([150, 50, 50])
HIGH_PINK = np.array([179, 255, 254])

LOW_ORANGE = np.array([5, 100, 70])
HIGH_ORANGE = np.array([15, 255, 255])

# Colores de las cosas
robot_colors = [[LOW_LIGHT_BLUE, HIGH_LIGHT_BLUE], [LOW_PINK, HIGH_PINK]]
ball_colors = [LOW_YELLOW, HIGH_YELLOW]
left_court_colors = [LOW_BLUE, HIGH_BLUE]
right_court_colors = [LOW_PINK, HIGH_PINK]


#=====================
#FUNCIONES CON IMAGENES

def rgb_to_hsv(img):
    '''Abrir una imagen y transformarla a HSV'''
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def make_bounding_box(img, coords: tuple, rounded:bool=False, text:str="", color:tuple=GREEN)->None:
    '''Funcion para agregar una bounding box a la imagen'''
    if coords is None:
        return
    
    if not rounded:
        x1, y1, x2, y2 = coords
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    else:
        cx, cy = coords
        cv2.circle(img, (cx, cy), 2, color, 5)
        cv2.putText(img, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def get_object_box(img, low, high, rounded:bool=False, name:str="", color:tuple=GREEN) -> tuple:
    '''Funcion para obtener bounding boxes de un objeto dado un rango de colores HSV'''
    hsv_img = rgb_to_hsv(img)
    mask = cv2.inRange(hsv_img, low, high)

    best_box = None
    min_area = 100
    best_circularity = 0

    #Filtrar contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        #Ver si circulo es suficientemente bueno
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue 
        circularity = 4*math.pi*(area/(perimeter**2))

        if rounded:
            if circularity > 0.5 and circularity > best_circularity:
                best_circularity = circularity
                best_box = cnt
        else:
            if circularity < 0.5:
                best_box = cnt

    #Dibujar
    if best_box is not None:
        x, y, w, h = cv2.boundingRect(best_box)
        box_coords = (x, y, x+w, y+h)
        if not rounded:
            make_bounding_box(img, box_coords, False, name, color)
            return (x+w//2, y+h//2)
        else:
            cx, cy = (x + w//2, y + h//2)
            make_bounding_box(img, (cx, cy), True, "", color)
            make_bounding_box(img, box_coords, False, name, color)
            return (cx, cy)
    

def update_all(img):
    '''Actualizar todo segun detecciones'''
    global CENTER_POS, LEFT_COURT_POS, RIGHT_COURT_POS, LEFT_QUARTER_POS, RIGHT_QUARTER_POS
    global SCREEN_HEIGHT, SCREEN_WIDTH
    global BALL_POS, OBJECTIVE_POS


    #Obtener objetos
    ball = get_object_box(img, *ball_colors, rounded=True, name="Pelota")
    # left_court = get_object_box(img, *left_court_colors, name="Arco izquierdo")
    # right_court = get_object_box(img, *right_court_colors, name="Arco derecho")
    robot_back = get_object_box(img, *robot_colors[0], rounded=True, name="Back")
    robot_front = get_object_box(img, *robot_colors[1], rounded=True, name="Front")

    # Actualizar posiciones globales
    with data_lock:
        #Para calculos
        SCREEN_WIDTH = int(img.shape[1])
        SCREEN_HEIGHT = int(img.shape[0])
        CENTER_POS = (SCREEN_WIDTH//2, SCREEN_HEIGHT//2)

        LEFT_COURT_POS = (SCREEN_WIDTH//7, SCREEN_HEIGHT//2)
        RIGHT_COURT_POS = ((SCREEN_WIDTH//7)*6, SCREEN_HEIGHT//2)
        LEFT_QUARTER_POS = (SCREEN_WIDTH//4, SCREEN_HEIGHT//2)
        RIGHT_QUARTER_POS = ((SCREEN_WIDTH//4)*3, SCREEN_HEIGHT//2)

        if robot_back and robot_front:
            ronaldinho.update([robot_back, robot_front])

        if ball:
            BALL_POS = ball
            if not executing_order:
                OBJECTIVE_POS = ball

    #Dibujar
    with data_lock:
        cv2.line(img, ronaldinho.positions[BACK], OBJECTIVE_POS, BLACK, 2)
        cv2.arrowedLine(img, ronaldinho.positions[BACK], ronaldinho.positions[FRONT], BLACK, 2, tipLength=0.1)
        cv2.circle(img, BALL_POS, 2, (0, 0, 0), -1)

        #Obtener vectores clave
        vec1 = ronaldinho.to_target_vector
        vec2 = ronaldinho.circles_vector


    d = round(np.linalg.norm(vec1))
    cv2.putText(img, f"Distancia: {d} pixeles", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 2)

    angle = round(get_angle(vec1, vec2), 2)
    cv2.putText(img, f"Angulo: {angle} grados", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 2)

    cv2.imshow('original', img)


#=====================

def align_to_ball():
    '''El robot debe girar hasta que su frente apunte directamente al balón, minimizando el ángulo de orientación'''
    update_target(BALL_POS)
    ronaldinho.align()


def go_to_center():
    '''El robot debe moverse hasta el centro de la pista, indicado por una línea transversal negra.'''
    update_target(CENTER_POS)
    ronaldinho.go_to()


def chase_ball():
    '''El robot debe acercarse al balón sin tocarlo.'''
    update_target(BALL_POS)
    ronaldinho.go_to(touch=False)
    

def straight_push():
    '''Desde una orientación inicial adecuada, el robot debe empujar el balón en línea recta hasta el final de la
cancha.'''
    global LEFT_COURT_POS, RIGHT_COURT_POS
    global LEFT_QUARTER_POS, RIGHT_QUARTER_POS

    with data_lock:
        left_court, right_court = LEFT_COURT_POS, RIGHT_COURT_POS
        left_quarter, right_quarter = LEFT_QUARTER_POS, RIGHT_QUARTER_POS

    chase_ball()
    if(TEAM_SIDE == LEFT):
        update_target(right_court)
    else:
        update_target(left_court)
    ronaldinho.go_to()       


def push_to_enemy():
    '''Desde posiciones aleatorias asignadas por el cuerpo docente, el robot debe localizar el balón y empujarlo
hacia el arco enemigo.'''
    global LEFT_COURT_POS, RIGHT_COURT_POS
    global LEFT_QUARTER_POS, RIGHT_QUARTER_POS

    with data_lock:
        left_court, right_court = LEFT_COURT_POS, RIGHT_COURT_POS
        left_quarter, right_quarter = LEFT_QUARTER_POS, RIGHT_QUARTER_POS
        pos = ronaldinho.circles_vector
    
    if(TEAM_SIDE == LEFT):
        update_target(left_quarter)
    else:
        update_target(right_quarter)
    
    ronaldinho.go_to()
    update_target(BALL_POS)
    ronaldinho.align()

    if(TEAM_SIDE == LEFT):
        update_target(right_court)
    else:
        update_target(left_court)
    ronaldinho.go_to()


    

def kick_to_enemy():
    '''Similar al caso anterior, pero ahora el robot debe golpear el balón (en lugar de empujarlo) hasta que llegue
al arco enemigo.'''
    pass

#=====================

def execute_order(order: int)->None:
    global executing_order
    print(f"Ejecutando: {order} -> ", end="")

    if(order == 24):
        print("Alinear con balon")
        align_to_ball()
    elif(order == 25):
        print("Ir al centro")
        go_to_center()
    elif(order == 26):
        print("Ir por pelota")
        chase_ball()
    elif(order == 27):
        print("Pushear pelota al otro lado de la cancha")
        straight_push()
    elif(order == 28):
        print("Pushear al arco rival")
        push_to_enemy()
    elif(order == 29):
        print("Patear al arco rival")
        kick_to_enemy()
    
    with data_lock:
        executing_order = False
    print("Orden completa :D")
    ronaldinho.send_speeds(0, 0)


def handle_input():
    global executing_order
    while(True):
        user = input("Comando a ejecutar: ")
        with data_lock:
            executing_order = True
        
        execute_order(int(user))

        while(True):
            with data_lock:
                if not(executing_order):
                    break
    
        
#Programa principal 
if __name__ == "__main__":
    #Obtener equipo
    print("Ronaldhino activado :D")

    #Abrir todo
    cv2.namedWindow('original', cv2.WINDOW_NORMAL)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    # cap = cv2.VideoCapture("PerceptionDataset.mp4")

    if not cap.isOpened():
        print("No se pudo abrir la cámara :(")
        exit()

    side = input("(I / D) Donde esta el arco enemigo?: ")
    TEAM_SIDE = LEFT if side.upper() == "I" else RIGHT

    if TEAM_SIDE == LEFT:
        print("Somos el equipo izquierdo (Azul)!")
    else:
        print("Somos el equipo derecho (Morado)!")

    t1 = Thread(target=handle_input, daemon=True)
    t1.start()

    #Desarrollo
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error leyendo frame :(")
            continue

        #Actualizar datos
        update_all(frame)

        #Tecla Q = salir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break