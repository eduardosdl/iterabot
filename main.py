import pybullet as p
import pybullet_data
import time
import paho.mqtt.client as mqtt
import json
import math
import threading
import numpy as np

# ==============
# configurações
# ==============

MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_USER = 'edu'
MQTT_PASSWORD = 'eduardo'
MQTT_COMMAND_TOPIC = 'robot/command'
MQTT_STATUS_TOPIC = 'robot/status'
MQTT_ACTION_TOPIC = 'robot/action'

PHYSICS_TIME_STEP = 1./240.
PHYSICS_SOLVER_ITERATIONS = 150
GRAVITY = -10

OBSTACLE_SIZE = 16
WALL_HEIGHT = 1
WALL_THICKNESS = 0.05
WALL_COLOR = [1, 1, 1, 1]
BORDER_LIMIT = OBSTACLE_SIZE / 2 - 0.5
OBSTACLE_QUANTITY = 10
OBSTACLE_COLOR = [0, 0, 1, 1]

DEFAULT_WHEEL_SPEED = 15.0
DEFAULT_WHEEL_FORCE = 10.0
ROBOT_RADIUS = 0.4
ROBOT_WHEEL_JOINTS = [2, 3, 6, 7]
ROBOT_LEFT_WHEELS = [2, 3]
ROBOT_RIGHT_WHEELS = [6, 7]

FORWARD_DETECTION_DISTANCE = 0.35
SAFE_MARGIN = 0.1
OBSTACLE_DETECTION_DISTANCE = 0.8
SENSOR_HEIGHT = 0.5

# =================
# variaveis globais
# =================

physics_client = None
robot_id = None
current_action = "stop"
robot_state = "stopped"
object_detected = False
mqtt_client = None

# =========================
# inicialização do ambiente
# =========================

def initialize_physics():
    # iniciação do pybullet
    global physics_client
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY)
    p.setPhysicsEngineParameter(fixedTimeStep=PHYSICS_TIME_STEP, numSolverIterations=PHYSICS_SOLVER_ITERATIONS)
    p.loadURDF("plane.urdf")

def create_wall(x, y, x_size, y_size):
    # criação de parede para criação da limitação do ambiente
    visual_shape = p.createVisualShape(
        p.GEOM_BOX, 
        halfExtents=[x_size, y_size, WALL_HEIGHT / 2], 
        rgbaColor=WALL_COLOR
    )
    collision_shape = p.createCollisionShape(
        p.GEOM_BOX, 
        halfExtents=[x_size, y_size, WALL_HEIGHT / 2]
    )
    p.createMultiBody(
        baseMass=0, 
        baseCollisionShapeIndex=collision_shape, 
        baseVisualShapeIndex=visual_shape, 
        basePosition=[x, y, WALL_HEIGHT / 2]
    )

def create_room_walls():
    # montagem das 4 paredes
    half_size = OBSTACLE_SIZE / 2
    half_thickness = WALL_THICKNESS / 2
    
    create_wall(0, half_size, half_size, half_thickness)
    create_wall(0, -half_size, half_size, half_thickness)
    create_wall(half_size, 0, half_thickness, half_size)
    create_wall(-half_size, 0, half_thickness, half_size)

def generate_obstacle_positions():
    # gerando obstáculos aleatórios dentro do ambiente
    obstacles = []
    attempts = 0
    max_attempts = 300
    min_distance_from_center = 1.5
    min_distance_between_obstacles = 1.2
    
    while len(obstacles) < OBSTACLE_QUANTITY and attempts < max_attempts:
        x = round(np.random.uniform(-OBSTACLE_SIZE/2 + 1.5, OBSTACLE_SIZE/2 - 1.5), 1)
        y = round(np.random.uniform(-OBSTACLE_SIZE/2 + 1.5, OBSTACLE_SIZE/2 - 1.5), 1)
        
        # removendo da posição de inicio do robo
        if abs(x) < min_distance_from_center and abs(y) < min_distance_from_center:
            attempts += 1
            continue
        
        # analisando a distancia dos obstáculos
        if all(np.linalg.norm(np.array([x, y]) - np.array([ox, oy])) > min_distance_between_obstacles 
               for ox, oy, _ in obstacles):
            obstacles.append((x, y, 0.5))
        
        attempts += 1
    
    return obstacles

def create_maze_obstacles():
    # criação dos obstáculos no ambiente
    obstacle_positions = generate_obstacle_positions()
    obstacle_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.8])
    
    for position in obstacle_positions:
        obstacle_id = p.createMultiBody(0, obstacle_collision_shape, -1, position)
        p.changeVisualShape(obstacle_id, -1, rgbaColor=OBSTACLE_COLOR)

def create_robot():
    # inicialização do robô
    global robot_id
    robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1], useFixedBase=False)

def setup_scene():
    # chamando todas as funções de configuração do ambiente
    initialize_physics()
    create_room_walls()
    create_maze_obstacles()
    create_robot()

# ==============================
# configurando movimento do robô
# ==============================

def set_joint_motor_control(joint_index, target_velocity, force=DEFAULT_WHEEL_FORCE):
    # movimento uma junta do robo (roda)
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=joint_index,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=target_velocity,
        force=force
    )

def set_joints_motor_controls(joints_array, target_velocity):
    # movimento varias juntas do robo (rodas)
    for joint in joints_array:
        set_joint_motor_control(joint, target_velocity)

### MOVIMENTOS DO ROBÔ ###

def move_forward():
    global robot_state
    robot_state = "moving_forward"
    set_joints_motor_controls(ROBOT_WHEEL_JOINTS, -DEFAULT_WHEEL_SPEED)

def move_backward():
    global robot_state
    robot_state = "moving_backward"
    set_joints_motor_controls(ROBOT_WHEEL_JOINTS, DEFAULT_WHEEL_SPEED)

def turn_left():
    global robot_state
    robot_state = "turning_left"
    set_joints_motor_controls(ROBOT_LEFT_WHEELS, -DEFAULT_WHEEL_SPEED)
    set_joints_motor_controls(ROBOT_RIGHT_WHEELS, DEFAULT_WHEEL_SPEED)

def turn_right():
    global robot_state
    robot_state = "turning_right"
    set_joints_motor_controls(ROBOT_LEFT_WHEELS, DEFAULT_WHEEL_SPEED)
    set_joints_motor_controls(ROBOT_RIGHT_WHEELS, -DEFAULT_WHEEL_SPEED)

def stop_robot():
    global robot_state
    robot_state = "stopped"
    set_joints_motor_controls(ROBOT_WHEEL_JOINTS, 0)

# ======================
# detecção de obstáculos
# ======================

def get_forward_position(position, orientation, distance=FORWARD_DETECTION_DISTANCE):
    # calcula a posição à frente do robô
    euler = p.getEulerFromQuaternion(orientation)
    yaw = euler[2]
    front_x = position[0] + distance * math.cos(yaw)
    front_y = position[1] + distance * math.sin(yaw)
    return front_x, front_y

def is_within_safe_area(position, orientation, margin=SAFE_MARGIN):
    # verifica se o robô está dentro da área segura
    front_x, front_y = get_forward_position(position, orientation)
    limit = OBSTACLE_SIZE / 2 - margin
    return -limit <= front_x <= limit and -limit <= front_y <= limit

def is_robot_in_area(point, margin=SAFE_MARGIN):
    # valida se o robô está dentro da área segura
    limit = OBSTACLE_SIZE / 2 - margin
    return -limit <= point[0] <= limit and -limit <= point[1] <= limit

def detect_forward_obstacles(position, orientation, distance=OBSTACLE_DETECTION_DISTANCE):
    # detecta obstáculos à frente do robô usando ray casting (algoritmo de tratamento de imagens)
    euler = p.getEulerFromQuaternion(orientation)
    quaternion = p.getQuaternionFromEuler([0, 0, euler[2]])
    rotation_matrix = np.array(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
    
    forward_direction = rotation_matrix @ np.array([0, 1, 0])
    lateral_direction = rotation_matrix @ np.array([1, 0, 0])
    
    lateral_offset = ROBOT_RADIUS * 0.6
    lateral_displacements = [-lateral_offset, 0, lateral_offset]
    
    for i, lateral in enumerate(lateral_displacements):
        ray_range = distance * 1.5 if lateral == 0 else distance
        ray_start = [
            position[0] + lateral_direction[0] * lateral,
            position[1] + lateral_direction[1] * lateral,
            SENSOR_HEIGHT
        ]
        ray_end = [
            ray_start[0] + forward_direction[0] * ray_range,
            ray_start[1] + forward_direction[1] * ray_range,
            ray_start[2] + forward_direction[2] * ray_range
        ]
        
        color = [0, 1, 0] if lateral == 0 else [0.5, 1, 0.5]
        p.addUserDebugLine(ray_start, ray_end, color, 2, 0.1)
        
        ray_result = p.rayTest(ray_start, ray_end)[0]
        hit_object_id = ray_result[0]
        
        if hit_object_id != -1 and hit_object_id != robot_id:
            return True
    
    return False

def detect_backward_obstacles(position, orientation, distance=OBSTACLE_DETECTION_DISTANCE):
    # detecta obstáculos atrás
    euler = p.getEulerFromQuaternion(orientation)
    quaternion = p.getQuaternionFromEuler([0, 0, euler[2]])
    rotation_matrix = np.array(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
    
    backward_direction = rotation_matrix @ np.array([0, -1, 0])
    lateral_direction = rotation_matrix @ np.array([1, 0, 0])
    
    lateral_offset = ROBOT_RADIUS * 0.6
    lateral_displacements = [-lateral_offset, 0, lateral_offset]
    
    for lateral in lateral_displacements:
        ray_start = [
            position[0] + lateral_direction[0] * lateral,
            position[1] + lateral_direction[1] * lateral,
            SENSOR_HEIGHT
        ]
        
        ray_range = distance * 1.5 if lateral == 0 else distance
        ray_end = [
            ray_start[0] + backward_direction[0] * ray_range,
            ray_start[1] + backward_direction[1] * ray_range,
            ray_start[2] + backward_direction[2] * ray_range
        ]
        
        color = [1, 0.5, 0] if lateral == 0 else [1, 0.8, 0.8]
        p.addUserDebugLine(ray_start, ray_end, color, 2, 0.1)
        
        ray_result = p.rayTest(ray_start, ray_end)[0]
        hit_object_id = ray_result[0]
        
        if hit_object_id != -1 and hit_object_id != robot_id:
            return True
    
    return False

# ============
# conexão MQTT
# ============

def send_current_action(action):
    # Envia a ação atual do robô via MQTT
    global mqtt_client
    try:
        position, _ = p.getBasePositionAndOrientation(robot_id)
        action_data = {
            "action": action,
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2])
            },
            "timestamp": time.time()
        }
        if mqtt_client:
            mqtt_client.publish(MQTT_ACTION_TOPIC, json.dumps(action_data))
            print(f"Action sent: {current_action} - State: {robot_state}")
    except Exception as e:
        print(f"Error sending action: {e}")

def send_object_detection_status():
    # envia status de detecção de objeto via MQTT
    global mqtt_client
    try:
        position, _ = p.getBasePositionAndOrientation(robot_id)
        status_data = {
            "status": "object_detected",
            "message": "Object detected - robot stopped",
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2])
            },
            "timestamp": time.time()
        }
        if mqtt_client:
            mqtt_client.publish(MQTT_STATUS_TOPIC, json.dumps(status_data))
            print("Object detection message sent to robot/status")
    except Exception as e:
        print(f"Error sending status: {e}")

def on_connect(client_instance, userdata, flags, rc):
    # callback para conexão MQTT
    print(f"Connected to MQTT broker with code: {rc}")
    client_instance.subscribe(MQTT_COMMAND_TOPIC)

def on_message(client_instance, userdata, msg):
    # callback para recebimento de mensagens MQTT
    command = msg.payload.decode()
    send_current_action(command)
    execute_command(command)
    print(f"Command received: {command}")

def execute_command(command):
    # altera o comando recebido via MQTT para executar a ação correspondente
    global current_action
    
    if command == "front":
        current_action = "forward"
    elif command == "back":
        current_action = "backward"
    elif command == "stop":
        current_action = "stop"
    elif command == "left":
        current_action = "turn_left"
    elif command == "right":
        current_action = "turn_right"


def mqtt_connection_thread():
    # cria uma thread para conexão MQTT
    global mqtt_client
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"MQTT connection error: {e}")

# ========================================
# execução principal de movimentos do robo
# ========================================

def movement_control_loop():
    # Loop de controle de movimento do robô
    global current_action, robot_state, object_detected
    
    try:
        while True:
            position, orientation = p.getBasePositionAndOrientation(robot_id)
            
            # detecta obstáculos à frente e atrás do robô
            obstacle_ahead = detect_forward_obstacles(position, orientation, distance=0.6)
            obstacle_behind = detect_backward_obstacles(position, orientation, distance=0.6)
            
            # verifica se o robô está dentro da área segura
            if current_action == "forward" and obstacle_ahead:
                if not object_detected:
                    object_detected = True
                    send_object_detection_status()
                stop_robot()
                
            elif current_action == "backward" and obstacle_behind:
                if not object_detected:
                    object_detected = True
                    send_object_detection_status()
                stop_robot()
                
            # executa a ação atual do robô
            elif current_action == "forward":
                if is_within_safe_area(position, orientation) and not obstacle_ahead:
                    move_forward()
                    object_detected = False
                else:
                    if not object_detected:
                        object_detected = True
                        send_object_detection_status()
                    stop_robot()
                    
            elif current_action == "backward":
                if is_robot_in_area(position) and not obstacle_behind:
                    move_backward()
                    object_detected = False
                else:
                    if not object_detected:
                        object_detected = True
                        send_object_detection_status()
                    stop_robot()
            
            elif current_action == "turn_left":
                turn_left()
                object_detected = False
                
            elif current_action == "turn_right":
                turn_right()
                object_detected = False
                
            else:  # stop
                stop_robot()
                object_detected = False
            
            p.stepSimulation()
            time.sleep(PHYSICS_TIME_STEP)
            
    except p.error:
        print("PyBullet server disconnected.")

# metodo principal
def main():
    # inicia cenário
    setup_scene()
    
    # inicia thrad de conexão MQTT
    mqtt_thread = threading.Thread(target=mqtt_connection_thread, daemon=True)
    mqtt_thread.start()
    
    # inicia thread de controle de movimento do robô
    movement_thread = threading.Thread(target=movement_control_loop, daemon=True)
    movement_thread.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()