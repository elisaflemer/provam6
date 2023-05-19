#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose

from collections import deque

MAX_DIFF = 0.1


class Pose(TPose):
    # Classe para guardar poses do robô no mundo

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):

        return f"(x={self.x:.2f}, theta={self.y:.2f})"
    
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other):
        # Checa igualdade dentro da margem de segurança
        return abs(self.x - other.x) < MAX_DIFF \
        and abs(self.y - other.y) < MAX_DIFF \

# Classe de pilha (FILO)
class Stack():
    def __init__(self):
        self.stack = []

    def push(self, item):
        self.stack.append(item)

    def pop(self):
        return self.stack.pop()

# Classe de controle da rota, em fila
class MissionControl(deque):
    def __init__(self):
        super().__init__()
        self.enqueue((0.0, 0.5))   
        self.enqueue((0.5, 0.0)) 
        self.enqueue((0.0, 0.5))   
        self.enqueue((0.5, 0.0))  
        self.enqueue((0.0, 1.0))   
        self.enqueue((1.0, 0.0))        
     
    def enqueue(self, item):
        super().append(item)

    def dequeue(self):
        return super().popleft()

class TurtleController(Node):
    # Controlador da tartaruga
    
    def __init__(self, mission_control, control_period=0.02, stack = Stack()):
        # Recebe a rota em fila, o período de controle e uma fila para guardar a rota já percorrida
        super().__init__('turtle_controller')

        self.pose = Pose(x=-40.0)
        self.setpoint = Pose(x=-40.0)
        self.mission_control = mission_control
        self.stack = stack
        self.initial_pose = Pose()
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )


        self.control_timer = self.create_timer(
                timer_period_sec=control_period,
                callback=self.control_callback
        )

    def control_callback(self):
        # Controla movimentação do robô

        # Se a pose estiver em -40.0, significa que não chegou nenhuma info ainda
        if self.pose.x == -40.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        
        # Compara distância do robô com o destino e envia velocidades de acordo
        msg = Twist()
        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.get_logger().info(f"Robô chegou em {self.pose}")
            self.update_setpoint()
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 0.4 if y_diff > 0 else -0.4
        else:
            msg.linear.y = 0.0
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.4 if x_diff > 0 else -0.4
        else:
            msg.linear.x = 0.0
        self.publisher.publish(msg)
        
    def pose_callback(self, msg):
        #Método de callback da posição da tartaruga
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)

        # Guarda pose inicial no stack e pega primeiro setpoint
        if self.setpoint.x == -40.0:
            print(f"Pose inicial: {self.pose}")
            self.update_setpoint()

    # Atualiza o setpoint
    def update_setpoint(self):
        # Se ainda houver elementos na fila de rota, vai para o próximo elemento
        if(len(self.mission_control) >= 1):
            self.stack.push(self.pose) # Salva posição atual no stack
            # Pega o próximo item da fila
            x, y = self.mission_control.dequeue() 
            self.setpoint = self.pose + Pose(x, y)
            print(f'Setpoint: {self.setpoint}')
            
        # Se a fila de pontos tiver terminado, utiliza o stack para voltar ao ponto inicial    
        elif(len(self.stack.stack) >= 1):
            last_pose = self.stack.pop()
            self.setpoint = last_pose
            print(f'Setpoint: {self.setpoint}')
        
        # Se o stack tiver terminado, chegou ao destino final
        else:
            print(f"Pose final: {self.pose}")
            exit()

# Código principal
def main(args=None):
    rclpy.init(args=args)
    mc = MissionControl()
    tc = TurtleController(mc)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()