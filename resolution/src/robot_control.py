#!/usr/bin/env python3
 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotControl():
 
    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.summit_laser_callback)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.summit_laser_msg = LaserScan()
        self.ctrl_c = False
        self.rate = rospy.Rate(20)
        self.right_wall_distance = 0.0
 
    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()
 
    def shutdownhook(self):
        self.ctrl_c = True
 
    def laser_callback(self, msg):
        self.laser_msg = msg
        self.right_wall_distance = min(msg.ranges[300:330])
 
    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg
 
    def get_front_laser(self):
                if len(self.laser_msg.ranges) > 360:  # Assurez-vous que l'index 360 est valide
                        return self.laser_msg.ranges[360]
                else:
                        return None  # Retourner None ou une autre valeur par défaut si l'index est invalide
                        
    def get_laser_summit(self, pos):
        return self.summit_laser_msg.ranges[pos]
 
    def get_laser_full(self):
        return self.laser_msg.ranges
 
    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
 
    def move_straight(self):
    # Vérifier s'il y a un obstacle directement devant (index 360 du tableau de données laser)
        if self.get_front_laser() < 0.4:  # Ajustez la distance limite selon vos besoins
        # Arrêter le robot et ajuster sa direction pour éviter la collision
            self.stop_robot()
            self.turn("left", 0.5, 0.1)  # Tourner à gauche (ou à droite selon la situation) pour éviter l'obstacle
        else:
             # Aucun obstacle détecté, avancer tout droit
            self.cmd.linear.x = 0.2  # Réduire la vitesse linéaire à 0.2 ou moins
            self.cmd.linear.y = 0
            self.cmd.linear.z = 0
            self.cmd.angular.x = 0
            self.cmd.angular.y = 0
            self.cmd.angular.z = 0
            self.publish_once_in_cmd_vel()

    def turn(self, clockwise, speed, time):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        if clockwise == "clockwise":
            self.cmd.angular.z = speed
        else:
            self.cmd.angular.z = -speed
            self.publish_once_in_cmd_vel()
            self.rate.sleep()
            self.stop_robot()

    def follow_right_wall(self):
        while not self.ctrl_c:
            if self.right_wall_distance > 0.4:
                self.turn("left", 0.5, 0.1)
            elif self.right_wall_distance < 0.15:
                self.stop_robot()
                self.turn("left", 0.5, 0.1)
            else:
                self.move_straight()
            self.publish_once_in_cmd_vel()
            self.rate.sleep()
 
if __name__ == '__main__':
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.publish_once_in_cmd_vel()
        robotcontrol_object.rate.sleep()
        robotcontrol_object.stop_robot()
        robotcontrol_object.follow_right_wall()
    except rospy.ROSInterruptException:
        pass

