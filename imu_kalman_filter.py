# Kamila G.
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

# Clase - Kalman 
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance  # Varianza del proceso (Q)
        self.measurement_variance = measurement_variance  # Varianza de medición (R)
        self.estimated_error = 1.0  # Error de estimación posterior inicial (P_posterior)
        self.current_estimate = 0.0  # Estado posterior inicial (x_posterior)

    def update(self, measurement):
        # predicción
        self.estimated_error += self.process_variance  # P_prior = P_posterior + Q

        # Hanancia de Kalman
        kalman_gain = self.estimated_error / (self.estimated_error + self.measurement_variance)  # K

        # Actualizamos el estado
        self.current_estimate += kalman_gain * (measurement - self.current_estimate)  # x_posterior

        # Actualizamos el error
        self.estimated_error = (1 - kalman_gain) * self.estimated_error  # P_posterior

        return self.current_estimate

# Nodo principal
def imu_kalman_node():
    rospy.init_node('imu_kalman_filter', anonymous=True)
    rospy.loginfo("Nodo Kalman Filter iniciado") 

    # instancias
    roll_filter = KalmanFilter(process_variance=1.0, measurement_variance=0.5)
    pitch_filter = KalmanFilter(process_variance=1.0, measurement_variance=0.5)
    yaw_filter = KalmanFilter(process_variance=1.0, measurement_variance=0.5)

    # publishers para los datos al topic
    roll_pub = rospy.Publisher('/imu_filtered/roll', Float32, queue_size=10)
    pitch_pub = rospy.Publisher('/imu_filtered/pitch', Float32, queue_size=10)
    yaw_pub = rospy.Publisher('/imu_filtered/yaw', Float32, queue_size=10)

    def roll_callback(data):
        rospy.loginfo(f"Datos recibidos en /imu_euler/roll: {data.data}")  
        filtered_roll = roll_filter.update(data.data) 
        rospy.loginfo(f"Roll filtrado: {filtered_roll}")  
        roll_pub.publish(filtered_roll)  

    def pitch_callback(data):
        rospy.loginfo(f"Datos recibidos en /imu_euler/pitch: {data.data}")  
        filtered_pitch = pitch_filter.update(data.data)  
        rospy.loginfo(f"Pitch filtrado: {filtered_pitch}")  
        pitch_pub.publish(filtered_pitch)  

    def yaw_callback(data):
        rospy.loginfo(f"Datos recibidos en /imu_euler/yaw: {data.data}")  
        filtered_yaw = yaw_filter.update(data.data) 
        rospy.loginfo(f"Yaw filtrado: {filtered_yaw}")  
        yaw_pub.publish(filtered_yaw) 

    
    rospy.Subscriber('/imu_euler/roll', Float32, roll_callback)
    rospy.Subscriber('/imu_euler/pitch', Float32, pitch_callback)
    rospy.Subscriber('/imu_euler/yaw', Float32, yaw_callback)

    # No mimir
    rospy.spin()

if __name__ == '__main__':
    imu_kalman_node()
