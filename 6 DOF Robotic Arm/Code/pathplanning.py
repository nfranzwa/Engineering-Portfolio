import sys
import serial
import numpy as np
from scipy.optimize import minimize
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QTextEdit
from PyQt5.QtCore import Qt, QTimer

# DH parameters [a, alpha, d, theta]
dhparams = np.array([
    [23.4200, -np.pi/2, 110.5000, 0],
    [180.0000, np.pi, 0, -np.pi/2],
    [-43.5000, np.pi/2, 0, np.pi],
    [0, -np.pi/2, -176.3500, 0],
    [0, np.pi/2, 0, 0],
    [0, np.pi, -125.0750, np.pi]
])

# Joint limits in radians
joint_limits = np.array([
    [-123.046875, 123.046875],
    [-55.0088, 86.625],
    [-72.134, 107.8675],
    [-105.46975, 105.46975],
    [-90, 90],
    [-360, 360]
]) * np.pi / 180  # Convert to radians

joint_weights = np.array([1.0, 1.1, 0.9, 1.4, 0.8, 1.0])

def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta):
    T = np.eye(4)
    for i in range(6):
        a, alpha, d, theta_offset = dhparams[i]
        T = T @ dh_matrix(a, alpha, d, theta[i] + theta_offset)
    return T

def objective_function(theta, target_position, previous_theta):
    T = forward_kinematics(theta)
    current_position = T[:3, 3]
    current_orientation = T[:3, :3]
    
    # Calculate position error
    position_error = np.sum((current_position - target_position)**2)
    
    # Calculate orientation error (we want the z-axis of the end-effector to point downward)
    desired_z_axis = np.array([0, 0, -1])  # Pointing downward
    current_z_axis = current_orientation[:, 2]
    orientation_error = np.sum((current_z_axis - desired_z_axis)**2)
    
    # Calculate joint change penalty
    joint_change_penalty = np.sum(joint_weights * (theta - previous_theta)**2)
    
    # Combine all error terms
    total_error = position_error + 10 * orientation_error + 0.1 * joint_change_penalty
    return total_error

def inverse_kinematics(target_position, previous_theta=None):
    if previous_theta is None:
        previous_theta = np.zeros(6)
    
    result = minimize(
        lambda x: objective_function(x, target_position, previous_theta),
        previous_theta,
        method='L-BFGS-B',
        bounds=joint_limits
    )
    return result.x

class RoboticArmGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm Control")
        self.setGeometry(100, 100, 400, 450)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.init_ui()
        self.init_serial()
        self.previous_theta = np.zeros(6)  # Initialize previous joint angles
        self.z_offset = self.calculate_z_offset()  # Initialize z-offset

    def init_ui(self):
        # Create buttons
        button_layout = QHBoxLayout()
        self.homing_button = QPushButton("Start Homing")
        self.shutdown_button = QPushButton("Start Shutdown")
        self.reset_button = QPushButton("Reset Joints")
        button_layout.addWidget(self.homing_button)
        button_layout.addWidget(self.shutdown_button)
        button_layout.addWidget(self.reset_button)
        self.layout.addLayout(button_layout)

        bldc_layout = QHBoxLayout()
        self.open_button = QPushButton("Open BLDC")
        self.close_button = QPushButton("Close BLDC")
        bldc_layout.addWidget(self.open_button)
        bldc_layout.addWidget(self.close_button)
        self.layout.addLayout(bldc_layout)

        # Create input fields for X, Y, Z coordinates
        coord_layout = QHBoxLayout()
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.z_input = QLineEdit()
        coord_layout.addWidget(QLabel("X (mm):"))
        coord_layout.addWidget(self.x_input)
        coord_layout.addWidget(QLabel("Y (mm):"))
        coord_layout.addWidget(self.y_input)
        coord_layout.addWidget(QLabel("Z (mm):"))
        coord_layout.addWidget(self.z_input)
        self.layout.addLayout(coord_layout)

        # Create "Move to Position" button
        self.move_button = QPushButton("Move to Position")
        self.layout.addWidget(self.move_button)

        # Create text area for displaying calculated angles
        self.angle_display = QTextEdit()
        self.angle_display.setReadOnly(True)
        self.layout.addWidget(self.angle_display)

        # Connect signals to slots
        self.homing_button.clicked.connect(self.start_homing)
        self.shutdown_button.clicked.connect(self.start_shutdown)
        self.reset_button.clicked.connect(self.reset_joints)
        self.move_button.clicked.connect(self.move_to_position)
        self.open_button.clicked.connect(self.open_bldc)
        self.close_button.clicked.connect(self.close_bldc)

    def init_serial(self):
        try:
            self.serial_port = serial.Serial("/dev/cu.usbmodem1101", 115200)  # Increased baud rate
        except serial.SerialException:
            print("Failed to open serial port. Make sure the Arduino is connected.")

    def calculate_z_offset(self):
        # Define the known points
        known_points = np.array([
            [100, 33],
            [150, 36],
            [200, 41],
            [250, 45],
            [300, 51],
            [350, 57]
        ])
        
        # Fit a quadratic function to the data
        coeffs = np.polyfit(known_points[:, 0], known_points[:, 1], 2)
        
        return coeffs

    def compensate_z(self, x, z):
        # Calculate the z-offset based on the x-coordinate
        z_compensation = np.polyval(self.z_offset, x)
        
        # Add the compensation to the original z to keep it constant
        return z + z_compensation - self.z_offset[2]  # Subtract the constant term to maintain the initial height

    def start_homing(self):
        self.send_command("H")
        self.homing_button.setEnabled(False)
        self.homing_button.setText("Homing...")
        QTimer.singleShot(5000, self.finish_homing)  # 5000 ms = 5 seconds

    def finish_homing(self):
        self.homing_button.setEnabled(True)
        self.homing_button.setText("Start Homing")
        self.previous_theta = np.zeros(6)  # Reset previous joint angles after homing

    def start_shutdown(self):
        self.send_command("S")
        self.shutdown_button.setEnabled(False)
        self.shutdown_button.setText("Shutting down...")
        QTimer.singleShot(5000, self.finish_shutdown)  # 5000 ms = 5 seconds

    def finish_shutdown(self):
        self.shutdown_button.setEnabled(True)
        self.shutdown_button.setText("Start Shutdown")

    def reset_joints(self):
        self.send_command("R")
        self.reset_button.setEnabled(False)
        self.reset_button.setText("Resetting...")
        QTimer.singleShot(5000, self.finish_reset)  # 5000 ms = 5 seconds

    def finish_reset(self):
        self.reset_button.setEnabled(True)
        self.reset_button.setText("Reset Joints")
        self.previous_theta = np.zeros(6)  # Reset previous joint angles after resetting

    def move_to_position(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())

            # Apply z-compensation
            compensated_z = self.compensate_z(x, z)

            target_position = np.array([x, y, compensated_z])
            
            joint_angles_rad = inverse_kinematics(target_position, self.previous_theta)
            self.previous_theta = joint_angles_rad  # Update previous joint angles
            joint_angles_deg = np.degrees(joint_angles_rad)

            # Invert angles for joints 1, 3, 4, and 5. Don't invert joints 2 and 6.
            inverted_angles = [-angle if i in [0, 2, 3, 4] else angle for i, angle in enumerate(joint_angles_deg)]

            # Make joint 6 rotate the opposite magnitude of joint 1
            inverted_angles[5] = -inverted_angles[0]

            command = "M" + ",".join(f"{angle:.5f}" for angle in inverted_angles)
            self.send_command(command)

            self.angle_display.append(f"Moving to position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            self.angle_display.append(f"Compensated Z: {compensated_z:.2f}")
            self.angle_display.append("Calculated and Inverted Joint Angles (degrees):")
            for i, (calc_angle, inv_angle) in enumerate(zip(joint_angles_deg, inverted_angles)):
                self.angle_display.append(f"Joint {i+1}: Calculated: {calc_angle:.5f}, Inverted: {inv_angle:.5f}")

            # Check final orientation
            final_pose = forward_kinematics(joint_angles_rad)
            final_z_axis = final_pose[:3, 2]
            self.angle_display.append(f"Final gripper z-axis: {final_z_axis}")

            # Check if the orientation is within acceptable limits
            desired_z_axis = np.array([0, 0, -1])
            orientation_error = np.sum((final_z_axis - desired_z_axis)**2)
            if orientation_error > 0.1:  # You can adjust this threshold
                self.angle_display.append("Warning: Gripper orientation may not be perfectly perpendicular to XY plane.")

        except ValueError:
            self.angle_display.setText("Invalid input. Please enter valid numbers for X, Y, and Z coordinates.")

    def open_bldc(self):
        self.send_command("OPEN")
        self.angle_display.append("Opening BLDC motor")

    def close_bldc(self):
        self.send_command("CLOSE")
        self.angle_display.append("Closing BLDC motor")

    def send_command(self, command):
        if hasattr(self, 'serial_port'):
            self.serial_port.write(f"{command}\n".encode())

    def closeEvent(self, event):
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoboticArmGUI()
    window.show()
    sys.exit(app.exec_())