import sys
import serial
import numpy as np
import cv2
import time
import math
from scipy.optimize import minimize
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

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
    
    position_error = np.sum((current_position - target_position)**2)
    
    desired_z_axis = np.array([0, 0, -1])  # Pointing downward
    current_z_axis = current_orientation[:, 2]
    orientation_error = np.sum((current_z_axis - desired_z_axis)**2)
    
    joint_change_penalty = np.sum(joint_weights * (theta - previous_theta)**2)
    
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

def inverse_kinematics_unrestricted(target_position, previous_theta=None):
    if previous_theta is None:
        previous_theta = np.zeros(6)
    
    def objective_function_unrestricted(theta):
        T = forward_kinematics(theta)
        current_position = T[:3, 3]
        position_error = np.sum((current_position - target_position)**2)
        
        joint_change_penalty = np.sum(joint_weights * (theta - previous_theta)**2)
        
        return position_error + 0.1 * joint_change_penalty

    result = minimize(
        objective_function_unrestricted,
        previous_theta,
        method='L-BFGS-B',
        bounds=joint_limits
    )
    return result.x

class VisionThread(QThread):
    update_frame = pyqtSignal(np.ndarray)
    update_object_position = pyqtSignal(tuple)
    update_debug_info = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = False
        self.vision_initialized = False
        self.cap = None
        self.corner_history = []
        self.stable_corners = None
        self.background_subtractor = None
        self.last_object_position = None

    def run(self):
        self.running = True
        self.init_camera()
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self.init_camera()
                time.sleep(1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture frame")
                self.init_camera()
                time.sleep(1)
                continue

            if self.vision_initialized:
                processed_frame, object_position = self.process_frame(frame)
                self.update_frame.emit(processed_frame)
                if object_position is not None:
                    self.update_object_position.emit(object_position)
            else:
                self.update_frame.emit(frame)

            time.sleep(0.03)  # Limit to about 30 FPS

    def init_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Failed to open camera")

    def initialize_vision(self):
        self.vision_initialized = True
        self.corner_history = []
        self.stable_corners = None
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=False)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, box_mask = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY_INV)

        # Debug: Draw the box_mask on the frame
        mask_overlay = cv2.cvtColor(box_mask, cv2.COLOR_GRAY2BGR)
        debug_frame = cv2.addWeighted(frame, 0.7, mask_overlay, 0.3, 0)

        corners = self.find_box_corners(box_mask)
        if corners is not None and len(corners) == 6:
            self.corner_history.append(corners)
            # Debug: Draw detected corners
            for corner in corners:
                cv2.circle(debug_frame, tuple(corner.astype(int)), 5, (0, 255, 0), -1)
            self.update_debug_info.emit(f"Corners detected: {len(corners)}")
        else:
            self.update_debug_info.emit("No corners detected")

        if self.stable_corners is None and len(self.corner_history) >= 150:
            self.stable_corners = self.median_corners(self.corner_history)
            if self.stable_corners is not None:
                self.update_debug_info.emit("Stable corners established")
            else:
                self.update_debug_info.emit("Failed to establish stable corners")

        if self.stable_corners is not None:
            # Debug: Draw stable corners
            for corner in self.stable_corners:
                cv2.circle(debug_frame, tuple(corner.astype(int)), 7, (255, 0, 0), -1)

            transform_matrix = self.perspective_transform(self.stable_corners, 
                                                          np.array([175, -5]),
                                                          np.array([185, -175]),
                                                          np.array([315, -175]),
                                                          np.array([305, 0]))

            box_area_mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.fillConvexPoly(box_area_mask, self.stable_corners[:4].astype(int), 255)

            masked_frame = cv2.bitwise_and(gray, gray, mask=box_area_mask)

            fg_mask = self.background_subtractor.apply(masked_frame, learningRate=0.0001)
            
            fg_mask = cv2.erode(fg_mask, None, iterations=2)
            fg_mask = cv2.dilate(fg_mask, None, iterations=2)

            _, object_mask = cv2.threshold(fg_mask, 244, 255, cv2.THRESH_BINARY)

            object_contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            min_contour_area = 500
            large_contours = [cnt for cnt in object_contours if cv2.contourArea(cnt) > min_contour_area]
            
            if large_contours:
                largest_contour = max(large_contours, key=cv2.contourArea)
                
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    cv2.drawContours(debug_frame, [largest_contour], 0, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cX, cY), 7, (255, 0, 0), -1)

                    object_position = self.calculate_object_position(transform_matrix, (cX, cY))
                    cv2.putText(debug_frame, f"X: {object_position[1]:.2f}, Y: {object_position[0]:.2f}", (cX, cY - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    self.last_object_position = object_position
                    return debug_frame, object_position

        return debug_frame, None

    def find_box_corners(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            epsilon = 0.02 * cv2.arcLength(largest_contour, True)
            corners = cv2.approxPolyDP(largest_contour, epsilon, True)

            if 6 <= len(corners) <= 8:
                return self.sort_corners(corners.reshape(-1, 2))
            else:
                self.update_debug_info.emit(f"Invalid number of corners: {len(corners)}")
        else:
            self.update_debug_info.emit("No contours found")

        return None

    def sort_corners(self, corners):
        sorted_corners = corners[np.argsort(corners[:, 1])]
        top_corners = sorted_corners[:3]
        bottom_corners = sorted_corners[3:]
        top_corners = top_corners[np.argsort(top_corners[:, 0])]
        bottom_corners = bottom_corners[np.argsort(bottom_corners[:, 0])]
        return np.vstack((top_corners, bottom_corners))[:6]

    def median_corners(self, corner_history):
        if len(corner_history) == 0:
            return None

        valid_history = [corners for corners in corner_history if corners is not None and len(corners) == 6]

        if len(valid_history) == 0:
            return None

        median_corners = np.median(valid_history, axis=0).astype(int)

        return median_corners

    def perspective_transform(self, corners, ref_point1, ref_point2, ref_point3, ref_point4):
        src_points = corners[:4].astype(np.float32)  # Points 1, 2, 3, 4
        dst_points = np.array([
            [ref_point1[0], ref_point1[1]],  # Y, X for point 1
            [ref_point2[0], ref_point2[1]],  # Y, X for point 2
            [ref_point3[0], ref_point3[1]],  # Y, X for point 3
            [ref_point4[0], ref_point4[1]]   # Y, X for point 4
        ], dtype=np.float32)

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        return matrix

    def calculate_object_position(self, transform_matrix, object_point):
        object_point = np.array([[[object_point[0], object_point[1]]]], dtype=np.float32)
        transformed_point = cv2.perspectiveTransform(object_point, transform_matrix)[0][0]

        y_mm = transformed_point[0]  # Left to right
        x_mm = transformed_point[1]  # Front to back

        return y_mm, x_mm  # Return (Y, X)

class RoboticArmGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm Control")
        self.setGeometry(100, 100, 800, 800)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.init_ui()
        self.init_serial()
        self.init_vision_thread()

        self.previous_theta = np.zeros(6)
        self.z_offset = self.calculate_z_offset()

        # Joint parameters
        self.joints = {
            'J1': {'speed': 6000, 'accel': 8000, 'gear_ratio': 6.4},
            'J2': {'speed': 12000, 'accel': 8000, 'gear_ratio': 20.0},
            'J3': {'speed': 14000, 'accel': 8000, 'gear_ratio': 18.0952381},
            'J4': {'speed': 8000, 'accel': 8000, 'gear_ratio': 4.0},
            'J5': {'speed': 10000, 'accel': 8000, 'gear_ratio': 4.0},
            'J6': {'speed': 12000, 'accel': 8000, 'gear_ratio': 10.0}
        }

    def init_ui(self):
        button_layout = QHBoxLayout()
        self.homing_button = QPushButton("Start Homing")
        self.shutdown_button = QPushButton("Start Shutdown")
        self.reset_button = QPushButton("Reset Joints")
        self.init_vision_button = QPushButton("Initialize Vision")
        button_layout.addWidget(self.homing_button)
        button_layout.addWidget(self.shutdown_button)
        button_layout.addWidget(self.reset_button)
        button_layout.addWidget(self.init_vision_button)
        self.layout.addLayout(button_layout)

        bldc_layout = QHBoxLayout()
        self.open_button = QPushButton("Open BLDC")
        self.close_button = QPushButton("Close BLDC")
        bldc_layout.addWidget(self.open_button)
        bldc_layout.addWidget(self.close_button)
        self.layout.addLayout(bldc_layout)

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

        self.move_button = QPushButton("Move to Position")
        self.layout.addWidget(self.move_button)

        self.detect_button = QPushButton("Detect Object")
        self.layout.addWidget(self.detect_button)

        self.camera_feed_label = QLabel(self)
        self.camera_feed_label.setAlignment(Qt.AlignCenter)
        self.camera_feed_label.setMinimumSize(640, 480)
        self.layout.addWidget(self.camera_feed_label)

        self.debug_label = QLabel(self)
        self.layout.addWidget(self.debug_label)

        self.homing_button.clicked.connect(self.start_homing)
        self.shutdown_button.clicked.connect(self.start_shutdown)
        self.reset_button.clicked.connect(self.reset_joints)
        self.move_button.clicked.connect(self.move_to_position)
        self.open_button.clicked.connect(self.open_bldc)
        self.close_button.clicked.connect(self.close_bldc)
        self.detect_button.clicked.connect(self.detect_and_move)
        self.init_vision_button.clicked.connect(self.initialize_vision)

    def init_serial(self):
        try:
            self.serial_port = serial.Serial("/dev/cu.usbmodem1101", 115200)
        except serial.SerialException:
            print("Failed to open serial port. Make sure the Arduino is connected.")

    def init_vision_thread(self):
        self.vision_thread = VisionThread(self)
        self.vision_thread.update_frame.connect(self.update_camera_feed)
        self.vision_thread.update_object_position.connect(self.update_object_position)
        self.vision_thread.update_debug_info.connect(self.update_debug_label)
        self.vision_thread.start()

    def calculate_z_offset(self):
        known_points = np.array([
            [100, 33],
            [150, 36],
            [200, 41],
            [250, 45],
            [300, 51],
            [350, 57]
        ])
        
        coeffs = np.polyfit(known_points[:, 0], known_points[:, 1], 2)
        
        return coeffs

    def compensate_z(self, x, z):
        z_compensation = np.polyval(self.z_offset, x)
        return z + z_compensation - self.z_offset[2]

    def update_camera_feed(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.camera_feed_label.setPixmap(pixmap.scaled(self.camera_feed_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def update_object_position(self, position):
        self.last_object_position = position

    def update_debug_label(self, info):
        self.debug_label.setText(info)

    def start_homing(self):
        self.send_command("H")
        self.homing_button.setEnabled(False)
        self.homing_button.setText("Homing...")
        QTimer.singleShot(5000, self.finish_homing)

    def finish_homing(self):
        self.homing_button.setEnabled(True)
        self.homing_button.setText("Start Homing")
        self.previous_theta = np.zeros(6)

    def start_shutdown(self):
        self.send_command("S")
        self.shutdown_button.setEnabled(False)
        self.shutdown_button.setText("Shutting down...")
        QTimer.singleShot(5000, self.finish_shutdown)

    def finish_shutdown(self):
        self.shutdown_button.setEnabled(True)
        self.shutdown_button.setText("Start Shutdown")

    def reset_joints(self):
        self.send_command("R")
        self.reset_button.setEnabled(False)
        self.reset_button.setText("Resetting...")
        QTimer.singleShot(5000, self.finish_reset)

    def finish_reset(self):
        self.reset_button.setEnabled(True)
        self.reset_button.setText("Reset Joints")
        self.previous_theta = np.zeros(6)

    def calculate_max_torque(self, joint):
        ARM_WEIGHT = 3  # kg
        GRAVITY = 9.81  # m/s^2
        if joint in ['J1', 'J2', 'J3']:
            return ARM_WEIGHT * GRAVITY * 0.7
        else:
            return ARM_WEIGHT * GRAVITY * 0.1

    def calculate_time(self, angle, speed, acceleration, gear_ratio, joint):
        VOLTAGE = 24  # V
        POWER = 350  # W
        EFFICIENCY = 0.7
        MOTOR_TORQUE = 0.5  # N·m

        speed_deg = speed * 1.8 / (gear_ratio * 16)
        accel_deg = acceleration * 1.8 / (gear_ratio * 16)
        
        max_torque = self.calculate_max_torque(joint)
        
        available_torque = min(MOTOR_TORQUE * gear_ratio, POWER * EFFICIENCY / speed_deg)
        torque_factor = min(1, available_torque / max_torque)
        adjusted_accel = accel_deg * torque_factor
        
        time_to_full_speed = speed_deg / adjusted_accel
        distance_accel = 0.5 * adjusted_accel * time_to_full_speed**2
        
        if distance_accel * 2 < abs(angle):
            full_speed_distance = abs(angle) - 2 * distance_accel
            time_at_full_speed = full_speed_distance / speed_deg
            total_time = 2 * time_to_full_speed + time_at_full_speed
        else:
            total_time = 2 * math.sqrt(abs(angle) / adjusted_accel)
        
        return total_time * 1.4

    def calculate_movement_time(self, joint_angles):
        longest_time = 0
        for i, (joint, params) in enumerate(self.joints.items()):
            angle = joint_angles[i]
            time = self.calculate_time(angle, params['speed'], params['accel'], params['gear_ratio'], joint)
            longest_time = max(longest_time, time)
        return longest_time + 0.5

    def move_to_position(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())

            compensated_z = self.compensate_z(x, z)

            target_position = np.array([x, y, compensated_z])
            
            joint_angles_rad = inverse_kinematics(target_position, self.previous_theta)
            self.previous_theta = joint_angles_rad
            joint_angles_deg = np.degrees(joint_angles_rad)

            inverted_angles = [-angle if i in [0, 2, 3, 4] else angle for i, angle in enumerate(joint_angles_deg)]

            inverted_angles[5] = -inverted_angles[0]

            command = "M" + ",".join(f"{angle:.5f}" for angle in inverted_angles)
            self.send_command(command)

            movement_time = self.calculate_movement_time(inverted_angles)

            return movement_time

        except ValueError:
            print("Invalid input. Please enter valid numbers for X, Y, and Z coordinates.")
            return 0

    def open_bldc(self):
        self.send_command("OPEN")

    def close_bldc(self):
        self.send_command("CLOSE")

    def send_command(self, command):
        if hasattr(self, 'serial_port'):
            self.serial_port.write(f"{command}\n".encode())

    def initialize_vision(self):
        self.vision_thread.initialize_vision()
        self.init_vision_button.setEnabled(False)
        self.init_vision_button.setText("Initializing...")
        QTimer.singleShot(5000, self.finish_vision_init)

    def finish_vision_init(self):
        self.init_vision_button.setEnabled(True)
        self.init_vision_button.setText("Re-Initialize Vision")

    def detect_and_move(self):
        if not self.vision_thread.vision_initialized or self.vision_thread.stable_corners is None:
            print("Error: Vision not initialized or corners not stabilized yet.")
            return

        if hasattr(self, 'last_object_position'):
            x, y = self.last_object_position
            z = 36  # Constant Z value
            self.move_to_position_and_close(x, y, z)
        else:
            print("No object detected.")

    def move_to_position_and_close(self, x, y, z):
        self.x_input.setText(str(x))
        self.y_input.setText(str(y))
        self.z_input.setText(str(z))
        movement_time = self.move_to_position()
        
        # Close the BLDC motor after the calculated movement time
        QTimer.singleShot(int(movement_time * 1000), self.close_bldc)
        
        # Move up to z = 100mm after closing the BLDC
        QTimer.singleShot(int(movement_time * 1000 + 500), lambda: self.move_up(x, y))

    def move_up(self, x, y):
        self.x_input.setText(str(x))
        self.y_input.setText(str(y))
        self.z_input.setText("100")
        movement_time = self.move_to_position()
        
        # Move to bowl location after ensuring z = 100mm movement is complete
        QTimer.singleShot(int(movement_time * 1000 + 500), self.move_to_bowl)

    def move_to_bowl(self):
        bowl_x, bowl_y, bowl_z = 215, -240, 100
        target_position = np.array([bowl_x, bowl_y, bowl_z])
        
        # Use inverse_kinematics_unrestricted for bowl movement
        joint_angles_rad = inverse_kinematics_unrestricted(target_position, self.previous_theta)
        self.previous_theta = joint_angles_rad
        joint_angles_deg = np.degrees(joint_angles_rad)

        inverted_angles = [-angle if i in [0, 2, 3, 4] else angle for i, angle in enumerate(joint_angles_deg)]
        inverted_angles[5] = -inverted_angles[0]

        command = "M" + ",".join(f"{angle:.5f}" for angle in inverted_angles)
        self.send_command(command)

        print(f"Moving to bowl location: ({bowl_x}, {bowl_y}, {bowl_z})")
        
        movement_time = self.calculate_movement_time(inverted_angles)
        
        # Schedule opening the BLDC after the movement is complete
        QTimer.singleShot(int(movement_time * 1000 + 500), self.open_bldc)
        
        return movement_time

    def closeEvent(self, event):
        self.vision_thread.running = False
        self.vision_thread.wait()
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoboticArmGUI()
    window.show()
    sys.exit(app.exec_())