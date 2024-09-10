import cv2
import numpy as np
import time

def find_box_corners(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        corners = cv2.approxPolyDP(largest_contour, epsilon, True)

        if 6 <= len(corners) <= 8:
            return sort_corners(corners.reshape(-1, 2))

    return None

def sort_corners(corners):
    sorted_corners = corners[np.argsort(corners[:, 1])]
    top_corners = sorted_corners[:3]
    bottom_corners = sorted_corners[3:]
    top_corners = top_corners[np.argsort(top_corners[:, 0])]
    bottom_corners = bottom_corners[np.argsort(bottom_corners[:, 0])]
    return np.vstack((top_corners, bottom_corners))[:6]

def median_corners(corner_history):
    if len(corner_history) == 0:
        return None

    valid_history = [corners for corners in corner_history if corners is not None and len(corners) == 6]

    if len(valid_history) == 0:
        return None

    median_corners = np.median(valid_history, axis=0).astype(int)

    return median_corners

def perspective_transform(corners, ref_point1, ref_point2, ref_point3, ref_point4):
    src_points = corners[:4].astype(np.float32)  # Points 1, 2, 3, 4
    dst_points = np.array([
        [ref_point1[0], ref_point1[1]],  # Y, X for point 1
        [ref_point2[0], ref_point2[1]],  # Y, X for point 2
        [ref_point3[0], ref_point3[1]],  # Y, X for point 3
        [ref_point4[0], ref_point4[1]]   # Y, X for point 4
    ], dtype=np.float32)

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    return matrix

def calculate_object_position(transform_matrix, object_point):
    object_point = np.array([[[object_point[0], object_point[1]]]], dtype=np.float32)
    transformed_point = cv2.perspectiveTransform(object_point, transform_matrix)[0][0]

    y_mm = transformed_point[0]  # Left to right
    x_mm = transformed_point[1]  # Front to back

    return y_mm, x_mm  # Return (Y, X)

def main():
    cap = cv2.VideoCapture(0)
    corner_history = []

    # Create a background subtractor with a slower learning rate
    background_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=False)

    # Variables for object tracking
    min_contour_area = 500  # Minimum area to consider as an object
    object_detected = False
    object_position = None
    frames_since_detection = 0
    max_frames_to_keep = 30  # Number of frames to keep showing the object after it stops moving

    # Reference points (all four corners)
    ref_point1 = np.array([175, -5])    # in mm (y, x)
    ref_point2 = np.array([185, -175])  # in mm (y, x)
    ref_point3 = np.array([315, -175])  # in mm (y, x)
    ref_point4 = np.array([305, 0])     # in mm (y, x)

    # Variables for corner stabilization
    start_time = time.time()
    stabilization_time = 5  # seconds
    stable_corners = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, box_mask = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY_INV)

        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time < stabilization_time:
            # During the first 5 seconds, collect corner data
            corners = find_box_corners(box_mask)
            if corners is not None and len(corners) == 6:
                corner_history.append(corners)
        elif stable_corners is None:
            # After 5 seconds, calculate the median corners and set them as stable
            stable_corners = median_corners(corner_history)
            print("Corners stabilized!")

        # Create mask display
        mask_display = cv2.cvtColor(box_mask, cv2.COLOR_GRAY2BGR)

        if stable_corners is not None:
            transform_matrix = perspective_transform(stable_corners, ref_point1, ref_point2, ref_point3, ref_point4)

            # Draw the 3D box
            for i in range(4):
                cv2.line(mask_display, tuple(stable_corners[i]), 
                         tuple(stable_corners[(i+1)%4]), (0, 255, 0), 2)
            for i in range(4):
                cv2.line(mask_display, tuple(stable_corners[i]), 
                         tuple(stable_corners[i+2]), (0, 255, 0), 2)

            # Draw and label corner points
            for i, corner in enumerate(stable_corners):
                cv2.circle(mask_display, tuple(corner), 5, (0, 0, 255), -1)
                cv2.putText(mask_display, str(i+1), (corner[0]+10, corner[1]+10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Create a mask for the area inside points 1-4
            box_area_mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.fillConvexPoly(box_area_mask, stable_corners[:4].astype(int), 255)

            # Apply the mask to the original frame
            masked_frame = cv2.bitwise_and(gray, gray, mask=box_area_mask)

            # Apply background subtraction
            fg_mask = background_subtractor.apply(masked_frame, learningRate=0.0001)
            
            # Reduce noise
            fg_mask = cv2.erode(fg_mask, None, iterations=2)
            fg_mask = cv2.dilate(fg_mask, None, iterations=2)

            # Threshold the foreground mask
            _, object_mask = cv2.threshold(fg_mask, 244, 255, cv2.THRESH_BINARY)

            # Find contours of objects
            object_contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter contours by area and find the largest one
            large_contours = [cnt for cnt in object_contours if cv2.contourArea(cnt) > min_contour_area]
            
            if large_contours:
                largest_contour = max(large_contours, key=cv2.contourArea)
                
                # Calculate the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Calculate object position with improved perspective correction
                    new_position = calculate_object_position(transform_matrix, (cX, cY))

                    # Check if the object has moved
                    if object_position is None or abs(new_position[0] - object_position[0]) > 2.54 or abs(new_position[1] - object_position[1]) > 2.54:
                        object_position = new_position
                        frames_since_detection = 0
                        object_detected = True
                    else:
                        frames_since_detection += 1

                    # Draw object center (green dot)
                    cv2.circle(mask_display, (cX, cY), 7, (0, 255, 0), -1)

                    # Display position in top left corner with red font
                    position_text = f"Object Position: Y (left-right): {object_position[0]:.2f} mm, X (front-back): {object_position[1]:.2f} mm"
                    cv2.putText(mask_display, position_text, (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                frames_since_detection += 1

            # Reset object detection if it hasn't moved for too long
            if frames_since_detection > max_frames_to_keep:
                object_detected = False
                object_position = None

        # Display stabilization countdown
        if elapsed_time < stabilization_time:
            countdown_text = f"Stabilizing corners: {stabilization_time - int(elapsed_time)}s"
            cv2.putText(mask_display, countdown_text, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow('Object Tracking within 3D Box', mask_display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()