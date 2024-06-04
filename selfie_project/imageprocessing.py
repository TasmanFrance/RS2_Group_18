import cv2
import numpy as np
from tkinter import *
from tkinter import messagebox
from PIL import Image, ImageTk
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import math
import svgwrite
# added by jacob
# import os
# from svg_processing import process_svg_file
# from gui_helper import root, on_closing

BG_COLOR = (192, 192, 192)  # gray
HAIR_COLOR = (0, 0, 255)     # Red for hair
BODY_COLOR = (0, 255, 0)     # Green for body
FACE_COLOR = (255, 0, 0)     # Blue for face
CLOTHES_COLOR = (0, 255, 255)  # Yellow for clothes
OTHERS_COLOR = (255, 255, 0)   # Cyan for others
EDGE_COLOR = (255, 255, 255)   # White for edges

DESIRED_HEIGHT = 500
DESIRED_WIDTH = 500

def create_blank_a4_paper():
    a4_width_px = int(2480)
    a4_height_px = int(3508)
    return np.full((a4_height_px, a4_width_px, 3), 255, dtype=np.uint8)

def draw_landmarks_on_image(rgb_image, detection_result, exclude_index):
    face_landmarks_list = detection_result.face_landmarks
    annotated_image = np.copy(rgb_image)
    thickness = 10  # Set thickness here

    for face_landmarks in face_landmarks_list:
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
        ])

        drawing_spec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 0), thickness=thickness)

        mp.solutions.drawing_utils.draw_landmarks(
            image=annotated_image,
            landmark_list=face_landmarks_proto,
            connections=exclude_connections(face_landmarks_proto, exclude_index),
            landmark_drawing_spec=None,
            connection_drawing_spec=drawing_spec
        )
    
    left_iris_landmarks = face_landmarks_proto.landmark[468:472]
    left_center_iris_landmark = left_iris_landmarks[0]
    left_outer_iris_landmark = face_landmarks_proto.landmark[159]
    left_radius = np.sqrt((left_outer_iris_landmark.x - left_center_iris_landmark.x) ** 2 +
                              (left_outer_iris_landmark.y - left_center_iris_landmark.y) ** 2) * annotated_image.shape[1]
    left_radius = int(left_radius)
    left_x = int(left_center_iris_landmark.x * rgb_image.shape[1])
    left_y = int(left_center_iris_landmark.y * rgb_image.shape[0])
    cv2.circle(annotated_image, (left_x, left_y), left_radius, (0, 0, 0), thickness=thickness)
    
    right_iris_landmarks = face_landmarks_proto.landmark[473:477]
    right_center_iris_landmark = right_iris_landmarks[0]
    right_outer_iris_landmark = face_landmarks_proto.landmark[385]
    right_radius = np.sqrt((right_outer_iris_landmark.x - right_center_iris_landmark.x) ** 2 +
                              (right_outer_iris_landmark.y - right_center_iris_landmark.y) ** 2) * annotated_image.shape[1]
    right_radius = int(right_radius)
    right_x = int(right_center_iris_landmark.x * rgb_image.shape[1])
    right_y = int(right_center_iris_landmark.y * rgb_image.shape[0])
    cv2.circle(annotated_image, (right_x, right_y), right_radius, (0, 0, 0), thickness=thickness)
    
    return annotated_image

def exclude_connections(landmarks_proto, exclude_index):
    connections = list(mp.solutions.face_mesh.FACEMESH_CONTOURS)
    
    for connection in connections:
        if exclude_index in connection and 10 in connection:
            connections.remove(connection)
    
    excluded_indices = [exclude_index]
    
    for idx in excluded_indices:
        connections = exclude_connections_for_index(connections, landmarks_proto, idx, excluded_indices)
    
    return connections

def exclude_connections_for_index(connections, landmarks_proto, index, excluded_indices):
    for connection in connections[:]:
        if index in connection:
            other_index = connection[0] if connection[0] != index else connection[1]
            if other_index not in excluded_indices:
                excluded_indices.append(other_index)
                connections.remove(connection)
                connections = exclude_connections_for_index(connections, landmarks_proto, other_index, excluded_indices)
    return connections

def process_image(image_path):
    base_options = python.BaseOptions(model_asset_path='face_landmarker.task')
    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        output_face_blendshapes=True,
        output_facial_transformation_matrixes=True,
        num_faces=1
    )
    detector = vision.FaceLandmarker.create_from_options(options)

    image = mp.Image.create_from_file(image_path)
    detection_result = detector.detect(image)
    exclude_index = 109

    blank_paper = create_blank_a4_paper()
    annotated_image = draw_landmarks_on_image(blank_paper, detection_result, exclude_index=109)

    base_options = python.BaseOptions(model_asset_path='selfie_multiclass.tflite')
    options = vision.ImageSegmenterOptions(base_options=base_options, output_category_mask=True)

    with vision.ImageSegmenter.create_from_options(options) as segmenter:
        image = mp.Image.create_from_file(image_path)
        segmentation_result = segmenter.segment(image)
        category_mask = segmentation_result.category_mask

        colors = [BG_COLOR, HAIR_COLOR, BODY_COLOR, FACE_COLOR, CLOTHES_COLOR, OTHERS_COLOR]
        image_data = image.numpy_view()
        output_image = np.zeros_like(image_data)

        # Create SVG drawing
        dwg = svgwrite.Drawing('test.svg', profile='tiny')

        svg_width = annotated_image.shape[1]
        svg_height = annotated_image.shape[0]
        dwg = svgwrite.Drawing('test.svg', size=(svg_width, svg_height), profile='tiny')

# Add width and height attributes to the SVG drawing
        dwg.attribs['width'] = f"{svg_width}px"
        dwg.attribs['height'] = f"{svg_height}px"
        for i in range(1, 6):
            category_edges = cv2.Canny((category_mask.numpy_view() == i).astype(np.uint8) * 255, 100, 200)
            category_edges = cv2.dilate(category_edges, np.ones((2, 2), dtype=np.uint8), iterations=1)
            output_image[category_edges > 0] = EDGE_COLOR

            if (category_mask.numpy_view() == i).any():
                category_i_edges = cv2.Canny((category_mask.numpy_view() == i).astype(np.uint8) * 255, 100, 200)
                contours, _ = cv2.findContours(category_i_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Create a single SVG path data string
                path_data = ""
                for contour in contours:
                    path_data += "M " + " L ".join(f"{point[0][0]},{point[0][1]}" for point in contour) + " "

                # Add the path to the SVG drawing
                path = dwg.path(d=path_data, fill="none", stroke="black", stroke_width=2)
                dwg.add(path)

        # Save the SVG drawing
        dwg.save()

    output_image_resized = cv2.resize(output_image, (annotated_image.shape[1], annotated_image.shape[0]))
    output_image_resized_inverted = cv2.bitwise_not(output_image_resized)
    combined_image = cv2.addWeighted(annotated_image, 0.5, output_image_resized_inverted, 0.5, 0)

    return combined_image


# GUI Code
root = Tk()
root.title("Selfie Drawing Robot")
root.geometry("800x600")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    messagebox.showerror("Error", "Cannot access the camera")
    root.destroy()
    exit()

A4_RATIO = 1.414

captured_image = None

def get_cropped_frame():
    ret, frame = cap.read()
    if not ret:
        return None

    height, width, _ = frame.shape
    new_width = int(height / A4_RATIO)
    if new_width > width:
        new_height = int(width * A4_RATIO)
        crop_img = frame[(height - new_height) // 2 : (height + new_height) // 2, :]
    else:
        crop_img = frame[:, (width - new_width) // 2 : (width + new_width) // 2]

    return crop_img

def show_frame():
    if captured_image is None:
        frame = get_cropped_frame()
        if frame is not None:
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            display1.imgtk = imgtk
            display1.configure(image=imgtk)
        display1.after(10, show_frame)

def capture():
    global captured_image
    frame = get_cropped_frame()
    if frame is not None:
        captured_image = frame
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)
        display1.imgtk = imgtk
        display1.configure(image=imgtk)

def retake():
    global captured_image
    captured_image = None
    show_frame()

# from Rohit
# def save_image():
#     if captured_image is not None:
#         cv2.imwrite("selfie.jpg", captured_image)
#         print("Image saved!")
#         combined_image = process_image("selfie.jpg")
#         display_combined_image(combined_image)

def save_image():
    if captured_image is not None:
        cv2.imwrite("selfie.jpg", captured_image)
        print("Image saved!")
        combined_image = process_image("selfie.jpg")
        display_combined_image(combined_image)
        process_svg_file()  #

# edited version of save_image
# def save_image():
#     if captured_image is not None:
#         cv2.imwrite("selfie.jpg", captured_image)
#         print("Image saved!")
#         combined_image = process_image("selfie.jpg")
#         display_combined_image(combined_image)
#         process_svg_file()  # Call the function from svg_processing.py

def display_combined_image(combined_image):
    window_width = display1.winfo_width()
    window_height = display1.winfo_height()
    
    combined_image_rgb = cv2.cvtColor(combined_image, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(combined_image_rgb)

    # Resize the image to fit the display window while maintaining aspect ratio
    img.thumbnail((window_width, window_height), Image.ANTIALIAS)
    imgtk = ImageTk.PhotoImage(image=img)
    display1.imgtk = imgtk
    display1.configure(image=imgtk)

def on_closing():
    cap.release()
    cv2.destroyAllWindows()
    root.destroy()

display1 = Label(root)
display1.pack(expand=True, fill=BOTH)

button_frame = Frame(root)
button_frame.pack(side=BOTTOM, fill=X)

capture_button = Button(button_frame, text="Capture", command=capture)
capture_button.pack(side=LEFT, padx=5, pady=5)

retake_button = Button(button_frame, text="Retake", command=retake)
retake_button.pack(side=LEFT, padx=5, pady=5)

accept_button = Button(button_frame, text="Accept", command=save_image)
accept_button.pack(side=LEFT, padx=5, pady=5)

root.protocol("WM_DELETE_WINDOW", on_closing)

show_frame()
root.mainloop()










