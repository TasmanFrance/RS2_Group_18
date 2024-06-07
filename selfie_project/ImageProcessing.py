import cv2
import numpy as np
from tkinter import *
from tkinter import messagebox
from PIL import Image, ImageTk
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import svgwrite

BG_COLOR = (192, 192, 192)  # gray
HAIR_COLOR = (0, 0, 255)    # Red for hair
BODY_COLOR = (0, 255, 0)    # Green for body
FACE_COLOR = (255, 0, 0)    # Blue for face
CLOTHES_COLOR = (0, 255, 255)  # Yellow for clothes
OTHERS_COLOR = (255, 255, 0)   # Cyan for others
EDGE_COLOR = (255, 255, 255)   # White for edges

DESIRED_HEIGHT = 500
DESIRED_WIDTH = 500

FACEMESH_NOSE = frozenset([(168, 6), (6, 197), (197, 195), (195, 5),
                           (5, 4), (4, 1), (1, 19), (19, 94), (94, 2), (98, 97),
                           (97, 2), (2, 326), (326, 327), (327, 294),
                           (294, 278), (278, 344), (344, 440), (440, 275),
                           (275, 4), (4, 45), (45, 220), (220, 115), (115, 48),
                           (48, 64), (64, 98)])

FACEMESH_LIPS = frozenset([(61, 146), (146, 91), (91, 181), (181, 84), (84, 17),
                           (17, 314), (314, 405), (405, 321), (321, 375),
                           (375, 291), (61, 185), (185, 40), (40, 39), (39, 37),
                           (37, 0), (0, 267), (267, 269), (269, 270), (270, 409), (409, 291),
                           (78, 95), (95, 88), (88, 178), (178, 87), (87, 14),
                           (14, 317), (317, 402), (402, 318), (318, 324),
                           (324, 308), (78, 191), (191, 80), (80, 81), (81, 82),
                           (82, 13), (13, 312), (312, 311), (311, 310),
                           (310, 415), (415, 308)])

FACEMESH_LEFT_EYE = frozenset([(263, 249), (249, 390), (390, 373), (373, 374),
                               (374, 380), (380, 381), (381, 382), (382, 362),
                               (263, 466), (466, 388), (388, 387), (387, 386),
                               (386, 385), (385, 384), (384, 398), (398, 362)])

FACEMESH_LEFT_EYEBROW = frozenset([(276, 283), (283, 282), (282, 295),
                                   (295, 285), (300, 293), (293, 334),
                                   (334, 296), (296, 336)])

FACEMESH_RIGHT_EYE = frozenset([(33, 7), (7, 163), (163, 144), (144, 145),
                                (145, 153), (153, 154), (154, 155), (155, 133),
                                (33, 246), (246, 161), (161, 160), (160, 159),
                                (159, 158), (158, 157), (157, 173), (173, 133)])

FACEMESH_RIGHT_EYEBROW = frozenset([(46, 53), (53, 52), (52, 65), (65, 55),
                                    (70, 63), (63, 105), (105, 66), (66, 107)])

def create_blank_a4_paper():
    a4_width_px = int(2480)
    a4_height_px = int(3508)
    return np.full((a4_height_px, a4_width_px, 3), 255, dtype=np.uint8)

def draw_features_as_paths(detection_result, dwg, image_shape, feature_connections, color="black"):
    face_landmarks_list = detection_result.face_landmarks

    for face_landmarks in face_landmarks_list:
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
        ])

        # Create a single SVG path data string for the features
        path_data = ""
        for connection in feature_connections:
            start_idx, end_idx = connection
            start_landmark = face_landmarks_proto.landmark[start_idx]
            end_landmark = face_landmarks_proto.landmark[end_idx]
            sx = int(start_landmark.x * image_shape[1])
            sy = int(start_landmark.y * image_shape[0])
            ex = int(end_landmark.x * image_shape[1])
            ey = int(end_landmark.y * image_shape[0])
            path_data += f"M {sx},{sy} L {ex},{ey} "

        # Add the path to the SVG drawing
        path = dwg.path(d=path_data, fill="none", stroke=color, stroke_width=2)
        dwg.add(path)

def draw_irises_as_paths(detection_result, dwg, image_shape):
    face_landmarks_list = detection_result.face_landmarks

    for face_landmarks in face_landmarks_list:
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
        ])

        # Draw left iris as SVG path
        left_iris_landmarks = face_landmarks_proto.landmark[468:472]
        left_center_iris_landmark = left_iris_landmarks[0]
        left_outer_iris_landmark = face_landmarks_proto.landmark[159]
        left_radius = np.sqrt((left_outer_iris_landmark.x - left_center_iris_landmark.x) ** 2 +
                              (left_outer_iris_landmark.y - left_center_iris_landmark.y) ** 2) * image_shape[1]
        left_x = int(left_center_iris_landmark.x * image_shape[1])
        left_y = int(left_center_iris_landmark.y * image_shape[0])
        left_iris_path = (
            f"M {left_x - left_radius},{left_y} "
            f"a {left_radius},{left_radius} 0 1,0 {left_radius * 2},0 "
            f"a {left_radius},{left_radius} 0 1,0 {-left_radius * 2},0"
        )
        dwg.add(dwg.path(d=left_iris_path, fill="none", stroke="black", stroke_width=2))

        # Draw right iris as SVG path
        right_iris_landmarks = face_landmarks_proto.landmark[473:477]
        right_center_iris_landmark = right_iris_landmarks[0]
        right_outer_iris_landmark = face_landmarks_proto.landmark[385]
        right_radius = np.sqrt((right_outer_iris_landmark.x - right_center_iris_landmark.x) ** 2 +
                               (right_outer_iris_landmark.y - right_center_iris_landmark.y) ** 2) * image_shape[1]
        right_x = int(right_center_iris_landmark.x * image_shape[1])
        right_y = int(right_center_iris_landmark.y * image_shape[0])
        right_iris_path = (
            f"M {right_x - right_radius},{right_y} "
            f"a {right_radius},{right_radius} 0 1,0 {right_radius * 2},0 "
            f"a {right_radius},{right_radius} 0 1,0 {-right_radius * 2},0"
        )
        dwg.add(dwg.path(d=right_iris_path, fill="none", stroke="black", stroke_width=2))

def draw_features_on_image(detection_result, image, feature_connections, color=(0, 0, 0)):
    face_landmarks_list = detection_result.face_landmarks

    for face_landmarks in face_landmarks_list:
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
        ])

        for connection in feature_connections:
            start_idx, end_idx = connection
            start_landmark = face_landmarks_proto.landmark[start_idx]
            end_landmark = face_landmarks_proto.landmark[end_idx]
            sx = int(start_landmark.x * image.shape[1])
            sy = int(start_landmark.y * image.shape[0])
            ex = int(end_landmark.x * image.shape[1])
            ey = int(end_landmark.y * image.shape[0])
            cv2.line(image, (sx, sy), (ex, ey), color, 2)

def draw_irises_on_image(detection_result, image, color=(0, 0, 0)):
    face_landmarks_list = detection_result.face_landmarks

    for face_landmarks in face_landmarks_list:
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
        ])

        # Draw left iris
        left_iris_landmarks = face_landmarks_proto.landmark[468:472]
        left_center_iris_landmark = left_iris_landmarks[0]
        left_outer_iris_landmark = face_landmarks_proto.landmark[159]
        left_radius = np.sqrt((left_outer_iris_landmark.x - left_center_iris_landmark.x) ** 2 +
                              (left_outer_iris_landmark.y - left_center_iris_landmark.y) ** 2) * image.shape[1]
        left_x = int(left_center_iris_landmark.x * image.shape[1])
        left_y = int(left_center_iris_landmark.y * image.shape[0])
        cv2.circle(image, (left_x, left_y), int(left_radius), color, 2)

        # Draw right iris
        right_iris_landmarks = face_landmarks_proto.landmark[473:477]
        right_center_iris_landmark = right_iris_landmarks[0]
        right_outer_iris_landmark = face_landmarks_proto.landmark[385]
        right_radius = np.sqrt((right_outer_iris_landmark.x - right_center_iris_landmark.x) ** 2 +
                               (right_outer_iris_landmark.y - right_center_iris_landmark.y) ** 2) * image.shape[1]
        right_x = int(right_center_iris_landmark.x * image.shape[1])
        right_y = int(right_center_iris_landmark.y * image.shape[0])
        cv2.circle(image, (right_x, right_y), int(right_radius), color, 2)

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

    blank_paper = create_blank_a4_paper()

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
        svg_width = blank_paper.shape[1]
        svg_height = blank_paper.shape[0]
        dwg = svgwrite.Drawing('edges_all_categories.svg', size=(svg_width, svg_height), profile='tiny')

        # Add width and height attributes to the SVG drawing
        dwg.attribs['width'] = f"{svg_width}px"
        dwg.attribs['height'] = f"{svg_height}px"

        # Draw features as paths
        draw_features_as_paths(detection_result, dwg, image_data.shape, FACEMESH_LIPS, color="black")
        draw_features_as_paths(detection_result, dwg, image_data.shape, FACEMESH_LEFT_EYE, color="black")
        draw_features_as_paths(detection_result, dwg, image_data.shape, FACEMESH_LEFT_EYEBROW, color="black")
        draw_features_as_paths(detection_result, dwg, image_data.shape, FACEMESH_RIGHT_EYE, color="black")
        draw_features_as_paths(detection_result, dwg, image_data.shape, FACEMESH_RIGHT_EYEBROW, color="black")
        draw_features_as_paths(detection_result, dwg, image_data.shape, FACEMESH_NOSE, color="black")

        # Draw irises as paths
        draw_irises_as_paths(detection_result, dwg, image_data.shape)

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

    output_image_resized = cv2.resize(output_image, (blank_paper.shape[1], blank_paper.shape[0]))
    output_image_resized_inverted = cv2.bitwise_not(output_image_resized)
    combined_image = cv2.addWeighted(blank_paper, 0.5, output_image_resized_inverted, 0.5, 0)

    # Draw features on the combined image
    draw_features_on_image(detection_result, combined_image, FACEMESH_LIPS, color=(0, 0, 0))
    draw_features_on_image(detection_result, combined_image, FACEMESH_LEFT_EYE, color=(0, 0, 0))
    draw_features_on_image(detection_result, combined_image, FACEMESH_LEFT_EYEBROW, color=(0, 0, 0))
    draw_features_on_image(detection_result, combined_image, FACEMESH_RIGHT_EYE, color=(0, 0, 0))
    draw_features_on_image(detection_result, combined_image, FACEMESH_RIGHT_EYEBROW, color=(0, 0, 0))
    draw_features_on_image(detection_result, combined_image, FACEMESH_NOSE, color=(0, 0, 0))

    # Draw irises on the combined image
    draw_irises_on_image(detection_result, combined_image, color=(0, 0, 0))

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

def save_image():
    if captured_image is not None:
        cv2.imwrite("selfie.jpg", captured_image)
        print("Image saved!")
        combined_image = process_image("selfie.jpg")
        display_combined_image(combined_image)

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










