# import cv2  # Import OpenCV library for working with computer vision
# import tkinter as tk  # Import Tkinter library for GUI creation
# from tkinter import ttk  # Import themed Tkinter widgets
# from PIL import Image, ImageTk  # Import Python Imaging Library for image manipulation
# import numpy as np  # Import NumPy library for numerical operations

# import os
# current_directory = os.getcwd()


# def contours_to_svg(contours, output_path, width, height):
    
#         # Open the SVG file for writing
#         with open(output_path, 'w') as f:
#             # Write the SVG header with the specified width and height
#             f.write(f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">\n')

#             # Loop through each contour
#             for contour in contours:
#                 # Convert contour to a string of coordinates
#                 path_data = ' '.join([f'{point[0][0]},{point[0][1]}' for point in contour])

#                 # Write the SVG path element with the contour coordinates
#                 f.write(f'<path d="M {path_data} Z" fill="none" stroke="black" />\n')

#             # Close the SVG tag
#             f.write('</svg>')


# def simple_outline(image, output_path):
#     # Convert to grayscale
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     # Apply GaussianBlur to smoothen image
#     blur = cv2.GaussianBlur(gray, (9, 9), 0)

#     # Apply thresholding to create a binary image
#     _, thresh = cv2.threshold(blur, 50, 200, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
#     # Find contours from the thresholded image
#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     blank_image = np.zeros(image.shape, dtype=np.uint8)
    
#     # Draw contours on the blank image
#     cv2.drawContours(blank_image, contours, -1, (0, 255, 0), 3)
    
#     # Resize the window to show the image smaller
#     window_name = 'contours'
#     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
#     cv2.resizeWindow(window_name, 800, 600)  # Resize the window to 800x600 pixels
    
#     # Show the image with contours
#     cv2.imshow(window_name, blank_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     # Create SVG from contours
#     contours_to_svg(contours, output_path, image.shape[1], image.shape[0])

# class WebcamApp:
#     def __init__(self, root):
#         self.root = root  # Initialize root window
#         self.root.title("Webcam Live View")  # Set title of the window

#         # Initialize webcam
#         self.cap = cv2.VideoCapture(0)  # Access the default camera (index 0)
#         self.capture_active = True  # Flag to control continuous frame capture

#         # Create GUI elements
#         self.label = ttk.Label(root)  # Create a label widget
#         self.label.pack()  # Display the label in the root window

#         # Create capture, accept, and retake buttons
#         self.capture_button = ttk.Button(root, text="Capture", command=self.capture_image)
#         self.capture_button.pack()  # Display the capture button in the root window

#         self.accept_button = ttk.Button(root, text="Accept", command=self.accept_image)
#         self.accept_button.pack()  # Display the accept button in the root window

#         self.retake_button = ttk.Button(root, text="Retake", command=self.retake_image)
#         self.retake_button.pack()  # Display the retake button in the root window

#         self.image = None  # Initialize image variable to store captured image
#         self.processed_image = None  # Initialize processed image variable to store processed image
#         self.show_live_camera()  # Start displaying live camera feed

#     def show_live_camera(self):
#         if self.capture_active:  # If continuous frame capture is active
#             ret, frame = self.cap.read()  # Read frame from the camera
#             if ret:  # If frame is successfully captured
#                 self.show_image(frame)  # Display the captured frame
#                 self.root.after(10, self.show_live_camera)  # Update live camera feed every 10 milliseconds

#     def capture_image(self):
#         self.capture_active = False  # Stop continuous frame capture
#         ret, frame = self.cap.read()  # Read frame from the camera
#         if ret:  # If frame is successfully captured
#             self.image = frame  # Store the captured frame
#             self.show_image(frame)  # Display the captured image

#     def show_image(self, frame):
#         img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR image to RGB
#         img_pil = Image.fromarray(img)  # Create a PIL Image from numpy array
#         img_tk = ImageTk.PhotoImage(image=img_pil)  # Create a Tkinter-compatible photo image
#         self.label.img_tk = img_tk  # Store reference to the photo image to prevent it from being garbage collected
#         self.label.config(image=img_tk)  # Update the label with the new image

#     def retake_image(self):
#         self.capture_active = True  # Resume continuous frame capture
#         self.image = None  # Clear the stored image
#         self.processed_image = None  # Clear the stored processed image
#         self.show_live_camera()  # Show live camera feed

#     def accept_image(self):
#         if self.image is not None:  # If an image has been captured
#             self.processed_image = self.image.copy()  # Store the captured image
            
            
#             filename = "camera_test.svg"
#             output_path = os.path.join(current_directory, filename)
            
#             # output_path = '/home/ubuntu/git/RS2_Group_18/project/test/camera_test.svg'  # Output path for SVG file
#             # output_path = 'camera_test.svg'  # Output path for SVG file
#             # Check if the SVG file is being created
#             print("Saving SVG file to:", output_path)
            
#             simple_outline(self.processed_image, output_path)  # Call simple outline function
#             self.show_image(self.processed_image)  # Display the processed image

# if __name__ == "__main__":
#     root = tk.Tk()  # Create a Tkinter root window
#     app = WebcamApp(root)  # Create an instance of WebcamApp
#     root.mainloop()  # Start the Tkinter event loop


import cv2
import tkinter as tk
from tkinter import ttk, filedialog
from PIL import Image, ImageTk
import numpy as np
import os

class WebcamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Webcam Live View")

        self.cap = cv2.VideoCapture(0)
        self.capture_active = True

        self.label = ttk.Label(root)
        self.label.pack()

        self.capture_button = ttk.Button(root, text="Capture", command=self.capture_image)
        self.capture_button.pack()

        self.accept_button = ttk.Button(root, text="Accept", command=self.accept_image)
        self.accept_button.pack()

        self.retake_button = ttk.Button(root, text="Retake", command=self.retake_image)
        self.retake_button.pack()

        self.image = None
        self.processed_image = None
        self.show_live_camera()

    def show_live_camera(self):
        if self.capture_active:
            ret, frame = self.cap.read()
            if ret:
                self.show_image(frame)
                self.root.after(10, self.show_live_camera)

    def capture_image(self):
        self.capture_active = False
        ret, frame = self.cap.read()
        if ret:
            self.image = frame
            self.show_image(frame)

    def show_image(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        self.label.img_tk = img_tk
        self.label.config(image=img_tk)

    def retake_image(self):
        self.capture_active = True
        self.image = None
        self.processed_image = None
        self.show_live_camera()

    def accept_image(self):
        if self.image is not None:
            filename = filedialog.asksaveasfilename(defaultextension=".svg", filetypes=[("SVG files", "*.svg")])
            if filename:
                contours = self.detect_contours(self.image)
                if contours:
                    output_path = filename
                    self.save_svg(contours, output_path)
                    self.show_image(self.processed_image)

    def detect_contours(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        _, thresh = cv2.threshold(blur, 50, 200, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def save_svg(self, contours, output_path):
        with open(output_path, 'w') as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg">\n')
            for contour in contours:
                path_data = ' '.join([f'{point[0][0]},{point[0][1]}' for point in contour])
                f.write(f'<path d="M {path_data} Z" fill="none" stroke="black" />\n')
            f.write('</svg>')

if __name__ == "__main__":
    root = tk.Tk()
    app = WebcamApp(root)
    root.mainloop()
