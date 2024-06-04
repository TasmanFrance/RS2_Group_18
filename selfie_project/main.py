import os
import subprocess
from imageprocessing import process_image
from svg_reader import read_svg_paths
# added
# from main import process_svg_file

# works 
# def process_svg_file():
#     svg_file_path = os.path.join(os.getcwd(), "test.svg")
#     read_svg_paths(svg_file_path)

# def run_imageprocessing():
#     process_image("selfie.jpg")

# if __name__ == "__main__":
#     # Run the imageprocessing script to capture and process the image
#     run_imageprocessing()

#     # Check if the test.svg file was created
#     if os.path.isfile("test.svg"):
#         # Run the svg_reader script to process the SVG file
#         process_svg_file()
#     else:
#         print("The test.svg file was not created.")

# test
import os

def process_svg_file():
    svg_file_path = os.path.join(os.getcwd(), "test.svg")
    read_svg_paths(svg_file_path)

if __name__ == "__main__":
    process_svg_file()