import tkinter as tk
from xml.dom import minidom
import math
import time

class SVGAnimator:
    def __init__(self, root, svg_file):
        self.root = root
        self.svg_file = svg_file
        self.canvas = tk.Canvas(root, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.path_segments = self.parse_svg()
        self.animate_path()

    def parse_svg(self):
        path_segments = []

        svg_doc = minidom.parse(self.svg_file)
        path_elements = svg_doc.getElementsByTagName("path")

        for path_element in path_elements:
            d = path_element.getAttribute("d")
            path_segments.append(d)

        return path_segments

    def animate_path(self):
        for segment in self.path_segments:
            self.draw_segment(segment)
            time.sleep(1)  # Adjust the delay as needed

    def draw_segment(self, segment):
        commands = self.parse_path_commands(segment)

        for command in commands:
            if command[0] == "M":
                x, y = command[1]
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill="red")
            elif command[0] == "L":
                x, y = command[1]
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill="blue")
            elif command[0] == "C":
                x1, y1 = command[1]
                x2, y2 = command[2]
                x3, y3 = command[3]
                self.canvas.create_oval(x1-2, y1-2, x1+2, y1+2, fill="green")
                self.canvas.create_oval(x2-2, y2-2, x2+2, y2+2, fill="green")
                self.canvas.create_oval(x3-2, y3-2, x3+2, y3+2, fill="green")

        self.root.update()

    def parse_path_commands(self, segment):
        commands = []
        tokens = segment.strip().split()

        i = 0
        while i < len(tokens):
            command = tokens[i]
            params = []
            i += 1

            while i < len(tokens) and tokens[i][0].isalpha() == False:
                param = tokens[i]
                params.append((float(param.split(",")[0]), float(param.split(",")[1])))
                i += 1

            commands.append((command, params))

        return commands

if __name__ == "__main__":
    root = tk.Tk()
    root.title("SVG Path Animator")



    svg_file = "/home/ubuntu/git/RS2_Group_18/project/test/yobombo.svg"  # Change this to your SVG file
    animator = SVGAnimator(root, svg_file)

    root.mainloop()