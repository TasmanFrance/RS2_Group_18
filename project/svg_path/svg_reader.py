import os
from svgpathtools import svg2paths

def extract_relevant_path_data(path_data):
    # Extract the 'd' attribute value
    d_attribute = path_data.get('d', '')
    
    # Remove the 'M ' prefix and ' Z' suffix
    if d_attribute.startswith('M '):
        d_attribute = d_attribute[2:]
    if d_attribute.endswith(' Z'):
        d_attribute = d_attribute[:-2]
    
    # Remove commas between points
    d_attribute = d_attribute.replace(',', ' ')
    
    return d_attribute

def read_svg_paths(file_path):
    # Check if the file exists
    if not os.path.isfile(file_path):
        print(f"The file {file_path} does not exist.")
        return
    
    # Read the paths and attributes from the SVG file
    paths, attributes = svg2paths(file_path)
    
    # Process and print only the relevant data for each path
    for i, attribute in enumerate(attributes):
        relevant_data = extract_relevant_path_data(attribute)
        #print(f'Path {i + 1}: "{relevant_data}"')
        print(f'"{relevant_data}"')

if __name__ == "__main__":
    # Specify the SVG file location in the current directory with file name 'test.svg'
    svg_file_path = os.path.join(os.getcwd(), "as.svg")
    
    # Call the function to read and print the SVG paths
    read_svg_paths(svg_file_path)
