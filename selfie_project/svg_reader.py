import os
from svgpathtools import svg2paths

# added
def process_svg_file():
    svg_file_path = os.path.join(os.getcwd(), "test.svg")
    read_svg_paths(svg_file_path)

def extract_relevant_path_data(path_data):
    # Extract the 'd' attribute value
    d_attribute = path_data.get('d', '')
    
    # Remove the 'M ' prefix and ' Z' suffix
    if d_attribute.startswith('M '):
        d_attribute = d_attribute[2:]
    if d_attribute.endswith(' Z'):
        d_attribute = d_attribute[:-2]

    # Remove 'L' characters
    d_attribute = d_attribute.replace('L', '')
    # Remove 'L' characters
    d_attribute = d_attribute.replace('M', '')
    # Remove 'L' characters
    d_attribute = d_attribute.replace('Z', '')
    
    # Remove commas between points
    d_attribute = d_attribute.replace(',', ' ')
    
    # Clean up extra spaces
    d_attribute = ' '.join(d_attribute.split())
    
    return d_attribute

def read_svg_paths(file_path):
    # Check if the file exists
    if not os.path.isfile(file_path):
        print(f"The file {file_path} does not exist.")
        return
    
    # Read the paths and attributes from the SVG file
    paths, attributes = svg2paths(file_path)
    
    # this is still good
    # Process and print only the relevant data for each path
    for i, attribute in enumerate(attributes):
        relevant_data = extract_relevant_path_data(attribute)
        # print(f'Path {i + 1}: "{relevant_data}",')
        print(f'"{relevant_data}",')
        # print(f'{relevant_data}')
        # print(f'"{attribute}"')

if __name__ == "__main__":
    # Specify the SVG file location in the current directory with file name 'test.svg'
    svg_file_path = os.path.join(os.getcwd(), "test.svg")
    
    # Call the function to read and print the SVG paths
    read_svg_paths(svg_file_path)
