import re

"""
Script to parse calibration data to json/dictionary data
Made to work with the command: 
    CV_CALIB_NOZZLE_PX_MM PRINT_POSITIONS=1
"""


data = """
X150.0 Y85.0:
X274.0 Y144.0 r23.0
X151.0 Y85.0:
X296.0 Y247.0 r23.0
X299.0 Y246.0 r23.0
X299.0 Y247.0 r23.0
X297.0 Y247.0 r23.0
X151.0 Y84.0:
X406.0 Y224.0 r23.0
X152.0 Y85.0:
X322.0 Y352.0 r23.0
X151.0 Y86.0:
X190.0 Y271.0 r23.0
"""

# Use a regular expression to extract the xy coordinates
pattern = re.compile(r"X(\d+\.\d+) Y(\d+\.\d+)")

# Initialize an empty dictionary
result = {}

# Iterate over the lines in the data
for line in data.split("\n"):
    if not len(line): 
        continue
    # If the line matches the pattern
    # pattern = re.compile(r"X(\d+\.\d+) Y(\d+\.\d+)")
    # print(line)
    match = pattern.match(line)
    if match and ":" in line:
        # Extract the x and y coordinates and convert them to floats
        x = float(match.group(1))
        y = float(match.group(2))
        # Add an entry to the dictionary with the coordinates as the key
        # and an empty array as the value
        result[(x, y)] = []
    elif match:
        x = float(match.group(1))
        y = float(match.group(2))
        result[list(result.keys())[-1]].append((x, y))

print(result)