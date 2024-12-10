# data_parser.py

import math

DATA_FILE = "data.txt"  # Path to your data file
current_line = 0  # Keeps track of the last read line

def parse_line(line):
    """
    Parse a single line to extract pitch, roll, yaw, and altitude, converting angles to radians.
    Example line: "Pitch:45.12,Roll:10.50,Yaw:90.75,Alt:500.00"
    """
    try:
        data = {}
        segments = line.split(",")
        for segment in segments:
            if ":" in segment:
                key, value = segment.split(":")
                key = key.strip().lower()
                data[key] = float(value)
        return {
            "x": math.radians(data.get("pitch", 0)),  # Convert pitch to radians
            "y": math.radians(data.get("roll", 0)),   # Convert roll to radians
            "z": math.radians(data.get("yaw", 0)),    # Convert yaw to radians
            "alt": data.get("alt", 0)                 # Altitude remains as is
        }
    except Exception as e:
        print(f"Error parsing line: {line}, error: {e}")
        return {"x": 0, "y": 0, "z": 0, "alt": 0}

def get_next_data():
    global current_line
    try:
        with open(DATA_FILE, "r") as file:
            lines = file.readlines()

            # If there are new lines to read, parse the next one
            if current_line < len(lines):
                line = lines[current_line].strip()
                current_line += 1
                parsed_data = parse_line(line)
                return parsed_data
            else:
                # No new data
                return None
    except FileNotFoundError:
        print("Data file not found")
        return {"error": "Data file not found"}