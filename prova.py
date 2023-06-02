import csv
import os

# Get the path of the file
assignment2_path = "/root/ros_ws/src/assignment2"

file_path = os.path.join(assignment2_path, 'room_coordinates.csv')

# Initialize an empty dictionary to store the room coordinates
room_coordinates = {}

# Open the file and read the contents
with open(file_path, 'r') as csv_file:
    reader = csv.reader(csv_file)
    next(reader)  # Skip the header
    for row in reader:
        # Convert the 'Coordinates' back to a tuple and store it in the dictionary
        room, coordinates = row[0], eval(row[1])
        room_coordinates[room] = coordinates

# Ask for the room you're interested in
room = input("Please enter the name of the room: ")

# Get the coordinates of the specific room
if room in room_coordinates:
    print(room_coordinates[room][0])
    print (type(room_coordinates[room][0]))
else:
    print("Room not found.")
