import csv
import matplotlib.pyplot as plt
import os

time = []
z = []
vz = []
roll = []
pitch = []

with open('telemetry.csv', newline="") as file:
    reader = csv.DictReader(file)
    for row in reader:
        time.append(float(row["Time"]))
        z.append(float(row["Z"]))
        vz.append(float(row["VZ"]))
        roll.append(float(row["Roll"]))
        pitch.append(float(row["Pitch"]))

## Altitude
plt.figure()
plt.plot(time, z)
plt.title("Drone Altitude over Time")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.grid(True)
plt.savefig("altitude.png")

## Vertical velocity
plt.figure()
plt.plot(time, vz)
plt.xlabel("time (s)")
plt.ylabel("Vertical Velocity vz (m/s)")
plt.title("Vertical Velocity vs Time")
plt.grid(True)
plt.savefig("Velocity.png")

## Orientation
plt.figure()
plt.plot(time, roll, label="Roll") 
plt.plot(time, pitch, label="Pitch")
plt.title("Drone Roll and Pitch over Time")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Roll & Pitch vs Time")
plt.legend()
plt.grid()
plt.savefig("orientation.png")

plt.show()