import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Read current array
current_array_file = open("./python-graph/data/current_array.txt", "r")
current_array = current_array_file.readlines()
current_array = [float(val) for val in current_array]


# Read voltage array
voltage_array_file = open("./python-graph/data/voltage_array.txt", "r")
voltage_array = voltage_array_file.readlines()
voltage_array = [float(val) for val in voltage_array]

plt.plot(voltage_array, current_array)
plt.show()

# def animate(i, xs, ys):

#     # Read data from STM (current voltage)
#     # Serial COM port (as usb)

#     # Add x (voltage) and y(current) to lists
#     xs.append(1)
#     ys.append(2)

#     # Draw x and y lists
#     ax.clear()
#     ax.plot(voltage_array, current_array)
#     ax.plot(xs, ys, "o")


#     # Format plot
#     plt.title('PV Emulation thesis')
#     plt.ylabel('Current (A)')
#     plt.xlabel("Voltage (V)")

#     xs.clear()
#     ys.clear()


# # Set up plot to call animate() function periodically
# ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=100)
# plt.show()
