import numpy as np

# Let's say your tracking system outputs a 2D array
# Columns represent: X, Y, Z_Rotation
theta = np.arange(0, 2*np.pi, 0.1)  # Simulated angle data
R = 2
xpos = R * np.cos(theta)  # Simulated X positions
ypos = R * np.sin(theta)  # Simulated Y positions
data_matrix = np.column_stack((xpos, ypos))
# Save it directly to a CSV
np.savetxt(
    'tracking_data_matrix.csv', 
    data_matrix, 
    delimiter=',', 
    header='Position_X,Position_Y', 
    comments='',  # Prevents NumPy from adding a '#' before the header row
    fmt='%.3f'    # Optional: Formats the output to exactly 3 decimal places
)