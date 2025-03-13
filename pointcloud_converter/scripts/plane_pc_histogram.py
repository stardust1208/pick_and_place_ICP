import numpy as np
import matplotlib.pyplot as plt

# TEST to see point cloud of the plane surface

# Step 1: Generate or load your float data
xyz = np.load("plane_pc.npy")
data = xyz[2, :]
print(data.mean(), data.min(), data.max())
# data = np.random.randn(1000)  # Example: 1000 random floats from a normal distribution

# Step 2: Choose the number of bins
num_bins = 20  # You can adjust this number based on your data

# Step 3: Plot the histogram
plt.hist(data, bins=num_bins, alpha=0.7, color='blue', edgecolor='black')

# Adding titles and labels
plt.title('Histogram of Float Variable')
plt.xlabel('Value')
plt.ylabel('Frequency')

# Show the plot
plt.show()
