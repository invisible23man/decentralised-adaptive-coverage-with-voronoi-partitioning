from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def plot_field_3d(x_values, y_values, z_values):
    """
    Function to plot a 3D representation of the field.
    Args:
    x_values : The x-coordinates of the field.
    y_values : The y-coordinates of the field.
    z_values : The z-coordinates (weed densities) of the field.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(x_values, y_values)
    ax.plot_surface(X, Y, z_values.reshape(X.shape), cmap='viridis')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Weed density')
    ax.set_title('3D plot of the field')
    plt.show()