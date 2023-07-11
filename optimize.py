from scipy.integrate import solve_ivp
from dintegrate import integrate_sensor_values
import numpy as np


def voronoi_ode(t, centers, finite_vertices, finite_regions,
                grid_points, weed_density, time_step, time_limit,
                sensor_func, neighbours, beta):
    """
    The system of ODEs describing the evolution of Voronoi centers.

    Parameters are the same as in `optimize_voronoi_centers_consensus`, except:
    t (float): Current time.
    centers (np.array): Flattened array of Voronoi center coordinates.

    Returns:
    np.array: Time derivatives of Voronoi center coordinates.
    """
    # Reshape the flattened centers array into a 2D array
    voronoi_centers = centers.reshape((-1, 2))

    # Initialize derivatives
    derivatives = np.zeros_like(voronoi_centers)

    # For each Voronoi center
    for i, center in enumerate(voronoi_centers):
        # Get the corresponding partition
        partition = finite_vertices[i]

        # Calculate the moment integrals
        mv, lv, cv = integrate_sensor_values(
            sensor_func, partition, center, grid_points, weed_density, time_step, time_limit)

        # Calculate the consensus term
        consensus_term = sum(
            voronoi_centers[j] - center for j in neighbours[i])

        # Calculate the derivative (negative of the derivative of the cost function, with consensus term)
        derivatives[i] = -mv * (cv - center) + beta * consensus_term

    # Return the flattened derivatives array
    return derivatives.flatten()


def optimize_voronoi_centers_consensus(voronoi_centers, finite_vertices, finite_regions,
                                       grid_points, weed_density, sensor_func, neighbours, time_step=0.05, time_limit=5, beta=0.1, t_span=(0, 10)):
    """
    Optimize Voronoi centers using scipy.integrate.solve_ivp.

    Parameters and return value are the same as in the previous version of `optimize_voronoi_centers_consensus`,
    except `t_span` is added to specify the interval of integration for solve_ivp.

    Returns:
    np.array: Updated coordinates of Voronoi centers at the final time.
    """
    # Solve the ODE
    solution = solve_ivp(
        fun=voronoi_ode,
        t_span=t_span,
        y0=voronoi_centers.flatten(),
        args=(finite_vertices, finite_regions,
              grid_points, weed_density, time_step, time_limit,
              sensor_func, neighbours, beta),
        method='RK45',  # You can choose other methods
        dense_output=True  # Allows to evaluate the solution at any point within t_span
    )

    # Evaluate the solution at the final time
    final_centers = solution.sol(t_span[1])

    # Reshape the flattened centers array into a 2D array
    voronoi_centers = final_centers.reshape((-1, 2))

    return voronoi_centers
