import sys
sys.path.append('/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts')
from decentralised_adaptive_coverage.scripts.review.initialization import generate_weed_distribution

# generate_weed_distribution(r = 50, num_gaussians=3, bandwidth=5.5, grid_resolution=1, plot=True)
generate_weed_distribution(r = 25, num_gaussians=2, bandwidth=1.5, grid_resolution=1, plot=True)