import sys
sys.path.append(r'/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts')

from utils import plots
import configparser

# Load configuration
config = configparser.ConfigParser()
config.read(
    '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini')

plots.plot_results(config)