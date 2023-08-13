import configparser
import os

def generate_experiment_tag(filter_config):
    tag = 'f'
    for k, v in filter_config.items():
        # Replace special characters with meaningful alternatives
        if v is not None:
            if not isinstance(v,str):
                v=str(v)
            v = v.replace('*', 'x').replace('(', 'I_').replace(')', '_I')
        else:
            v = '0'  # Default value when v is None
        
        abbreviated_key = ''.join(word[0] for word in k.split('_'))  # Takes the first letter of each word in the key
        tag += f'_{abbreviated_key}{v}'
    
    # Truncate the tag if it's too long (e.g., 100 characters)
    max_length = 100
    if len(tag) > max_length:
        tag = tag[:max_length]
    
    return tag


def read_config_file(filename):
    config = configparser.ConfigParser()
    config.read(filename)

    field_section = config['FIELD']
    size = int(field_section.get('size', 50))
    grid_resolution = int(field_section.get('grid_resolution', 1))
    weed_centers = eval(field_section.get('weed_centers', '[[8, -5], [20, 22]]'))
    weed_cov = eval(field_section.get('weed_cov', '[[5, 0], [0, 5]]'))

    team_section = config['TEAM']
    drone_count = int(team_section.get('drone_count', 8))
    formation_pattern = team_section.get('formation_pattern', 'circle')

    run_section = config['RUN']
    iterations = int(run_section.get('iterations', 10))
    disable_warnings = run_section.getboolean('disable_warnings', True)

    planner_section = config['PLANNER']
    reordermode = planner_section.get('planner_algorithm', None)
    planner_config = {
        "planner_algorithm": reordermode,
        "formation_pattern": formation_pattern
    }

    sampling_section = config['SAMPLING']
    sampling_time = int(sampling_section.get('sampling_time', 30))

    estimator_section = config['ESTIMATOR']
    weigh_uncertainity = estimator_section.get('weigh_uncertainity', None)
    estimator_name = estimator_section.get('estimator_name', None)
    num_particles = int(estimator_section.get('num_particles', None)) if estimator_section.get('num_particles') else None
    temperature = float(estimator_section.get('temperature', None)) if estimator_section.get('temperature') else None
    cooling = float(estimator_section.get('cooling', None)) if estimator_section.get('cooling') else None
    kernel = estimator_section.get('kernel', None)

    if estimator_name == "GPR":
        estimator_config = {
            "estimator_name": estimator_name,
            "kernel": kernel,
            "weigh_uncertainity": weigh_uncertainity
        }
    elif estimator_name == 'Particle Filter':
        estimator_config = {
            "estimator_name": estimator_name,
            "num_particles": num_particles,
            "temperature": temperature,
            "cooling": cooling,
            "weigh_uncertainity": weigh_uncertainity
        }
    else:
        raise ValueError("Invalid Estimator Specified. Choose from GPR, Particle Filter")

    results_section = config['RESULTS']
    experiment_logging_directory = results_section.get('experiment_logging_directory', None)
    anim_save_directory = results_section.get('anim_save_directory', None)


    experiment_timestamp = ''
    experiment_filtertag = f's-{sampling_time}-it{iterations}-{generate_experiment_tag(estimator_config)}'
    experiment_filename = os.path.join(experiment_logging_directory,experiment_timestamp,
                                       f'{experiment_filtertag}-data.pkl')
    animation2d_filename = os.path.join(experiment_logging_directory,experiment_timestamp,
                                        f'{experiment_filtertag}-animation2d.gif')
    animation3d_filename = os.path.join(experiment_logging_directory,experiment_timestamp,
                                        f'{experiment_filtertag}-animation3d.gif')

    return config, size, grid_resolution, weed_centers, weed_cov, drone_count, \
        iterations, disable_warnings, planner_config, sampling_time, estimator_config, \
            experiment_filename, animation2d_filename, animation3d_filename

