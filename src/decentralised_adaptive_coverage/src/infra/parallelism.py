import multiprocessing
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
from tqdm import tqdm
from robots import Robot
from scipy.spatial import Voronoi

def run_iterative_process(drone:Robot, vor:Voronoi, grid_points, weed_density, config):
    n_iter = config.getint('ITERATIVE_PROCESS', 'n_iterations')
    for _ in range(n_iter):
        drone.move_and_sense(vor, grid_points, weed_density)
        drone.update()
        drone.calculate_new_voronoi_center()

def parallelize_iterations(drones, vor, grid_points, weed_density, config):
    progress_lock = Lock()
    progress_counter = 0
    parallelism_mode = config.get('INFRA_SETUP', 'parallelism_mode')
    
    def update_progress():
        with progress_lock:
            pbar.update()
    
    def worker(drone):
        run_iterative_process(drone, vor, grid_points, weed_density, config)
        update_progress()
    
    # Determine the appropriate executor based on the multiprocessing flag
    if parallelism_mode == 'multiprocessing':
        executor = multiprocessing.Pool()
        imap_unordered = executor.imap_unordered
        worker_count = multiprocessing.cpu_count()
    if parallelism_mode == 'threading':
        executor = ThreadPoolExecutor()
        imap_unordered = executor.map
        # worker_count = multiprocessing.cpu_count() * 5  # Adjust the thread count as needed
        worker_count = len(drones)
        executor = ThreadPoolExecutor(worker_count)


    with tqdm(total=len(drones)) as pbar:
        if parallelism_mode == 'multiprocessing':
            for _ in imap_unordered(worker, drones):
                pass
        if parallelism_mode == 'threading':
            futures = []
            for drone in drones:
                future = executor.submit(worker, drone)
                futures.append(future)
            
            for future in futures:
                future.result()
    
    executor.shutdown()

    return worker_count
