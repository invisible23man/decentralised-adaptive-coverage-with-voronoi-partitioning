import rospy
from decentralised_adaptive_coverage.srv import VoronoiUpdate, VoronoiUpdateResponse

class Service:
    def __init__(self, drone):
        self.drone = drone

    def setup_services(self):
        self.drone.update_voronoi_completed = False
        service_name = f'/drone{self.drone.drone_id+1}/update_voronoi_service'
        rospy.Service(service_name, VoronoiUpdate, self.drone_update_voronoi_service)
        rospy.wait_for_service(service_name)

    def drone_update_voronoi_service(self, request):
        return VoronoiUpdateResponse(success=self.drone.update_voronoi_completed, iteration_timestamp=self.drone.current_iteration)


    def wait_for_update_voronoi_for_all_drones(self):
        drone_update_voronoi_completed = [False] * self.drone.drone_count
        while not all(drone_update_voronoi_completed):
            rospy.loginfo(f"Drone {self.drone.drone_id+1} waiting for {self.drone.drone_count-sum(drone_update_voronoi_completed)} Drones at iteration: {self.drone.current_iteration+1}")
            for i in range(self.drone.drone_count):
                try:
                    check_update = rospy.ServiceProxy(
                        '/drone{}/update_voronoi_service'.format(i+1), VoronoiUpdate)
                    resp = check_update()
                    drone_update_voronoi_completed[i] = resp.success and resp.iteration_timestamp == self.drone.current_iteration
                except rospy.ServiceException as e:
                    rospy.loginfo(f"Service call failed: {e}")
            rospy.sleep(5)