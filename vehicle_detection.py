# implementation for vehicle detection
import carla
import math

# runs one step in the pre-crash simulation
# returns a tuple of location and velocity
def simulation_run_step(location, velocity, acceleration, interval):
    # use kinematic formula: delta x = v0*t + (1/2)a*t^2
    displacement_x = velocity.x * interval + (1/2)*acceleration.x*(interval**2)
    displacement_y = velocity.y * interval + (1/2)*acceleration.y*(interval**2)
    displacement_z = velocity.z * interval + (1/2)*acceleration.z*(interval**2)

    sx = displacement_x + location.x
    sy = displacement_y + location.y
    sz = displacement_z + location.z

    vx = velocity.x + acceleration.x*interval
    vy = velocity.y + acceleration.y*interval
    vz = velocity.z + acceleration.z*interval

    new_location = carla.Location(sx, sy, sz)
    new_velocity = carla.Vector3D(vx, vy, vz)
    
    return (new_location, new_velocity)

def check_for_collision(location1, location2):
    # naive implementation
    return location1.distance(location2) < 5

def time_to_collision(world, ego, vehicle, ttc_threshold, fps):    
    ego_location = ego.get_location()
    ego_velocity = ego.get_velocity()
    ego_acceleration = ego.get_acceleration()

    npc_location = vehicle.get_location()
    npc_velocity = vehicle.get_velocity()
    npc_acceleration = vehicle.get_acceleration()

    min_ttc = ttc_threshold
    interval = 1.0/fps
    t = 0.0
    int_t = int(t)
    while int_t < min_ttc:
        # check if a crash occurs
        (ego_location, ego_velocity) = simulation_run_step(ego_location, ego_velocity, ego_acceleration, interval)
        (npc_location, npc_velocity) = simulation_run_step(npc_location, npc_velocity, npc_acceleration, interval)

        if (check_for_collision(ego_location, npc_location)):
            print("fps is {}, time interval is 1/fps = {}".format(fps, interval))
            print("ego velocity after {} seconds: {}".format(t, ego_velocity))
            ego_speed = 3.6 * math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
            print("ego speed: {} km/h".format(ego_speed))
            print("npc velocity after {} seconds: {}".format(t, npc_velocity))
            npc_speed = 3.6 * math.sqrt(npc_velocity.x**2 + npc_velocity.y**2 + npc_velocity.z**2)
            print("ego speed: {} km/h".format(npc_speed))

            print("ego location: " + str(ego_location))
            print("npc location: " + str(npc_location))
            cur_waypoint = world.map.get_waypoint(ego_location)
            print("lane width: {}".format(str(cur_waypoint.lane_width)))
            print("crash distance: {} (should be less than 5)\n".format(ego_location.distance(npc_location)))
            return t
        t += interval
        int_t = int(t)
    return min_ttc



def is_safe_ttc(world, fps):
    # set the ttc threshold to 4 seconds
    min_ttc = 4

    ego = world.player

    #..., -2, -1, 0(reference line), 1, 2,...
    # same signedness indicates same direction
    # cur_lane = cur_waypoint.lane_id 
    # cur_road = cur_waypoint.road_id

    # left_lane = cur_waypoint.get_left_lane()
    # right_lane = cur_waypoint.get_right_lane()

    for vehicle in world.world.get_actors().filter('vehicle.*'):
        # only consider vehicles within a 200 meter radius
        if (vehicle.id != ego.id and ego.get_location().distance(vehicle.get_location()) < 200):
            ttc = time_to_collision(world, ego, vehicle, min_ttc, fps)
            if (ttc < min_ttc):
                print("potential crash with {}".format(vehicle.type_id))
                print("at distance {}\n".format(ego.get_location().distance(vehicle.get_location())))
                return (False, ttc)
    return (True, min_ttc)

# detect the lateral distance between the ego vehicle and a specified npc vehicle
def lateral_distance(ego, npc):
    ego_x = ego.get_location().x 
    npc_x = npc.get_location().x

    #set a constant safe distance of 2 meters for now (temp)
    safe_dist = 2 
    return abs(ego_x - npc_x) >= safe_dist

# detect the longitudinal distance between the ego vehicle and a specified npc vehicle
def longitudinal_distance(ego, npc):
    ego_y = ego.get_location().y
    npc_y = npc.get_location().y

    #set a constant safe distance of 2 meters for now (temp)
    safe_dist = 2 
    return abs(ego_y - npc_y) >= safe_dist

#return the shortest lateral distance between the ego vehicle and all other vehicles
def shortest_lat_dist():
    return -1

#return the shortest longitudinal distance between the ego vehicle and all other vehicles
def shortest_long_dist():
    return -1

def distance(ego, other):
    dist_x = ego.get_transform().location.x - other.get_location().x
    dist_y = ego.get_transform().location.y - other.get_location().y
    return math.sqrt(dist_x**2 + dist_y**2)



#return whether the ego vehicle is at a safe distance from all other vehicles on the road
#check before each run step to adjust speed and maintain a safe distance in Autonomous mode
def is_safe_from_vehicles(world):
    # set safe Euclidean distance to 50 meterss
    safe_distance = 50

    ego = world.player
    min_dist = -1
    closest_vehicle = None
    for vehicle in world.world.get_actors().filter('vehicle.*'):
        if(vehicle.id != ego.id):
            dist = distance(ego, vehicle)
            if(min_dist < 0 or dist < min_dist):
                closest_vehicle = vehicle
                min_dist = dist
            if(dist < safe_distance):
                return (False, vehicle, dist)
    return (True, closest_vehicle, min_dist)

def is_safe_longitudinal(world):
    ego = world.player
    cur_waypoint = world.map.get_waypoint(ego.get_location())

    cur_lane = cur_waypoint.lane_id 
    cur_road = cur_waypoint.road_id

    safe_distance = 50
    list_waypoints = cur_waypoint.next(safe_distance)

    for vehicle in world.world.get_actors().filter('vehicle.*'):
        if (vehicle.id != ego.id):
            vehicle_waypoint = world.map.get_waypoint(vehicle.get_location())
            if (vehicle_waypoint in list_waypoints): #need testing
                return (False, vehicle, vehicle_waypoint)
    return (True)


def is_safe_lateral(world):
    ego = world.player
    cur_waypoint = world.map.get_waypoint(ego.get_location())

    #..., -2, -1, 0(reference line), 1, 2,...
    # same signedness indicates same direction
    cur_lane = cur_waypoint.lane_id 
    cur_road = cur_waypoint.road_id

    left_lane = cur_waypoint.get_left_lane()
    right_lane = cur_waypoint.get_right_lane()
    
    # if left_lane:
    #     #do something
        
    # if right_lane:
    #     #do something
    
