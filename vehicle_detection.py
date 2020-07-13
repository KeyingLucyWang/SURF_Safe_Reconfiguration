# implementation for vehicle detection
import carla
import math

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

