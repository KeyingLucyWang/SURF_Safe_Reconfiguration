import carla 
import math

def distance(ego, obstacle):
    dist_x = ego.get_transform().location.x - obstacle.get_location().x
    dist_y = ego.get_transform().location.y - obstacle.get_location().y
    return math.sqrt(dist_x**2 + dist_y**2)


def is_safe_from_obstacles(world):
    # set safe Euclidean distance to 5 meterss
    safe_distance = 5

    ego = world.player
    min_dist = -1
    closest_obstacle = None
    
    valid_obstacle = True
    for obstacle in world.world.get_actors():
        # for actor in world.world.get_actors().filter('sensor.*'):
        #     print(actor.type_id)
        # print(obstacle.type_id)
        # if(obstacle.type_id not in world.world.get_actors().filter('vehicle.*') 
        # and obstacle.type_id not in world.world.get_actors().filter('sensor.*')
        # and obstacle.id != ego.id):
        for vehicle in world.world.get_actors().filter('vehicle.*'):
            if obstacle.type_id == vehicle.type_id:
                valid_obstacle = False
        if(not valid_obstacle):
            break
        
        for sensor in world.world.get_actors().filter('sensor.*'):
            if obstacle.type_id == sensor.type_id:
                print("obstacle" + obstacle.type_id + ". sensor" + sensor.type_id)
                valid_obstacle = False
        if(not valid_obstacle):
            break

        dist = distance(ego, obstacle)
        if(min_dist < 0 or dist < min_dist):
            closest_obstacle = obstacle
            min_dist = dist
        if(dist < safe_distance and dist != 0):
            print((obstacle in world.world.get_actors().filter('sensor.*')))
            return (False, obstacle, dist)
    return (True, closest_obstacle, min_dist)