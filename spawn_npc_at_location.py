# transform = world.player.get_transform()
# nearest_waypoint = world.map.get_waypoint(transform.location, project_to_road=True)
# left_wp = nearest_waypoint.get_left_lane()
# right_wp = nearest_waypoint.get_right_lane()
import carla

def spawn_vehicle(world, map, player, direction, host, port):
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # transform = player.get_transform()
    # print(str(transform))
    # transform.location.x -= 10
    # transform.location.y += 5
    current_waypoint = map.get_waypoint(carla.Location(10, -170, 30))
    blueprint = world.get_blueprint_library().find("vehicle.ford.mustang")
    blueprint.set_attribute('role_name', 'autopilot')
    print(str(current_waypoint.transform))

    batch = []
    client = carla.Client(host, port)

    if(direction == "left"):
        left_waypoint = current_waypoint.get_left_lane()
        print(str(left_waypoint.transform))
        batch.append(SpawnActor(blueprint, left_waypoint.transform).then(SetAutopilot(FutureActor, True)))
        print("spawned vehicle on the left")
        # response = client.apply_batch_sync(batch)
        for response in client.apply_batch_sync(batch):
            if response.error:
                print("error")
            else:
                print("successful")
        # for response in client.apply_batch_sync(batch):
        #     if response.error:
        #         logging.error(response.error)
        #     else:
        #         vehicles_list.append(response.actor_id)
        return True
    elif(direction == "right"):
        right_waypoint = current_waypoint.get_right_lane()
        batch.append(SpawnActor(blueprint, right_waypoint.transform).then(SetAutopilot(FutureActor, True)))
        client.apply_batch_sync(batch)
        return True
    # failed to spawn npc vehicle
    return False
    
