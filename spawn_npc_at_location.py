#!/usr/bin/env python

"""Spawn NPC vehicles into the simulation at a specific location"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=1,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.audi.a2',
        help='vehicles filter (default: "vehicle.audi.a2")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--player_id',
        metavar="ID",
        default=0, #have to be given in the command line
        type=int,
        help='player id (have to be specified)')
    argparser.add_argument(
        "--distance",
        metavar="SPAWN_DISTANCE",
        default=1,
        help="spawn distance of npc vehicle")
    argparser.add_argument(
        "--scenario",
        metavar="SCENARIO",
        default="left",
        help="spawn a vehicle on the left"
    )
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    try:

        world = client.get_world()
        blueprints = world.get_blueprint_library().filter(args.filterv)
        # blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        # spawn_points = world.get_map().get_spawn_points()

        player = world.get_actor(args.player_id)
        if not player:
            print("player not found. Please initialize player first")
            return
        player_location = player.get_location()
        print("player at location: " + str(player_location))
        cur_waypoint = world.get_map().get_waypoint(player_location)

        if args.scenario == "left":
        # spawn vehicle on the left
            spawn_point = cur_waypoint.get_left_lane().next(int(args.distance))[-1].transform
        elif args.scenario == "right":
            spawn_point = cur_waypoint.get_right_lane().next(int(args.distance))[-1].transform
        elif args.scenario == "lead":
            spawn_point = cur_waypoint.next(int(args.distance))[-1].transform
        elif args.scenario == "follow":
            spawn_point = cur_waypoint.next(int(args.distance))[-1].transform
        spawn_points = [spawn_point]
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)
            
        print('spawned %d vehicles, press Ctrl+C to exit.' % (len(vehicles_list)))

        while True:
            world.wait_for_tick()

    
    finally:

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')


# def spawn_vehicle(world, map, player, direction, host, port):
#     SpawnActor = carla.command.SpawnActor
#     SetAutopilot = carla.command.SetAutopilot
#     FutureActor = carla.command.FutureActor

#     # transform = player.get_transform()
#     # print(str(transform))
#     # transform.location.x -= 10
#     # transform.location.y += 5
#     current_waypoint = map.get_waypoint(player.get_location())
#     blueprint = world.get_blueprint_library().find("vehicle.ford.mustang")
#     blueprint.set_attribute('role_name', 'autopilot')
#     print(str(current_waypoint.transform))

#     batch = []
#     client = carla.Client(host, port)

#     if(direction == "left"):
#         left_waypoint = current_waypoint.get_left_lane()
#         print(str(left_waypoint.transform))
#         batch.append(SpawnActor(blueprint, left_waypoint.transform).then(SetAutopilot(FutureActor, True)))
#         print("spawned vehicle on the left")
#         # response = client.apply_batch_sync(batch)
#         for response in client.apply_batch_sync(batch):
#             if response.error:
#                 print("error")
#             else:
#                 print("successful")
#         # for response in client.apply_batch_sync(batch):
#         #     if response.error:
#         #         logging.error(response.error)
#         #     else:
#         #         vehicles_list.append(response.actor_id)
#         return True
#     elif(direction == "right"):
#         right_waypoint = current_waypoint.get_right_lane()
#         batch.append(SpawnActor(blueprint, right_waypoint.transform).then(SetAutopilot(FutureActor, True)))
#         client.apply_batch_sync(batch)
#         return True
#     # failed to spawn npc vehicle
#     return False
    
