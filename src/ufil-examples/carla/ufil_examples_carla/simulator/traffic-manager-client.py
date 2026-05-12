#!/usr/bin/env python

# Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import time

import carla

import argparse
import logging
import yaml
from numpy import random

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host', metavar='H', default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port', metavar='P', default=2000, type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles', metavar='N', default=30, type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers', metavar='W', default=10, type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe', action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv', metavar='PATTERN', default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv', metavar='G', default='All',
        help='restrict to certain vehicle generation (values: "2","3","All" - default: "All")')
    argparser.add_argument(
        '--filterw', metavar='PATTERN', default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw', metavar='G', default='All',
        help='restrict to certain pedestrian generation (values: "2","3","All" - default: "All")')
    argparser.add_argument(
        '--tm-port', metavar='P', default=8000, type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch', action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid', action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed', metavar='S', type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw', metavar='S', default=0, type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on', action='store_true', default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero', action='store_true', default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn', action='store_true', default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering', action='store_true', default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '--spawn-rate', type=float, default=0.1,
        help='Baseline probability (0.0 to 1.0) of a spawn attempt per tick')
    argparser.add_argument(
        '--config', type=str, default='/scenario-config.yml',
        help='Path to the config.yml file')

    args = argparser.parse_args()
    
    player_transform = None
    custom_spawn_points = []
    area_of_interest = None
    try:
        with open(args.config, 'r') as f:
            data = yaml.safe_load(f)
            
            area_of_interest = data.get('area_of_interest')
                
            if 'custom_spawns' in data:
                for sp in data['custom_spawns']:
                    loc = carla.Location(x=sp['x'], y=sp['y'], z=sp['z'])
                    rot = carla.Rotation(yaw=sp['yaw'])
                    custom_spawn_points.append(carla.Transform(loc, rot))
            
            if 'player_spawn' in data:
                ps = data.get('player_spawn')
                if ps:
                    player_transform = carla.Transform(
                        carla.Location(x=ps['x'], y=ps['y'], z=ps['z']),
                        carla.Rotation(
                            yaw=ps['yaw'], 
                            pitch=ps.get('pitch', 0.0), 
                            roll=ps.get('roll', 0.0)
                        )
                    )
    except FileNotFoundError:
        logging.warning("No config file found. Using default global spawning.")

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()
        
        if player_transform:
            spec_loc = player_transform.location
            spec_trans = carla.Transform(spec_loc, player_transform.rotation)
            world.get_spectator().set_transform(spec_trans)

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode, and traffic might experience some issues")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        if custom_spawn_points:
            logging.info("Custom waypoints detected. Activating AOI filtering.")
            spawn_points = custom_spawn_points
            
            if area_of_interest:
                spawn_points = [sp for sp in spawn_points if is_within_rect(sp.location, area_of_interest)]
        else:
            logging.info("No custom waypoints provided. Using all default map spawn points.")
            spawn_points = world.get_map().get_spawn_points()
        
        target_vehicle_count = args.number_of_vehicles

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero

        # Randomize spawn points for initial distribution
        random.shuffle(spawn_points)
        for n, transform in enumerate(spawn_points):
            if n >= target_vehicle_count:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        if args.car_lights_on:
            all_vehicle_actors = world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers
        # -------------
        percentagePedestriansRunning = 0.0      
        percentagePedestriansCrossing = 0.0     
        if args.seedw:
            world.set_pedestrians_seed(args.seedw)
            random.seed(args.seedw)
        spawn_points_walkers = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                if area_of_interest and not is_within_rect(loc, area_of_interest):
                    continue
                spawn_point.location = loc
                spawn_point.location.z += 2
                spawn_points_walkers.append(spawn_point)

        batch = []
        walker_speed = []
        for spawn_point in spawn_points_walkers:
            walker_bp = random.choice(blueprintsWalkers)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            all_actors[i].start()
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

            current_actors = world.get_actors(vehicles_list)
            valid_ids = []
            
            # Clean up out-of-bounds or destroyed actors
            to_destroy = []
            for actor in current_actors:
                if actor is None: continue
                if custom_spawn_points and area_of_interest:
                    if not is_within_rect(actor.get_location(), area_of_interest):
                        to_destroy.append(actor.id)
                        continue 
                valid_ids.append(actor.id)

            if to_destroy:
                client.apply_batch([carla.command.DestroyActor(x) for x in to_destroy])
            
            # Sync the tracked list
            vehicles_list = valid_ids

            # Stochastic Respawning logic
            current_count = len(vehicles_list)
            if current_count < target_vehicle_count:
                # 1.0 when empty, 0.0 when full
                spawn_pressure = 1.0 - (current_count / target_vehicle_count)
                
                if spawn_points and random.random() < (args.spawn_rate * spawn_pressure):
                    # Attempt to spawn a single vehicle at a random valid point
                    transform = random.choice(spawn_points)
                    
                    is_occupied = False
                    all_vehicles = world.get_actors().filter('vehicle.*')
                    for vehicle in all_vehicles:
                        if vehicle.get_location().distance(transform.location) < 5.0:
                            is_occupied = True
                            break
                    if not is_occupied:
                        blueprint = random.choice(blueprints)
                        if blueprint.has_attribute('color'):
                            blueprint.set_attribute('color', random.choice(blueprint.get_attribute('color').recommended_values))
                        blueprint.set_attribute('role_name', 'autopilot')

                        # Non-blocking spawn attempt
                        respawn_op = SpawnActor(blueprint, transform).then(
                            SetAutopilot(FutureActor, True, traffic_manager.get_port())
                        )
                        
                        response = client.apply_batch_sync([respawn_op], synchronous_master)[0]
                        if not response.error:
                            vehicles_list.append(response.actor_id)

    finally:
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        
        time.sleep(0.5)
        
def is_within_rect(location, rect):
    """Checks if a location (x, y) is inside [min_x, min_y, max_x, max_y]"""
    if rect is None: return True
    return rect[0] <= location.x <= rect[2] and rect[1] <= location.y <= rect[3]

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')