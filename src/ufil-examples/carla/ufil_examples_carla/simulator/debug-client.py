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


import carla
import time
import argparse
import yaml

def main():
    argparser = argparse.ArgumentParser(description="CARLA Visualizer with Boundary Support")
    argparser.add_argument('--host', default='127.0.0.1', help='IP of the host server')
    argparser.add_argument('-p', '--port', default=2000, type=int, help='TCP port')
    
    # Visualization Toggles
    argparser.add_argument('--no-sensors', action='store_false', dest='show_sensors', help='Hide sensors')
    argparser.add_argument('--no-spawn', action='store_false', dest='show_spawn', help='Hide spawn points')
    argparser.add_argument('--no-arrows', action='store_false', dest='show_arrows', help='Hide orientation arrows')
    argparser.add_argument('--no-labels', action='store_false', dest='show_labels', help='Hide labels')
    argparser.add_argument(
            '--config', type=str, default='/scenario-config.yml',
            help='Path to the config.yml file')
    
    args = argparser.parse_args()
    
    
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    debug = world.debug

    LIFE_TIME = 1.1 
    
    spawn_points = world.get_map().get_spawn_points()
    
    area_of_interest = None
    has_custom_points = False
    try:
        with open(args.config, 'r') as f:
            config_data = yaml.safe_load(f)
            if config_data:
                area_of_interest = config_data.get('area_of_interest')
                
                if 'custom_spawns' in config_data:
                    has_custom_points = True
                    spawn_points = []
                    for sp in config_data['custom_spawns']:
                        loc = carla.Location(x=sp['x'], y=sp['y'], z=sp['z'])
                        rot = carla.Rotation(yaw=sp['yaw'], pitch=sp.get('pitch', 0.0))
                        spawn_points.append(carla.Transform(loc, rot))
    except Exception as e:
        print(f"Error loading YAML: {e}")
    
    try:
        while True:
            if has_custom_points and area_of_interest:
                draw_boundary(debug, area_of_interest, carla.Color(255, 165, 0), "AOI", LIFE_TIME)

            # Logic for Sensors/Static Actors
            if args.show_sensors:
                actors = world.get_actors()
                objects = [a for a in actors if "sensor" in a.type_id or "static" in a.type_id]
                for actor in objects:
                    t = actor.get_transform()
                    if args.show_labels:
                        debug.draw_string(t.location + carla.Location(z=0.5), actor.type_id.split('.')[-1], 
                                            color=carla.Color(255, 255, 0), life_time=LIFE_TIME)
                    debug.draw_box(carla.BoundingBox(t.location, carla.Vector3D(0.05, 0.05, 0.05)), t.rotation, 0.05, life_time=LIFE_TIME)
                    if args.show_arrows:
                        draw_axes(debug, t, LIFE_TIME)

            # Logic for Spawn Points
            if args.show_spawn:
                for i, sp in enumerate(spawn_points):
                    debug.draw_point(sp.location + carla.Location(z=0.1), 0.1, carla.Color(0, 255, 255), LIFE_TIME)
                    if args.show_labels:
                        debug.draw_string(sp.location + carla.Location(z=0.3), f"N: {i}", False, carla.Color(0, 255, 255), LIFE_TIME)
                    if args.show_arrows:
                        debug.draw_arrow(sp.location, sp.location + sp.get_forward_vector() * 0.8, 0.05, 0.1, carla.Color(0, 255, 255), LIFE_TIME)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped.")

def draw_boundary(debug, rect, color, label, life_time):
    min_x, min_y, max_x, max_y = rect
    z = 0.5
    
    p1 = carla.Location(x=min_x, y=min_y, z=z)
    p2 = carla.Location(x=max_x, y=min_y, z=z)
    p3 = carla.Location(x=max_x, y=max_y, z=z)
    p4 = carla.Location(x=min_x, y=max_y, z=z)
    
    debug.draw_line(p1, p2, thickness=0.1, color=color, life_time=life_time)
    debug.draw_line(p2, p3, thickness=0.1, color=color, life_time=life_time)
    debug.draw_line(p3, p4, thickness=0.1, color=color, life_time=life_time)
    debug.draw_line(p4, p1, thickness=0.1, color=color, life_time=life_time)
    
    debug.draw_string(p1 + carla.Location(z=1.0), label, False, color=color, life_time=life_time)

def draw_axes(debug, transform, life_time):
    loc = transform.location
    debug.draw_arrow(loc, loc + transform.get_forward_vector() * 0.8, 0.02, 0.1, carla.Color(255, 0, 0), life_time)
    debug.draw_arrow(loc, loc + transform.get_right_vector() * 0.8, 0.02, 0.1, carla.Color(0, 255, 0), life_time)
    debug.draw_arrow(loc, loc + transform.get_up_vector() * 0.8, 0.02, 0.1, carla.Color(0, 0, 255), life_time)

if __name__ == "__main__":
    main()