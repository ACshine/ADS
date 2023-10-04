import carla
import random
from map_parsing import *

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)
traffic_manager.global_percentage_speed_difference(85)
# Set a seed
traffic_manager.set_random_device_seed(0)
random.seed(0)
fixed_initial_location = carla.Location(x=-20.14, y=136.7, z=0.6)
#fixed_initial_location = carla.Location(x=-68.17, y=24.16, z=0.6)
initial_rotation = carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0)
vehicle_blueprint = world.get_blueprint_library().find('vehicle.mercedes.coupe')
vehicle = world.spawn_actor(vehicle_blueprint, carla.Transform(fixed_initial_location, initial_rotation))
world.tick()
simulation_start_time = world.get_snapshot().timestamp.elapsed_seconds
#print(waypoint.transform)
#print(waypoint.road_id)
#print(waypoint.lane_id)
print(vehicle.get_transform());
vehicle.set_autopilot(True)
spectator = world.get_spectator()
spectator_transform = carla.Transform(vehicle.get_transform().location + carla.Location(z=20), carla.Rotation(pitch=-90))
spectator.set_transform(spectator_transform)
red_color = carla.Color(r=255, g=0, b=0)

p1=carla.Location(x=-18.68,y=136.79,z=0)
p2=carla.Location(x=8.82,y=140.76,z=0)
p3=carla.Location(x=7.77,y=140.18,z=0)
p4=carla.Location(x=22.95,y=137.16,z=0)
p5=carla.Location(x=29.26,y=140.6,z=0)
p6=carla.Location(x=34.99,y=140.73,z=0)
p7=carla.Location(x=54.44,y=140.73,z=0)
p8=carla.Location(x=77.53,y=138.99,z=0)
p9=carla.Location(x=82.51,y=132.62,z=0)
p10=carla.Location(x=84.13,y=128.37,z=0)
p11=carla.Location(x=-93.87,y=112.11,z=0)
p12=carla.Location(x=-106.94,y=60.41,z=0)
p13=carla.Location(x=-103.45,y=28.63,z=0)
p14=carla.Location(x=-68.17,y=24.16,z=0)
p15=carla.Location(x=-52.58,y=22.9,z=0)
p16=carla.Location(x=-49.6,y=20.81,z=0)
p17=carla.Location(x=-42.14,y=9.80,z=0)
p18=carla.Location(x=-40.7,y=5.14,z=0)
p19=carla.Location(x=-41.7,y=-17.00,z=0)
route=[p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p17,p18,p19]
#route=[p17,p18,p19]

for i, spawn_point in enumerate(route):
    world.debug.draw_string(spawn_point, str(i), life_time=200)

traffic_manager.set_path(vehicle,route)
backward=False

road_id_list=[16,15,315,19,18,795,5,515,4,8,1]
#road_id_list=[19,315,15,16]
cur=8
#cur=0
road_list = []
get_xml_data('Town10HD.xodr', road_list)
road_node = get_road_node(road_list, road_id_list[cur])
r=Road(road_node)
r.set_backward(True)
r.set_s(19.95)
r.set_s(9.9)
output_file = f'vehicle_info.txt'
with open(output_file, 'w') as f:
    f.write("")  # 清空文件内容
while True:
    world.tick()
    simulation_time = world.get_snapshot().timestamp.elapsed_seconds
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    vehicle_x = transform.location.x
    vehicle_y = -transform.location.y
    #print(vehicle_x,vehicle_y)
    spectator_transform = carla.Transform(transform.location + carla.Location(z=20), carla.Rotation(pitch=-90))
    #spectator.set_transform(spectator_transform)
    (px, py) = r.get_vehicle_project_point(vehicle_x,vehicle_y)
    if px == 1e9:
        #print(r.s)
        if not backward:
            cur += 1
            if cur >= len(road_id_list):
                break
        else:
            cur-=1
            if cur<0:
                break
        #print(road_id_list[cur])
        road_node = get_road_node(road_list, road_id_list[cur])
        r = Road(road_node)
        if backward and cur>=9:
            r.set_backward(True)
    else:
        r.get_offset_to_start(px,py)
        d=calculate_distance(px,py,vehicle_x,vehicle_y)
        #print(d)
        lane_id=r.get_lane_id(d)
        #lane_id = 1000
        if road_id_list[cur]==1 and lane_id==1:
            r.set_backward(True)
            backward=True

        print(f"road_id:{road_id_list[cur]},lane_id:{lane_id},s:{r.s}")
        with open(output_file, "a") as f:
            f.write(f"road_id:{road_id_list[cur]},lane_id:{lane_id},s:{r.s}\n")
        r.to_next_referenceline()
        location = carla.Location(x=px, y=-py, z=0)  # y是反的
        world.debug.draw_point(location, size=0.03, color=red_color, life_time=60.0)



