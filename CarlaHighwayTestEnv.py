import carla
import argparse
import numpy as np

from srunner.tools.scenario_helper import generate_target_waypoint
from srunner.challenge.utils.route_manipulation import interpolate_trajectory
from srunner.scenariomanager.timer import GameTime

import random
import time
import weakref
import json
# import re
import collections
import math
import copy
import traceback

import controller2d_m
import configparser
import local_planner
import behavioural_planner
import gym
from gym import spaces
from copy import deepcopy

from PIL import Image, ImageDraw

COLOR_R = (255, 0, 0)
COLOR_G = (0, 255, 0)
COLOR_B = (0, 0, 255)
COLOR_GRAY = (128, 128, 128)
COLOR_WIHTE = (128, 128, 128)
PIXELS_PER_METER = 12

NUM_PATHS = 1
BP_LOOKAHEAD_BASE = 15.0  # m 15.0 8.0
BP_LOOKAHEAD_TIME = 1.0  # s 1.0 0.8
PATH_OFFSET = 0.7  # m
CIRCLE_OFFSETS = [-1.0, 1.0, 3.0]  # m
CIRCLE_RADII = [1.5, 1.5, 1.5]  # m
TIME_GAP = 1.0  # s
PATH_SELECT_WEIGHT = 10
A_MAX = 2.0  # m/s^2
SLOW_SPEED = 2.0  # m/s
STOP_LINE_BUFFER = 3.5  # m
LEAD_VEHICLE_LOOKAHEAD = 30.0  # m
LP_FREQUENCY_DIVISOR = 3  # Frequency divisor to make the
# local planner operate at a lower
# frequency than the controller
# (which operates at the simulation
# frequency). Must be a natural
# number.
INTERP_DISTANCE_RES = 0.01  # distance between interpolated points
DESIRE_SPEED = 60.0 / 3.6  # m/s


class MapImage(object):
    """Class encharged of rendering a 2D image from top view of a carla world.
    Please note that a cache system is used, so if the OpenDrive content
    of a Carla town has not changed, it will read and use the stored image
    if it was rendered in a previous execution"""

    def __init__(self, carla_world, carla_map, pixels_per_meter):
        """ Renders the map image generated based on the world,
        its map and additional flags that provide extra information about the road network"""
        self._pixels_per_meter = pixels_per_meter
        self.scale = 1.0

        waypoints = carla_map.generate_waypoints(2)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        # Maximum size of a Pygame surface
        width_in_pixels = (1 << 14) - 1

        # Adapt Pixels per meter to make world fit in surface
        surface_pixel_per_meter = int(width_in_pixels / self.width)
        if surface_pixel_per_meter > PIXELS_PER_METER:
            surface_pixel_per_meter = PIXELS_PER_METER

        self._pixels_per_meter = surface_pixel_per_meter
        width_in_pixels = int(self._pixels_per_meter * self.width)

        self.image = Image.open('highway_{}_crop.png'.format(PIXELS_PER_METER))
        # box = (0, 0, 6600, 988)
        # self.image = self.image.crop(box)
        # print(self.image.mode)

    def world_to_pixel(self, location, offset=(0, 0)):
        """Converts the world coordinates to pixel coordinates"""
        x = self.scale * self._pixels_per_meter * (location.x - self._world_offset[0])
        y = self.scale * self._pixels_per_meter * (location.y - self._world_offset[1])
        return (int(x - offset[0]), int(y - offset[1]))


def distance_vehicle(waypoint, vehicle_position):
    dx = waypoint['lat'] - vehicle_position[0]
    dy = waypoint['lon'] - vehicle_position[1]

    return math.sqrt(dx * dx + dy * dy)


def get_distance(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


def get_current_pose(transform):
    x = transform.location.x
    y = transform.location.y
    yaw = transform.rotation.yaw
    r_yaw = yaw * math.pi / 180.0
    if r_yaw > math.pi:
        r_yaw -= 2 * math.pi
    elif r_yaw < -math.pi:
        r_yaw += 2 * math.pi
    return x, y, r_yaw


def get_speed(current_pos, prev_pos, delta_time):
    dx = current_pos[0] - prev_pos[0]
    dy = current_pos[1] - prev_pos[1]
    # dz = current_pos[2] - prev_pos[2]
    # dis = math.sqrt(dx * dx + dy * dy + dz * dz)
    dis = math.sqrt(dx * dx + dy * dy)
    if delta_time == 0:
        return 0
    else:
        return dis / delta_time


def calculate_speed(v):
    speed = math.sqrt(v[0] * v[0] + v[1] * v[1])
    if speed < 1e-2:
        speed = 0
    return speed


def generate_route(world, vehicle_location, hop_resolution=1.0):
    # generate a route for current scenario
    # based on current scenario and map
    current_map = world.get_map()

    # get initial location of ego_vehicle
    start_waypoint = current_map.get_waypoint(vehicle_location)

    # generate a dense route according to current scenario
    # could ref to <scenario_helper> module
    turn_flag = 0  # turn_flag by current scenario
    mid_waypoint = generate_target_waypoint(start_waypoint, turn_flag)
    end_waypoint_list = mid_waypoint.next(20)
    end_waypoint = end_waypoint_list[-1]
    turn_flag = -1  # turn_flag by current scenario
    mid_waypoint = generate_target_waypoint(end_waypoint, turn_flag)
    end_waypoint_list = mid_waypoint.next(20)
    end_waypoint = end_waypoint_list[-1]

    # generate a dense route
    # Setting up global router
    waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
    gps_route, trajectory = interpolate_trajectory(world, waypoints, hop_resolution)
    return gps_route, trajectory


def draw_trajectory(world, global_plan_world_coord, persistency=-1, vertical_shift=1):
    for index in range(len(global_plan_world_coord)):
        waypoint = global_plan_world_coord[index][0]
        location = waypoint.location + carla.Location(z=vertical_shift)
        world.debug.draw_point(location, size=0.1, color=carla.Color(34, 125, 81), life_time=persistency)


def draw_waypoints(world, waypoints, ego_location, persistency=-1.0, vertical_shift=1.5, color=(255, 0, 0)):
    if waypoints is None or len(waypoints) == 0:
        return
    for position in waypoints:
        location = carla.Location(x=position[0], y=position[1], z=ego_location.z + vertical_shift)
        # height = world.get_map().get_waypoint(location).transform.location.z
        # location = carla.Location(x=position[0], y=position[1], z=height+vertical_shift)
        draw_color = carla.Color(color[0], color[1], color[2])
        if location.distance(ego_location) > 2.5:
            world.debug.draw_point(location, size=0.1, color=draw_color, life_time=persistency)


def set_sespector(world, ego_trans, wp_angle=None):
    angle = wp_angle if wp_angle is not None else ego_trans.rotation.yaw
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(x=10, z=60),
                                            carla.Rotation(pitch=-90)))


def get_local_coordinate_frame(world, origin_transform, axis_length_scale=3.0, persistency=1.0):
    yaw = np.deg2rad(origin_transform.rotation.yaw)
    # x axis
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    origin_coord = np.array(
        [origin_transform.location.x, origin_transform.location.y, origin_transform.location.z + 2])
    # elevate z coordinate
    Origin_location = carla.Location(origin_coord[0], origin_coord[1], origin_coord[2])
    # x axis destination
    x_des_coord = origin_coord + axis_length_scale * np.array([cy, sy, 0])
    x_des = carla.Location(x_des_coord[0], x_des_coord[1], x_des_coord[2])
    # y axis destination
    y_des_coord = origin_coord + axis_length_scale * np.array([-sy, cy, 0])
    y_des = carla.Location(y_des_coord[0], y_des_coord[1], y_des_coord[2])
    # z axis destination
    z_des_coord = origin_coord + axis_length_scale * np.array([0, 0, 1])
    z_des = carla.Location(z_des_coord[0], z_des_coord[1], z_des_coord[2])

    # axis feature
    # thickness = 0.1f
    # arrow_size = 0.1f
    if persistency > 0.0:
        x_axis_color = carla.Color(255, 0, 0)
        y_axis_color = carla.Color(0, 255, 0)
        z_axis_color = carla.Color(0, 0, 255)
        world.debug.draw_arrow(Origin_location, x_des, color=x_axis_color, life_time=persistency)
        world.debug.draw_arrow(Origin_location, y_des, color=y_axis_color, life_time=persistency)
        world.debug.draw_arrow(Origin_location, z_des, color=z_axis_color, life_time=persistency)

    return x_des, y_des, z_des


class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        # self.collision = False
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        # self.collision = True
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


class CarlaEnv(gym.Env):
    action_space = spaces.Discrete(3)
    shape = (64, 64, 3)
    observation_space = spaces.Box(low=0, high=255, shape=shape, dtype=np.uint8)

    def __init__(self, carla_world, traffic_manager=None, port=2000, client=None, logdir='', record=False):
        self.world = carla_world
        self.port = port
        self.client = carla.Client('localhost', self.port) if client is None else client
        self.client.set_timeout(100.0)
        self.logdir = logdir
        if record:
            self.logdir.mkdir(parents=True, exist_ok=True)
        self.timestamp = None
        self.count_time = 0
        self.index = 60

        self.map = self.world.get_map()
        start_waypoint_1 = self.map.get_waypoint(carla.Location(x=0.0, y=1.25, z=0.5))
        end_waypoint_1 = self.map.get_waypoint(carla.Location(x=1000.0, y=1.25, z=0.5))
        start_waypoint_2 = self.map.get_waypoint(carla.Location(x=0.0, y=5.25, z=0.5))
        end_waypoint_2 = self.map.get_waypoint(carla.Location(x=1000.0, y=5.25, z=0.5))
        start_waypoint_3 = self.map.get_waypoint(carla.Location(x=0.0, y=7.25, z=0.5))
        end_waypoint_3 = self.map.get_waypoint(carla.Location(x=1000.0, y=7.25, z=0.5))
        topology = [(start_waypoint_1, end_waypoint_1), (start_waypoint_2, end_waypoint_2), (start_waypoint_3, end_waypoint_3)]
        def get_topology():
            return topology
        self.map.get_topology = get_topology
        def get_map():
            return self.map
        self.world.get_map = get_map

        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicles = self.blueprint_library.filter('vehicle.*')
        self.cars = [x for x in self.vehicles if int(x.get_attribute('number_of_wheels')) == 4]
        self.spawn_points = self.map.get_spawn_points()

        self.map_image = MapImage(
            carla_world=self.world,
            carla_map=self.map,
            pixels_per_meter=PIXELS_PER_METER)

        self.actor_list = []
        self.actor_dic = []
        self.hero = None
        self.sensor = None
        self.imu = None
        # self.reset()
        self.recording_enabled = record
        self.recording_start = 0
        self._waypoints = None
        self.multi_waypoints = []
        self.routes = []
        self.scenario = None
        self.little_scenario = None
        self.actor_speed_scenario = None
        self.test_scen = 0
        self.vehicle_list = []

        self.lp = None
        self.bp = None
        self.local_planner = None
        self.controller = None
        self.current_timestamp = 0.0
        self.prev_timestamp = 0.0
        self.start_time = 0.0
        self.end_time = 0.0
        self.start_pos = 0.0
        self.count_frame = 0
        self.count_frame_time = 0
        self.count_step = 0
        self.count_stop = 0
        self.lane = 1
        self.init_lane = self.lane
        self.local_waypoints = None
        self.lead_car_speed = 0.0
        self.desired_speed = 0.0
        self.dis_to_goal = BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * DESIRE_SPEED
        self.run_traffic_light = False
        self.tl_waypoint = None
        self.collision = False
        self.collision_times = 0
        self.collision_his = 0
        self.speed_sum = 0.0
        self.speed_step = 0
        self.lane_change_times = 0
        self.max_acc = 0.0
        self.ttc = np.Inf
        self.min_ttc = self.ttc
        self.min_dis = np.Inf
        self.last_speed = 0.0
        self.last_time = 0.0
        self.scenario_start = False

        self.current_control = carla.VehicleControl()
        self.current_control.steer = 0.0
        self.current_control.throttle = 0.0
        self.current_control.brake = 1.0
        self.current_control.hand_brake = False
        self.world.on_tick(self._update_timestamp)

        self.traffic_manager = None
        if traffic_manager is not None:
            self.traffic_manager = traffic_manager
            self.traffic_manager.set_global_distance_to_leading_vehicle(5.0)
            # print("traffic_manager set")

        # test results
        self.success_num = 0
        self.collsion_num = 0
        self.change_failure_num = 0
        self.exceed_acc_num = 0
        self.collision_condition_list = []
        self.change_failure_condition_list = []
        self.acc_condition_list = []

        # scenarios
        self.speed_refer_dic = {0:0, 5:5.5, 10:11.6, 15:18.4, 20:24.3, 25:31.0, 30:38.2, 
                                35:45.9, 40:54.0, 45:62.3, 50:66.3, 55:66.31, 60:68.6, 65:73.7}
        self.test_scenario = [i for i in range(1,6)]
        self.car_num_dic = {1:1, 2:2, 3:2, 4:2, 5:2}
        self.ego_lane_dic = {1:2, 2:2, 3:1, 4:2, 5:2}
        self.other_lane_dic = {1:[2], 2:[2,1], 3:[1,0], 4:[2,1], 5:[2,2]}
        front_car_dis = 100
        self.distance_dic = {1:{0:[front_car_dis], }, 
                             2:{0:[front_car_dis], 1:[-100+10*i for i in range(6)]},
                             3:{0:[front_car_dis], 1:[-100+20*i for i in range(11)]},
                             4:{0:[front_car_dis], 1:[50+10*i for i in range(6)]},
                             5:{0:[front_car_dis], 1:[front_car_dis+40+10*i for i in range(5)]}}
        self.speed_dic = {1:{0:[5*i for i in range(10)], },
                          2:{0:[0], 1:[30+5*i for i in range(5)]},
                          3:{0:[25+10*i for i in range(3)], 1:[5+10*i for i in range(7)]},
                          4:{0:[25+10*i for i in range(3)], 1:[5+10*i for i in range(7)]},
                          5:{0:[30+5*i for i in range(5)], 1:[0]}}
        self.test_scenario_dic = {}
        self.scen_num = 0
        self.scen_refer_dic = {}
        self.length_dic = {}
        length_speed_dic = {0:200, 5:200, 10:250, 15:250, 20:300, 25:300, 30:400, 35:400, 
                            40:500, 45:500, 50:600, 55:800}
        for scen in self.test_scenario:
            self.test_scenario_dic[scen] = self.scen_num
            if self.car_num_dic[scen] > 1:
                for i in self.distance_dic[scen][0]:
                    for j in self.distance_dic[scen][1]:
                        for k in self.speed_dic[scen][0]:
                            for l in self.speed_dic[scen][1]:
                                self.scen_refer_dic[self.scen_num] = [i,j,k,l]
                                self.length_dic[self.scen_num] = length_speed_dic[k]
                                self.scen_num += 1
            else:
                for i in self.distance_dic[scen][0]:
                    for k in self.speed_dic[scen][0]:
                        self.scen_refer_dic[self.scen_num] = [i,k]
                        self.length_dic[self.scen_num] = length_speed_dic[k]
                        self.scen_num += 1
        self.test_scenario_dic[scen+1] = self.scen_num
        # print(self.scen_num, self.test_scenario_dic)

    def _update_timestamp(self, snapshot):
        self.timestamp = snapshot.timestamp
        if self.scenario == 6:
            self.count_time += 1

    def reset(self, scenario=None, test_scen=None):
        if (self.test_scen >= self.scen_num and scenario is None) \
                or (scenario in self.test_scenario and self.test_scen >= self.test_scenario_dic[scenario+1]):
            self.test_scen = 0 if scenario is None else self.test_scenario_dic[scenario]
            print('A test loop has finished, begin next test loop.')
            print('collision scenarios:', self.collision_condition_list)
            print('change fail scenarios:', self.change_failure_condition_list)
            print('exceed acc scenarios:', self.acc_condition_list)

        if scenario is not None and self.scenario is None:
            self.scenario = scenario
            if scenario in self.test_scenario:
                self.test_scen = self.test_scenario_dic[scenario]
        else:
            if test_scen is not None:
                if test_scen in range(self.scen_num):
                    self.test_scen = test_scen
                else:
                    raise ValueError(f'Test scenario must in range [0, {self.scen_num}] !')
            idx = 0
            for scen in self.test_scenario:
                if self.test_scen >= self.test_scenario_dic[scen]:
                    idx += 1
            self.scenario = idx if scenario is None else scenario

        self.reset_para()
        blueprint = random.choice(self.vehicles.filter('vehicle.audi.tt'))
        blueprint.set_attribute('role_name', 'autopilot')
        def spawn_npc(x, y, multi_wapoints):
            num_npc = 0
            loc = multi_wapoints[x][y]
            wp = self.map.get_waypoint(carla.Location(x=loc[0], y=loc[1]))
            transform = carla.Transform(wp.transform.location +
                                        carla.Location(z=0.5), wp.transform.rotation)
            npc = self.world.try_spawn_actor(blueprint, transform)
            if npc is not None:
                self.actor_list.append(npc)
                num_npc += 1
                if self.scenario == 6:
                    npc_control = carla.VehicleControl(throttle=0.0, steer=0, brake=1)
                    npc.apply_control(npc_control)
            return num_npc

        # prepare the scenario
        loc = self.hero.get_transform().location
        _, closest_index = behavioural_planner.get_closest_index(self._waypoints, [loc.x, loc.y])
        multi_wps = self.multi_waypoints
        if self.scenario == 0:  # no other car
            num_npcs = 0
        elif self.scenario in self.test_scenario:
            num_npcs = 0
            for car in range(self.car_num_dic[self.scenario]):
                car_lane = self.other_lane_dic[self.scenario][car]
                car_index = closest_index + self.scen_refer_dic[self.test_scen][car]
                num_npcs += spawn_npc(car_lane, car_index, multi_wps)
        else:  # random spawn cars
            num_npcs = 0
            expect_num_npcs = np.random.randint(4, 9)  # np.random.randint(6, 13)
            while num_npcs < expect_num_npcs:
                x = np.random.randint(3)
                y = np.random.randint(self.index-40, self.index+180)
                num_npcs += spawn_npc(x, y, multi_wps)

        self.world.tick()
        self.actor_dic = []
        # npc speed
        if self.scenario in self.test_scenario:
            for i in range(len(self.actor_list)):
                actor = self.actor_list[i]
                actor.set_autopilot(True, self.port+6000)
                self.traffic_manager.auto_lane_change(actor, False)
                actor_speed = self.scen_refer_dic[self.test_scen][i+2] \
                                if self.scenario > 1 else self.scen_refer_dic[self.test_scen][i+1]
                actor.set_velocity(carla.Vector3D(x=actor_speed/3.6, y=0., z=0.))
                actor_speed_limit = actor.get_speed_limit()
                percentage_v_dif = (1.0 - self.speed_refer_dic[actor_speed] / actor_speed_limit) * 100
                self.traffic_manager.vehicle_percentage_speed_difference(actor, percentage_v_dif)
                dis_to_lead = BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * actor_speed / 3.6
                dic = {'id':actor.id, 'target speed': actor_speed, 'desired gap': dis_to_lead}
                self.actor_dic.append(dic)
        else:
            for actor in self.actor_list:
                actor.set_autopilot(True, self.port+6000)
                self.traffic_manager.auto_lane_change(actor, False)
                dis_to_lead = np.random.randint(15)
                self.traffic_manager.distance_to_leading_vehicle(actor, dis_to_lead)
                actor_speed_limit = actor.get_speed_limit()
                target_speed = np.random.randint(11, 54)
                percentage_v_dif = (1.0 - target_speed / actor_speed_limit) * 100
                self.traffic_manager.vehicle_percentage_speed_difference(actor, percentage_v_dif)
                dic = {'id':actor.id, 'target speed': target_speed, 'desired gap': dis_to_lead}
                self.actor_dic.append(dic)

        self.world.tick()
        if self.scenario == 5:
            self.traffic_manager.force_lane_change(self.actor_list[0], False)
        if self.scenario in self.test_scenario:
            print("Current scenario: {}, index: {}, parameters: {}, length: {}".format(
                  self.scenario, self.test_scen, self.scen_refer_dic[self.test_scen], 
                  self.length_dic[self.test_scen]))
            lane_length = self.length_dic[self.test_scen]
            if len(self.multi_waypoints[0]) > self.index + lane_length:
                self.multi_waypoints = self.multi_waypoints[:, :self.index + lane_length]
        else:
            print("number of other cars: {}, scenario: {}".format(num_npcs, self.scenario))
            self.multi_waypoints = self.multi_waypoints[:, :self.index + 400]
            for _ in range(10):
                self.step_one_loop(0)
        self.count_frame_time = 0
        self.count_step = 0
        self.scenario_start = True
        GameTime.on_carla_tick(self.timestamp)
        self.start_time = GameTime.get_time()
        self.start_pos = self.hero.get_location().x
        if self.recording_enabled:  # self.scenario in self.test_scenario and
            self.client.start_recorder(str(self.logdir.resolve()) + f'/{self.test_scen}.log')
        ob, _, done, _ = self.get_state(self.hero)
        if done:
            ob = self.reset(scenario, test_scen)
            self.test_scen -= 1
        return ob

    def reset_para(self, ticks=30):
        self.world.set_weather(carla.WeatherParameters.ClearNoon)  # ClearNoon, ClearSunset, etc.
        self.destroy()
        self.lp = local_planner.LocalPlanner(NUM_PATHS,
                                             PATH_OFFSET,
                                             CIRCLE_OFFSETS,
                                             CIRCLE_RADII,
                                             PATH_SELECT_WEIGHT,
                                             TIME_GAP,
                                             A_MAX,
                                             SLOW_SPEED,
                                             STOP_LINE_BUFFER)
        self.bp = behavioural_planner.BehaviouralPlanner(BP_LOOKAHEAD_BASE,
                                                         LEAD_VEHICLE_LOOKAHEAD)
        self.controller = None
        self.count_time = 0
        self.current_timestamp = 0.0
        self.prev_timestamp = 0.0
        self.count_frame = 0
        self.count_frame_time = 0
        # self.vehicles = None
        self._waypoints = None
        self.multi_waypoints = []
        self.local_waypoints = None
        self.lead_car_speed = 0.0
        self.desired_speed = 0.0
        self.run_traffic_light = False
        self.tl_waypoint = None
        self.vehicle_list = []
        self.max_acc = 0.0
        self.ttc = np.Inf
        self.min_ttc = self.ttc
        self.min_dis = np.Inf
        self.last_speed = 0.0
        self.last_time = 0.0
        self.scenario_start = False
        self.lane_change_times = 0
        self.count_step = 0
        self.count_stop = 0
        self.speed_sum = 0.0
        self.speed_step = 0

        success, trans = self.setup_hero()
        if success:
            if self.scenario in self.test_scenario:
                self.world.tick()
                GameTime.on_carla_tick(self.timestamp)
                timestamp = GameTime.get_time()
                self.controller.vars.t_prev = timestamp
                ego_speed = 0.0
                while ego_speed < DESIRE_SPEED - 0.2:
                    ego_speed = self.step_one_loop(0)
                self.last_speed = ego_speed
                GameTime.on_carla_tick(self.timestamp)
                self.last_time = GameTime.get_time()
                # set_sespector(self.world, self.hero.get_transform())
            else:
                self.current_control.steer = 0.0
                self.current_control.throttle = 0.0
                self.current_control.brake = 1.0
                self.current_control.hand_brake = False
                self.hero.apply_control(self.current_control)
                for _ in range(ticks):
                    self.world.tick()
                GameTime.on_carla_tick(self.timestamp)
                timestamp = GameTime.get_time()
                self.controller.vars.t_prev = timestamp
                set_sespector(self.world, self.hero.get_transform())

    def setup_hero(self, model='vehicle.lincoln.mkz2017', random_location=False):
        # model = 'vehicle.tesla.model3'
        blueprint = random.choice(self.vehicles.filter(model))
        blueprint.set_attribute('role_name', 'hero')
        if random_location:
            transform = random.choice(self.spawn_points)
        else:
            transform = carla.Transform(carla.Location(x=50.0, y=5.25, z=0.5))
        if self.hero is None:
            # self.hero = self.world.try_spawn_actor(blueprint, transform)
            # gps_route, trajectory = generate_route(self.world, transform.location)
            self.multi_waypoints = []
            start_waypoint = self.map.get_waypoint(transform.location)
            end_waypoint = start_waypoint.next(self.index + 850.5)[-1]
            left_start_waypoint = start_waypoint.get_left_lane()
            left_end_waypoint = end_waypoint.get_left_lane()
            right_start_waypoint = start_waypoint.get_right_lane()
            right_end_waypoint = end_waypoint.get_right_lane()
            waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
            left_waypoints = [left_start_waypoint.transform.location, left_end_waypoint.transform.location]
            right_waypoints = [right_start_waypoint.transform.location, right_end_waypoint.transform.location]
            gps_route, trajectory = interpolate_trajectory(self.world, waypoints, 1.0)
            left_gps_route, left_trajectory = interpolate_trajectory(self.world, left_waypoints, 1.0)
            right_gps_route, right_trajectory = interpolate_trajectory(self.world, right_waypoints, 1.0)
            self.set_global_plan(gps_route, trajectory)

            left_waypoints_list = self._get_waypoints(left_trajectory)
            waypoints_list = self._get_waypoints(trajectory)
            right_waypoints_list = self._get_waypoints(right_trajectory)
            self.multi_waypoints.append(left_waypoints_list)
            self.multi_waypoints.append(waypoints_list)
            self.multi_waypoints.append(right_waypoints_list)
            self.multi_waypoints = np.array(self.multi_waypoints)
            
            self.lane = self.ego_lane_dic[self.scenario] \
                        if self.scenario in self.test_scenario else np.random.randint(3)
            self.init_lane = self.lane
            if self.lane == 0:
                spawn_ego_trans = carla.Transform(left_trajectory[self.index][0].location +
                                                  carla.Location(z=0.5), left_trajectory[self.index][0].rotation)
            elif self.lane == 2:
                spawn_ego_trans = carla.Transform(right_trajectory[self.index][0].location +
                                                  carla.Location(z=0.5), right_trajectory[self.index][0].rotation)
            else:
                spawn_ego_trans = carla.Transform(trajectory[self.index][0].location +
                                                  carla.Location(z=0.5), trajectory[self.index][0].rotation)
            self.hero = self.world.try_spawn_actor(blueprint, spawn_ego_trans)
            if self._waypoints is None and self._global_plan is not None:
                self._waypoints = self.multi_waypoints[self.lane]
                self.controller = controller2d_m.Controller2D(self._waypoints)
            if self.hero is not None:
                self.sensor = CollisionSensor(self.hero)
                return True, transform
            else:
                print('create ego car failed')
                return False, transform
        else:
            self.hero.set_transform(transform)
            start_waypoint = self.map.get_waypoint(transform.location)
            end_waypoint = self.map.get_waypoint(carla.Location(x=9.0, y=124.67))
            waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
            gps_route, trajectory = interpolate_trajectory(self.world, waypoints, 1.0)
            self.set_global_plan(gps_route, trajectory)
            if self._waypoints is None and self._global_plan is not None:
                self._waypoints = self._get_waypoints(self._global_plan_world_coord)
                self.controller = controller2d_m.Controller2D(self._waypoints)
            return True, transform

    def step(self, action):
        self.count_step += 1
        if action < -0.001:  # MOBIL policy
            action = self.change_lane_policy()
        # -------------------------------------------------------------------------------------------
        """ Some rules are added to ensure basic safety (https://arxiv.org/abs/1904.00231). 
        Commet this if you wish. """
        ego_x = self.hero.get_location().x
        ego_y = self.hero.get_location().y
        ego_yaw = self.hero.get_transform().rotation.yaw
        ego_lane = self.lane
        front_dis = 60.1
        behind_conditions = []
        for actor in self.world.get_actors().filter('vehicle.audi*'):
            actor_x = actor.get_location().x
            actor_y = actor.get_location().y
            actor_yaw = actor.get_transform().rotation.yaw
            actor_lane = -2 - (self.map.get_waypoint(carla.Location(x=actor_x, y=actor_y)).lane_id)
            y_relative = actor_x - ego_x
            front_dis = min(front_dis, y_relative) if (actor_lane == ego_lane and 0 < y_relative) else front_dis
            behind_condition = (-20 < y_relative < 20) and ((action*(ego_lane-actor_lane)==1 or action*(ego_lane-actor_lane)==-2))
            behind_conditions.append(behind_condition)

        if (any(behind_conditions) or front_dis > 60.0) and action != 0:
            action = 0
        # -------------------------------------------------------------------------------------------

        self.step_one_loop(action, is_new_action=True)
        # for _ in range(2):
        #     self.step_one_loop(action, is_new_action=False)
        ob, reward, done, info = self.get_state(self.hero, show_message=False)
        return ob, reward, done, info

    def step_one_loop(self, action, is_new_action=False):
        self.count_frame += 1
        self.count_frame_time += 1
        GameTime.on_carla_tick(self.timestamp)
        timestamp = GameTime.get_time()
        delta_time = timestamp - self.prev_timestamp
        # print("timestamp: {}, delta_time: {}".format(timestamp, delta_time))
        if delta_time == 0:
            return
        self.prev_timestamp = self.current_timestamp
        self.current_timestamp = timestamp

        trans = self.hero.get_transform()
        current_x, current_y, current_yaw = get_current_pose(trans)
        v_xy = self.hero.get_velocity()
        current_speed = math.sqrt(v_xy.x ** 2 + v_xy.y ** 2 + v_xy.z ** 2)
        ego_wp = self.get_vehicle_wp(self.hero)
        ego_s = ego_wp.s
        ego_lane = ego_wp.lane_id
        if current_speed < 0.01:
            self.count_stop += 1
        lead_car = None
        lead_v = None
        d_cf = np.Inf
        ttc = np.Inf
        dis = np.Inf
        # if self.scenario >= 6:
        for i, actor in enumerate(self.world.get_actors().filter('vehicle.audi*')):
            actor_wp = self.get_vehicle_wp(actor)
            actor_lane = actor_wp.lane_id
            actor_s = actor_wp.s
            if actor_lane == (-self.lane - 2) or actor_lane == ego_lane:
                if actor_s >= ego_s and actor_s - ego_s < d_cf:
                    d_cf = actor_s - ego_s
                    lead_car = actor
        if lead_car is not None:
            v_xy = lead_car.get_velocity()
            lead_v = math.sqrt(v_xy.x ** 2 + v_xy.y ** 2 + v_xy.z ** 2)
            if current_speed > lead_v:
                ttc = d_cf / (current_speed - lead_v)
            dis = self.get_vehicle_wp(lead_car).s - ego_s
            self.min_ttc = min(ttc, self.min_ttc)
            self.min_dis = min(dis, self.min_dis)

        if self.scenario_start and self.recording_enabled:
            with (self.logdir / f'speed{self.test_scen}.jsonl').open('a') as f:
                f.write(json.dumps({'current_speed': current_speed, 
                                    'timestamp': timestamp,
                                    'ttc': ttc,
                                    'dis': dis}) + '\n')

        if self.scenario_start and (self.count_frame_time % 20 == 0):
            acc = (current_speed - self.last_speed) / (timestamp - self.last_time)
            abs_acc = math.fabs(acc)
            self.max_acc = max(abs_acc, self.max_acc)
            if math.fabs(abs_acc - self.max_acc) < 0.001:
                self.ttc = ttc
            safe_dis = 0.8 * current_speed + 8.0
            if self.recording_enabled:
                with (self.logdir / f'acc{self.test_scen}.jsonl').open('a') as f:
                    f.write(json.dumps({'current_speed': current_speed, 
                                        'last_speed': self.last_speed,
                                        'timestamp': timestamp,
                                        'delta_time': (timestamp - self.last_time),
                                        'acc': acc,
                                        'abs_acc': abs_acc,
                                        'lead_v': lead_v,
                                        'safe_dis': safe_dis,
                                        'ttc': ttc,
                                        'dis': dis,
                                        'max_acc': self.max_acc,
                                        'min_ttc': self.min_ttc,
                                        'min_dis': self.min_dis}) + '\n')
            self.last_speed = current_speed
            self.last_time = timestamp

        lead_car_pos = []
        lead_car_speed = []
        parkedcar_box_pts = []
        self.bp._stopsign_fences = []
        # best_path = []

        if self.count_frame % LP_FREQUENCY_DIVISOR == 0 or is_new_action:
            self.count_frame = 1
            # ------------------------- change lane -------------------------
            last_lane = self.lane
            if action == 0 and is_new_action:
                self._waypoints = self.multi_waypoints[self.lane]
            elif action == 1 and (ego_lane == -3 or ego_lane == -4) and is_new_action:
                if self.lane >= 1:
                    self.lane -= 1
                # self._waypoints = self.multi_waypoints[self.lane]
            elif action == 2 and (ego_lane == -3 or ego_lane == -2) and is_new_action:
                # self.lane = self.lane + 1 if self.lane <= 1 else self.lane
                if self.lane <= 1:
                    self.lane += 1
                # self._waypoints = self.multi_waypoints[self.lane]
            intent_lane = -self.lane - 2
            if abs(intent_lane - ego_lane) == 2:
                self.lane = 1
            self._waypoints = self.multi_waypoints[self.lane]
            if last_lane != self.lane:
                self.lane_change_times += 1
            # self.local_planner.set_global_plan(self.routes[self.lane])
            # ---------------------------------------------------------------

            # ------------------------- get lead car info -------------------------
            d_cf = self.bp._lookahead
            # d_cf = BP_LOOKAHEAD_BASE + (BP_LOOKAHEAD_TIME - 0.5) * current_speed
            lead_car = None
            # if self.scenario >= 6:
            for i, actor in enumerate(self.world.get_actors().filter('vehicle.audi*')):
                actor_wp = self.get_vehicle_wp(actor)
                actor_lane = actor_wp.lane_id
                actor_s = actor_wp.s
                if actor_lane == (-self.lane - 2):
                    if actor_s >= ego_s and actor_s - ego_s < d_cf:
                        d_cf = actor_s - ego_s
                        lead_car = actor
            if lead_car is not None:
                # print(lead_car)
                lead_loc = lead_car.get_location()
                lead_car_pos.append([lead_loc.x, lead_loc.y])
                lead_v = lead_car.get_velocity()
                lead_car_speed.append(math.sqrt(lead_v.x ** 2 + lead_v.y ** 2 + lead_v.z ** 2))
            # ---------------------------------------------------------------------

            open_loop_speed = self.lp._velocity_planner.get_open_loop_speed(delta_time)
            # print('open_loop_speed: {}'.format(open_loop_speed))
            ego_state = [current_x, current_y, current_yaw, open_loop_speed]
            # print(ego_state)
            self.bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)
            self.bp.transition_state(self._waypoints, ego_state, current_speed, action, is_new_action)

            if len(lead_car_pos) != 0:
                self.bp.check_for_lead_vehicle(ego_state, lead_car_pos[0])
            goal_state_set = self.lp.get_goal_state_set(self.bp._goal_index,
                                                        self.bp._goal_state, self._waypoints, ego_state)

            # ------------------------- draw goal states -------------------------
            # for goal_state in goal_state_set:
            #     goal_x = ego_state[0] + goal_state[0]*math.cos(ego_state[2]) - \
            #                                     goal_state[1]*math.sin(ego_state[2])
            #     goal_y = ego_state[1] + goal_state[0]*math.sin(ego_state[2]) + \
            #                                     goal_state[1]*math.cos(ego_state[2])
            #     location = carla.Location(goal_x, goal_y, 1.0)
            #     color = carla.Color(0, 255, 255)
            #     self.world.debug.draw_point(location, size=0.1, color=color, life_time=0.1)
            # ---------------------------------------------------------------------

            paths, path_validity = self.lp.plan_paths(goal_state_set)
            paths = local_planner.transform_paths(paths, ego_state)
            collision_check_array = self.lp._collision_checker.collision_check(paths, parkedcar_box_pts)
            goal_state = self.bp._goal_state
            best_index = self.lp._collision_checker.select_best_path_index(paths, collision_check_array,
                                                                           goal_state)
            if best_index == None:
                best_path = self.lp._prev_best_path
            else:
                best_path = paths[best_index]
                self.lp._prev_best_path = best_path

            # ------------------------- draw paths -------------------------
            # for path in paths:
            #     t_path = list(map(list, zip(*path)))
            #     draw_waypoints(self.world, t_path, self.hero.get_location(), 0.1, 1.0, (0, 0, 255))
            # --------------------------------------------------------------

            if self.bp._state == behavioural_planner.FOLLOW_LANE:
                self.desired_speed = DESIRE_SPEED
            elif self.bp._state == behavioural_planner.DECELERATE:
                self.desired_speed -= 0.2
                self.desired_speed = max(self.desired_speed, 0.0)
            else:
                self.desired_speed = 0.0
            if len(lead_car_pos) >= 1:
                lead_car_state = [lead_car_pos[0][0], lead_car_pos[0][1], lead_car_speed[0]]
            else:
                lead_car_state = []
            decelerate = self.bp._state == behavioural_planner.DECELERATE
            self.local_waypoints = self.lp._velocity_planner.compute_velocity_profile(
                best_path, self.desired_speed, ego_state, current_speed, decelerate,
                lead_car_state, self.bp._follow_lead_vehicle)

            if self.local_waypoints is not None:
                wp_distance = []  # distance array
                self.local_waypoints_np = np.array(self.local_waypoints)
                for i in range(1, self.local_waypoints_np.shape[0]):
                    wp_distance.append(
                        np.sqrt((self.local_waypoints_np[i, 0] - self.local_waypoints_np[i - 1, 0]) ** 2 +
                                (self.local_waypoints_np[i, 1] - self.local_waypoints_np[i - 1, 1]) ** 2))
                wp_distance.append(0)
                wp_interp = []  # interpolated values
                # (rows = waypoints, columns = [x, y, v])
                for i in range(self.local_waypoints_np.shape[0] - 1):
                    wp_interp.append(list(self.local_waypoints_np[i]))
                    num_pts_to_interp = int(np.floor(wp_distance[i] / \
                                                     float(INTERP_DISTANCE_RES)) - 1)
                    wp_vector = self.local_waypoints_np[i + 1] - self.local_waypoints_np[i]
                    wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                    for j in range(num_pts_to_interp):
                        next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
                        wp_interp.append(list(self.local_waypoints_np[i] + next_wp_vector))
                wp_interp.append(list(self.local_waypoints_np[-1]))
                self.controller.update_waypoints(wp_interp)

        if self.local_waypoints is not None and self.local_waypoints != []:
            self.controller.update_values(current_x, current_y, current_yaw,
                                          current_speed, self.current_timestamp, self.count_frame)
            self.controller.update_controls()
            cmd_throttle, cmd_steer, cmd_brake = self.controller.get_commands()
        else:
            cmd_throttle = 0.0
            cmd_steer = 0.0
            cmd_brake = 0.0

        if not self.scenario_start and (self.bp._goal_index != len(self._waypoints) - 1):
            cmd_throttle = 1.0
            cmd_steer = 0.0
            cmd_brake = 0.0

        self.current_control.steer = cmd_steer
        self.current_control.throttle = cmd_throttle
        self.current_control.brake = cmd_brake
        self.current_control.hand_brake = False
        self.hero.apply_control(self.current_control)
        draw_waypoints(self.world, self.local_waypoints, self.hero.get_location(), 0.05, 1.0)

        if ego_lane == -2:
            sespector_wp = ego_wp.get_right_lane()
        elif ego_lane == -4:
            sespector_wp = ego_wp.get_left_lane()
        else:
            sespector_wp = ego_wp
        set_sespector(self.world, sespector_wp.transform, sespector_wp.transform.rotation.yaw)
        for i, actor in enumerate(self.world.get_actors().filter('vehicle.audi*')):
            actor_wp = self.get_vehicle_wp(actor)
            actor_s = actor_wp.s
            if actor_s > 900:
                actor.set_autopilot(False, self.port+6000)
                npc_control = carla.VehicleControl(throttle=0, steer=0, brake=1)
                actor.apply_control(npc_control)
        self.world.tick()
        return current_speed

    def get_state(self, vehicle, show_message=False):
        # TODO: define reward function
        v_xy = vehicle.get_velocity()
        current_speed = math.sqrt(v_xy.x ** 2 + v_xy.y ** 2 + v_xy.z ** 2) * 3.6
        speed = copy.copy(current_speed)
        # print("speed: {}".format(current_speed))
        trans = vehicle.get_transform()
        loc = trans.location
        rot = trans.rotation
        vehicle_waypoint = self.map.get_waypoint(loc)
        vehicle_lane = vehicle_waypoint.lane_id
        ego_lane = -(vehicle_lane + 3.0)

        dis_to_goal = get_distance(loc.x, loc.y, self._waypoints[-1][0], self._waypoints[-1][1])
        # print(self.dis_to_goal, dis_to_goal)
        if dis_to_goal >= self.dis_to_goal:
            desire_speed = DESIRE_SPEED
        else:
            desire_speed = DESIRE_SPEED * (dis_to_goal / self.dis_to_goal)
        if self.bp._goal_index == len(self._waypoints) - 1:
            target = 1
            self.scenario_start = False
        else:
            target = -1

        if desire_speed > DESIRE_SPEED:
            desire_speed = DESIRE_SPEED
        elif desire_speed < 0:
            desire_speed = 0.0
        if speed > DESIRE_SPEED:
            speed = DESIRE_SPEED
        speed = speed / DESIRE_SPEED
        target_speed = desire_speed / DESIRE_SPEED

        list_vehicles = self.world.get_actors().filter('vehicle.*')
        map_image = copy.copy(self.map_image.image)
        draw = ImageDraw.Draw(map_image)
        self.render_vehicles(draw, list_vehicles, self.map_image.world_to_pixel)
        # angle = 0.0 if self.hero is None else rot.yaw + 90.0
        angle = 0.0 if self.hero is None else vehicle_waypoint.transform.rotation.yaw + 90.0
        hero_location_screen = self.map_image.world_to_pixel(loc)
        resultIm = map_image.rotate(angle, center=hero_location_screen)
        resultIm_center = hero_location_screen
        box = (resultIm_center[0]-150, resultIm_center[1]-600, 
               resultIm_center[0]+150, resultIm_center[1]+300)
        cropIm = resultIm.crop(box)
        cropIm = cropIm.resize((64, 64), Image.ANTIALIAS)
        ob = cropIm

        done = False
        collision = False
        success = False
        # if self.scenario in self.test_scenario:
        if abs(ego_lane) > 1:
            done = True
        closest_len, closest_index = behavioural_planner.get_closest_index(self._waypoints, [loc.x, loc.y])
        if closest_index == len(self._waypoints) - 1:
            done = True
        if dis_to_goal < 2.5:
            done = True
        if target_speed < 0.40 and speed < 0.05:
            done = True
        if self.count_stop >= 30:
            done = True
        # else:
        #     if self.count_step >= 200:
        #         done = True
            
        self.run_traffic_light = False

        if self.sensor is not None:
            if len(self.sensor.history) > 0:
                collision = True
                self.collision_condition_list.append([self.scenario, self.test_scen])
                if (self.scenario in self.test_scenario) and self.recording_enabled:
                    with (self.logdir.parent / 'collision.jsonl').open('a') as f:
                        f.write(json.dumps({'scenario': self.scenario, 'index': self.test_scen,
                                            'parameters': self.scen_refer_dic[self.test_scen]}) + '\n')
                self.collsion_num += 1
                # if self.scenario in self.test_scenario:
                done = True
        if self.collision_his != len(self.sensor.history):
            self.collision_times += 1
            self.collision_his = len(self.sensor.history)
        if done and (self.lane != self.init_lane) and (not collision):
            success = True
            # print(self.lane, self.init_lane)
            self.success_num +=1
        if done and (not success) and (not collision):
            self.change_failure_condition_list.append([self.scenario, self.test_scen])
            if (self.scenario in self.test_scenario) and self.recording_enabled:
                with (self.logdir.parent / 'fail.jsonl').open('a') as f:
                    f.write(json.dumps({'scenario': self.scenario, 'index': self.test_scen,
                                        'parameters': self.scen_refer_dic[self.test_scen]}) + '\n')
            self.change_failure_num += 1
        if done and self.max_acc >= 2.0:
            self.acc_condition_list.append([self.scenario, self.test_scen])
            if (self.scenario in self.test_scenario) and self.recording_enabled:
                with (self.logdir.parent / 'exceed_acc.jsonl').open('a') as f:
                    f.write(json.dumps({'scenario': self.scenario, 'index': self.test_scen,
                                        'parameters': self.scen_refer_dic[self.test_scen],
                                        'max acc': self.max_acc, 'ttc': self.ttc, 
                                        'min ttc': self.min_ttc, 'min dis': self.min_dis}) + '\n')
            self.exceed_acc_num +=1 

        self.speed_sum += current_speed
        self.speed_step += 1
        if done:
            GameTime.on_carla_tick(self.timestamp)
            self.end_time = GameTime.get_time()

        info = {
            'index': self.test_scen,
            'success': success,
            'max acc': round(self.max_acc, 3),
            'collision': collision,
            # 'ttc': round(self.ttc, 3),
            'min ttc': round(self.min_ttc, 3),
            'min dis': round(self.min_dis, 3),
            'lane change times': self.lane_change_times,
            # 'collision times': self.collision_times,
            'steps': self.count_step,
            'frames': self.count_frame_time,
            'average speed': self.speed_sum / self.speed_step,
            'success, collision, fail and exceed acc': [self.success_num, self.collsion_num, 
                                                        self.change_failure_num, self.exceed_acc_num]
        }
        if done:
            print(info, f'episode time: {self.end_time - self.start_time}', f'length: {loc.x - self.start_pos}')
            if (self.scenario in self.test_scenario) and self.recording_enabled:
                with (self.logdir.parent / 'test_result.jsonl').open('a') as f:
                    f.write(json.dumps({'scenario': self.scenario,
                                        'scen index': self.test_scen,
                                        'scen parameters': self.scen_refer_dic[self.test_scen],
                                        'success': success,
                                        'max acc': self.max_acc,
                                        'collision': collision,
                                        'min ttc': self.min_ttc,
                                        'min dis': self.min_dis,
                                        'lane change times': self.lane_change_times,
                                        'steps': self.count_step,
                                        'frames': self.count_frame_time,
                                        'average speed': self.speed_sum / self.speed_step,
                                        # 'collision scenarios': self.collision_condition_list, 
                                        # 'change fail scenarios': self.change_failure_condition_list, 
                                        # 'exceed acc scenarios': self.acc_condition_list
                                        }) + '\n')
            elif self.recording_enabled:
                with (self.logdir.parent / 'test_result.jsonl').open('a') as f:
                    f.write(json.dumps({'scenario': self.scenario,
                                        'num of cars': len(self.actor_list),
                                        'index': self.test_scen,
                                        'success': success,
                                        'max acc': self.max_acc,
                                        'collision': collision,
                                        'min ttc': self.min_ttc,
                                        'min dis': self.min_dis,
                                        'lane change times': self.lane_change_times,
                                        'steps': self.count_step,
                                        'frames': self.count_frame_time,
                                        'start time': self.start_time,
                                        'end time': self.end_time,
                                        'episode time': self.end_time - self.start_time,
                                        'length': loc.x - self.start_pos,
                                        'average speed': self.speed_sum / self.speed_step,
                                        }) + '\n')
            self.destroy()
            if self.scenario in self.test_scenario and self.recording_enabled:
                self.client.stop_recorder()
            self.test_scen += 1
        return ob, 0., done, info

    def render_vehicles(self, draw, list_v, world_to_pixel):
        """Renders the vehicles' bounding boxes"""
        for v in list_v:
            color_depth = 255
            if v.attributes['role_name'] == 'hero':
                color = (0, 0, color_depth)
            else:
                color = (0, color_depth, 0)
            # Compute bounding box points
            bb = v.bounding_box.extent
            corners = [carla.Location(x=-bb.x, y=-bb.y),
                       carla.Location(x=bb.x - 0.8, y=-bb.y),
                       carla.Location(x=bb.x, y=0),
                       carla.Location(x=bb.x - 0.8, y=bb.y),
                       carla.Location(x=-bb.x, y=bb.y),
                       carla.Location(x=-bb.x, y=-bb.y)
                       ]
            v.get_transform().transform(corners)
            corners = [world_to_pixel(p) for p in corners]
            draw.polygon(corners, fill=color)

    def get_vehicle_wp(self, vehicle):
        loc = vehicle.get_location()
        waypoint = self.map.get_waypoint(loc, project_to_road=True)
        return waypoint

    def destroy_hero(self):
        if self.sensor is not None:
            # self.sensor.stop()
            self.sensor.sensor.destroy()
        if self.imu is not None:
            self.imu.sensor.destroy()
        if self.hero is not None:
            self.hero.destroy()
        self.hero = None
        self.sensor = None
        self.imu = None
        # print("destroy hero and sensor done")

    def destroy(self):
        # print('\ndestroying actors')
        self.destroy_hero()
        actor_ids = []
        for v in self.world.get_actors().filter('vehicle*'):
            actor_ids.append(v.id)
        self.client.apply_batch([carla.command.DestroyActor(x) for x in actor_ids])
        self.actor_list = []
        # self.lane = 1
        self.bp = None
        self.lp = None
        self.controller = None
        self.world.tick()
        self.collision_times = 0
        self.collision_his = 0
        self.speed_sum = 0.0
        self.speed_step = 0
        self.lane_change_times = 0
        self.count_time = 0
        self.count_frame_time = 0
        # print('done.')

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        self._global_plan = global_plan_gps
        self._global_plan_world_coord = global_plan_world_coord

    def _get_waypoints(self, global_plan_world_coord):
        waypoints = []
        for index in range(len(global_plan_world_coord)):
            waypoint = global_plan_world_coord[index][0]
            waypoints.append([waypoint.location.x, waypoint.location.y, DESIRE_SPEED])
        return waypoints

    def change_lane_policy(self):
        ego_wp = self.get_vehicle_wp(self.hero)
        ego_lane = ego_wp.lane_id
        ego_s = ego_wp.s
        # decide to make a lane change
        action = 0
        lane_index = ego_lane - 1
        if lane_index in [-2, -3, -4]:
            # Does the MOBIL model recommend a lane change?
            if self.mobil(lane_index, ego_s, ego_lane):
                action = 2
        lane_index = ego_lane + 1
        if lane_index in [-2, -3, -4]:
            # Does the MOBIL model recommend a lane change?
            if self.mobil(lane_index, ego_s, ego_lane):
                action = 1
        return action

    def mobil(self, lane_index, ego_s, ego_lane):
        d_preceding = np.Inf
        d_following = np.Inf
        new_preceding = None
        new_following = None
        old_d_preceding = np.Inf
        old_d_following = np.Inf
        old_preceding = None
        old_following = None
        for i, actor in enumerate(self.world.get_actors().filter('vehicle.audi*')):
            actor_wp = self.get_vehicle_wp(actor)
            actor_lane = actor_wp.lane_id
            actor_s = actor_wp.s
            if actor_lane == lane_index:
                if actor_s >= ego_s and actor_s - ego_s < d_preceding:
                    d_preceding = actor_s - ego_s
                    new_preceding = actor
                if actor_s < ego_s and ego_s - actor_s < d_following:
                    d_following = ego_s - actor_s
                    new_following = actor
            elif actor_lane == ego_lane:
                if actor_s >= ego_s and actor_s - ego_s < old_d_preceding:
                    old_d_preceding = actor_s - ego_s
                    old_preceding = actor
                if actor_s < ego_s and ego_s - actor_s < old_d_following:
                    old_d_following = ego_s - actor_s
                    old_following = actor
        if (d_preceding <= 20 or d_following <= 20):
            return False
        new_following_a = self.acceleration(ego_vehicle=new_following, front_vehicle=new_preceding)
        new_following_pred_a = self.acceleration(ego_vehicle=new_following, front_vehicle=self.hero)
        new_preceding_pred_a = self.acceleration(ego_vehicle=self.hero, front_vehicle=new_preceding)
        if new_following_pred_a < -A_MAX or new_preceding_pred_a < -A_MAX:
            return False

        # Is there an acceleration advantage for me and/or my followers to change lane?
        self_a = self.acceleration(ego_vehicle=self.hero, front_vehicle=old_preceding)
        old_following_a = self.acceleration(ego_vehicle=old_following, front_vehicle=self.hero)
        old_following_pred_a = self.acceleration(ego_vehicle=old_following, front_vehicle=old_preceding)
        jerk = new_preceding_pred_a - self_a + 0. * (new_following_pred_a - new_following_a
                                                             + old_following_pred_a - old_following_a)
        if jerk < 0.2:
            return False

        return True

    def acceleration(self, ego_vehicle, front_vehicle):
        if ego_vehicle is None or front_vehicle is None:
            return 0
        v_xy = ego_vehicle.get_velocity()
        current_speed = math.sqrt(v_xy.x ** 2 + v_xy.y ** 2 + v_xy.z ** 2)
        ego_target_speed = DESIRE_SPEED
        ego_desired_gap = BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * current_speed
        for dic in self.actor_dic:
            if dic['id'] == ego_vehicle.id:
                ego_target_speed = dic['target speed']
                ego_desired_gap = dic['desired gap']
        acceleration = 1.5 * (
                1 - np.power(max(current_speed, 0) / (ego_target_speed+1e-5), 4.0))

        if front_vehicle:
            front_vehicle_wp = self.get_vehicle_wp(front_vehicle)
            front_vehicle_s = front_vehicle_wp.s
            ego_vehicle_wp = self.get_vehicle_wp(ego_vehicle)
            ego_vehicle_s = ego_vehicle_wp.s
            d = front_vehicle_s - ego_vehicle_s
            acceleration -= 1.5 * \
                np.power(ego_desired_gap / (d+1e-5), 2)
        return acceleration


class PlayGame(object):
    def __init__(self):
        self.world = None
        self.env = None

    @staticmethod
    def setup_world(host='localhost', port=3000,
                    fixed_delta_seconds=0.05, town_name='highway', reload=False):
        try:
            client = carla.Client(host, port)
            # client = carla.Client('localhost', 2000)
            client.set_timeout(30.0)
            world = client.get_world()
            settings = world.get_settings()

            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = fixed_delta_seconds
            world.apply_settings(settings)
            world.wait_for_tick()

            town_map = world.get_map()
            if town_map.name != town_name or reload:
                world = client.load_world(town_name)
                # town_map = world.get_map()

            world.wait_for_tick()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = fixed_delta_seconds
            world.apply_settings(settings)
            # Wait for the world to be ready
            if world.get_settings().synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()

            world.set_weather(carla.WeatherParameters.ClearNoon)  # ClearNoon, ClearSunset, etc.
            print("World setup")

            return world, client

        except:
            traceback.print_exc()
            print("Setup world failed")
            return None, None

    @staticmethod
    def replay(directory, scen=0, port=3000):
        host='localhost'
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        world.wait_for_tick()
        world = client.load_world('highway')
        world.wait_for_tick()
        world.set_weather(carla.WeatherParameters.ClearNoon)  # ClearNoon, ClearSunset, etc.
        world.wait_for_tick()

        file = directory+"/carla_log/{}.log".format(scen)
        info = client.show_recorder_file_info(file, False)
        index_end = info.index(': vehicle.lincoln.mkz2017')
        index_begin = index_end - 1
        while info[index_begin] in [str(i) for i in range(10)]:
            index_begin -= 1
        ego = int(info[index_begin:index_end])
        client.replay_file(file, 0, 0, ego)
        world.wait_for_tick()
        while world.get_actors().filter('vehicle.lincoln.mkz2017'):
            ego = world.get_actors().filter('vehicle.lincoln.mkz2017')[0]
            world.wait_for_tick()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, required=True, help='Directory of test results')
    parser.add_argument('--scen', type=int, default=0, help='Test scenario index')
    parser.add_argument('--port', type=int, default=2000, help='CARLA client port')
    args = parser.parse_args()
    try:
        play_game = PlayGame()
        play_game.replay(args.dir, args.scen, args.port)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception:
        traceback.print_exc()
    finally:
        if play_game is not None:
            if play_game.env is not None:
                print("destroy last time")
                play_game.env.destroy()
            del play_game


if __name__ == '__main__':
    main()
