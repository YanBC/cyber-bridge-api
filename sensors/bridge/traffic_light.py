import math
from time import time,time_ns
import carla
from sensors.base_sensor import Sensor
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection, TrafficLight

class TrafficLightAlter(Sensor):
    
    _apollo_channel = '/apollo/perception/traffic_light'
    _apollo_msgType = 'apollo.perception.TrafficLightDetection'
    _apollo_pbCls = TrafficLightDetection

    def __init__(self, ego_vehicle: carla.Vehicle, world) -> None:
        super().__init__(ego_vehicle)
        self._world = world

    def update(self):
        self._pbCls = self._apollo_pbCls()
        def _carla_traffic_light_to_pb(carla_tl: carla.TrafficLight):
            color = TrafficLight.Color.UNKNOWN
            
            # set color
            if carla_tl is not None:
                tl_state = carla_tl.get_state()
                if tl_state == carla.TrafficLightState.Red:
                    color = TrafficLight.Color.RED
                elif tl_state == carla.TrafficLightState.Yellow:
                    color = TrafficLight.Color.YELLOW
                elif tl_state == carla.TrafficLightState.Green:
                    color = TrafficLight.Color.GREEN
                elif tl_state == carla.TrafficLightState.off:
                    color = TrafficLight.Color.BLACK
                elif tl_state == carla.TrafficLightState.Unknow:
                    color = TrafficLight.Color.UNKNOWN
            return color

        def _retrieve_traffic_landmark(distance=30): # around 30m
            tmp_map = self._world.get_map()
            tmp_vehicle = self.ego_vehicle
            traffic_light_landmarks = []
            waypoint_ego_near = tmp_map.get_waypoint(tmp_vehicle.get_location(), \
                                                    project_to_road=True, \
                                                    lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
            if waypoint_ego_near is not None:
                # print(f'roadid:{waypoint_ego_near.road_id}, sectionid[{waypoint_ego_near.section_id }], laneid[{waypoint_ego_near.lane_id }]')
                landmarks = waypoint_ego_near.get_landmarks(distance)                
                for landmark in landmarks:
                #    print(f'roadid:{landmark.road_id}, name[{landmark.name}], id[{landmark.id}], type[{landmark.type}], s[{landmark.s}], t{landmark.t}')
                   if landmark.type == '1000001': 
                       if len(traffic_light_landmarks) == 0:
                            traffic_light_landmarks.append(landmark) 
                       elif len(traffic_light_landmarks) > 0 and landmark.id != traffic_light_landmarks[-1].id:
                            traffic_light_landmarks.append(landmark) 
            else:
                print('Generate ego vehicle waypoint failed!')

            return traffic_light_landmarks

        def _test_print_tlg(traffic_light):
            tlg=traffic_light.get_group_traffic_lights()
            print(f'tlg num:{len(tlg)}')
            for tl in tlg:
                print(tl.get_state())

        tl_lms = _retrieve_traffic_landmark()            
        for tl_lm in tl_lms:            
            traffic_light = self._world.get_traffic_light(tl_lm)
            # _test_print_tlg(traffic_light)
            rtl = TrafficLight()
            rtl.color = _carla_traffic_light_to_pb(traffic_light)
            rtl.id = tl_lm.name
            rtl.confidence = 1.0
            rtl.blink = False
            self._pbCls.traffic_light.append(rtl)
            # print(traffic_light.get_elapsed_time())  # just print to refer

        if tl_lms is not None and len(tl_lms) > 0:
            self._pbCls.contain_lights = True
        else:
            self._pbCls.contain_lights = False           

        # if self.ego_vehicle.is_at_traffic_light():
        #     tl_id = _retrieve_tl_opendriver_id()
        #     if tl_id is not None:
        #         traffic_light = self.ego_vehicle.get_traffic_light()
        #         rtl = TrafficLight()
        #         rtl.color = _carla_traffic_light_to_pb(traffic_light)
        #         rtl.id = tl_id
        #         rtl.confidence = 1.0
        #         rtl.blink = False

        #         self._pbCls.contain_lights = True                
        #         self._pbCls.traffic_light.append(rtl)

            # print(f"pole index={rtl.id}")        

        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._pbCls.header.camera_timestamp = time_ns()
        self._updated = True
