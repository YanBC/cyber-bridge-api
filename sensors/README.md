## Available Sensors
1. `ChassisAlter`
2. `BestPose`
3. `Odometry`
4. `InsStatus`
5. `CorrectedImu`
6. `Obstacles`
7. `TrafficLightAlter`
8. `ClockSensor`

Common attributes for all sensors:
- "name": string, id of the sensor, used for identification, should be different for different sensor instances
- "type": string, type of the sensor class
- "frequency": number, set it to -1.0 if the sensor should be run as fast as it can

### ChassisAlter
This sensor sends data about the vehicle chassis.

### BestPose
This sensor outputs the GPS location of the vehicle in Longitude/Latitude and Northing/Easting coordintates.

### Odometry
This sensor outputs the GPS location of the vehicle in Longitude/Latitude and Northing/Easting coordintates and the vehicle velocity.

Since some maps do not centred at (0,0), there are two additional attributes for this sensor:
- "x_offset": number, offset along the x axis
- "y_offset": number, offset along the y axis


### InsStatus
This sensor outputs the status of the GPS correction due to INS. The simulator is an ideal environment in which GPS is always corrected.

### CorrectedImu
This sensor output the data about the vehicle imu status.


### Obstacles
This sensor returns 3D ground truth data for training and creates bounding boxes around the detected objects.

Additional attributes:
- "x_offset": number, offset along the x axis
- "y_offset": number, offset along the y axis
- "vehicle_distance": number, vehicles outside of this radius will not be detected
- "walker_distance": number, pedestrians outside of this radius will not be detected


### TrafficLightAlter
This sensor returns ground truth data for traffic light signals connected to the current lane of ego vehicle.

### Clock
This sensor outputs simulated time to CyberRT as clock messages.

