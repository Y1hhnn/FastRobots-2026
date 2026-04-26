+++
title = "Lab 11: Localization on the Real Robot"
date = "2026-04-29"
+++

# Simulation

{{ image(path="content/posts/lab11/sim.png", alt="sim", width=1200, class="center" )}}

# Algorithm

```python
from notebooks import ble

class RealRobot():
    """A class to interact with the real robot
    """
    def __init__(self, commander, ble):
        # Load world config
        self.world_config = os.path.join(str(pathlib.Path(os.getcwd()).parent), "config", "world.yaml")
        
        self.config_params = load_config_params(self.world_config)
        
        # Commander to commuincate with the Plotter process
        # Used by the Localization module to plot odom and belief
        self.cmdr = commander

        # ArtemisBLEController to communicate with the Robot
        self.ble = ble
        self.times = []
        self.yaws = []
        self.tof_1_readings = []
        self.tof_2_readings = []

    def log_handler(self, sender, data: bytearray):
        parts = self.ble.bytearray_to_string(data).split("|")
        if len(parts) == 4:
            timestamp = float(parts[0][2:])
            yaw = float(parts[1][3:])
            tof_1 = float(parts[2][3:])
            tof_2 = float(parts[3][3:])
            self.times.append(float(timestamp))
            self.yaws.append(float(yaw))
            self.tof_1_readings.append(float(tof_1))
            self.tof_2_readings.append(float(tof_2))
            print (f"Received log: timestamp={timestamp}, yaw={yaw}, tof_1={tof_1}, tof_2={tof_2}")

    async def perform_observation_loop(self, rot_vel=120):
        """Perform the observation loop behavior on the real robot, where the robot does  
        a 360 degree turn in place while collecting equidistant (in the angular space) sensor
        readings, with the first sensor reading taken at the robot's current heading. 
        The number of sensor readings depends on "observations_count"(=18) defined in world.yaml.
        
        Keyword arguments:
            rot_vel -- (Optional) Angular Velocity for loop (degrees/second)
                        Do not remove this parameter from the function definition, even if you don't use it.
        Returns:
            sensor_ranges   -- A column numpy array of the range values (meters)
            sensor_bearings -- A column numpy array of the bearings at which the sensor readings were taken (degrees)
                               The bearing values are not used in the Localization module, so you may return a empty numpy array
        """
        _ = rot_vel  
        # 1. Clear previous readings
        self.times = []
        self.yaws = []
        self.tof_1_readings = []
        self.tof_2_readings = []

        # 2. Send commands to configure the 360 degree rotation, set mode, and start
        observations_count = self.config_params["mapper"]["observations_count"]
        # self.ble.send_command(CMD.SET_MAP_COUNT, str(observations_count))
        self.ble.send_command(CMD.SET_MAP_DEGREES, "360")
        
        self.ble.send_command(CMD.SET_MODE, "5")
        self.ble.send_command(CMD.UPDATE_ORIENT_PID, "2.5|1.2|0.25")
        self.ble.send_command(CMD.SET_SAMPLE_RATE, "50")
        self.ble.send_command(CMD.SET_DURATION, "25000")
        self.ble.send_command(CMD.START_RECORD, "")

        await asyncio.sleep(25)

        # 3. Collect sensor readings until the loop is done
        self.ble.start_notify(self.ble.uuid['RX_STRING'], self.log_handler)
        await asyncio.sleep(3)
        self.ble.send_command(CMD.SEND_LOG, "")
        await asyncio.sleep(50)
        self.ble.stop_notify(self.ble.uuid['RX_STRING'])

        # 4. Format the output
        tof_2 = np.array(self.tof_2_readings)
        yaw_unwrapped = np.degrees(np.unwrap(np.radians(self.yaws)))
        yaw_continuous = yaw_unwrapped - yaw_unwrapped[0]

        valid_indices_2 = (tof_2 > 0) & (tof_2 < 6000)
        valid_angles_2 = np.radians(yaw_continuous[valid_indices_2])
        valid_distances_2 = tof_2[valid_indices_2] + 69.85


        if len(valid_distances_2) < observations_count:
            raise RuntimeError(
                f"Insufficient observation samples: tof={len(valid_distances_2)}, expected={observations_count}"
            )

        ranges_in_meters = [reading / 1000.0 for reading in valid_distances_2[:observations_count]]
        sensor_ranges = np.array(ranges_in_meters)[np.newaxis].T
        sensor_bearings = np.array(valid_angles_2[:observations_count])[np.newaxis].T

        print(f"Sensor Ranges (m): {sensor_ranges.flatten()}")
        print(f"Sensor Bearings (rad): {sensor_bearings.flatten()}")

        return sensor_ranges, sensor_bearings

```

# Testing

## Pose at (-3,-2)

{{ image(path="content/posts/lab11/-3_-2.png", alt="map_-3_-2", width=1200, class="center" )}}

## Pose at (0,0)

{{ image(path="content/posts/lab11/0_0.png", alt="map_0_0", width=1200, class="center" )}}

## Pose at (0,3)

{{ image(path="content/posts/lab11/0_3.png", alt="map_0_3", width=1200, class="center" )}}


## Pose at (5,3)

{{ image(path="content/posts/lab11/5_3.png", alt="map_5_3", width=1200, class="center" )}}

## Pose at (5,-3)

{{ image(path="content/posts/lab11/5_3.png", alt="map_5_-3", width=1200, class="center" )}}

# Analysis

{{ image(path="content/posts/lab11/final_map.png", alt="final_map", width=1200, class="center" )}}
