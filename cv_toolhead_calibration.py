import time
import cv2
import numpy as np
import math
import requests
from requests.exceptions import InvalidURL, HTTPError, RequestException, ConnectionError

class CVToolheadCalibration:
    def __init__(self, config):
        self.camera_address = config.get('nozzle_cam_url')
        self.camera_position = config.getlist('camera_position', ('x','y'), count=2)
        self.camera_position = (float(self.camera_position[0]), float(self.camera_position[1]))
        self.speed = config.getfloat('speed', 50., above=10.)

        self.calib_iterations = config.getint('calib_iterations', 1, minval=1, maxval=25)
        self.calib_value = config.getfloat('calib_value', 1.0, above=0.25)

        self.printer = config.get_printer()
        self.config = config

        if not config.has_section('dual_carriage'):
            raise self.printer.config_error("No dual carriage section found in config, CVNozzleCalib wont work")

        self.streamer = MjpegStreamReader(self.camera_address)
        # TODO: Move to printer connect event
        # if not self.streamer.can_read_stream(self.printer):
        #     raise self.printer.config_error("Could not read configured stream url %s" % (self.camera_address))

        self.cv_tools = CVTools()

        # TODO: Parameterize detector
        self.detector = self.cv_tools._create_detector()

        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command('CV_TEST', self.cmd_SIMPLE_TEST, desc=self.cmd_SIMPLE_TEST_help)
        self.gcode.register_command('CV_CENTER_TOOLHEAD', self.cmd_center_toolhead, desc=self.cmd_center_toolhead_help)
        self.gcode.register_command('CV_SIMPLE_NOZZLE_POSITION', self.cmd_SIMPLE_NOZZLE_POSITION, desc=self.cmd_SIMPLE_NOZZLE_POSITION_help)
        self.gcode.register_command('CV_CALIB_NOZZLE_PX_MM', self.cmd_CALIB_NOZZLE_PX_MM, desc=self.cmd_CALIB_NOZZLE_PX_MM_help)
        self.gcode.register_command('CV_CALIB_OFFSET', self.cmd_CALIB_OFFSET, desc=self.cmd_CALIB_OFFSET_help)

    cmd_SIMPLE_TEST_help = "Tests if the CVNozzleCalib extension works"
    def cmd_SIMPLE_TEST(self, gcmd):
        gcmd.respond_info("CVNozzleCalib extension works. OpenCV version is %s and the nozzle cam url is configured to be %s" % (cv2.__version__, self.camera_address))

    cmd_center_toolhead_help = "Positions the current toolhead at the camera nozzle position"
    def cmd_center_toolhead(self, gcmd):
        self._center_toolhead()

    cmd_SIMPLE_NOZZLE_POSITION_help = "Detects if a nozzle is found in the current image"
    def cmd_SIMPLE_NOZZLE_POSITION(self, gcmd):
        self.streamer.open_stream()
        position = self._recursively_find_nozzle_position()
        if position:
            gcmd.respond_info("Found nozzle")
            # position = position[0]
            gcmd.respond_info("X%.3f Y%.3f, radius %.3f" % (position[0], position[1], position[2]))
        else:
            gcmd.respond_info("No nozzles found")
        self.streamer.close_stream()

    cmd_CALIB_OFFSET_help = "Calibraties T0 and T1 XY offsets based on the configured center point"
    def cmd_CALIB_OFFSET(self, gcmd):
        self.streamer.open_stream()
        # Ensure we are using T0
        self.t0()

        skip_center = gcmd.get('SKIP_CENTER', False)
        calib_value = gcmd.get_float('CALIB_VALUE', self.calib_value)
        calib_iterations = gcmd.get_int('CALIB_ITERATIONS', self.calib_iterations)

        # Get positions from the calibration function
        positions = self.calibrate_toolhead_movement(skip_center, calib_value, calib_iterations)

        # avg_positions would be {(klipper_x_in_mm, klipper_y_in_mm): (cv_pixel_x_in_px, cv_pixel_y_in_px), {...}, ...}
        avg_points = self.cv_tools.get_average_positions(positions)

        # Get px/mm from averages
        px_mm = self.cv_tools.calculate_px_to_mm(avg_points, self.camera_position)

        top_point = self.cv_tools.get_edge_point(positions, 'top')
        center_point = self.cv_tools.get_edge_point(positions, 'center')

        point_ideal_top = (
            int(avg_points[center_point][0]), 
            0
        )
        point_center = (
            int(avg_points[center_point][0]), 
            int(avg_points[center_point][1])
        )
        point_top = (
            int(avg_points[top_point][0]), 
            int(avg_points[top_point][1])
        )

        slope1 = self.cv_tools.slope(point_center, point_ideal_top)
        slope2 = self.cv_tools.slope(point_center, point_top)

        ang = self.cv_tools.angle(slope1, slope2)

        if point_center[1] < point_top[1]: 
            ang += 180

        rads = math.radians(ang)

        center_deviation = self.cv_tools.get_center_point_deviation(positions[center_point])
        gcmd.respond_info("""
            T0 calibration
            Center point: (%.2f,%.2f)
            Calibration accuracy: X%dpx Y%dpx
            px/mm: %.4f
            Camera rotation %.2f
        """ % (
            avg_points[center_point][0], 
            avg_points[center_point][1], 
            center_deviation[0], 
            center_deviation[1], 
            px_mm, 
            ang
        ))

        toolhead = self.printer.lookup_object('toolhead')

        # Get a new T0 center position after running the calibration function
        # TODO: If this position deviates too much from calibration function center point, abort
        t0_nozzle_pos = self._recursively_find_nozzle_position()
        if t0_nozzle_pos is None:
            gcmd.respond_info("Did not find nozzle after initial T0 move to center, aborting")
            self.streamer.close_stream()
            return

        # X home T0
        self._x_home_current_toolhead()

        # Switch to T1 and center above camera
        self.t1()
        self._center_toolhead()

        # Find initial T1 position to caluclate offset from
        t1_nozzle_pos = self._recursively_find_nozzle_position()
        if t1_nozzle_pos is None:
            gcmd.respond_info("Did not find nozzle after initial T1 move to center, aborting")
            self.streamer.close_stream()
            return

        t1_pos = (int(t1_nozzle_pos[0]), int(t1_nozzle_pos[1]))
        t1_rotated = self.cv_tools.rotate_around_origin(avg_points[center_point], t1_pos, -rads)

        # TODO: If t1_nozzle_pos nozzle radius differs a lot from T0 nozzle radius there will be issues

        # Calculate the X and Y offsets
        x_offset_px = t0_nozzle_pos[0]-t1_rotated[0]
        y_offset_px = t0_nozzle_pos[1]-t1_rotated[1]

        # Convert the px offset values to real world mm
        x_offset_mm = x_offset_px/px_mm
        y_offset_mm = y_offset_px/px_mm

        # TODO: Add early return if movement is bigger then some we can assume is outside of the cameras vision
        new_pos_t1 = toolhead.get_position()
        new_pos_t1[0] = center_point[0]+x_offset_mm
        new_pos_t1[1] = center_point[1]+y_offset_mm

        # Move T1 to "center" + offsets
        toolhead.move(new_pos_t1, self.speed)
        toolhead.wait_moves()

        second_t1_pos = self._recursively_find_nozzle_position()
        if second_t1_pos is None:
            gcmd.respond_info("Tried to use MM offsets X%.4f Y%.4f, but did not find T1 nozzle after move..." % (x_offset_mm, y_offset_mm))
            self.streamer.close_stream()
            return

        # TODO: Add early return if resulting virtual offset is not within spec
        # (t0_nozzle_pos[0]-second_t1_pos[0]),
        # (t0_nozzle_pos[1]-second_t1_pos[1])

        gcmd.respond_info("""
            Done calibrating! 
            Using MM offsets X%.4f Y%.4f got:
            Initial virtual offset: X%d Y%d
            Resulting virtual offset: X%d Y%d
        """ % (
            x_offset_mm, 
            y_offset_mm, 
            (t0_nozzle_pos[0]-t1_nozzle_pos[0]), # Initial virtual offset between t0 and t1
            (t0_nozzle_pos[1]-t1_nozzle_pos[1]), # Initial virtual offset between t0 and t1
            (t0_nozzle_pos[0]-second_t1_pos[0]), # Calibrated virtual offset between t0 and t1
            (t0_nozzle_pos[1]-second_t1_pos[1]) # Calibrated virtual offset between t0 and t1
        ))

        self.streamer.close_stream()

        self._x_home_current_toolhead()
        # Restore state to t0
        self.t0()

    cmd_CALIB_NOZZLE_PX_MM_help = "Calibrates the movement of the active nozzle around the point it started at"
    def cmd_CALIB_NOZZLE_PX_MM(self, gcmd):
        self.streamer.open_stream()
        skip_center = gcmd.get('SKIP_CENTER', False)
        calib_value = gcmd.get_float('CALIB_VALUE', self.calib_value)
        calib_iterations = gcmd.get_int('CALIB_ITERATIONS', self.calib_iterations)

        # Get positions from the calibration function
        positions = self.calibrate_toolhead_movement(skip_center, calib_value, calib_iterations)

        # avg_positions would be {(klipper_x_in_mm, klipper_y_in_mm): (cv_pixel_x_in_px, cv_pixel_y_in_px), {...}, ...}
        avg_points = self.cv_tools.get_average_positions(positions)

        # Get px/mm from averages
        px_mm = self.cv_tools.calculate_px_to_mm(avg_points, self.camera_position)

        top_point = self.cv_tools.get_edge_point(positions, 'top')
        center_point = self.cv_tools.get_edge_point(positions, 'center')

        point_ideal_top = (
            int(avg_points[center_point][0]), 
            0
        )
        point_center = (
            int(avg_points[center_point][0]), 
            int(avg_points[center_point][1])
        )
        point_top = (
            int(avg_points[top_point][0]), 
            int(avg_points[top_point][1])
        )

        slope1 = self.cv_tools.slope(point_center, point_ideal_top)
        slope2 = self.cv_tools.slope(point_center, point_top)

        ang = self.cv_tools.angle(slope1, slope2)

        if point_center[1] < point_top[1]: 
            ang += 180

        print_positions = gcmd.get('PRINT_POSITIONS', False)
        if print_positions != False:
            debug_string = self.cv_tools.positions_dict_to_string(positions)
            gcmd.respond_info(debug_string)

        center_deviation = self.cv_tools.get_center_point_deviation(positions[self.camera_position])
            
        gcmd.respond_info("""
            Calibration results:
            Center point: (%.2f,%.2f)
            Deviation: X%dpx Y%dpx
            px/mm: %.4f
            Camera rotation %.4f
        """ % (avg_points[self.camera_position][0], avg_points[self.camera_position][1], center_deviation[0], center_deviation[1], px_mm, ang))

        self.streamer.close_stream()

    def calibrate_toolhead_movement(self, skip_move_toolhead_to_center=False, calib_value=1.0, iterations=1):
        if not skip_move_toolhead_to_center:
            self._center_toolhead()

        toolhead = self.printer.lookup_object('toolhead')
        starting_pos = toolhead.get_position()
        start_x = starting_pos[0]
        start_y = starting_pos[1]

        # These points most likely don't need to be this extensive
        calib_points = [ 
            (start_x-calib_value, start_y), # Most left
            # (start_x-(calib_value/2), start_y-(calib_value/2)), # Top left corner
            (start_x, start_y-calib_value), # Most top
            # (start_x+(calib_value/2), start_y-(calib_value/2)), # Top right corner
            (start_x+calib_value, start_y), # Most right
            # (start_x+(calib_value/2), start_y+(calib_value/2)), # Bottom right corner
            (start_x, start_y+calib_value), # Most bottom
            # (start_x-(calib_value/2), start_y+(calib_value/2)) # Bottom left corner
        ]

        # Would have data like {(klipper_x_in_mm, klipper_y_in_mm): [(cv_pixel_x_in_px, cv_pixel_y_in_px), ...], ...}
        positions = {}

        # Itterations is most likely not needed for the same reason as mentioned for calib_points
        for _ in range(iterations):
            for calib_point in calib_points:
                # Go to calib point and get a position
                new_pos = toolhead.get_position()
                new_pos[0] = calib_point[0]
                new_pos[1] = calib_point[1]
                toolhead.move(new_pos, self.speed)
                toolhead.wait_moves()

                nozzle_pos = self._recursively_find_nozzle_position()
                if nozzle_pos:
                    positions.setdefault(calib_point,[]).append(nozzle_pos)
                else:
                    raise self.gcode.error("No nozzle detected for this position (%d, %d)" % calib_point)

                # Go back to center and get a position
                new_pos = toolhead.get_position()
                new_pos[0] = start_x
                new_pos[1] = start_y
                toolhead.move(new_pos, self.speed)
                toolhead.wait_moves()

                nozzle_pos = self._recursively_find_nozzle_position()
                if nozzle_pos:
                    positions.setdefault((start_x, start_y),[]).append(nozzle_pos)

        return positions

    def t0(self):
        dc = self.printer.lookup_object('dual_carriage')
        status = dc.get_status()
        if status['active_carriage'] != 'CARRIAGE_0':
            self.gcode.run_script_from_command('ACTIVATE_EXTRUDER EXTRUDER=extruder')
            self.gcode.run_script_from_command('SET_DUAL_CARRIAGE CARRIAGE=0')

    def t1(self):
        dc = self.printer.lookup_object('dual_carriage')
        status = dc.get_status()
        if status['active_carriage'] != 'CARRIAGE_1':
            self.gcode.run_script_from_command('ACTIVATE_EXTRUDER EXTRUDER=extruder1')
            self.gcode.run_script_from_command('SET_DUAL_CARRIAGE CARRIAGE=1')

    def _x_home_current_toolhead(self):
        toolhead = self.printer.lookup_object('toolhead')
        dc = self.printer.lookup_object('dual_carriage')
        status = dc.get_status()
        pos = toolhead.get_position()

        if status['active_carriage'] == 'CARRIAGE_0':
            stepper_x = self.config.getsection('stepper_x')
            x_endstop = stepper_x.getfloat('position_endstop')
            pos[0] = x_endstop
        elif status['active_carriage'] == 'CARRIAGE_1':
            dual_carriage = self.config.getsection('dual_carriage')
            x_endstop = dual_carriage.getfloat('position_endstop')
            pos[0] = x_endstop

        toolhead.move(pos, self.speed)
        toolhead.wait_moves()

    def _center_toolhead(self):
        toolhead = self.printer.lookup_object('toolhead')
        starting_pos = toolhead.get_position()
        if starting_pos[0] != self.camera_position[0] or starting_pos[1] != self.camera_position[1]:
            center_pos = starting_pos
            center_pos[0] = self.camera_position[0]
            center_pos[1] = self.camera_position[1]
            toolhead.move(center_pos, self.speed)
            toolhead.wait_moves()

    def _find_nozzle_positions(self):
        image = self.streamer.get_single_frame()
        if image is None:
            return None
        return self.cv_tools.detect_nozzles(image)

    def _recursively_find_nozzle_position(self):
        start_time = time.time()  # Get the current time

        CV_TIME_OUT = 5 # If no nozzle found in this time, timeout the function
        CV_MIN_MATCHES = 5 # Minimum amount of matches to confirm toolhead position after a move

        last_pos = (0,0)
        pos_matches = 0
        while time.time() - start_time < CV_TIME_OUT:
            positions = self._find_nozzle_positions()
            if not positions:
                continue

            pos = positions[0]
            # Only compare XY position, not radius...
            if pos and last_pos and pos[0] == last_pos[0] and pos[1] and last_pos[1]:
            # if pos and last_pos and pos == last_pos:
                pos_matches += 1
                if pos_matches >= CV_MIN_MATCHES:
                    return pos
            else:
                pos_matches = 0

            last_pos = pos
        return None

class CVTools:
    def __init__(self):
        self.detector = self._create_detector()

    def get_average_positions(self, positions):
        avg_positions = {}
        for position in positions:
            mm_positions = positions[position]
            transposed = zip(*mm_positions)
            averages = [np.mean(col) for col in transposed]
            avg_positions[position] = averages
        return avg_positions

    def calculate_px_to_mm(self, positions, center_point):
        mm_center_point = (center_point[0], center_point[1])
        px_center_point = (positions[mm_center_point][0], positions[mm_center_point][1])

        px_mm_calibs = []
        for key in positions:
            if key == mm_center_point:
                continue
            position = positions[key]

            px_distance = self.get_distance((position[0], position[1]), px_center_point)
            mm_distance = self.get_distance((key[0], key[1]), mm_center_point)

            px_mm_calibs.append((px_distance / mm_distance))

        avg = (sum(px_mm_calibs)/len(px_mm_calibs))
        return avg

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def get_center_point_deviation(self, positions):
        center_min = np.min(positions, axis=0)
        center_max = np.max(positions, axis=0)
        return (center_max[0]-center_min[0], center_max[1]-center_min[1])

    def positions_dict_to_string(self, dictionary):
        string = ""
        for key, value in dictionary.items():
            string += f"X{key[0]} Y{key[1]}:\n"
            for val in value:
                string += f"  X{val[0]} Y{val[1]} r{val[2]}\n"
        return string

    def get_edge_point(self, positions, edge):
        points_np = np.array(list(positions.keys()))

        x_median = np.median(points_np[:,0])
        y_median = np.median(points_np[:,1])
        if edge == 'left':
            min_x = np.min(points_np[:,0])
            return (min_x, y_median)
        if edge == 'right':
            max_x = np.max(points_np[:,0])
            return (max_x, y_median)
        if edge == 'top':
            min_y = np.min(points_np[:,1])
            return (x_median, min_y)
        if edge == 'bottom':
            max_y = np.max(points_np[:,1])
            return (x_median, max_y)
        if edge == 'center':
            return (x_median, y_median)
        return None

    def rotate_around_origin(self, origin, point, angle):
        ox, oy = (int(origin[0]), int(origin[1]))
        px, py = (int(point[0]), int(point[1]))

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy
    
    # Taken straight from TAMV: https://github.com/DanalEstes/TAMV/blob/master/TAMV.py#L149
    def _create_detector(self, t1=5, t2=250, all=0.5, area=100):
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = t1
        params.maxThreshold = t2
        params.filterByArea = True
        params.minArea = area
        params.filterByCircularity = True
        params.minCircularity = all
        params.filterByConvexity = True
        params.minConvexity = all
        params.filterByInertia = True
        params.minInertiaRatio = all
        detector = cv2.SimpleBlobDetector_create(params)
        return detector

    def detect_nozzles(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        keypoints = self.detector.detect(gray)
        if len(keypoints) < 1:
            return None

        data = []
        for point in keypoints:
            pos = np.around(point.pt)
            r = np.around(point.size/2) # Radius of the detected nozzle
            data.append((pos[0], pos[1], r))

        return data

    def slope(self, p1, p2):
        a1 = p2[1]-p1[1]
        a2 = p2[0]-p1[0]
        if a1 == 0:
            return a2
        if a2 == 0:
            return a1
        return a1/a2

    def angle(self,  s1, s2):
        return math.degrees(math.atan((s2-s1)/(1+(s2*s1))))

class MjpegStreamReader:
    def __init__(self, camera_address):
        self.camera_address = camera_address
        self.session = requests.Session()

    def can_read_stream(self, printer):
        # TODO: Clean this up and return actual errors instead of this...stuff...
        try:
            with self.session.get(self.camera_address) as _:
                return True
        except InvalidURL as _:
            raise printer.config_error("Could not read nozzle camera address, got InvalidURL error %s" % (self.camera_address))
        except ConnectionError as _:
            raise printer.config_error("Failed to establish connection with nozzle camera %s" % (self.camera_address))
        except Exception as e:
            raise printer.config_error("Nozzle camera request failed %s" % str(e))

    def open_stream(self):
        # TODO: Raise error, stream already running 
        self.session = requests.Session()

    def get_single_frame(self):
        if self.session is None: 
            # TODO: Raise error: stream is not running
            return None

        with self.session.get(self.camera_address, stream=True) as stream:
            if stream.ok:
                chunk_size = 1024
                bytes_ = b''
                for chunk in stream.iter_content(chunk_size=chunk_size):
                    bytes_ += chunk
                    a = bytes_.find(b'\xff\xd8')
                    b = bytes_.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_[a:b+2]
                        return cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        return None

    def close_stream(self):
        if self.session is not None:
            self.session.close()
            self.session = None

def load_config(config):
    return CVToolheadCalibration(config)
