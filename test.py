import math
from cv_toolhead_calibration import CVTools

class Tests:
    def __init__(self):
        self.cv_tools = CVTools()
    
    def run_tests(self):
        debug_data = self.get_debug_data()
        for (positions, t1_nozzle_pos) in debug_data:
            self.run_test_on_data(positions, t1_nozzle_pos)

    def run_test_on_data(self, positions, t1_nozzle_pos):
        center_point = self.cv_tools.get_edge_point(positions, 'center')
        left_point = self.cv_tools.get_edge_point(positions, 'left')
        right_point = self.cv_tools.get_edge_point(positions, 'right')
        top_point = self.cv_tools.get_edge_point(positions, 'top')
        bottom_point = self.cv_tools.get_edge_point(positions, 'bottom')

        avg_points = self.cv_tools.get_average_positions(positions)

        rads_debug_data = {}

        right_rads_wanted = self.cv_tools.get_angle_between_points(center_point, right_point)
        right_rads = self.cv_tools.get_angle_between_points(avg_points[center_point], avg_points[right_point]) 
        right_rads_calc = right_rads_wanted+right_rads
        rads_debug_data['right'] = (right_rads_wanted, right_rads, right_rads_calc)


        left_rads_wanted = self.cv_tools.get_angle_between_points(center_point, left_point)
        left_rads = self.cv_tools.get_angle_between_points(avg_points[center_point], avg_points[left_point])
        left_rads_calc = left_rads_wanted+left_rads
        rads_debug_data['left'] = (left_rads_wanted, left_rads, left_rads_calc)


        top_rads_wanted = self.cv_tools.get_angle_between_points(center_point, top_point)
        top_rads = self.cv_tools.get_angle_between_points(avg_points[center_point], avg_points[top_point])
        top_rads_calc = top_rads-math.remainder(top_rads_wanted, math.pi)
        rads_debug_data['top'] = (top_rads_wanted, top_rads, top_rads_calc)

        bottom_rads_wanted = self.cv_tools.get_angle_between_points(center_point, bottom_point)
        bottom_rads = self.cv_tools.get_angle_between_points(avg_points[center_point], avg_points[bottom_point])
        bottom_rads_calc = bottom_rads_wanted+math.remainder(bottom_rads, math.pi)
        rads_debug_data['bottom'] = (bottom_rads_wanted, bottom_rads, bottom_rads_calc)

        rads_calc = (right_rads_calc+left_rads_calc+top_rads_calc+bottom_rads_calc)/4
        rads_debug_data['total'] = (0, 0, rads_calc);
        
        print ("| %s | %s | %s | %s |" % ('Name', 'Perfect','Actual','Calc'))
        for k, v in rads_debug_data.items():
            perfect, actual, calc = v
            print ("| %s | %.3f | %.3f | %.3f |" % (k, perfect, actual, calc))

        print("-------------")

        t0_nozzle_pos = positions[center_point][0]

        x_offset_px = (t0_nozzle_pos[0]-t1_nozzle_pos[0])
        y_offset_px = (t0_nozzle_pos[1]-t1_nozzle_pos[1])

        # Get px/mm from averages
        px_mm = self.cv_tools.calculate_px_to_mm(avg_points, center_point)

        # Convert the px offset values to real world mm offset
        x_offset_mm = x_offset_px/px_mm
        y_offset_mm = y_offset_px/px_mm

        x_center_offset_mm = center_point[0]+x_offset_mm
        y_center_offset_mm = center_point[1]+y_offset_mm
        rotated_offsets = self.cv_tools.rotate_around_origin(center_point, [x_center_offset_mm, y_center_offset_mm], -rads_calc)

        x_offset_mm_rotated = rotated_offsets[0]-center_point[0]
        y_offset_mm_rotated = rotated_offsets[1]-center_point[1]

        print("Camera rotatation %.5f" % (rads_calc))
        print("Pre rotation mm offsets X %.3f Y %.3f" % (x_offset_mm, y_offset_mm))
        print("After rotation mm offsets X %.3f Y %.3f" % (x_offset_mm_rotated, y_offset_mm_rotated))
        print("-------------")

    def get_debug_data(self):
        # Looking for offsets like this...
        # x: 0.09
        # y: -0.8423
        # Looks like later on the offset moved to x: -0.01 too, gotta do test prints

        calibration_data = []

        ################################
        # Supposed 'good' calibration data with camera in the orientation that would work in the old script
        positions = {(149.0, 75.0): [(220.0, 214.0)], (150.0, 75.0): [(323.0, 219.0), (325.0, 217.0), (326.0, 218.0), (326.0, 220.0)], (150.0, 74.0): [(330.0, 112.0)], (151.0, 75.0): [(428.0, 223.0)], (150.0, 76.0): [(321.0, 327.0)]}
        t1_position = (304.00, 310.00)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        positions = {(149.0, 75.0): [(209.0, 212.0)], (150.0, 75.0): [(313.0, 217.0), (313.0, 215.0), (314.0, 215.0), (313.0, 217.0)], (150.0, 74.0): [(318.0, 109.0)], (151.0, 75.0): [(417.0, 221.0)], (150.0, 76.0): [(308.0, 324.0)]}
        t1_position = (308.0, 308.0)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        ##################################
        # Rotated camera 180 degrees

        positions = {(149.0, 75.0): [(440.0, 220.0)], (150.0, 75.0): [(338.0, 241.0), (338.0, 243.0), (337.0, 243.0), (337.0, 241.0)], (150.0, 74.0): [(359.0, 347.0)], (151.0, 75.0): [(236.0, 264.0)], (150.0, 76.0): [(315.0, 136.0)]}
        t1_position = (320.0, 152.0)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        positions = {(149.0, 75.0): [(443.0, 221.0)], (150.0, 75.0): [(341.0, 242.0), (340.0, 244.0), (339.0, 244.0), (339.0, 241.0)], (150.0, 74.0): [(361.0, 347.0)], (151.0, 75.0): [(238.0, 264.0)], (150.0, 76.0): [(318.0, 137.0)]}
        t1_position = (320.0, 152.0)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        #######################
        # Rotated randomly

        positions = {(149.0, 75.0): [(407.0, 167.0)], (150.0, 75.0): [(344.0, 251.0), (344.0, 253.0), (343.0, 254.0), (342.0, 252.0)], (150.0, 74.0): [(429.0, 316.0)], (151.0, 75.0): [(281.0, 336.0)], (150.0, 76.0): [(257.0, 187.0)]}
        t1_position = (267.0, 197.0)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        ###############
        # Saturday
        # Pre camera move

        positions = {(148.5, 80.5): [(417.0, 184.0)], (149.5, 80.5): [(323.0, 230.0), (324.0, 232.0), (322.0, 233.0), (323.0, 230.0)], (149.5, 79.5): [(371.0, 330.0)], (150.5, 80.5): [(226.0, 279.0)], (149.5, 81.5): [(274.0, 132.0)]}
        t1_position = (281.0, 148.0)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        ######
        # After camera move

        positions = {(150.0, 85.0): [(274.0, 144.0)], (151.0, 85.0): [(296.0, 247.0), (299.0, 246.0), (299.0, 247.0), (297.0, 247.0)], (151.0, 84.0): [(406.0, 224.0)], (152.0, 85.0): [(322.0, 352.0)], (151.0, 86.0): [(190.0, 271.0)]}
        t1_position = (204.0, 267.0)
        calibration_data.append(
            (
                positions,
                t1_position
            )
        )

        return calibration_data


tests = Tests()
tests.run_tests()
