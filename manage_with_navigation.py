# """
# DonkeyCar vehicle setup with integrated navigation system
# Combines lane following, object detection, and pathfinding navigation
# """

# from donkeycar.vehicle import Vehicle
# from donkeycar.parts.camera import PiCamera
# from donkeycar.parts.actuator import PWMSteering, PWMThrottle, PCA9685
# from lane_part import LaneTFLite
# from det_part import DetTFLite
# from planner_part import LaneChangePlanner
# from integrated_navigation import IntegratedNavigationController

# # Initialize vehicle
# V = Vehicle()

# # Camera
# cam = PiCamera(image_w=160, image_h=120, image_d=3)
# V.add(cam, outputs=['cam/image_array'], threaded=True)

# # Lane detection
# lane = LaneTFLite('models/lanes.tflite')
# V.add(lane, inputs=['cam/image_array'], outputs=['lane/offset', 'lane/heading'])

# # Object detection
# det = DetTFLite('models/detector.tflite')
# V.add(det, inputs=['cam/image_array'], outputs=['det/objs'])

# # Integrated navigation controller (combines pathfinding + lane planning)
# nav_controller = IntegratedNavigationController(
#     graph_path="graph.json",
#     max_throttle=0.25,
#     safety_gap_m=1.2
# )
# V.add(
#     nav_controller,
#     inputs=['lane/offset', 'lane/heading', 'det/objs'],
#     outputs=['angle', 'throttle', 'nav/info']
# )

# # Actuators
# steering = PWMSteering(channel=0)
# throttle = PWMThrottle(channel=1)
# V.add(steering, inputs=['angle'])
# V.add(throttle, inputs=['throttle'])

# # To start navigation, call this before V.start():
# # nav_controller.start_navigation("A", "D")  # Example: navigate from A to D

# # Start vehicle
# # V.start()


