from donkeycar.vehicle import Vehicle
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.actuator import PWMSteering, PWMThrottle, PCA9685
from lane_part import LaneTFLite
from det_part import DetTFLite
from planner_part import LaneChangePlanner

V = Vehicle()
cam = PiCamera(image_w=160, image_h=120, image_d=3); V.add(cam, outputs=['cam/image_array'], threaded=True)
lane = LaneTFLite('models/lanes.tflite'); V.add(lane, inputs=['cam/image_array'], outputs=['lane/offset','lane/heading'])
det  = DetTFLite('models/detector.tflite'); V.add(det, inputs=['cam/image_array'], outputs=['det/objs'])
plan = LaneChangePlanner(); V.add(plan, inputs=['lane/offset','lane/heading','det/objs'], outputs=['angle','throttle'])