#!/usr/bin/env python

from __future__ import print_function
import time
from svgpathtools import svg2paths, wsvg, Path, Line
from struct import pack, unpack
from enum import Enum
from pathlib import Path as FilePath
from random import choice
import math
import csv

#Real RF24/GPIO

from RF24 import *
import RPi.GPIO as GPIO
'''

# Fake RF24/GPIO
RF24_PA_LOW = None

class RF24():
    def __init__(self,x,y):
        pass
    def begin(self):
        pass
    def setPALevel(self, x):
        pass
    def setAutoAck(self, x):
        pass
    def enableAckPayload(self):
        pass
    def setRetries(self, x, y):
        pass
    def printDetails(self):
        pass
    def openWritingPipe(self, address):
        pass
    def openReadingPipe(self, pipe, address):
        pass
    def startListening(self):
        pass
    def stopListening(self):
        pass
    def write(self, data, length=-1):
        print("{},{}".format(data, length))
        return True
    def read(self, maxLen):
        return pack('<LL', 42, 42)
    def available(self):
        return choice([True, True, False])
    payloadSize = 0

class GPIO():
    @staticmethod
    def output(x,y):
        pass
    @staticmethod
    def setmode(x):
        pass
    @staticmethod
    def setup(x,y):
        pass
    BCM = None
    OUT = None
    HIGH = None
    LOW = None
'''

ce_gpio_pin = 22
irq_gpio_pin = None
step_gpio_pin = 23
dir_gpio_pin = 24

class Addresses(Enum):
    main_unit = 0xF0F0F0F0E1
    left_unit = 0xF0F0F0F0D2
    right_unit = 0xF0F0F0F0C3
    pen_unit = 0xF0F0F0F0B4

def saveInstructions(csv_file, instructions):
    with open(csv_file, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for instr in instructions:
            writer.writerow(["{},{}".format(instr[0].real, instr[0].imag), instr[1].name, "{},{},{}".format(*(instr[2]))])

class CsvPlot:
    def __init__(self, csv_path, start_index=(0,0,0)):
        self.csv_path = csv_path
        self.start_index = start_index
        self.instructions = None

    def bbox(self):
        if not self.instructions:
            self.genInstructions()
        bbox = (float("inf"), 0, float("inf"), 0)
        for instr in self.instructions:
            point = instr[0]
            bbox = (min(bbox[0], point.real), max(bbox[1], point.real), min(bbox[2], point.imag), max(bbox[3], point.imag))
        return bbox

    def genInstructions(self):
        instructions = []
        dictReader = \
            csv.DictReader(open(self.csv_path, 'r'), fieldnames=['point', 'pen', 'index'], delimiter=',', quotechar='"')
        for row in dictReader:
            point = row['point'].split(',')
            point = complex(float(point[0]), float(point[1]))
            pen = PenPosition[row['pen']]
            index = row['index'].split(',')
            index = (int(index[0]), int(index[1]), float(index[2]))
            instructions.append((point, pen, index))
            #print((point, pen, index))
        print("{} instructions loaded".format(len(instructions)))
        self.instructions = instructions
        return instructions

class SvgPlot:
    def __init__(self, svg_path, start_index=(0,0,0)):
        self.svg_path = svg_path
        self.paths, self.attributes = svg2paths(svg_path)
        self.start_index = start_index

    def bbox(self):
        bbox = (float("inf"), 0, float("inf"), 0)
        for path in self.paths:
            bb = path.bbox()
            bbox = (min(bbox[0], bb[0]), max(bbox[1], bb[1]), min(bbox[2], bb[2]), max(bbox[3], bb[3]))
        return bbox

    def segmentFinished(self, index):
        path_i, seg_i, dist = index
        if path_i > len(self.paths) - 1:
            return True
        path = self.paths[path_i]
        if seg_i > len(path) - 1:
            return True
        segment = path[seg_i]
        return dist >= segment.length()

    def pathFinished(self, index):
        path_i, seg_i, dist = index
        if path_i > len(self.paths) - 1:
            return True
        path = self.paths[path_i]
        if seg_i > len(path) - 1:
            return True
        segment = path[seg_i]
        return seg_i == len(path) - 1 and dist >= segment.length()

    def pathsFinished(self, index):
        path_i, seg_i, dist = index
        return path_i >= len(self.paths) - 1 and self.pathFinished(index)

    def nextIndex(self, index, resolution):
        path_i, seg_i, dist = index
        rewind = False
        if self.segmentFinished((path_i, seg_i, dist)):
            seg_i += 1
            dist = 0
            rewind = True
        if self.pathFinished((path_i, seg_i, dist)):
            path_i += 1
            seg_i = 0
            rewind = True
        if not rewind and not self.pathsFinished((path_i, seg_i, dist)):
            dist += resolution
        return (path_i, seg_i, dist)

    def getPathsStart(self):
        return (0, 0, 0)

    def getPathsEnd(self):
        return (float("inf"), float("inf"), float("inf"))

    def getPathStart(self, index):
        path_i, seg_i, dist = index
        return (path_i, 0, 0)

    def getPathEnd(self, index):
        path_i, seg_i, dist = index
        return (path_i, float("inf"), float("inf"))

    def getSegStart(self, index):
        path_i, seg_i, dist = index
        return (path_i, seg_i, 0)

    def getSegEnd(self, index):
        path_i, seg_i, dist = index
        return (path_i, seg_i, float("inf"))

    def getPointForIndex(self, index):
        path_i, seg_i, dist = index
        seg_i = min(seg_i, len(self.paths[path_i]))
        segment = self.paths[path_i][seg_i]
        seg_length = segment.length()
        parametric_point = segment.ilength(dist) if dist < seg_length else 1.0
        return segment.point(parametric_point)

    def currentPath(self, index):
        return self.paths[min(index[0], len(self.paths))]

    def currentSegment(self, index):
        return self.currentPath(index)[min(index[1],len(self.currentPath(index)))]

    def genInstructions(self, resolution):
        instructions = []
        index = self.start_index
        instructions.append((self.getPointForIndex(index), PenPosition.up, index))
        while True:
            print("Progess: {}".format(index), end='\r')
            if self.pathFinished(index):
                if self.currentPath(index).iscontinuous() and self.currentPath(index).isclosed():
                    point = self.getPointForIndex(self.getPathStart(index))
                    instructions.append((point, PenPosition.down, index))
                    print('')
                if self.pathsFinished(index):
                    break
                index = self.nextIndex(index, resolution)
                point = self.getPointForIndex(self.getPathStart(index))
                instructions.append((point, PenPosition.up, index))
            else:
                index = self.nextIndex(index, resolution)
                point = self.getPointForIndex(index)
                instructions.append((point, PenPosition.down, index))
        print("Progess: Done - #{} instructions".format(len(instructions)))
        return instructions

def readInt(text, default_value):
  try:
    return int(input(text))
  except ValueError:
    return default_value

def readFloat(text, default_value):
  try:
    return float(input(text))
  except ValueError:
    return default_value

MICROSTEPS = 16
# Steps Per Revolution
SPR = 200 * MICROSTEPS
# Step length, the distance a step increases the string in mm
step_length = 40.0 / SPR
# microseconds delay between each step
STEP_DELAY_US = 100
# seconds delay between each step
step_delay = 100/1000/1000

radio = RF24(ce_gpio_pin, 0)

class State(Enum):
  READY = 0
  DRAW = 1
  MOVER = 2
  MOVEL = 3
  MOVEU = 4
  MOVED = 5
  MOTOR_RF = 6
  MOTOR_RB = 7
  MOTOR_LF = 8
  MOTOR_LB = 9

class Direction(Enum):
  backward = 0
  forward = 1

class PenPosition(Enum):
  up = 0
  down = 1

class Command(Enum):
  backward_direct = 0
  forward_direct = 1
  backward_parallel = 2
  forward_parallel = 3
  start_parallel = 4
  debug_device = 5

current_state = State.READY
motor_distance = 1350.0 # the distance between the two motors in mm
current_a = 0 # the number of steps the pen is away from the left motor
current_b = 0 # the number of steps the pen is away from the right motor
origin = complex(0,0)
img_offset = complex(0,0)
img_scale = 1.0
pen_updown = True
plot_index = (0,0,0)
paths, attributes = None, None
instructions = None
instr_index = 0
path = ""
slowdown = 16

def pack_msg(value1, code):
    return pack('<LL', value1, code)

def unpack_msg(byte_array):
    return unpack('<LL', byte_array)

def setup():    
    radio.begin()
    radio.setPALevel(RF24_PA_LOW)
    radio.setAutoAck(True)            # Ensure autoACK is enabled
    radio.enableAckPayload()       # Allow optional ack payloads
    radio.setRetries(0,15)         # Smallest time between retries, max no. of retries
    radio.payloadSize = 16
    radio.printDetails()           # Dump the configuration of the rf unit for debugging

    # initialize a/current_b by calibration
    calibrate()

def calibrate():
  global current_a, current_b, origin
  current_a = round(math.sqrt(2) * (motor_distance/2) / step_length)
  current_b = round(math.sqrt(2) * (motor_distance/2) / step_length)
  origin = complex(motor_distance / 2.0, motor_distance / 2.0 )
  print("origin saved {}".format(origin))
  print("ab: {},{}".format(current_a,current_b))

def sendMessage(address, data):
  radio.openWritingPipe(address)
  radio.stopListening()
  return radio.write(data)

def controlBothMotor(dir_left, steps_left, dir_right, steps_right, total_steps=0, sleep=True):
  total_steps = max(steps_left, steps_right, total_steps)
  print("Set Left:{} ,{}".format(dir_left.name, steps_left))
  if not sendMessage(Addresses.left_unit.value, pack_msg(
    steps_left,
    Command.backward_parallel.value if dir_left == Direction.backward else Command.forward_parallel.value)
  ):
    print("Failed")
    return False
  print("Set Right:{} ,{}".format(dir_right.name, steps_right))
  if not sendMessage(Addresses.right_unit.value, pack_msg(
    steps_right, 
    Command.backward_parallel.value if dir_right == Direction.backward else Command.forward_parallel.value)
  ):
    print("Failed")
    return False
  print("Send Start Left")
  if not sendMessage(Addresses.left_unit.value, pack_msg(total_steps, Command.start_parallel.value)):
    print("Failed")
    return False
  print("Send Start Right")
  if not sendMessage(Addresses.right_unit.value, pack_msg(total_steps, Command.start_parallel.value)):
    print("Failed silent")
  if sleep:
    time.sleep(total_steps * step_delay)
  return True

def controlLeftMotor(dir, num_steps, sleep=True):
  print("Set Left:{} ,{}".format(dir.name, num_steps))
  if sendMessage(Addresses.left_unit.value, pack_msg(num_steps, dir.value)):
    if sleep:
      time.sleep(num_steps * step_delay)
    return True
  else: 
    return False

def controlRightMotor(dir, num_steps, sleep=True):
  print("Set Right: {}, {}".format(dir.name, num_steps))
  if sendMessage(Addresses.right_unit.value, pack_msg(num_steps, dir.value)):
      if sleep:
          time.sleep(num_steps * step_delay)
      return True
  else: 
      return False

def controlPen(pen_pos):
  print("Set Pen: {}".format(pen_pos.name))
  return sendMessage(Addresses.pen_unit.value, pack_msg(0, pen_pos.value))

def abToPoint(steps_a, steps_b, motor_distance):
  length_a = steps_a * step_length
  length_b = steps_b * step_length
  s = 0.5 * (length_a + length_b + motor_distance)
  y = (2.0 / motor_distance) * math.sqrt(s*(s-length_a)*(s-length_b)*(s-motor_distance))
  x = math.sqrt(length_a**2 - y**2)
  return (x,y)

def moveToXY(x, y, quickmode=False):
  target_a = round(math.sqrt(x**2+y**2) / step_length)
  target_b = round(math.sqrt((motor_distance-x)**2+y**2) / step_length)
  return moveToAB(target_a, target_b, quickmode)

def moveToAB(target_a, target_b, quickmode=False):
    global current_a, current_b
    dir_a = Direction.backward if target_a < current_a else Direction.forward
    dir_b = Direction.backward if target_b < current_b else Direction.forward
    step_count_a = target_a - current_a
    step_count_b = target_b - current_b

    # update the current position of the pen
    current_a += step_count_a
    current_b += step_count_b

    # drop direction information
    step_count_a = abs(step_count_a)
    step_count_b = abs(step_count_b)

    if quickmode:
      return controlBothMotor(dir_a,step_count_a,dir_b,step_count_b)
    else:
      return controlBothMotor(dir_a,step_count_a,dir_b,step_count_b,max(step_count_a,step_count_b)*slowdown)

def transformPoint(point):
    global origin
    return (point * img_scale) + origin + img_offset

def debugAddress(addr):
  if not sendMessage(addr.value, pack_msg(0, Command.debug_device.value)):
    print("Failed")
  radio.openReadingPipe(1, Addresses.main_unit.value)
  radio.startListening()
  time.sleep(0.5) # sleep for 0.5s to receive
  while(radio.available()):
    data = radio.read(8)
    print(data)
    print(unpack_msg(data))
  radio.stopListening()

setup()

### Main Loop
while True:
  inp_cmd = str(input('Enter Cmd: ')).lower()
  if inp_cmd == 'exit':
    break
  if inp_cmd == 'open':
    path = str(input('file name: '))
    if not FilePath(path).is_file():
        print("Not Found")
        continue
    file_extension = path.split('.')[-1]
    if file_extension == 'svg':
        plot = SvgPlot(path)
        resolution = float(input('Open with resolution: '))
        bbox = plot.bbox()
        print("Dimensions: {:.2f}mm x {:.2f}mm".format(bbox[1] - bbox[0], bbox[3] - bbox[2]))
        print("Generate Instructions:")
        instructions = plot.genInstructions(resolution)
    elif file_extension == 'csv':
        plot = CsvPlot(path)
        instructions = plot.genInstructions()
        bbox = plot.bbox()
        print("Dimensions: {:.2f}mm x {:.2f}mm".format(bbox[1]-bbox[0], bbox[3]-bbox[2]))
    else:
        plot = None
        instructions = []

    print("Done")
  elif inp_cmd == 'save':
    saveInstructions(path+".csv", instructions)
  elif inp_cmd == 'simulate':
    paths = []
    lines = []
    attributes = []
    last_point = origin
    for instr in instructions:
        point, pen, index = instr
        point = transformPoint(point)
        if pen == PenPosition.down:
            lines.append(Line(last_point, point))
        if pen == PenPosition.up:
            # push black path
            if lines:
                paths.append(Path(*lines))
                attributes.append({'style': 'stroke:black;stroke-width: 0.26499999;stroke-opacity: 1;fill:none'})
            lines = []
            # push red path
            paths.append(Path(Line(last_point, point)))
            attributes.append({'style': 'stroke:red;stroke-width: 0.26499999;stroke-opacity: 1;fill:none'})
        last_point = point
    if lines:
        paths.append(Path(*lines))
        attributes.append({'style': 'stroke:black;stroke-width: 0.26499999;stroke-opacity: 1;fill:none'})
    print("Paths: {}".format([len(path) for path in paths]))
    wsvg(paths, filename=path+'.output.svg', attributes=attributes)

  elif inp_cmd == 'draw':
    instr_count = readInt('Draw # of instructions: ', 0)
    # move to resume point
    if instr_index > 0:
        print("Resume Plot")
        point, pen, index = instructions[instr_index-1]
        point = transformPoint(point)
        controlPen(PenPosition.up)
        moveToXY(point.real, point.imag, True)

    for _ in range(0,instr_count):
      if instr_index >= len(instructions):
        print("Finish")
        break
      point, pen, index = instructions[instr_index]
      print("{}: {}, {}, {}".format(instr_index, index, pen.name, point))
      if pen == PenPosition.up:
          controlPen(PenPosition.up)
      else:
          controlPen(PenPosition.down)
      point = transformPoint(point)
      moveToXY(point.real, point.imag, True if pen == PenPosition.up else False)

      instr_index += 1
  elif inp_cmd == 'peek':
      print("Now at #{}: {}".format(instr_index, instructions[instr_index]))
  elif inp_cmd == 'skip':
      instr_count = readInt('Skip # of instructions: ', 0)
      instr_index += instr_count
      print("Now at #{}: {}".format(instr_index, instructions[instr_index]))
  elif inp_cmd == 'restart':
      instr_index = 0
      print("Instruction Index: {}".format(instr_index))
  elif inp_cmd == 'calibrate':
      calibrate()
  elif inp_cmd == 'scale':
      img_scale = readFloat('scale image by factor: ', img_scale)
  elif inp_cmd == 'slow':
      slowdown = readInt('slowdown multiplier: ', slowdown)
  elif inp_cmd == 'offset':
      img_offset = abToPoint(current_a, current_b, motor_distance)
      img_offset = complex(img_offset[0],img_offset[1])
      img_offset = img_offset - origin
      print("offset saved: {}".format(img_offset))
  elif inp_cmd == 'penup':
      success = controlPen(PenPosition.up)
      print(success)
  elif inp_cmd == 'pendown':
      success = controlPen(PenPosition.down)
      print(success)
  elif inp_cmd == 'right':
    count = readInt('mm right: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0]+count, current_position[1])
  elif inp_cmd == 'left':
    count = readInt('mm left: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0]-count, current_position[1])
  elif inp_cmd == 'up':
    count = readInt('mm up: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0], current_position[1] - count)
  elif inp_cmd == 'down':
    count = readInt('mm down: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0], current_position[1] + count)
  elif inp_cmd == 'rightq':
    count = readInt('mm right quick: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0]+count, current_position[1], True)
  elif inp_cmd == 'leftq':
    count = readInt('mm left quick: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0]-count, current_position[1], True)
  elif inp_cmd == 'upq':
    count = readInt('mm up quick: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0], current_position[1] - count, True)
  elif inp_cmd == 'downq':
    count = readInt('mm down quick: ', 0)
    current_position = abToPoint(current_a, current_b, motor_distance)
    moveToXY(current_position[0], current_position[1] + count, True)
  elif inp_cmd == 'leftin':
      count = readInt('steps in: ', 0)
      if controlLeftMotor(Direction.backward, count):
        current_a = current_a-count
      else:
        print("Failed")
  elif inp_cmd == 'leftout':
      count = readInt('steps out: ', 0)
      if controlLeftMotor(Direction.forward, count):
        current_a = current_a+count
      else:
        print("Failed")
  elif inp_cmd == 'rightin':
      count = readInt('steps in: ', 0)
      if controlRightMotor(Direction.backward, count):
        current_b = current_b-count
      else:
        print("Failed")
  elif inp_cmd == 'rightout':
      count = readInt('steps out: ', 0)
      if controlRightMotor(Direction.forward, count):
        current_b = current_b+count
      else:
        print("Failed")
  elif inp_cmd == 'debug':
    for i,v in enumerate(Addresses):
      print('[{}] {}'.format(i, str(v)))
    count = readInt('Select device:', 99)
    if count < len(Addresses):
      debugAddress(list(Addresses)[count])
    else:
      print("Invalid Device")
    
