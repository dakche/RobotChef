# Import all possible libraries --------------------

# Serial comms and control flow stuff
import serial
import serial.tools.list_ports
import time
import os
import sys
import colorsys
import yaml

# Math stuff
import numpy as np
import random
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import math

# AI stuff
import cv2
from pynq_dpu import DpuOverlay
import pytesseract
from pytesseract import Output


# Predefined variables ------------------------------

# ------- Robot arm physical features ---------------
# Base height to shoulder pivot in mm
L_ZOFFSET = 311

# Base X-distance offset from shoulder pivot in mm
L_XOFFSET = 124

# Upper arm length in mm (shoulder pivot --> elbow pivot)
L_UARM = 260

# Upper arm Angle offset in deg (Upper arm --> Elbow point)
A_UOFFSET = 31.7

# Forearm length in mm (elbow pivot --> wrist pivot)
L_4ARM = 274

# Hand Length in mm (wrist pivot --> fingers/grabbing location)
L_HAND = 150

# Wrist pivot to camera X distance in mm
CAM_XDIST = 89

# Wrist_pivot to camera Y distance in mm
CAM_YDIST = 63.5

# Servo value for open claw
S_OPEN = 1700

# Servo value for close claw
S_CLOSE = 1300


# ------------- Target object heights ---------------
# Height of Egg (to center) in mm
H_Egg = 65

# Height of Buttons in mm
H_Button = 86

# Height of strainer in mm
H_Strainer = 300

# Height of handle in mm
H_Handle = 280

# Useful constants ------------------------------------
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# Usefule files ---------------------------------------
classes_path = "./Egg-Detector-8/data.yaml"
xmodel = "yolov5_kr260.xmodel"

# Arm Control functions -------------------------------
# Find the Teensy's USB port
def initArm():
    ports = list(serial.tools.list_ports.comports())

    for p in ports:
        if "USB Serial" in p.description:
            print(p.device)
            print("Connected!")
            break

    # Connect to that port
    arm = serial.Serial(port = p.device, baudrate = 115200, timeout = 0.1)
    time.sleep(3)
    arm.reset_input_buffer()
    return arm

# Poll the current arm angles
def getCurrPos(arm):
    arm.write(b"p\n")

    # Wait for Teensy's response
    while arm.in_waiting == 0:
        continue
    
    # Grab the data
    decoded = arm.readline().decode('utf-8')
    arm.reset_input_buffer()
    resp = decoded.rstrip(";\r\n").split(";")

    # Convert strings to floats
    curr_pos = np.array(resp).astype(float).tolist()
    return curr_pos

# Sends commands angles to arm
def driveArm(arm, angles):
    command = (
        str(round(angles[0], 2)) + "; " 
        + str(round(angles[1], 2)) + "; " 
        + str(round(angles[2], 2)) + "; " 
        + str(round(angles[3], 2)) + "; " 
        + str(angles[4]) + "; "  
        + str(angles[5]) + "\n"
    )
    arm.write(command.encode('ascii'))
    
# Robot Arm Motion Functions ----------------------------
# Calculates inverse law of cosines
# Input triangle side lengths a, b, c
# Output angle value for the angle opposite of side c
def inv_lawOfCosines(a, b, c):
    return ((math.acos(((a**2)+(b**2)-(c**2))/(2*a*b))) * RAD2DEG)

# The opposite of inv_lawOfCosines
def lawOfCosines(a, b, C):
    return math.sqrt((a**2)+(b**2)-(2*a*b*math.cos(C*DEG2RAD)))

# Calculates distance between two points
def dist(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    return np.sqrt((x2-x1)**2 + (y2 - y1)**2)

# Inverse Kinematics solver. 
# Input a 2D location in space with respect to the base of the robot
# Input the target device (claw or camera)
# Also input desired camera or claw angle, business end wrt the ground
# Output is an array of joint angles
def IK(loc, device, angle, tt, holding):
    # Reference location (aka where the shoulder joint is)
    ref = [-L_XOFFSET, L_ZOFFSET]
    target = loc
    
    angle_sin = math.sin(angle * DEG2RAD)
    angle_cos = math.cos(angle * DEG2RAD)
    
    # Compensating desired position based on device
    if (device == 'cam'):
        target[0] = target[0] - (CAM_XDIST * angle_cos) - (CAM_YDIST * angle_sin)
        target[1] = target[1] - (CAM_XDIST * angle_sin) + (CAM_YDIST * angle_cos)
        
    elif (device == 'claw'):
        target[0] -= L_HAND * angle_sin
        target[1] += L_HAND * angle_cos
    
    # Distance calculation
    v_side = dist(target, ref)
    v_angle = 180 - math.asin((target[1]-L_ZOFFSET) / v_side) * RAD2DEG
    
    # Shoulder angle
    virt_s_angle = v_angle - inv_lawOfCosines(v_side, L_UARM, L_4ARM)
    s_angle = virt_s_angle - A_UOFFSET
    
    # Elbow angle
    virt_e_angle = inv_lawOfCosines(L_4ARM, L_UARM, v_side)
    e_angle = virt_e_angle - A_UOFFSET
    
    # Wrist bend angle
    y_angle = 180 + angle - (virt_e_angle - (virt_s_angle - 90))
    
    if (device == 'cam'):
        y_angle += 90
        
    if (holding == True):
        c_val = S_CLOSE
    else:
        c_val = S_OPEN

    return [s_angle, e_angle, -3, y_angle, tt, c_val]
    
# Forward Kinematics solver.
# Input joint angles (pos)
# Also input desired device
# Returns point in space and device angle wrt the ground
def FK(pos, device):
    s_angle = pos[0]
    e_angle = pos[1]
    y_angle = pos[3]
    
    if (device == 'cam'):
        y_angle -= 90
        
    virt_e_angle = e_angle + A_UOFFSET
    virt_s_angle = s_angle + A_UOFFSET
    
    v_side = lawOfCosines(L_4ARM, L_UARM, virt_e_angle)
    v_angle = virt_s_angle + inv_lawOfCosines(v_side, L_UARM, L_4ARM)
    
    angle = y_angle - 180 + (virt_e_angle - (virt_s_angle - 90))
    
    ypos = (math.sin((180 - v_angle) * DEG2RAD) * v_side) + L_ZOFFSET
    xpos = (v_side * math.cos((180 - v_angle) * DEG2RAD)) - L_XOFFSET
    
    angle_sin = math.sin(angle * DEG2RAD)
    angle_cos = math.cos(angle * DEG2RAD)
    
    if (device == 'cam'):
        xpos = xpos + (CAM_XDIST * angle_cos) + (CAM_YDIST * angle_sin)
        ypos = ypos + (CAM_XDIST * angle_sin) - (CAM_YDIST * angle_cos)
        
    elif (device == 'claw'):
        xpos += L_HAND * angle_sin
        ypos -= L_HAND * angle_cos
        
    return [xpos, ypos, angle]

# Camera functions --------------------------------------
# Get the camera
def openCam():
    cap = cv2.VideoCapture(-1)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

# Close the camera and garbage collection
def closeCam(cap):
    cap.release()
    cv2.destroyAllWindows()

# DPU Vision functions ----------------------------------
# Create the overlay
def loadModel(model):
    overlay = DpuOverlay("dpu.bit")
    overlay.load_model(model)
    return overlay

# Queue up the model
def startDPU(overlay):
    dpu = overlay.runner
    return dpu

# Garbage collection
def closeDPU(overlay, dpu):
    del overlay
    del dpu

# Gets the names of object detection classes
def get_class(classes_path):
    with open(classes_path) as f:
        data = yaml.safe_load(f)
        classes = data.get('names', [])
        return classes
    
# Initialize things related to object detection
class_names = get_class(classes_path)
num_classes = len(class_names)
hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))
random.seed(0)
random.shuffle(colors)
random.seed(None)

# Start the DPU so it is ready when we need it
overlay = loadModel(xmodel)
dpu = startDPU(overlay)

anchor_list = [10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326]
anchor_float = [float(x) for x in anchor_list]
anchors = np.array(anchor_float).reshape(-1, 2)

# Grab info about the model
inputTensors = dpu.get_input_tensors()
outputTensors = dpu.get_output_tensors()

shapeIn = tuple(inputTensors[0].dims)

shapeOut0 = (tuple(outputTensors[0].dims))
shapeOut1 = (tuple(outputTensors[1].dims))
shapeOut2 = (tuple(outputTensors[2].dims))

outputSize0 = int(outputTensors[0].get_data_size() / shapeIn[0])
outputSize1 = int(outputTensors[1].get_data_size() / shapeIn[0])
outputSize2 = int(outputTensors[2].get_data_size() / shapeIn[0])
input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
output_data = [np.empty(shapeOut0, dtype=np.float32, order="C"), 
           np.empty(shapeOut1, dtype=np.float32, order="C"),
           np.empty(shapeOut2, dtype=np.float32, order="C")]
image = input_data[0]
    
    
# Resize image with unchanged aspect ratio using padding
def letterbox_image(image, size):
    ih, iw, _ = image.shape
    w, h = size
    scale = min(w/iw, h/ih)
    
    nw = int(iw*scale)
    nh = int(ih*scale)

    image = cv2.resize(image, (nw,nh), interpolation=cv2.INTER_LINEAR)
    new_image = np.ones((h,w,3), np.uint8) * 128
    h_start = (h-nh)//2
    w_start = (w-nw)//2
    new_image[h_start:h_start+nh, w_start:w_start+nw, :] = image
    return new_image


# Image preprocessing
def pre_process(image, model_image_size):
    image = image[...,::-1]
    image_h, image_w, _ = image.shape
 
    if model_image_size != (None, None):
        assert model_image_size[0]%32 == 0, 'Multiples of 32 required'
        assert model_image_size[1]%32 == 0, 'Multiples of 32 required'
        boxed_image = letterbox_image(image, tuple(reversed(model_image_size)))
    else:
        new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
        boxed_image = letterbox_image(image, new_image_size)
    image_data = np.array(boxed_image, dtype='float32')
    image_data /= 255.
    image_data = np.expand_dims(image_data, 0) 	
    return image_data

# Grabs the features
def _get_feats(feats, anchors, num_classes, input_shape):
    num_anchors = len(anchors)
    anchors_tensor = np.reshape(np.array(anchors, dtype=np.float32), [1, 1, 1, num_anchors, 2])
    grid_size = np.shape(feats)[1:3]
    nu = num_classes + 5
    predictions = np.reshape(feats, [-1, grid_size[0], grid_size[1], num_anchors, nu])
    grid_y = np.tile(np.reshape(np.arange(grid_size[0]), [-1, 1, 1, 1]), [1, grid_size[1], 1, 1])
    grid_x = np.tile(np.reshape(np.arange(grid_size[1]), [1, -1, 1, 1]), [grid_size[0], 1, 1, 1])
    grid = np.concatenate([grid_x, grid_y], axis = -1)
    grid = np.array(grid, dtype=np.float32)

    box_xy = (1/(1+np.exp(-predictions[..., :2])) + grid) / np.array(grid_size[::-1], dtype=np.float32)
    box_wh = np.exp(predictions[..., 2:4]) * anchors_tensor / np.array(input_shape[::-1], dtype=np.float32)
    box_confidence = 1/(1+np.exp(-predictions[..., 4:5]))
    box_class_probs = 1/(1+np.exp(-predictions[..., 5:]))
    return box_xy, box_wh, box_confidence, box_class_probs

# Correct the boxes back to the original picture
def correct_boxes(box_xy, box_wh, input_shape, image_shape):
    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]
    input_shape = np.array(input_shape, dtype = np.float32)
    image_shape = np.array(image_shape, dtype = np.float32)
    new_shape = np.around(image_shape * np.min(input_shape / image_shape))
    offset = (input_shape - new_shape) / 2. / input_shape
    scale = input_shape / new_shape
    box_yx = (box_yx - offset) * scale
    box_hw *= scale

    box_mins = box_yx - (box_hw / 2.)
    box_maxes = box_yx + (box_hw / 2.)
    boxes = np.concatenate([
        box_mins[..., 0:1],
        box_mins[..., 1:2],
        box_maxes[..., 0:1],
        box_maxes[..., 1:2]
    ], axis = -1)
    boxes *= np.concatenate([image_shape, image_shape], axis = -1)
    return boxes


def boxes_and_scores(feats, anchors, classes_num, input_shape, image_shape):
    box_xy, box_wh, box_confidence, box_class_probs = _get_feats(feats, anchors, classes_num, input_shape)
    boxes = correct_boxes(box_xy, box_wh, input_shape, image_shape)
    boxes = np.reshape(boxes, [-1, 4])
    box_scores = box_confidence * box_class_probs
    box_scores = np.reshape(box_scores, [-1, classes_num])
    return boxes, box_scores

'''Draw detection frame'''
def draw_bbox(image, bboxes, classes):
    """
    bboxes: [x_min, y_min, x_max, y_max, probability, cls_id] format coordinates.
    """
    num_classes = len(classes)
    image_h, image_w, _ = image.shape
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))

    random.seed(0)
    random.shuffle(colors)
    random.seed(None)

    for i, bbox in enumerate(bboxes):
        coor = np.array(bbox[:4], dtype=np.int32)
        fontScale = 0.5
        score = bbox[4]
        class_ind = int(bbox[5])
        bbox_color = colors[class_ind]
        bbox_thick = int(0.6 * (image_h + image_w) / 600)
        c1, c2 = (coor[0], coor[1]), (coor[2], coor[3])
        cv2.rectangle(image, c1, c2, bbox_color, bbox_thick)
    return image


def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.

    # Arguments
        boxes: ndarray, boxes of objects.
        scores: ndarray, scores of objects.

    # Returns
        keep: ndarray, index of effective boxes.
    """
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2-x1+1)*(y2-y1+1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 1)
        h1 = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= 0.45)[0]  # threshold
        order = order[inds + 1]

    return keep

def draw_boxes(image, boxes, scores, classes):
    _, ax = plt.subplots(1)
    ax.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    image_h, image_w, _ = image.shape

    for i, bbox in enumerate(boxes):
        [top, left, bottom, right] = bbox
        width, height = right - left, bottom - top
        center_x, center_y = left + width*0.5, top + height*0.5
        score, class_index = scores[i], classes[i]
        label = '{}: {:.4f}'.format(class_names[class_index], score) 
        color = tuple([color/255 for color in colors[class_index]])
        ax.add_patch(Rectangle((left, top), width, height,
                               edgecolor=color, facecolor='none'))
        ax.annotate(label, (center_x, center_y), color=color, weight='bold', 
                    fontsize=12, ha='center', va='center')
    return ax

# Functions needed for modified Krustal's Algorithm
def find(parent, i):
    if parent[i] == i:
        return i
    return find(parent, parent[i])

def union(parent, rank, x, y):
    xroot = find(parent, x)
    yroot = find(parent, y)

    if rank[xroot] < rank[yroot]:
        parent[xroot] = yroot
    elif rank[xroot] > rank[yroot]:
        parent[yroot] = xroot
    else:
        parent[yroot] = xroot
        rank[xroot] += 1

# Calculate centroid of a set of points
def centroid(pts):
    point_x = [point[0] for point in pts]
    point_y = [point[1] for point in pts]
    
    centroid_x = np.mean(point_x)
    centroid_y = np.mean(point_y)
    
    return (centroid_x, centroid_y)
        
        
# Gets clusters from a set of points using Union Find
def getClusters(pts, thresh):
    #print("Pts: ", pts)
    distances = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            distance = dist(pts[i], pts[j])
            if (distance <= thresh):
                distances.append((distance, pts[i], pts[j]))
    
    distances_ = sorted(distances, key=lambda x: x[0])
    
    # Initialize parent and rank arrays for union-find
    parent = list(range(len(pts)))
    rank = [0] * len(pts)

    # Perform union-find on nearby points
    for _, point1, point2 in distances_:
        point1_index = np.where((pts == point1).all(axis=1))[0][0]
        point2_index = np.where((pts == point2).all(axis=1))[0][0]
        union(parent, rank, point1_index, point2_index)

    # Find the connected components (clusters)
    clusters = {}
    for i in range(len(pts)):
        root = find(parent, i)
        if root not in clusters:
            clusters[root] = []
        clusters[root].append(i)

    # Only save clusters that are bigger than 5 elements
    new_clusters = {}
    for cluster_root, points_indices in clusters.items():
        if len(points_indices) >= 5:
            new_clusters[cluster_root] = points_indices
            
    eggs = []
    # Get centroids of the clusters
    for cluster_root, points_indices in new_clusters.items():
        eggs.append(centroid([pts[i] for i in points_indices]))
        
    return eggs
    
# Evaluates the DPU output
def evaluate(yolo_outputs, image_shape, class_names, anchors):
    score_thresh = 0.1
    anchor_mask = [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
    boxes = []
    box_scores = []
    input_shape = np.shape(yolo_outputs[0])[1 : 3]
    input_shape = np.array(input_shape)*32

    for i in range(len(yolo_outputs)):
        _boxes, _box_scores = boxes_and_scores(
            yolo_outputs[i], anchors[anchor_mask[i]], len(class_names), 
            input_shape, image_shape)
        boxes.append(_boxes)
        box_scores.append(_box_scores)
    boxes = np.concatenate(boxes, axis = 0)
    box_scores = np.concatenate(box_scores, axis = 0)

    mask = box_scores >= score_thresh
    boxes_ = []
    scores_ = []
    classes_ = []
    for c in range(len(class_names)):
        class_boxes_np = boxes[mask[:, c]]
        class_box_scores_np = box_scores[:, c]
        class_box_scores_np = class_box_scores_np[mask[:, c]]
        nms_index_np = nms_boxes(class_boxes_np, class_box_scores_np) 
        class_boxes_np = class_boxes_np[nms_index_np]
        class_box_scores_np = class_box_scores_np[nms_index_np]
        classes_np = np.ones_like(class_box_scores_np, dtype = np.int32) * c
        boxes_.append(class_boxes_np)
        scores_.append(class_box_scores_np)
        classes_.append(classes_np)
    boxes_ = np.concatenate(boxes_, axis = 0)
    scores_ = np.concatenate(scores_, axis = 0)
    classes_ = np.concatenate(classes_, axis = 0)

    return boxes_, scores_, classes_

# Returns the location of obj screen based on input feed cap
def runYolo(cap, obj):
    found = False
    
    if (obj == 'egg'):
        obj_x = 0
        obj_y = 0
    elif (obj == 'strainer'):
        obj_x = 350
        obj_y = 0
    
    count = 0
    
    while ((not found) and (count <= 24)):
        ret, input_image = cap.read()
        
        # Pre-processing
        image_size = input_image.shape[:2]
        image_data = np.array(pre_process(input_image, (640, 640)), dtype=np.float32)
    
        # Fetch data to DPU and trigger it
        image[0,...] = image_data.reshape(shapeIn[1:])
        job_id = dpu.execute_async(input_data, output_data)
        dpu.wait(job_id)

        # Retrieve output data
        conv_out0 = np.reshape(output_data[0], shapeOut0)
        conv_out1 = np.reshape(output_data[1], shapeOut1)
        conv_out2 = np.reshape(output_data[2], shapeOut2)
        yolo_outputs = [conv_out0, conv_out1, conv_out2]

        # Decode output from YOLOv5
        boxes, _, classes = evaluate(yolo_outputs, image_size, class_names, anchors)

        image_h, image_w, _ = input_image.shape
        
        # Init vars for calculating centroids
        egg_pts = []
        count_eggs = 0

        sumX_strainer = 0
        sumY_strainer = 0
        count_strainer = 0

        sumX_handle = 0
        sumY_handle = 0
        count_handle = 0

        for i, bbox in enumerate(boxes):
            [top, left, bottom, right] = bbox
            width, height = right - left, bottom - top
            center_x, center_y = left + width*0.5, top + height*0.5
            class_index = classes[i]
            
            # Grabs the center of each box found
            # For the sake of this demonstration, we assume there is only one strainer and one handle
            # But the number of eggs is unknown, so we will run a slightly different algorithm
            if (class_index == 0):
                egg_pts.append([center_x, center_y])
                count_eggs += 1
            elif (class_index == 2):
                sumX_strainer += center_x
                sumY_strainer += center_y
                count_strainer += 1
            elif (class_index == 1):
                sumX_handle += center_x
                sumY_handle += center_y
                count_handle += 1
        
        count += 1
        
        # Circles the average of all boxes
        clusters = []
        if ((count_eggs > 0) and (obj == 'egg')):
            all_pts = np.array(egg_pts)
            clusters = getClusters(all_pts, 20)
            colorEgg = tuple([color for color in colors[0]])
            for x_coor, y_coor in clusters:
                cv2.circle(input_image, (int(x_coor), int(y_coor)), radius = 5, color = colorEgg, thickness = 1)
                obj_x = x_coor
                obj_y = y_coor
            found = True
        if ((count_strainer > 0) and (obj == 'strainer')):
            ctrX_Str = int(sumX_strainer / count_strainer)
            ctrY_Str = int(sumY_strainer / count_strainer)
            colorStr = tuple([color for color in colors[2]])
            cv2.circle(input_image, (ctrX_Str, ctrY_Str), radius = 5, color = colorStr, thickness = 1)
            obj_x = ctrX_Str
            obj_y = ctrY_Str
            found = True
        if ((count_handle > 0) and (obj == 'handle')):
            ctrX_Han = int(sumX_handle / count_handle)
            ctrY_Han = int(sumY_handle / count_handle)
            colorHan = tuple([color for color in colors[1]])
            cv2.circle(input_image, (ctrX_Han, ctrY_Han), radius = 5, color = colorHan, thickness = 1)
            obj_x = ctrX_Str
            obj_y = ctrY_Str
            found = True
            
        # time2 = time.time()
        # fps = 1/(time2-time1)
        # fpsStats = 'FPS: {:.2f}'.format(fps)
        # cv2.putText(input_image, fpsStats, (0,30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
        cv2.imshow('frame', input_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return found, [obj_x, obj_y]

# Motions -------------------------------------------------------------------
# Am I holding something right now?
holding_ = False
prev_state = 'none'

# Find an object, distance far or near
def locate(arm, cap, obj, dist, tilt, tt_ctrl):
    global prev_state
    global holding_
    
    cam_pos = [0, 0]
    
    if (dist == 'far'):
        cam_pos1 = [250, 660]
        if (prev_state == 'near') or (prev_state == 'none'):
            posp = IK(cam_pos1, 'cam', 0, 0, holding_)
            driveArm(arm, posp)
            time.sleep(5)
        prev_state = 'far'
        cam_pos = cam_pos1
    elif (dist == 'near'):
        cam_pos2 = [230, 305]
        if (prev_state == 'far') or (prev_state == 'none'):
            posp = IK(cam_pos2, 'cam', 0, 0, holding_)
            driveArm(arm, posp)
            time.sleep(5)
        prev_state = 'near'
        cam_pos = cam_pos2
    
    pos = IK(cam_pos, 'cam', 0, tt_ctrl, holding_)
    driveArm(arm, pos)
    
    if (holding_):
        claw = S_CLOSE
    else:
        claw = S_OPEN
    
    # Hold the position of the arm, whatever it is
    hold_pos = getCurrPos(arm)
    new_hold_pos = [hold_pos[0], hold_pos[1], hold_pos[2], hold_pos[3], tt_ctrl, claw]
    driveArm(arm, new_hold_pos)
    
    cur_angle = FK(hold_pos, 'cam')[2]
    new_hold_pos[3] -= cur_angle
    
    driveArm(arm, new_hold_pos)
    
    f, x = runYolo(cap, obj)
    if (tilt == True):
        if (not f):
            new_hold_pos[2] = -20
            driveArm(arm, new_hold_pos)

        f, x = runYolo(cap, obj)
        if (not f):
            new_hold_pos[2] = 20
            driveArm(arm, new_hold_pos)
            
        f, x = runYolo(cap, obj)
    
    new_hold_pos[2] = -3
    driveArm(arm, new_hold_pos)
    time.sleep(0.5)
    
    return f, x

def clawCtrl(arm, action):
    global holding_
    cur = getCurrPos(arm)
    angs = [cur[0], cur[1], cur[2], cur[3], 0, 1600]
    if (action == 'grab'):
        print("Grab")
        angs[5] = S_CLOSE
        driveArm(arm, angs)
        holding_ = True
    elif (action == 'place'):
        print("Let go")
        angs[5] = S_OPEN
        driveArm(arm, angs)
        holding_ = False
    time.sleep(2)

# Spins the base so it lines up with desired object
# direction is the rough location of the item
def goTo(arm, cap, obj, dist, direction):
    tt_ctrl = int(17 * (240 - direction[0]))
    
    if (abs(tt_ctrl) < 1700):
        if (tt_ctrl < 0):
            tt_ctrl = -1700
        else:
            tt_ctrl = 1700
            
    f, x = locate(arm, cap, obj, dist, False, tt_ctrl)
    while ((x[0] < 240) or (x[0] > 270)):
        f, x = locate(arm, cap, obj, dist, False, tt_ctrl)
        tt_ctrl = int(17 * (240 - x[0]))
        if (abs(tt_ctrl) < 1500):
            if (tt_ctrl < 0):
                tt_ctrl = -1500
            else:
                tt_ctrl = 1500
    f, x = locate(arm, cap, obj, dist, False, 0)
    print("Exited goTo")

# Pick up something at coordates loc
# Orientation defines whether object is horizontal or vertical
def pickUpPlace(arm, cap, obj, orientation):
    global holding_
    
    if (orientation == 'hor'):
        claw_angle = 90
    elif (orientation == 'vert'):
        claw_angle = 0
        
    if (obj == 'egg'):
        dist = 'near'
    else:
        dist = 'far'
    
    f, x = locate(arm, cap, obj, dist, False, 0)
    if ((x[0] < 220) or (x[0] > 260)):
        print("final touches")
        goTo(arm, cap, obj, dist, x)
    print("adjusting angle")
    cam_tilt = 0
    curPos = getCurrPos(arm)
    
    if (obj == 'egg'):
        upper = 260
        lower = 220
    if (obj == 'strainer'):
        upper = 260
        lower = 220
        
    if (holding_ == True):
        c = S_CLOSE
    elif (holding_ == False):
        c = S_OPEN
    
    while ((x[1] < lower) or (x[1] > upper)):
        cam_tilt += 0.1
        if (x[1] < lower):
            newTilt = curPos[3] + cam_tilt
        elif (x[1] > upper):
            newTilt = curPos[3] - cam_tilt
        
        newPos = [curPos[0], curPos[1], curPos[2], newTilt, 0, c]
        driveArm(arm, newPos)
        time.sleep(0.01)
        _, x = runYolo(cap, obj)
        
    curPos = getCurrPos(arm)
    newPos = [curPos[0], curPos[1], curPos[2], curPos[3], 0, c]
    driveArm(arm, newPos)
    
    print("angle set, now grabbing")
    data = FK(getCurrPos(arm), 'cam')
    curr_x = data[0]
    curr_y = data[1]
    curr_ang = data[2]
    
    offset = 0
    if (obj == 'egg'):
        offset = H_Egg
    elif (obj == 'strainer'):
        offset = H_Strainer
    elif (obj == 'handle'):
        offset = H_Handle
    
    new_x = ((curr_y - offset) * math.tan(curr_ang * DEG2RAD)) + curr_x + 35
    
    angs = IK([new_x, offset], 'claw', claw_angle, 0, holding_)
    driveArm(arm, angs)
    time.sleep(2)

# Press the button that says word
def press(arm, word):
    print("Pressing {}".format(word))
    return 0

# Put one thing in another thing
def put(arm, cap, src, dest):
    # Find object, get approx location (left or right)
    _, x = locate(arm, cap, src, 'far', True, 0)
    goTo(arm, cap, src, 'near', x)
    pickUpPlace(arm, cap, src, 'vert')
    clawCtrl(arm, 'grab')
    _, x = locate(arm, cap, dest, 'far', True, 0)
    goTo(arm, cap, dest, 'far', x)
    pickUpPlace(arm, cap, dest, 'vert')
    clawCtrl(arm, 'place')
    _, x = locate(arm, cap, dest, 'far', True, 0)
    
    return 0

action_verbs = ['put', 'wait', 'press']
button_words = ['power', 'high', 'low']
object_nouns = ['egg', 'strainer', 'handle']

# Process the recipe
def process(file):
    text = file.read()
    sentences = text.split('\n')
    all_cmds = []
    
    for sentence in sentences:
        cmds = []
        sentence = sentence.lower().strip('.')
        words = sentence.split(' ')
        for key in action_verbs:
            if key in words:
                verb = key
                cmds.append(verb)
                break
        if (verb == 'put'):
            idx = words.index('in')
            first = words[:idx]
            last = words[idx:]
            
            for key in object_nouns:
                if key in first:
                    src = key
                if key in last:
                    dest = key
            
            cmds.append(src)
            cmds.append(dest)
        
        elif (verb == 'press'):
            idx = words.index('button')
            cmds.append(words[idx-1])
            
        elif (verb == 'wait'):
            idx = words.index('wait')
            cmds.append(words[idx+1])
        
        all_cmds.append(cmds)
    return all_cmds

# Main Function -------------------------------------------------------------
def main(file):
    # Process the recipe file
    line_count = 0
    try:
        with open(file_path, 'r') as file:
            cmds = process(file)
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
      
     # Initialize arm and camera
    arm = initArm()
    cap = openCam()
    
    # Do what the recipe says
    for cmd in cmds:
        if (cmd[0] == 'put'):
            print("Put {} in {}".format(cmd[1], cmd[2]))
            put(arm, cap, cmd[1], cmd[2])
        elif (cmd[0] == 'press'):
            press(arm, cmd[1])
        elif (cmd[0] == 'wait'):
            print("Waiting for {} seconds".format(cmd[1]))
            time.sleep(int(cmd[1]))
    
if __name__ == "__main__":
    # Check for recipe file
    if len(sys.argv) < 2:
        print("Usage: python3 chef.py <file_path>")
    else:
        file_path = sys.argv[1]
        main(file_path)