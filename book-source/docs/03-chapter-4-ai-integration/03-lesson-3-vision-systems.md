---
title: "Lesson 3: Vision Systems for Robotics"
chapter: 4
lesson: 3

# Robotics-Specific Metadata
simulation_required: []
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []

# Learning Objectives
learning_objectives:
  - "Complete this lesson"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation)"
---

<PersonalizedLesson lessonPath="03-chapter-4-ai-integration/03-lesson-3-vision-systems.md">

# Lesson 3: Vision Systems for Robotics

## Learning Objectives

By the end of this lesson, you will:
1. Implement object detection (YOLO-v8) to identify objects in scenes
2. Perform semantic segmentation to classify pixels by object type
3. Integrate depth data for 3D perception
4. Create ROS 2 nodes that publish detection and segmentation results
5. Understand real hardware challenges in vision (lighting, occlusion, object variability)

## What are Vision Systems?

A **vision system** gives robots the ability to "see" and understand their environment. It answers three questions:

### 1. Object Detection: "What objects are in the scene?"

```
Input Image:    [Red cube, blue sphere, shelf, table, ...]
                ↓
Detector (YOLO-v8)
                ↓
Output:         [{class: "cube", bbox: [100,50,150,100], confidence: 0.95},
                 {class: "sphere", bbox: [200,80,250,130], confidence: 0.92},
                 ...]
```

**Uses**: Identifying targets for manipulation, finding obstacles, scene inventory

### 2. Semantic Segmentation: "What is each pixel?"

```
Input Image:    [Camera image - 640×480 pixels]
                ↓
Segmentation (FCN/DeepLab)
                ↓
Output:         [Label for each pixel: 0=background, 1=table, 2=cube, ...]
                Visualization: Color-coded segmentation mask
```

**Uses**: Scene understanding, free space detection, grasping surface identification

### 3. Depth Sensing: "How far is everything?"

```
Input:          Stereo cameras or depth sensor
                ↓
Depth Estimation
                ↓
Output:         [Depth for each pixel: distance from camera in meters]
```

**Uses**: 3D object localization, obstacle avoidance, grasp planning

## Vision Pipeline Architecture

```
┌──────────────────┐
│  Camera Stream   │ (RGB images @ 10+ Hz)
└────────┬─────────┘
         │
         ↓
┌─────────────────────────────┐
│  Object Detection Node      │
│  (YOLO-v8)                  │
├─────────────────────────────┤
│ • Load model                │
│ • Run inference (<100ms)    │
│ • Extract bounding boxes    │
│ • Filter by confidence      │
└────────┬────────────────────┘
         │ (/robot/detections)
         ↓
┌─────────────────────────────┐
│  Segmentation Node          │
│  (FCN/DeepLab)              │
├─────────────────────────────┤
│ • Load model                │
│ • Run inference             │
│ • Generate pixel labels     │
│ • Map to class names        │
└────────┬────────────────────┘
         │ (/robot/segmentation)
         ↓
┌──────────────────────────────┐
│  Combined Perception Result  │
│ • Detected objects           │
│ • Scene segmentation         │
│ • Ready for planning         │
└──────────────────────────────┘
```

## Object Detection: YOLO-v8

### What is YOLO?

**YOLO** = "You Only Look Once" — a real-time object detection framework

- **Fast**: &lt;50ms inference on consumer GPU
- **Accurate**: ≥85% mean Average Precision (mAP)
- **Easy to use**: Pre-trained models available
- **Trainable**: Can fine-tune on custom objects

### Installation

```bash
pip install ultralytics torch torchvision

# Verify
python -c "from ultralytics import YOLO; print('YOLO installed')"
```

### Basic Usage

```python
from ultralytics import YOLO
import cv2

# Load pre-trained model
model = YOLO('yolov8n.pt')  # nano model, fastest

# Run detection on image
results = model('image.jpg')

# Extract detections
for result in results:
    for detection in result.boxes:
        x1, y1, x2, y2 = detection.xyxy[0]
        confidence = detection.conf[0]
        class_id = int(detection.cls[0])
        class_name = result.names[class_id]

        print(f"{class_name}: {confidence:.2f} at [{x1},{y1},{x2},{y2}]")
```

### ROS 2 Detection Node

File: `chapter4_vision/src/yolo_detector_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from datetime import datetime

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Load YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')  # nano model
        self.get_logger().info('YOLO model loaded')

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.image_callback,
            10
        )

        self.detections_pub = self.create_publisher(
            String,  # For simplicity, publish as JSON string
            '/robot/detections',
            10
        )

        self.annotated_image_pub = self.create_publisher(
            Image,
            '/robot/camera/annotated',
            10
        )

        self.cv_bridge = CvBridge()
        self.confidence_threshold = 0.5

        self.get_logger().info('YOLO Detector Node initialized')

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO inference
            results = self.model(cv_image)

            # Extract detections
            detections = []
            for result in results:
                for detection in result.boxes:
                    conf = float(detection.conf[0])

                    # Filter by confidence threshold
                    if conf < self.confidence_threshold:
                        continue

                    # Extract bounding box
                    bbox = detection.xyxy[0]
                    x1, y1, x2, y2 = [int(v) for v in bbox]

                    # Extract class info
                    class_id = int(detection.cls[0])
                    class_name = result.names[class_id]

                    detection_dict = {
                        'class': class_name,
                        'confidence': round(conf, 3),
                        'bbox': {
                            'x1': x1, 'y1': y1,
                            'x2': x2, 'y2': y2,
                            'width': x2 - x1,
                            'height': y2 - y1
                        }
                    }
                    detections.append(detection_dict)

                # Publish detections as JSON
                import json
                msg_out = String()
                msg_out.data = json.dumps({
                    'detections': detections,
                    'timestamp': datetime.now().isoformat(),
                    'image_size': list(cv_image.shape)
                })
                self.detections_pub.publish(msg_out)

                # Annotate image and publish
                annotated = self._annotate_image(cv_image, detections)
                annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                self.annotated_image_pub.publish(annotated_msg)

                self.get_logger().info(f'Detected {len(detections)} objects')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def _annotate_image(self, image, detections):
        """Draw bounding boxes on image"""
        annotated = image.copy()

        for det in detections:
            bbox = det['bbox']
            x1, y1 = bbox['x1'], bbox['y1']
            x2, y2 = bbox['x2'], bbox['y2']
            label = f"{det['class']} {det['confidence']:.2f}"

            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return annotated

def main(args=None):
    rclpy.init(args=args)
    detector_node = YOLODetectorNode()
    rclpy.spin(detector_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Semantic Segmentation

### What is Semantic Segmentation?

**Semantic segmentation** assigns a class label to every pixel in the image.

```
Input Image:        [640×480 RGB image]
                    ↓
Segmentation Model  (FCN or DeepLab)
                    ↓
Output Mask:        [640×480 array with class IDs]
Visualization:      [Color-coded mask showing regions]
```

### Installation

```bash
pip install segmentation-models-pytorch albumentations

# Or use pre-trained models from torchvision
pip install torch torchvision
```

### ROS 2 Segmentation Node

File: `chapter4_vision/src/segmentation_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
from torchvision.models.segmentation import fcn_resnet50
import numpy as np
import json

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        # Load segmentation model
        self.get_logger().info('Loading segmentation model...')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = fcn_resnet50(pretrained=True).to(self.device)
        self.model.eval()
        self.get_logger().info(f'Segmentation model loaded on {self.device}')

        # Class mapping for COCO dataset
        self.coco_classes = {
            0: 'background', 1: 'person', 2: 'bicycle', 3: 'car',
            # ... (21 classes total)
            15: 'cat', 16: 'dog', 18: 'table', 19: 'chair', 20: 'other'
        }

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.image_callback,
            10
        )

        self.segmentation_pub = self.create_publisher(
            Image,
            '/robot/segmentation',
            10
        )

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Prepare image for model
            image_tensor = self._prepare_image(cv_image)

            # Run segmentation inference
            with torch.no_grad():
                output = self.model(image_tensor)

            # Extract segmentation mask
            segmentation = output['out'][0].argmax(dim=0).cpu().numpy()

            # Create color-coded visualization
            colored_mask = self._colorize_segmentation(segmentation)

            # Publish segmentation mask
            seg_msg = self.cv_bridge.cv2_to_imgmsg(colored_mask, encoding='rgb8')
            self.segmentation_pub.publish(seg_msg)

            self.get_logger().info('Published segmentation mask')

        except Exception as e:
            self.get_logger().error(f'Error in segmentation: {str(e)}')

    def _prepare_image(self, cv_image):
        """Prepare image for segmentation model"""
        # Resize to 512×512 (model input size)
        resized = cv2.resize(cv_image, (512, 512))

        # Normalize
        normalized = resized.astype(np.float32) / 255.0
        normalized = (normalized - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]

        # Convert to tensor
        tensor = torch.from_numpy(normalized.transpose(2, 0, 1)).unsqueeze(0)
        return tensor.to(self.device)

    def _colorize_segmentation(self, segmentation):
        """Create color-coded segmentation visualization"""
        h, w = segmentation.shape
        colored = np.zeros((h, w, 3), dtype=np.uint8)

        # Color map for different classes
        colors = {
            0: (0, 0, 0),           # background - black
            15: (255, 0, 0),        # cat - red
            16: (0, 255, 0),        # dog - green
            18: (0, 0, 255),        # table - blue
            19: (255, 255, 0),      # chair - yellow
        }

        for class_id, color in colors.items():
            mask = (segmentation == class_id)
            colored[mask] = color

        return colored

def main(args=None):
    rclpy.init(args=args)
    seg_node = SegmentationNode()
    rclpy.spin(seg_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Sensing

### Depth Cameras in Isaac Sim

Isaac Sim can simulate depth cameras (RGB-D sensors):

```python
# Isaac Sim provides depth as float32 array
depth_image = get_depth_from_camera()  # meters

# Convert to visualization
depth_visual = (depth_image * 1000).astype(np.uint8)  # millimeters
depth_visual = cv2.applyColorMap(depth_visual, cv2.COLORMAP_TURBO)

# Use depth for 3D object localization
# If object center is at pixel (x, y) and depth is d:
# 3D position = camera.project_pixel_to_3d(x, y, d)
```

### 3D Object Localization

```python
def get_object_3d_position(detection, depth_image, camera_intrinsics):
    """Convert 2D detection + depth to 3D position"""
    # Get bounding box center
    bbox = detection['bbox']
    center_x = (bbox['x1'] + bbox['x2']) / 2
    center_y = (bbox['y1'] + bbox['y2']) / 2

    # Get depth at center
    depth = depth_image[int(center_y), int(center_x)]

    # Project to 3D using camera intrinsics
    fx, fy, cx, cy = camera_intrinsics
    x_3d = (center_x - cx) * depth / fx
    y_3d = (center_y - cy) * depth / fy
    z_3d = depth

    return [x_3d, y_3d, z_3d]  # meters
```

## Exercises

### Exercise L3-1: Object Detection

**Objective**: Detect objects in a scene using YOLO-v8

**Steps**:
1. Install YOLO: `pip install ultralytics`
2. Download model: `model = YOLO('yolov8n.pt')`
3. Detect in image: `results = model('image.jpg')`
4. Print detections with confidence scores
5. Visualize with bounding boxes

**Success Criteria**:
- ✓ YOLO model loads and runs
- ✓ Detects ≥3 objects with confidence scores
- ✓ Bounding boxes correct (match visual inspection)
- ✓ Inference time &lt;100ms

### Exercise L3-2: ROS 2 Vision Integration

**Objective**: Create vision nodes that publish to ROS 2 topics

**Steps**:
1. Create `yolo_detector_node.py` subscribing to `/robot/camera/image_raw`
2. Publish detections to `/robot/detections`
3. Launch Isaac Sim with camera
4. Run detector node
5. Monitor output: `ros2 topic echo /robot/detections`

**Success Criteria**:
- ✓ Nodes launch without errors
- ✓ Subscribe to camera topic successfully
- ✓ Publish detection JSON with objects + confidence
- ✓ Latency &lt;200ms (inference + ROS overhead)

## Real Hardware Considerations

### Differences from Simulation

**Simulation (Isaac Sim)**:
- Perfect lighting, no shadows or glare
- Known object dimensions and colors
- No occlusion or overlapping objects
- Camera calibration perfect
- No sensor noise

**Real Hardware**:
- Lighting varies: indoor/outdoor, shadows, reflections
- Objects partially occluded by other objects
- Camera calibration drift over time
- Sensor noise: electrical noise in images
- Object appearance changes: different angles, worn surfaces
- Real-time constraints: must process at robot control rate

### Robustness Strategies

1. **Data augmentation**: Train on varied lighting, angles, scales
2. **Multiple detectors**: Use ensemble of YOLO + other models
3. **Depth validation**: Check depth values for outliers
4. **Temporal filtering**: Smooth detections across frames
5. **Confidence thresholding**: Only trust high-confidence detections
6. **Fallback behaviors**: What to do if vision fails?

### Camera Setup

**For this course (Simulation)**:
- Isaac Sim provides perfect RGB-D cameras
- Configure camera position on humanoid robot

**For real robots**:
- Mount camera on robot wrist or head
- Ensure stable mounting (no vibration)
- Calibrate camera intrinsics once
- Regular cleaning (dust on lens degrades performance)
- Consider wide-angle lens for larger field of view

## Key Takeaways

1. **Object detection (YOLO) is fast and accurate for real-time robotics**
2. **Semantic segmentation gives pixel-level scene understanding**
3. **Depth sensing enables 3D object localization**
4. **ROS 2 nodes can integrate deep learning models easily**
5. **Real hardware has significantly different challenges (lighting, occlusion, noise)**
6. **Confidence thresholding and error handling are critical**

## Next Steps

In **Lesson 4**, you'll integrate vision with **LLM-based planning**:
- Use detected objects as input to LLM
- LLM plans how to manipulate specific objects
- Voice + Vision + Planning together

See you in Lesson 4! 🧠🎬


</PersonalizedLesson>
