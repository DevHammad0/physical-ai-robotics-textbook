---
title: "Lesson 5: Manipulation, Grasping & Motion Planning"
chapter: 4
lesson: 5

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

<PersonalizedLesson lessonPath="03-chapter-4-ai-integration/05-lesson-5-manipulation-grasping.md">

# Lesson 5: Manipulation, Grasping & Motion Planning

## Learning Objectives

By the end of this lesson, you will:
1. Understand grasp planning: how to compute stable grasping strategies for objects
2. Implement motion planning: compute collision-free arm trajectories
3. Integrate grasp + motion planning into cohesive manipulation pipeline
4. Validate grasps for stability and feasibility before execution
5. Handle gripper control with force feedback
6. Understand real hardware challenges in manipulation (friction, slippage, deformation)

## What is Grasp Planning?

**Grasp planning** answers: "How should the robot hold this object?"

```
Input: Detected object (position, shape, material)
       ↓
   Grasp Planner
       ↓
Output: Multiple grasp candidates ranked by quality
        - Gripper position [x, y, z]
        - Gripper orientation (roll, pitch, yaw)
        - Grasp quality score [0.0, 1.0]
        - Required gripper force [Newtons]
        - Stability prediction
```

### Grasp Anatomy

```
       Gripper
         ↓↑
       [==]
       ││││
       ││││  ← Fingers/jaws
       ││││
    ┌──┴┴──┐
    │      │  ← Object
    │  ◯   │     (center of mass here)
    │      │
    └──────┘

Good grasp:
- Center gripper over object center of mass
- Apply force perpendicular to grasp surface
- Avoid unstable contact points
- Minimize torque that could cause rolling

Bad grasp (will fail):
- Contact point far from COM → rotation
- Insufficient contact area → slippage
- Gripper too narrow/wide → can't grip
```

### Grasp Quality Metrics

| Metric | Formula | Interpretation |
|--------|---------|-----------------|
| **Grasp Closure** | Force-closure test | Can gripper prevent any motion? |
| **Margin to Failure** | Minimum friction needed | How robust is grasp? |
| **Contact Stability** | Distance from contact edge | Stability against perturbations |
| **Dexterity** | Condition number of grasp matrix | Can gripper achieve all wrist motions? |

## Motion Planning: Computing Collision-Free Paths

**Motion planning** answers: "What path should the arm take?"

```
       Goal
       ↗
      /
     / (obstacle-free path)
    /
   /
Start ---- (blocked)
  ↓ Can't go straight (obstacle in way)
Obstacle
```

### RRT (Rapidly-Exploring Random Trees) Algorithm

One popular algorithm: **RRT** — rapidly explores workspace

```
1. Start: home position (joint angles)
2. Goal: target position
3. Repeat:
   - Sample random configuration
   - Find nearest node in tree
   - Extend tree toward sample
   - Check collision-free
   - If collision-free, add node
   - If reached goal, return path
4. Smooth path (remove unnecessary waypoints)
```

**Why RRT?**
- ✓ Works in high-dimensional spaces (7-DOF arm)
- ✓ Probabilistically complete (will find path if one exists)
- ✓ Fast (often &lt;1 second)
- ✗ Doesn't guarantee optimal path
- ✗ May find awkward trajectories

## Manipulation Pipeline

```
┌────────────────┐
│ Detected Object│
│ (position, size)│
└────────┬────────┘
         │
         ↓
┌──────────────────────┐
│  Grasp Planner       │
│  • Sample grasps     │
│  • Score each grasp  │
│  • Rank by quality   │
└────────┬─────────────┘
         │
         ↓ GraspCandidate[]
┌──────────────────────────┐
│  Grasp Validator         │
│  • Check gripper limits  │
│  • Check workspace bounds│
│  • Validate stability    │
│  • Rank feasible grasps  │
└────────┬─────────────────┘
         │
         ↓ Best grasp
┌──────────────────────────┐
│  Motion Planner          │
│  • Plan approach path    │
│  • Plan lift trajectory  │
│  • Collision checking    │
└────────┬─────────────────┘
         │
         ↓ Trajectory
┌──────────────────────────┐
│  Trajectory Executor     │
│  • Send to robot control │
│  • Monitor gripper force │
│  • Handle failures       │
└──────────────────────────┘
```

## Grasp Planner Implementation

File: `chapter4_manipulation/src/grasp_planner_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from datetime import datetime
import uuid

class GraspPlannerNode(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')

        # Subscribe to detections
        self.detections_sub = self.create_subscription(
            String,
            '/robot/detections',
            self.detections_callback,
            10
        )

        # Publish grasp candidates
        self.grasps_pub = self.create_publisher(
            String,
            '/robot/grasp_candidates',
            10
        )

        # Gripper configuration
        self.gripper_width = 0.08  # Max opening width (m)
        self.gripper_max_force = 100.0  # Max force (N)
        self.gripper_fingers = 2  # Number of fingers

        # Workspace bounds
        self.workspace_min = np.array([-0.5, -0.5, 0.3])
        self.workspace_max = np.array([1.0, 0.5, 2.0])

        self.get_logger().info('Grasp Planner Node initialized')

    def detections_callback(self, msg):
        """Process detected object and generate grasps"""
        try:
            detections = json.loads(msg.data)
            for detection in detections.get('detections', []):
                grasps = self.plan_grasps(detection)
                if grasps:
                    self._publish_grasps(grasps)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse detections')

    def plan_grasps(self, detection: dict, num_candidates: int = 5) -> list:
        """Generate grasp candidates for detected object"""
        grasps = []

        try:
            # Extract object properties
            bbox = detection.get('bbox', {})
            class_name = detection.get('class', 'unknown')
            confidence = detection.get('confidence', 0.5)

            # Object center (in pixel space)
            pixel_x = (bbox.get('x1', 0) + bbox.get('x2', 0)) / 2.0
            pixel_y = (bbox.get('y1', 0) + bbox.get('y2', 0)) / 2.0
            pixel_z = 0.5  # Assume center depth (would come from depth camera)

            # Estimate 3D position (simplified - would use depth)
            obj_pos_3d = np.array([pixel_x / 640 * 1.5, pixel_y / 480 * 1.0, pixel_z])

            # Generate grasps from multiple angles
            for approach_angle in np.linspace(0, 2*np.pi, num_candidates):
                grasp = self._generate_grasp_candidate(
                    obj_pos_3d, approach_angle, class_name, confidence
                )

                # Validate grasp
                if self._validate_grasp(grasp):
                    quality = self._calculate_grasp_quality(grasp)
                    grasp['grasp_quality'] = quality
                    grasps.append(grasp)

            # Sort by quality (descending)
            grasps.sort(key=lambda g: g['grasp_quality'], reverse=True)

            self.get_logger().info(f'Generated {len(grasps)} valid grasps for {class_name}')
            return grasps

        except Exception as e:
            self.get_logger().error(f'Grasp planning failed: {e}')
            return []

    def _generate_grasp_candidate(self, object_pos: np.ndarray, angle: float,
                                 class_name: str, confidence: float) -> dict:
        """Generate single grasp candidate"""
        # Approach direction (circular around object)
        approach_offset = 0.1  # 10cm away from object
        approach_x = object_pos[0] + approach_offset * np.cos(angle)
        approach_y = object_pos[1] + approach_offset * np.sin(angle)
        approach_z = object_pos[2] + 0.05  # Slightly above

        # Grasp orientation (fingers point down)
        gripper_roll = angle
        gripper_pitch = np.pi / 4  # 45 degrees down
        gripper_yaw = 0.0

        return {
            'grasp_id': str(uuid.uuid4()),
            'object_class': class_name,
            'object_confidence': float(confidence),
            'gripper_position': [approach_x, approach_y, approach_z],
            'gripper_orientation': [gripper_roll, gripper_pitch, gripper_yaw],
            'approach_angle': float(angle),
            'required_force': 50.0  # Default gripper force in Newtons
        }

    def _validate_grasp(self, grasp: dict) -> bool:
        """Check if grasp is physically feasible"""
        pos = np.array(grasp['gripper_position'])

        # Check workspace bounds
        if not np.all(pos >= self.workspace_min) or not np.all(pos <= self.workspace_max):
            return False

        # Check force is reasonable
        if grasp['required_force'] > self.gripper_max_force:
            return False

        # Check gripper width
        if self.gripper_width < 0.02 or self.gripper_width > 0.2:
            return False

        return True

    def _calculate_grasp_quality(self, grasp: dict) -> float:
        """Calculate grasp quality score [0.0, 1.0]"""
        # Combine multiple factors
        quality = 0.5  # Base score

        # Factor 1: Object detection confidence
        obj_conf = grasp.get('object_confidence', 0.5)
        quality += 0.2 * obj_conf

        # Factor 2: Distance to workspace center (grasps in center are more stable)
        workspace_center = (self.workspace_min + self.workspace_max) / 2
        pos = np.array(grasp['gripper_position'])
        distance_to_center = np.linalg.norm(pos - workspace_center)
        max_distance = np.linalg.norm(self.workspace_max - self.workspace_min) / 2
        centering_score = 1.0 - (distance_to_center / max_distance)
        quality += 0.3 * max(0, centering_score)

        # Clamp to [0.0, 1.0]
        return min(1.0, max(0.0, quality))

    def _publish_grasps(self, grasps: list) -> None:
        """Publish grasp candidates"""
        msg = String()
        msg.data = json.dumps({
            'timestamp': datetime.now().isoformat(),
            'grasps': grasps,
            'num_candidates': len(grasps)
        })
        self.grasps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    grasp_planner = GraspPlannerNode()
    rclpy.spin(grasp_planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Motion Planning with RRT

File: `chapter4_manipulation/src/motion_planner_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from datetime import datetime
import uuid
import time

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # Subscribe to grasps
        self.grasps_sub = self.create_subscription(
            String,
            '/robot/grasp_candidates',
            self.grasps_callback,
            10
        )

        # Publish trajectories
        self.trajectory_pub = self.create_publisher(
            String,
            '/robot/planned_trajectory',
            10
        )

        # Robot configuration
        self.num_joints = 7
        self.home_config = [0.0] * 7  # Neutral joint angles

        # Motion planning parameters
        self.rrt_iterations = 1000
        self.step_size = 0.1
        self.planning_timeout = 5.0

        self.get_logger().info('Motion Planner Node initialized')

    def grasps_callback(self, msg):
        """Process grasp candidates and plan motion"""
        try:
            grasps_data = json.loads(msg.data)
            for grasp in grasps_data.get('grasps', []):
                trajectory = self.plan_trajectory(grasp)
                if trajectory:
                    self._publish_trajectory(trajectory)
                    return  # Use first valid trajectory
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse grasps')

    def plan_trajectory(self, grasp: dict) -> dict:
        """Plan collision-free trajectory to grasp"""
        try:
            # Simplified RRT implementation
            start = self.home_config
            goal_pos = grasp['gripper_position']

            # Plan approach phase
            approach_path = self._plan_rrt(start, goal_pos)
            if not approach_path:
                self.get_logger().warn('Failed to plan approach trajectory')
                return None

            # Create complete trajectory (approach + grasp + retract)
            trajectory = {
                'trajectory_id': str(uuid.uuid4()),
                'phases': [
                    {'name': 'approach', 'waypoints': approach_path},
                    {'name': 'grasp', 'waypoints': [approach_path[-1]]},  # Close gripper
                    {'name': 'retract', 'waypoints': [self.home_config]}   # Return home
                ],
                'estimated_duration': len(approach_path) * 0.5,  # ~0.5s per waypoint
                'collision_free': True
            }

            return trajectory

        except Exception as e:
            self.get_logger().error(f'Motion planning failed: {e}')
            return None

    def _plan_rrt(self, start: list, goal: list, max_iterations: int = 1000) -> list:
        """Plan path using RRT algorithm"""
        tree = [start]
        goal_reached = False

        for iteration in range(max_iterations):
            # Sample random configuration
            if np.random.rand() < 0.1:  # 10% goal bias
                random_config = goal
            else:
                random_config = [np.random.uniform(-np.pi, np.pi) for _ in range(self.num_joints)]

            # Find nearest node in tree
            nearest = self._nearest_node(tree, random_config)

            # Extend toward random sample
            new_node = self._extend(nearest, random_config)

            # Check collision-free
            if self._is_collision_free(nearest, new_node):
                tree.append(new_node)

                # Check if goal reached
                if self._distance(new_node, goal) < 0.1:
                    goal_reached = True
                    break

        if goal_reached:
            # Reconstruct path (simplified)
            return tree[-20:]  # Return last 20 nodes as path
        else:
            self.get_logger().warn(f'RRT failed to find path after {max_iterations} iterations')
            return None

    def _nearest_node(self, tree: list, target: list) -> list:
        """Find nearest node in tree to target"""
        return min(tree, key=lambda node: self._distance(node, target))

    def _extend(self, start: list, target: list) -> list:
        """Extend from start toward target"""
        direction = np.array(target) - np.array(start)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm
        return (np.array(start) + self.step_size * direction).tolist()

    def _distance(self, config1: list, config2: list) -> float:
        """Euclidean distance between configurations"""
        return np.linalg.norm(np.array(config1) - np.array(config2))

    def _is_collision_free(self, config1: list, config2: list) -> bool:
        """Check if motion is collision-free (simplified)"""
        # In real implementation, would check against scene geometry
        # For now, always return True (simulation assumes no obstacles)
        return True

    def _publish_trajectory(self, trajectory: dict) -> None:
        """Publish planned trajectory"""
        msg = String()
        msg.data = json.dumps(trajectory)
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'Published trajectory with {len(trajectory["phases"])} phases')


def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlannerNode()
    rclpy.spin(motion_planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Force Control & Gripper

```python
class GripperController:
    """Control gripper with force feedback"""

    def __init__(self, max_force=100.0, finger_width=0.08):
        self.max_force = max_force
        self.finger_width = finger_width
        self.current_force = 0.0
        self.current_width = finger_width

    def open(self):
        """Open gripper fully"""
        self.current_width = self.finger_width
        self.current_force = 0.0

    def close(self, target_force: float):
        """Close gripper to target force"""
        if target_force > self.max_force:
            self.get_logger().warn(f'Force {target_force}N exceeds limit {self.max_force}N')
            target_force = self.max_force

        # Simulate closing until force reached
        self.current_force = target_force
        self.current_width = max(0.0, self.finger_width - 0.04)  # Close to ~4cm

    def adjust_force(self, delta_force: float):
        """Adjust gripper force by delta"""
        self.current_force = max(0, min(self.max_force, self.current_force + delta_force))

    def get_state(self) -> dict:
        return {
            'current_force': self.current_force,
            'current_width': self.current_width,
            'is_open': self.current_width > 0.05
        }
```

## Exercises

### Exercise L5-1: Grasp Planning

**Objective**: Understand grasp quality metrics and planning

**Steps**:
1. Load an image of an object in the workspace
2. Manually identify:
   - Good grasp points (center of mass, stable contact)
   - Bad grasp points (edge, unstable contact)
3. Implement grasp candidate sampling (at least 6 angles)
4. Score each grasp by stability
5. Rank and visualize top 3 grasps

**Success Criteria**:
- ✓ Generate ≥6 grasp candidates
- ✓ Quality scores reflect stability
- ✓ Identify which grasps are feasible
- ✓ Can explain why top grasp is best

### Exercise L5-2: RRT Motion Planning

**Objective**: Implement collision-free path planning

**Steps**:
1. Create 2D workspace with obstacles
2. Implement RRT algorithm (pseudo-code provided)
3. Plan path from start to goal
4. Visualize tree and final path
5. Measure planning time and path quality

**Success Criteria**:
- ✓ RRT finds collision-free path
- ✓ Planning time &lt;1 second
- ✓ Path is continuous and smooth
- ✓ Can adjust parameters (iterations, step size)

## Real Hardware Considerations

### Differences from Simulation

**Simulation**:
- Perfect physics (same object → same grasp success rate)
- No slip or deformation
- Gripper force perfectly calibrated

**Real Hardware**:
- Object friction unknown (can slip unexpectedly)
- Gripper jaws deform under load
- Sensor calibration drift over time
- Unexpected object properties (wet surface, soft material)
- Collision detection imperfect

### Handling Grasp Failures

On real robots, grasps fail regularly. Solutions:

1. **Tactile Feedback**: Monitor gripper force to detect slip
2. **Vision-based Validation**: Check if object moved as expected
3. **Replanning**: If grasp fails, try different grasp
4. **Adaptive Force**: Increase gripper force if object slips

```python
def execute_grasp_with_recovery(grasp_candidates):
    for grasp in grasp_candidates:
        try:
            # Plan trajectory
            trajectory = motion_planner.plan(grasp)

            # Execute approach
            execute_trajectory(trajectory['approach'])

            # Close gripper
            gripper.close(force=grasp['required_force'])

            # Check if grasp stable
            if gripper.detect_slip():
                gripper.increase_force(10)  # Add 10N

            # Lift
            execute_trajectory(trajectory['lift'])

            # Success!
            return grasp

        except GraspFailure:
            continue  # Try next grasp

    raise Exception("All grasps failed")
```

## Key Takeaways

1. **Grasp planning** generates multiple candidates ranked by stability
2. **Motion planning** (RRT) computes collision-free paths
3. **Validation** is essential before execution
4. **Real robots** need force feedback and failure recovery
5. **Replanning** on failure is standard practice

## Next Steps

In **Lesson 6**, you'll integrate grasp planning + motion planning + LLM planning into a **unified orchestration system** that ties everything together.

See you in Lesson 6! 🎯


</PersonalizedLesson>
