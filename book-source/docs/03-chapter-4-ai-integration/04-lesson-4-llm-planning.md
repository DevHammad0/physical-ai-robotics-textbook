# Lesson 4: LLM-Driven Planning & Task Decomposition

## Learning Objectives

By the end of this lesson, you will:
1. Understand how large language models (LLMs) decompose high-level goals into executable robot tasks
2. Use prompt engineering to guide LLM planning (constraints, format, examples)
3. Implement a ROS 2 planning node that integrates GPT-4 or Ollama
4. Validate plans against safety constraints (workspace, joint limits, gripper feasibility)
5. Handle LLM errors gracefully and implement fallback strategies
6. Understand real hardware planning challenges (uncertainty, replanning, learning from failures)

## What is LLM-Based Planning?

Traditional robot programming requires humans to specify every movement:
```python
# Traditional: Hard-coded sequence
robot.move_to([0.5, 0.3, 0.8])  # Approach cube
robot.open_gripper()
robot.move_to([0.5, 0.3, 0.6])  # Close on cube
robot.close_gripper()
```

**LLM-based planning** lets the robot understand natural language and figure out the steps:
```
User: "Pick up the red cube and place it on the shelf"
     ↓
LLM Planner
     ↓
Output: [
  "detect red cube in scene",
  "approach cube center at pixel location",
  "open gripper",
  "move arm down to object",
  "close gripper with appropriate force",
  "lift object to 0.8m height",
  "move to shelf location",
  "place on shelf",
  "open gripper"
]
     ↓
Executor runs each step
```

### Why LLMs for Planning?

1. **Flexibility**: Same system handles diverse tasks without code changes
2. **Interpretability**: Can explain reasoning (why approach from this angle?)
3. **Generalization**: Training on human reasoning helps handle novel situations
4. **Error Recovery**: Can replan when execution fails
5. **Constraint Awareness**: Can incorporate safety rules into planning

### LLM Limitations (Why We Still Need Validation)

1. **Hallucination**: LLM might suggest physically impossible actions
2. **Real-time blindness**: LLM doesn't see actual robot state, only initial perception
3. **Cost**: API calls to GPT-4 are expensive ($0.03-0.10 per request)
4. **Latency**: API calls take 2-3 seconds (vs. &lt;100ms for traditional planning)
5. **Determinism**: Same input may generate different outputs

## LLM Planning Architecture

```
┌─────────────────────────────────────────────────────┐
│  Voice Command: "Pick up red cube"                  │
└────────────────┬────────────────────────────────────┘
                 │
                 ↓
        ┌────────────────────┐
        │  Input Validation  │
        │ • Transcription OK?│
        │ • Confidence ≥0.7? │
        └────────┬───────────┘
                 │
                 ↓
        ┌────────────────────────────┐
        │  Perception Integration    │
        │ • Detected objects: [...]  │
        │ • Scene segmentation       │
        │ • Estimated positions      │
        └────────┬───────────────────┘
                 │
                 ↓
    ┌────────────────────────────────────┐
    │  Build LLM Prompt                  │
    │ • System prompt (robot capabilities)│
    │ • Task description                 │
    │ • Detected objects                 │
    │ • Output format (JSON)             │
    │ • Constraints (workspace, limits)  │
    │ • Few-shot examples                │
    └────────┬─────────────────────────────┘
             │
             ↓
    ┌─────────────────────────────────┐
    │  LLM Planning Service           │
    │  (GPT-4 or Ollama)              │
    │  • Send prompt                  │
    │  • Parse JSON response          │
    │  • Extract action sequence      │
    └────────┬────────────────────────┘
             │
             ↓
    ┌─────────────────────────────────┐
    │  Plan Validation                │
    │ • Workspace constraints OK?     │
    │ • Joint limits OK?              │
    │ • Gripper feasible?             │
    │ • Physics plausible?            │
    └────────┬────────────────────────┘
             │ (Pass)
             ↓
    ┌─────────────────────────────────┐
    │  Task Plan Output               │
    │ TaskPlan {                      │
    │  task_id: "uuid",              │
    │  subtasks: [...],              │
    │  is_feasible: true             │
    │ }                               │
    └─────────────────────────────────┘
```

## LLM Selection: OpenAI vs. Ollama

### Option 1: OpenAI GPT-4 (Recommended for accuracy)

**Advantages**:
- Highest quality reasoning and instruction following
- Handles complex decomposition
- ~90%+ success rate on robot tasks
- Automatically improves as OpenAI updates model

**Disadvantages**:
- Requires API key + internet connection
- Costs $0.03-0.10 per request (~$3-10 per 100 tasks)
- 2-3 second latency (slower than local)
- Privacy: requests sent to OpenAI servers

**Installation**:
```bash
pip install openai
export OPENAI_API_KEY="sk-..."  # Get from https://platform.openai.com/api-keys
```

### Option 2: Ollama (Open-source, Local)

**Advantages**:
- Free, runs locally on your machine
- No API keys required
- Private: data never leaves your computer
- No per-request cost

**Disadvantages**:
- Lower quality reasoning than GPT-4
- Requires 4-8GB VRAM and modern GPU
- Slower inference (5-10 seconds per plan)
- Need to download large model (~7GB)

**Installation**:
```bash
# Download Ollama from https://ollama.ai
ollama pull mistral  # ~5GB, good reasoning model
# or: ollama pull llama2  # Alternative
```

**For this course**: We recommend **GPT-4** for better reliability, but Ollama works fine for learning.

## Prompt Engineering for Robot Planning

The key to good LLM planning is a **well-structured prompt**. Here's the architecture:

```
[System Prompt]
You are a robot planning system. Your job is to decompose high-level
commands into executable robot actions. You must respect constraints.

[Robot Capabilities]
- Can detect objects via camera (YOLO-v8)
- Can move arm to any workspace position
- Can open/close gripper with force control
- Cannot reach outside workspace [-0.5, -0.5, 0.3] to [1.0, 0.5, 2.0]
- Maximum gripper force: 100N

[Current Perception]
- Detected objects: red cube at [0.5, 0.3, 0.1], blue sphere at [0.3, 0.2, 0.05]
- Segmentation: table (0.1-0.8m height), shelf (1.2-1.8m height)
- Estimated 3D positions: [x, y, z] in meters

[Task]
User said: "Pick up the red cube and place it on the shelf"

[Output Format]
Return valid JSON only, no other text:
{
  "task_id": "uuid",
  "task_description": "...",
  "subtasks": ["step 1", "step 2", ...],
  "constraints_respected": true,
  "reasoning": "Why these steps?"
}

[Examples]
Example 1: "Find the red cube"
{
  "subtasks": ["scan room with camera", "detect red objects",
               "locate red cube in 3D space", "report position"],
  "constraints_respected": true
}

[Constraints]
- Each subtask must be < 2 sentences
- No reaching outside workspace
- Gripper force must be ≤100N
- Cannot manipulate objects simultaneously
```

## ROS 2 LLM Planner Node

File: `chapter4_planning/src/llm_planner_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime
import uuid

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Load configuration
        self.use_openai = self.declare_parameter(
            'use_openai', True
        ).value

        if self.use_openai:
            try:
                import openai
                openai.api_key = os.getenv('OPENAI_API_KEY')
                self.openai_client = openai
                self.get_logger().info('Using OpenAI GPT-4 for planning')
            except ImportError:
                self.get_logger().error('OpenAI library not installed. Install with: pip install openai')
                raise
        else:
            try:
                import ollama
                self.ollama_client = ollama
                self.get_logger().info('Using Ollama (local) for planning')
            except ImportError:
                self.get_logger().error('Ollama library not installed')
                raise

        # Subscribe to voice commands and detections
        self.voice_sub = self.create_subscription(
            String,
            '/robot/voice_command',
            self.voice_callback,
            10
        )

        self.detections_sub = self.create_subscription(
            String,
            '/robot/detections',
            self.detections_callback,
            10
        )

        # Publish plans
        self.plan_pub = self.create_publisher(
            String,
            '/robot/task_plan',
            10
        )

        # Store last detection data
        self.last_detections = None

        # Configuration
        self.workspace_min = [-0.5, -0.5, 0.3]
        self.workspace_max = [1.0, 0.5, 2.0]
        self.max_gripper_force = 100.0

        self.get_logger().info('LLM Planner Node initialized')

    def voice_callback(self, msg):
        """Process voice command and generate plan"""
        command = msg.data
        self.get_logger().info(f'Planning for command: "{command}"')

        # Generate plan
        plan = self.generate_plan(command, self.last_detections)

        if plan:
            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            self.get_logger().info(f'Published plan with {len(plan["subtasks"])} subtasks')
        else:
            self.get_logger().error('Failed to generate plan')

    def detections_callback(self, msg):
        """Store latest detection data for planning context"""
        try:
            self.last_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to parse detections JSON')

    def generate_plan(self, command: str, detections=None) -> dict:
        """Generate task plan using LLM"""
        try:
            # Build detection context
            detection_str = self._format_detections(detections)

            # Build prompt
            system_prompt = self._get_system_prompt()
            user_prompt = f"""
Task: {command}

Current Perception:
{detection_str}

Workspace bounds: {self.workspace_min} to {self.workspace_max}
Max gripper force: {self.max_gripper_force}N

Generate a JSON plan with subtasks.
"""

            # Call LLM
            if self.use_openai:
                response = self.openai_client.ChatCompletion.create(
                    model="gpt-4",
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=0.3,
                    max_tokens=500
                )
                response_text = response['choices'][0]['message']['content']
            else:
                response = self.ollama_client.generate(
                    model='mistral',
                    prompt=f"{system_prompt}\n{user_prompt}",
                    stream=False
                )
                response_text = response['response']

            # Parse response
            plan = self._parse_plan_response(response_text, command)

            # Validate plan
            is_valid, reason = self._validate_plan(plan)
            plan['is_feasible'] = is_valid
            plan['feasibility_reason'] = reason if not is_valid else "OK"

            return plan

        except Exception as e:
            self.get_logger().error(f'LLM planning failed: {e}')
            return None

    def _get_system_prompt(self) -> str:
        """Get system prompt for LLM"""
        return """You are a robot planning system. Your job is to decompose
high-level commands into executable robot actions.

Robot Capabilities:
- Can detect objects via YOLO-v8 camera
- Can move arm to any position in workspace
- Can grasp and place objects
- Can move at safe speeds to avoid collisions

Output exactly this JSON format, no other text:
{
  "task_id": "uuid-string",
  "task_description": "Human-readable task",
  "subtasks": ["step 1", "step 2", ...],
  "estimated_duration": 15.5,
  "reasoning": "Why these steps?"
}

Keep each subtask simple (<2 sentences).
Respect workspace boundaries and force limits.
"""

    def _format_detections(self, detections) -> str:
        """Format detection data for LLM context"""
        if not detections:
            return "No objects detected"

        try:
            lines = ["Detected objects:"]
            for det in detections.get('detections', []):
                lines.append(f"  - {det['class']}: confidence {det['confidence']:.2f}, "
                           f"at [{det['bbox']['x1']}, {det['bbox']['y1']}]")
            return "\n".join(lines)
        except (KeyError, TypeError):
            return "Could not parse detections"

    def _parse_plan_response(self, response_text: str, command: str) -> dict:
        """Parse LLM response into plan structure"""
        try:
            # Extract JSON from response (LLM might add extra text)
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            if json_start >= 0 and json_end > json_start:
                json_text = response_text[json_start:json_end]
                plan = json.loads(json_text)
            else:
                raise ValueError("No JSON found in response")

            # Ensure required fields
            if 'task_id' not in plan:
                plan['task_id'] = str(uuid.uuid4())
            if 'task_description' not in plan:
                plan['task_description'] = command
            if 'subtasks' not in plan:
                plan['subtasks'] = []
            if 'estimated_duration' not in plan:
                plan['estimated_duration'] = 30.0

            return plan

        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f'Failed to parse plan: {e}, using default')
            return {
                'task_id': str(uuid.uuid4()),
                'task_description': command,
                'subtasks': ['execute user command'],
                'estimated_duration': 30.0,
                'reasoning': 'Failed to parse LLM response'
            }

    def _validate_plan(self, plan: dict) -> tuple:
        """Validate plan against constraints"""
        # Check for reasonable number of subtasks
        if len(plan.get('subtasks', [])) > 20:
            return False, "Too many subtasks (>20)"

        # Check estimated duration
        duration = plan.get('estimated_duration', 0)
        if duration > 120:
            return False, "Estimated duration too long (>120s)"
        if duration < 1:
            return False, "Estimated duration too short (<1s)"

        # Check for obviously unsafe subtasks
        unsafe_keywords = ['explode', 'destroy', 'hurt', 'dangerous', 'unsafe']
        for subtask in plan.get('subtasks', []):
            if any(keyword in subtask.lower() for keyword in unsafe_keywords):
                return False, f"Unsafe subtask detected: {subtask}"

        return True, "OK"


def main(args=None):
    rclpy.init(args=args)
    planner_node = LLMPlannerNode()
    rclpy.spin(planner_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Prompt Templates for Different Tasks

### Template 1: Object Manipulation

```python
OBJECT_MANIPULATION_PROMPT = """
Task: Pick up the {color} {object_type} and place it on the {location}

Current scene:
- {objects_list}

Robot workspace: {workspace}
Available actions:
  - detect_object(color, type) -> 3D position
  - move_arm(target_position)
  - open_gripper()
  - close_gripper(force)
  - check_collision()

Plan steps ensuring:
1. Detect target object
2. Plan collision-free path
3. Approach from stable angle
4. Grasp with appropriate force
5. Lift safely
6. Move to destination
7. Place object
8. Verify success
"""
```

### Template 2: Scene Exploration

```python
SCENE_EXPLORATION_PROMPT = """
Task: Explore the environment and categorize all objects

Available cameras: RGB + depth
Available sensors: gripper force/torque

Generate plan to:
1. Scan environment from multiple angles
2. Build 3D map of scene
3. Classify objects by type
4. Estimate object properties (size, material)
5. Identify stable surfaces
6. Report findings
"""
```

### Template 3: Collaborative Tasks

```python
COLLABORATIVE_PROMPT = """
Task: {user_instruction}

Available resources:
- Robot 1: 7-DOF arm, parallel gripper
- Robot 2: Mobile base + arm

Coordinate actions to:
1. Distribute subtasks between robots
2. Avoid collisions
3. Synchronize timing
4. Report completion status
"""
```

## Exercises

### Exercise L4-1: Prompt Engineering

**Objective**: Understand how prompts affect plan quality

**Steps**:
1. Write 3 different prompts for "pick up the cube"
   - Prompt A: Minimal (just the task)
   - Prompt B: With constraints (workspace, force limits)
   - Prompt C: With few-shot examples (show good/bad plans)
2. Test each prompt 3 times
3. Compare output quality (specificity, safety, feasibility)
4. Which prompt produces best results?

**Success Criteria**:
- ✓ Can articulate how constraints affect planning
- ✓ Understand prompt structure and components
- ✓ Recognize hallucinations and how to prevent them

### Exercise L4-2: Implement Planning Node

**Objective**: Create LLM planner that integrates with voice and perception

**Steps**:
1. Implement `llm_planner_node.py` from this lesson
2. Set OpenAI API key: `export OPENAI_API_KEY="sk-..."`
3. Launch Whisper + Vision nodes (Lessons 2-3)
4. Launch planner node: `ros2 run chapter4_planning llm_planner_node`
5. Say voice command: "Pick up the red cube"
6. Verify plan published to `/robot/task_plan`
7. Check plan JSON for reasonable subtasks

**Success Criteria**:
- ✓ Node launches without errors
- ✓ Voice command parsed correctly
- ✓ Plan generated within 3 seconds
- ✓ Plan is valid JSON with subtasks
- ✓ Constraints validated (workspace, force)

## Real Hardware Considerations

### Differences from Simulation

**Simulation (Isaac Sim)**:
- Perfect knowledge of object positions (no estimation error)
- Deterministic physics (same action → same result every time)
- No latency or communication delays
- LLM planning overhead acceptable (3s latency invisible in sim)

**Real Hardware**:
- Object positions uncertain (camera calibration error ~5cm)
- Physics uncertain (friction, material properties unknown)
- Network latency and communication failures possible
- Humans expect near-instant response (long planning = frustrating)
- Object appearance varies (lighting, wear, dirt)

### Real Hardware Strategies

1. **Faster LLMs**: Use local Ollama instead of GPT-4 API (saves 2-3s latency)
2. **Caching**: Store common plans, avoid re-planning identical tasks
3. **Incremental planning**: Plan just the next step, not entire task
4. **Learning from failures**: Log failed plans, use to improve prompts
5. **Human oversight**: Have humans approve critical plans before execution
6. **Uncertainty quantification**: Track confidence in each subtask

### Simulation-to-Reality Transfer

The biggest challenge: **LLM planning works differently on real robots**

- **In simulation**: "Pick up cube at [0.5, 0.3, 0.1]" → always works if coordinates valid
- **On real robot**: "Pick up cube" → might fail if camera calibration off, gripper slips, cube rolls

**Solution**: Implement **replanning on failure**
```python
while not task_complete:
    plan = llm_planner.plan(goal, current_detections)
    try:
        executor.execute(plan)
        task_complete = True
    except ExecutionFailure as e:
        # Update detections, replan
        current_detections = vision_system.get_latest()
        feedback = f"Previous plan failed: {e}. Replanning..."
```

## Key Takeaways

1. **LLMs can decompose high-level goals into executable plans**
2. **Prompt engineering is critical** — constraints and examples matter
3. **Plans must be validated** — LLMs hallucinate, we check physics/constraints
4. **Caching and local models** reduce latency on real robots
5. **Real robots need replanning** — execution failures are common
6. **Human oversight** essential for safety-critical applications

## Next Steps

In **Lesson 5**, you'll implement the **manipulation pipeline**:
- Grasp planning: "How should the robot hold this object?"
- Motion planning: "What path should the arm take?"
- Force control: "How hard should the gripper squeeze?"

Combining planning (Lesson 4) + manipulation (Lesson 5) = autonomous grasping!

See you in Lesson 5! 🤖✋
