---
title: "آپ کا پہلا ROS 2 Publisher"
chapter: 1
lesson: 4
duration_minutes: 60
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "B1"
hardware_prerequisites: []
learning_objectives:
  - "rclpy.Node کا استعمال کرتے ہوئے ایک ROS 2 node class بنائیں"
  - "ایک publisher نافذ کریں اور topic پر پیغامات publish کریں"
  - "وقتاً فوقتاً پیغامات کی اشاعت (publishing) کو متحرک کرنے کے لیے timer callbacks کا استعمال کریں"
  - "Python میں ایک مکمل، چلنے کے قابل ROS 2 node لکھیں"
  - "ROS 2 CLI tools کا استعمال کرتے ہوئے publisher کے مسائل کو debug کریں"
  - "ROS 2 nodes میں error handling اور logging کا اطلاق کریں"
primary_layer: "Layer 1 (Manual Foundation) + Layer 2 (AI Collaboration)"
---

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/04-publisher.md">

## تعارف (Introduction)

اب آپ کا پہلا ROS 2 code لکھنے کا وقت ہے! اس سبق میں، آپ ایک Python node بنائیں گے جو Turtlesim کو velocity commands (رفتار کے احکامات) publish کرے گا، جس سے turtle حرکت کرے گا۔

**آپ کیا بنائیں گے**: ایک `SimpleTurtlePublisher` node جو:
- `/turtle1/cmd_vel` ٹاپک پر ایک publisher بناتا ہے
- 10 Hz پر `Twist` پیغامات (velocity commands) بھیجتا ہے
- Turtle کو غیر معینہ مدت تک آگے بڑھاتا ہے
- شائستہ بندش (graceful shutdown) کو سنبھالتا ہے

**Simulation environment**: Turtlesim (سبق 2 سے)

**وقت کا تخمینہ**: 60 منٹ (چلانے اور debugging سمیت)

آخر تک، آپ سمجھ جائیں گے:
- ROS 2 node class کی ساخت کیسے بنائی جائے
- Publishers کیسے کام کرتے ہیں (create, initialize, publish)
- Timer callbacks کیسے متواتر اشاعت (periodic publishing) کو فعال کرتے ہیں
- `ros2 topic echo` اور `ros2 topic hz` کا استعمال کرتے ہوئے debug کیسے کریں

---

## بنیادی علم کا جائزہ

Code میں غوطہ لگانے سے پہلے، آئیے اسباق 1-3 کے اہم تصورات کا جائزہ لیتے ہیں:

### Node Structure (سبق 3 سے)

ہر ROS 2 node:
1. `rclpy.node.Node` سے وراثت (inherits) حاصل کرتا ہے
2. اس کا ایک `__init__` طریقہ کار (method) ہوتا ہے جو publishers، subscribers، اور timers کو سیٹ اپ کرتا ہے
3. واقعات (timers, subscriptions) کو سنبھالنے کے لیے callback methods ہوتے ہیں
4. `rclpy.init()` کے ساتھ initialize ہوتا ہے
5. `rclpy.spin(node)` کے ساتھ چلتا ہے

### Publisher API (نیا)

ایک publisher، node constructor میں بنایا جاتا ہے:

```python
self.publisher = self.create_publisher(
    msg_type,          # e.g., geometry_msgs.msg.Twist
    topic_name,        # e.g., '/turtle1/cmd_vel'
    queue_size         # e.g., 10 (max queued messages)
)
````

پیغام publish کرنا:

```python
msg = Twist()          # Create message
msg.linear.x = 1.0     # Set fields
self.publisher.publish(msg)  # Send it
```

### Timer Callbacks (نیا)

متواتر کاموں کے لیے، ایک timer بنائیں:

```python
self.timer = self.create_timer(
    0.1,                    # Period in seconds (10 Hz = 0.1s)
    self.timer_callback    # Function to call periodically
)
```

Callback ہر 0.1 سیکنڈ میں چلتا ہے:

```python
def timer_callback(self):
    msg = Twist()
    msg.linear.x = 1.0
    self.publisher.publish(msg)
```

### Message Types (سبق 3 سے)

`Twist` پیغام velocity کی نمائندگی کرتا ہے:

```
geometry_msgs.msg.Twist:
  linear:
    x: float (m/s)       # forward/backward
    y: float (m/s)       # sideways
    z: float (m/s)       # vertical
  angular:
    x: float (rad/s)     # roll
    y: float (rad/s)     # pitch
    z: float (rad/s)     # yaw (turning)
```

Turtlesim کے لیے:

  - آگے (مثبت) یا پیچھے (منفی) جانے کے لیے `linear.x` سیٹ کریں
  - مڑنے کے لیے `angular.z` سیٹ کریں (مثبت = گھڑی کی مخالف سمت/counter-clockwise)

-----

## تصور 1: ROS 2 کے لیے Python Package Structure

ROS 2 nodes **پیکیجز (packages)** میں رہتے ہیں۔ ایک پیکیج ایک ڈائریکٹری ہے جس میں آپ کے node کے بارے میں میٹا ڈیٹا ہوتا ہے۔

### ڈائریکٹری کا ڈھانچہ (Directory Structure)

```
ros2_workspace/
├── src/
│   └── my_first_package/
│       ├── my_first_package/
│       │   ├── __init__.py
│       │   └── publisher_node.py      # Your node code
│       ├── setup.py
│       ├── setup.cfg
│       ├── package.xml
│       └── resource/
│           └── my_first_package
└── install/
└── build/
```

**اہم فائلیں**:

  - `package.xml`: پیکیج میٹا ڈیٹا (نام، ورژن، dependencies، maintainer)
  - `setup.py`: Python پیکیج کنفیگریشن (entry points، پیکیج ڈیٹا)
  - `setup.cfg`: Python پیکیج میٹا ڈیٹا
  - `publisher_node.py`: آپ کا اصل node code

ہم ایک لمحے میں یہ ڈھانچہ بنائیں گے۔ ابھی کے لیے، سمجھ لیں کہ ROS 2 اس ترتیب (layout) کی توقع کرتا ہے۔

-----

## تصور 2: The rclpy.Node Base Class

ہر ROS 2 node `rclpy.node.Node` سے وراثت (inherits) حاصل کرتا ہے۔ آئیے اہم طریقوں (methods) کو سمجھیں:

### ایک Node Class بنانا

```python
from rclpy.node import Node

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')  # Node name

        # Create publishers, subscribers, timers here
        self.publisher = self.create_publisher(...)
        self.timer = self.create_timer(...)
```

`__init__` طریقہ:

1.  Node کے نام کے ساتھ `super().__init__()` کو call کرتا ہے
2.  Publishers، subscribers، parameters، timers کو سیٹ اپ کرتا ہے
3.  ابھی تک spin loop میں داخل نہیں ہوتا

### Node میں کلیدی طریقے (Key Methods)

| Method | مقصد (Purpose) | Example |
|---|---|---|
| `create_publisher()` | Publisher بنائیں | `self.create_publisher(Twist, '/cmd_vel', 10)` |
| `create_subscription()` | Subscriber بنائیں | `self.create_subscription(Image, '/camera/image', callback, 10)` |
| `create_timer()` | Periodic callback شیڈول کریں | `self.create_timer(0.1, self.callback)` |
| `get_logger()` | اس node کے لیے logger حاصل کریں | `self.get_logger().info("message")` |
| `declare_parameter()` | ایک قابل ترتیب (configurable) parameter کا اعلان کریں | `self.declare_parameter('speed', 1.0)` |

-----

## تصور 3: Timer Callbacks اور Periodic Publishing

### Timers کو سمجھنا

ایک timer باقاعدہ وقفوں پر ایک فنکشن کو call کرتا ہے:

```python
# Create timer: call self.timer_callback() every 0.1 seconds (10 Hz)
self.timer = self.create_timer(0.1, self.timer_callback)

def timer_callback(self):
    msg = Twist()
    msg.linear.x = 1.0
    self.publisher.publish(msg)
```

### فریکوئنسی بمقابلہ پیریڈ (Frequency vs. Period)

  - **Frequency** (Hz): فی سیکنڈ پیغامات
      - 10 Hz = 10 پیغامات/سیکنڈ
      - 1 Hz = 1 پیغام/سیکنڈ
      - 30 Hz = 30 پیغامات/سیکنڈ

  - **Period** (سیکنڈز): پیغامات کے درمیان وقت
      - 10 Hz → period = 1/10 = 0.1 سیکنڈ
      - 1 Hz → period = 1/1 = 1.0 سیکنڈ
      - 30 Hz → period = 1/30 ≈ 0.033 سیکنڈ

**تبدیلی (Conversion)**: `period = 1.0 / frequency`

### Periodic Publishing کیوں؟

Sensor اور command ڈیٹا کو مسلسل بھیجنے کی ضرورت ہوتی ہے:

  - **Sensor data** (30 Hz): Camera images, lidar scans
  - **Control commands** (10-50 Hz): Motor commands, velocity
  - **Status updates** (1 Hz): بیٹری لیول، درجہ حرارت

اگر آپ ایک بار publish کریں اور رک جائیں، تو subscriber ایک پیغام دیکھتا ہے اور پھر کچھ نہیں۔ متواتر اشاعت (periodic publishing) مسلسل ڈیٹا کے بہاؤ کو یقینی بناتی ہے۔

-----

## Code Example 1: Simple Publisher (کم از کم)

آئیے اپنا پہلا node بنائیں۔ تصورات کو سمجھنے کے لیے یہ بالکل کم سے کم ہے۔

### Node فائل بنائیں

یہاں ایک فائل بنائیں: `publisher_node.py`

```python
"""
Simple ROS 2 Publisher Node
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleTurtlePublisher(Node):
    """Publishes velocity commands to Turtlesim at 10 Hz."""

    def __init__(self):
        super().__init__('simple_turtle_publisher')

        # Create publisher on /turtle1/cmd_vel topic
        self.publisher = self.create_publisher(
            Twist,                  # Message type
            '/turtle1/cmd_vel',     # Topic name
            10                      # Queue size
        )

        # Create timer to call callback every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Publishes a Twist message every 0.1 seconds."""
        msg = Twist()
        msg.linear.x = 1.0      # 1 m/s forward
        msg.angular.z = 0.0     # No rotation

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**کل لائنیں**: 38 | **استعمال شدہ تصورات**: 5

### کوڈ چلانا

دو terminals کھولیں:

**Terminal 1: Turtlesim لانچ کریں**

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2: اپنا publisher چلائیں**

```bash
source /opt/ros/humble/setup.bash
cd path/to/publisher_node.py
python3 publisher_node.py
```

**متوقع رویہ**: Turtle مسلسل سیدھی لائن میں آگے بڑھتا ہے۔

### ROS 2 Tools کے ساتھ تصدیق کرنا

ایک **تیسرا terminal** کھولیں اور تصدیق کریں کہ publisher کام کر رہا ہے:

```bash
source /opt/ros/humble/setup.bash

# Check topic exists
ros2 topic list | grep cmd_vel

# Watch messages
ros2 topic echo /turtle1/cmd_vel

# Check publish frequency
ros2 topic hz /turtle1/cmd_vel
```

**متوقع آؤٹ پٹ**:

```
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

اور فریکوئنسی \~10 Hz ظاہر ہونی چاہیے۔

-----

## Code Example 2: Publisher with Logging

حقیقی دنیا کے nodes اپنی سرگرمی کو log کرتے ہیں۔ آئیے publisher میں logging شامل کریں:

```python
"""
ROS 2 Publisher with Logging
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class LoggingTurtlePublisher(Node):
    """Publishes velocity commands with logging output."""

    def __init__(self):
        super().__init__('logging_turtle_publisher')

        # Get logger for this node
        self.get_logger().info('LoggingTurtlePublisher initializing...')

        # Create publisher
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Counter for messages
        self.msg_count = 0

        self.get_logger().info('LoggingTurtlePublisher ready!')

    def timer_callback(self):
        """Publishes velocity and logs every 10th message."""
        msg = Twist()
        msg.linear.x = 1.5      # 1.5 m/s forward
        msg.angular.z = 0.0     # No rotation

        self.publisher.publish(msg)

        self.msg_count += 1

        # Log every 10 messages (once per second at 10 Hz)
        if self.msg_count % 10 == 0:
            self.get_logger().info(
                f'Published {self.msg_count} messages. '
                f'Moving at {msg.linear.x} m/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LoggingTurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**چلانا**:

```bash
python3 logging_publisher.py
```

**متوقع آؤٹ پٹ**:

```
[INFO] [logging_turtle_publisher]: LoggingTurtlePublisher initializing...
[INFO] [logging_turtle_publisher]: LoggingTurtlePublisher ready!
[INFO] [logging_turtle_publisher]: Published 10 messages. Moving at 1.5 m/s
[INFO] [logging_turtle_publisher]: Published 20 messages. Moving at 1.5 m/s
[INFO] [logging_turtle_publisher]: Published 30 messages. Moving at 1.5 m/s
...
```

-----

## Code Example 3: Publisher with Parameters

حقیقی nodes کنفیگریشن کے لیے parameters کا استعمال کرتے ہیں۔ آئیے رفتار کو قابل ترتیب (configurable) بنائیں:

```python
"""
ROS 2 Publisher with Parameters
Simulation environment: Turtlesim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ParameterizedTurtlePublisher(Node):
    """Publisher with configurable speed parameter."""

    def __init__(self):
        super().__init__('parameterized_turtle_publisher')

        # Declare parameters with default values
        self.declare_parameter('linear_velocity', 1.0)   # Default: 1 m/s
        self.declare_parameter('angular_velocity', 0.0)  # Default: no rotation
        self.declare_parameter('publish_rate', 10.0)      # Default: 10 Hz

        self.get_logger().info('Parameters declared. Waiting for config...')

        # Get initial parameter values
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        pub_rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create timer based on publish_rate
        timer_period = 1.0 / pub_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Publishing at {pub_rate} Hz with '
            f'linear={self.linear_vel} m/s, '
            f'angular={self.angular_vel} rad/s'
        )

    def timer_callback(self):
        """Publishes with configured velocities."""
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_vel

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedTurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**اپنی مرضی کے parameters کے ساتھ چلانا**:

```bash
# Run with default parameters
python3 param_publisher.py

# Or set parameters via ROS 2
python3 param_publisher.py &
ros2 param set /parameterized_turtle_publisher linear_velocity 2.5
ros2 param set /parameterized_turtle_publisher angular_velocity 0.5
```

**متوقع آؤٹ پٹ**:

```
[INFO] [parameterized_turtle_publisher]: Parameters declared. Waiting for config...
[INFO] [parameterized_turtle_publisher]: Publishing at 10.0 Hz with linear=1.0 m/s, angular=0.0 rad/s
```

Turtle 1 m/s کی رفتار سے سیدھا چلتا ہے، یا آپ `ros2 param set` کے ساتھ متحرک طور پر رفتار تبدیل کر سکتے ہیں۔

-----

## Code Example 4: Publisher with Error Handling

پروڈکشن کوڈ کو error handling کی ضرورت ہوتی ہے۔ یہاں ایک مضبوط (robust) ورژن ہے:

```python
"""
ROS 2 Publisher with Error Handling and Logging
Simulation environment: Turtlesim
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobustTurtlePublisher(Node):
    """Publisher with comprehensive error handling."""

    def __init__(self):
        super().__init__('robust_turtle_publisher')

        self.logger = self.get_logger()

        try:
            # Declare parameters
            self.declare_parameter('linear_velocity', 1.0)
            self.declare_parameter('angular_velocity', 0.0)
            self.declare_parameter('publish_rate', 10.0)

            # Validate parameter values
            self.linear_vel = self.get_parameter('linear_velocity').value
            self.angular_vel = self.get_parameter('angular_velocity').value
            pub_rate = self.get_parameter('publish_rate').value

            if pub_rate <= 0.0:
                raise ValueError(f'publish_rate must be positive, got {pub_rate}')

            # Create publisher
            self.publisher = self.create_publisher(
                Twist,
                '/turtle1/cmd_vel',
                10
            )

            # Create timer
            timer_period = 1.0 / pub_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

            # Statistics
            self.msg_count = 0
            self.error_count = 0

            self.logger.info(
                f'RobustTurtlePublisher initialized successfully. '
                f'Publishing at {pub_rate} Hz'
            )

        except Exception as e:
            self.logger.error(f'Initialization failed: {e}')
            raise

    def timer_callback(self):
        """Publishes with error handling."""
        try:
            msg = Twist()
            msg.linear.x = self.linear_vel
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.angular_vel

            # Validate message before publishing
            if not self._validate_message(msg):
                self.error_count += 1
                return

            self.publisher.publish(msg)
            self.msg_count += 1

            # Log statistics every 100 messages
            if self.msg_count % 100 == 0:
                self.logger.debug(
                    f'Published {self.msg_count} messages. '
                    f'Errors: {self.error_count}'
                )

        except Exception as e:
            self.logger.error(f'Error in timer_callback: {e}')
            self.error_count += 1

    @staticmethod
    def _validate_message(msg):
        """Validate message fields."""
        # Check that velocities are reasonable
        if abs(msg.linear.x) > 10.0:
            return False  # Limit to 10 m/s max
        if abs(msg.angular.z) > 3.14:
            return False  # Limit to 1π rad/s max
        return True


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RobustTurtlePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupt received, shutting down...')
    except Exception as e:
        print(f'Fatal error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**خصوصیات (Features)**:

  - Parameter validation (چیک کرتا ہے کہ pub_rate > 0 ہے)
  - Callbacks میں error handling (استثنیٰ/exceptions پر کریش نہیں ہوتا)
  - Message validation (یقینی بناتا ہے کہ velocities مناسب ہیں)
  - Statistics tracking (کامیاب publishes اور errors کو گنتا ہے)
  - Error status کے ساتھ شائستہ بندش (Graceful shutdown)

-----

## اپنا پہلا Publisher کیسے چلائیں

### مرحلہ 1: ایک Working Directory بنائیں

```bash
mkdir -p ~/ros2_workspace/src/my_first_package/my_first_package
cd ~/ros2_workspace/src/my_first_package
```

### مرحلہ 2: Code Example 1 کاپی کریں

`my_first_package/publisher_node.py` بنائیں اور اس میں Code Example 1 کاپی کریں۔

### مرحلہ 3: package.xml بنائیں

پیکیج ڈائریکٹری میں `package.xml` بنائیں:

```xml
<?xml version="1.0"?>
<?xml-model href="[http://download.ros.org/schema/package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)" schematypens="[http://www.w3.org/2001/XMLSchema](http://www.w3.org/2001/XMLSchema)"?>
<package format="3">
  <name>my_first_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package with a publisher node</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### مرحلہ 4: setup.py بنائیں

پیکیج ڈائریکٹری میں `setup.py` بنائیں:

```python
from setuptools import find_packages, setup

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_cmake_core/cmake/package_cmake_template',
         ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My first ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_first_package.publisher_node:main',
        ],
    },
)
```

### مرحلہ 5: **init**.py بنائیں

`my_first_package/__init__.py` (خالی فائل) بنائیں:

```python
# Marker file for Python package
```

### مرحلہ 6: Package کو Build کریں

```bash
cd ~/ros2_workspace
colcon build --symlink-install
source install/setup.bash
```

**متوقع آؤٹ پٹ**:

```
Starting >>> my_first_package
Finished <<< my_first_package [0.0s]

Summary: 1 package finished [0.0s]
```

### مرحلہ 7: Node کو چلائیں

اب آپ اسے `ros2 run` کے ساتھ چلا سکتے ہیں:

```bash
# In one terminal
ros2 run turtlesim turtlesim_node

# In another
ros2 run my_first_package publisher_node
```

یا براہ راست:

```bash
python3 my_first_package/publisher_node.py
```

-----

## اپنے Publisher کو Debug کرنا

### مسئلہ 1: "ModuleNotFoundError: No module named 'geometry\_msgs'"

**وجہ**: ROS 2 environment کو source نہیں کیا گیا، یا پیکیج انسٹال نہیں ہے۔

**حل**:

```bash
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

### مسئلہ 2: "Turtle حرکت نہیں کرتا"

**وجہ**: Topic کا نام غلط ہے، یا turtlesim نہیں چل رہا۔

**حل**:

1.  تصدیق کریں کہ turtlesim چل رہا ہے: `ros2 node list` (ظاہر کرنا چاہیے `/turtlesim`)
2.  اپنے topic کی تصدیق کریں: `ros2 topic list` (ظاہر کرنا چاہیے `/turtle1/cmd_vel`)
3.  topic info چیک کریں: `ros2 topic info /turtle1/cmd_vel` (1 publisher دکھانا چاہیے)

### مسئلہ 3: "Connection timeout" یا "Cannot connect to ROS network"

**وجہ**: `rclpy.init()` call نہیں کیا گیا، یا ROS 2 daemon کریش ہو گیا ہے۔

**حل**:

```bash
ros2 daemon stop
ros2 daemon start
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

### مسئلہ 4: "Turtle حرکت کرتا ہے لیکن بہت آہستہ/تیز"

**وجہ**: Velocity کی قدر (value) غلط ہے۔

**حل**: اپنی `linear.x` کی قدر چیک کریں:

  - `linear.x = 0.5` → 0.5 m/s پر چلتا ہے (آہستہ)
  - `linear.x = 2.0` → 2.0 m/s پر چلتا ہے (تیز)
  - `linear.x = 0.0` → حرکت نہیں کرتا

اپنے code میں قدر کو ایڈجسٹ کریں اور دوبارہ چلائیں۔

### مسئلہ 5: "Topics پیغامات echo نہیں کرتے"

**وجہ**: Publisher کو call نہیں کیا جا رہا ہے، یا timer کام نہیں کر رہا ہے۔

**حل**:

1.  timer\callback میں logging شامل کریں
2.  چیک کریں کہ timer period درست ہے
3.  تصدیق کریں کہ `rclpy.spin()` چل رہا ہے (ٹرمینل کو block کرتا ہے)

-----

## خود تشخیص چیک لسٹ (Self-Assessment Checklist)

سبق 5 پر جانے سے پہلے، تصدیق کریں:

  - [ ] میں rclpy.Node class structure کی وضاحت کر سکتا ہوں
  - [ ] میں جانتا ہوں کہ `create_publisher()` کے ساتھ publisher کیسے بنایا جاتا ہے
  - [ ] میں timer callbacks اور periodic publishing کو سمجھتا ہوں
  - [ ] میں شروع سے ایک مکمل ROS 2 node لکھ سکتا ہوں
  - [ ] میں اپنے node کو `ros2 run` کے ساتھ چلا سکتا ہوں (colcon build کے بعد)
  - [ ] میں تصدیق کر سکتا ہوں کہ میرا publisher `ros2 topic echo` کے ساتھ کام کرتا ہے
  - [ ] میں frequency-to-period تبدیلی (period = 1/frequency) کو سمجھتا ہوں
  - [ ] میں ROS 2 tools کا استعمال کرتے ہوئے عام publisher مسائل کو debug کر سکتا ہوں
  - [ ] میں تسلیم کرتا ہوں کہ pub/sub communication غیر ہم آہنگ (asynchronous) ہے

اگر کوئی چیک ناکام ہو گیا، تو متعلقہ سیکشن کو دوبارہ پڑھیں۔

-----

## مزید ایکسپلوریشن (اختیاری)

1.  **Publisher میں ترمیم کریں**:
       - Turtle کو دائرے میں گھمائیں (linear.x اور angular.z دونوں سیٹ کریں)
       - publish rate کو 5 Hz، 30 Hz میں تبدیل کریں، اور فرق کا مشاہدہ کریں
       - ایک مربع حرکت کا نمونہ (square motion pattern) بنائیں

2.  **Advanced Logging**:
       - مختلف log levels استعمال کریں: `info()`, `warn()`, `error()`, `debug()`
       - logger level سیٹ کریں: `self.get_logger().set_level(logging.DEBUG)`

3.  **ROS 2 دستاویزات**:
       - Publisher اور Subscriber دستاویزات: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
       - Geometry Msgs دستاویزات: https://docs.ros.org/en/humble/Concepts/Intermediate-Concepts/About-Builtin-Interfaces.html

-----

## آگے کیا ہے؟

**سبق 5: آپ کا پہلا ROS 2 Subscriber** (باب 1، سبق 5 میں آ رہا ہے)

آپ سیکھیں گے:

  - Subscriber node کیسے بنایا جائے
  - پیغامات آنے پر callbacks کیسے کام کرتے ہیں
  - ایک مکمل pub/sub system کیسے بنایا جائے (publisher + subscriber)
  - ایک سے زیادہ nodes کے درمیان data کو کیسے synchronize کیا جائے

**تخمینہ شدہ وقت**: 60 منٹ

**ضروری شرائط**: اس سبق سے publishers کو سمجھیں

-----

## Layer 2: AI Collaboration Prompts

### Node Structure کو سمجھنے کے لیے

> "میں نے ابھی اپنا پہلا ROS 2 node لکھا ہے۔ Claude سے پوچھیں: 'اگر میں node بنانے سے پہلے `rclpy.init()` کو call کرنا بھول جاؤں تو کیا ہوگا؟ یہ کیوں ضروری ہے؟'"

### مسائل کو Debug کرنے کے لیے

> "میرا turtle حرکت نہیں کر رہا ہے حالانکہ میرا code درست لگ رہا ہے۔ Claude سے پوچھیں: 'مجھے debugging کے عمل سے گزاریں۔ وہ کون سی جگہیں ہیں جہاں پیغام publish کرنے اور turtle کے حرکت کرنے کے درمیان چیزیں غلط ہو سکتی ہیں؟'"

### جدید خصوصیات (Advanced Features) کے لیے

> "Claude سے پوچھیں: 'میرا publisher 10 Hz پر publish کرتا ہے، لیکن Turtlesim مسلسل احکامات کی توقع کرتا ہے۔ اگر turtle کو 1 سیکنڈ تک کوئی پیغام موصول نہ ہو تو کیا ہوتا ہے؟ کیا یہ رک جاتا ہے یا چلتا رہتا ہے؟'"

-----

## Concept Count & CEFR Validation

**کل B1 تصورات متعارف کرائے گئے**: 8

1.  rclpy.Node class وراثت اور ساخت
2.  `create_publisher()` کے ساتھ Publisher کی تخلیق
3.  Message instantiation اور field assignment
4.  `publisher.publish()` کے ساتھ پیغامات کی اشاعت
5.  متواتر عمل (periodic execution) کے لیے Timer callbacks
6.  Frequency اور period تبدیلی (Hz ↔ seconds)
7.  `get_logger()` کے ساتھ Logging اور debugging
8.  Error handling اور شائستہ بندش (graceful shutdown)

**CEFR B1 Alignment**: ✓ Node lifecycle (A2) کو سمجھنے کی ضرورت ہے، ٹھوس آؤٹ پٹ کے ساتھ ہینڈز آن کوڈنگ کا اضافہ کرتا ہے

**Code Complexity**: 4 کام کرنے والی مثالیں (پیچیدگی 38 سے \~90 لائنوں تک بڑھ رہی ہے)

-----

**باب 1، سبق 4 مکمل**

آپ نے کامیابی کے ساتھ اپنا پہلا ROS 2 publisher لکھ لیا ہے!

اگلا: سبق 5 — آپ کا پہلا ROS 2 Subscriber (باب 1 کا حصہ)

</PersonalizedLesson>
