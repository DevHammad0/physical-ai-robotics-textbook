---
title: "ROS 2 کیا ہے؟"
chapter: 1
lesson: 1
duration_minutes: 30

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "A2"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Define ROS 2 and explain its role in robotics development"
  - "Identify the three fundamental communication patterns in ROS 2"
  - "Describe the relationship between nodes, topics, and messages"
  - "Explain why simulation-first development is essential"
  - "Recognize real-world robotics applications powered by ROS 2"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation)"
---

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/01-what-is-ros2.md">

## تعارف (Introduction)

ROS 2 Fundamentals میں خوش آمدید! اگلے 7 اسباق میں، آپ Robot Operating System 2 (ROS 2) کا استعمال کرتے ہوئے ذہین robotic systems بنانا سیکھیں گے۔ لیکن کوڈ کی ایک لائن لکھنے سے پہلے، آئیے سمجھتے ہیں کہ ROS 2 کیا ہے، یہ کیوں موجود ہے، اور یہ autonomous vehicles سے لے کر humanoid robots تک ہر چیز کو کیسے طاقت فراہم کرتا ہے۔

ROS 2 کو **robots کے لیے operating system** سمجھیں—جس طرح Windows یا macOS آپ کے کمپیوٹر پر پروگرامز کو کوآرڈینیٹ کرتا ہے، اسی طرح ROS 2 robot پر sensors، processors، اور actuators کو کوآرڈینیٹ کرتا ہے۔ یہ ایک اہم مسئلہ حل کرتا ہے: robots پیچیدہ سسٹمز ہوتے ہیں جن میں بہت سے آزاد اجزاء (cameras، motors، lidar sensors) ہوتے ہیں جنہیں قابل اعتماد طریقے سے communicate کرنا اور real-time میں مل کر کام کرنا ہوتا ہے۔

اس سبق کے اختتام تک، آپ ان بنیادی تصورات کو سمجھ جائیں گے جو ہر ROS 2 developer کو جاننا ضروری ہیں۔

---

## ROS 2 کیا ہے؟ (60 سیکنڈ میں)

**ROS 2** (Robot Operating System 2) ایک **مفت، اوپن سورس middleware framework** ہے جو robots پر distributed computing کو فعال بناتا ہے۔ یہ فراہم کرتا ہے:

1. **Communication layer**: Software components کے درمیان Hardware سے آزاد message passing
2. **Hardware abstraction**: ایک بار لکھیں، کسی بھی robot پر چلائیں (wheelchair، humanoid، autonomous car)
3. **Scalability**: ایک ہی robot پر 10 processes چلائیں یا 1,000 processes
4. **Reliability**: متعدد quality-of-service (QoS) سیٹنگز کے ساتھ Real-time کی صلاحیت
5. **Developer tools**: Visualization، debugging، اور monitoring کی سہولیات built-in ہیں

**اہم بات**: ROS 2 robot کو براہ راست control نہیں کرتا۔ بلکہ، یہ ایک **communication framework** ہے جو software کے مختلف ٹکڑوں کو ایک دوسرے سے بات کرنے دیتا ہے۔ ایک camera driver تصاویر publish کرتا ہے، ایک motion planner ان تصاویر کو subscribe کرتا ہے اور motor commands پبلش کرتا ہے، اور motor controller ان commands کو سبسکرائب کرتا ہے اور robot کو حرکت دیتا ہے۔

اس command کو verify کرنے کے لیے چلائیں:
```bash
ros2 --version
````

-----

## ROS 2 نے ROS 1 کی جگہ کیوں لی؟

ROS 1 (2007–2019) ایک اہم پیشرفت تھی لیکن جدید robotics کے لیے اس میں کچھ حدود تھیں:

| Limitation | Problem | ROS 2 Solution |
|---|---|---|
| Single central server | Master node کی ناکامی = مکمل سسٹم کی ناکامی | Distributed architecture، ناکامی کا کوئی واحد نقطہ نہیں (no single point of failure) |
| Poor real-time support | غیر متوقع latencies، ناقابل اعتماد ٹائمنگ | Real-time kernel سپورٹ، deterministic شیڈولنگ |
| Legacy Python 2 | Python 2 کا اختتام (2020) | Native Python 3.8+ سپورٹ |
| Weak security | کوئی encryption، authentication، یا رسائی کا کنٹرول نہیں | Built-in سیکیورٹی: DTLS، authentication، encryption |
| Android/Windows gaps | صرف Linux پر اچھا چلتا تھا | مکمل Windows اور Android سپورٹ |

**ROS 2 فلسفہ**: اسے research prototypes کے بجائے production robotics کے لیے بنیاد سے (ground up) بنایا گیا ہے۔

-----

## بنیادی تصورات: Nodes، Topics، اور Messages

اس سے پہلے کہ ہم مزید گہرائی میں جائیں، آئیے ROS 2 سسٹمز کے تین بنیادی اجزاء (building blocks) کا تعارف کراتے ہیں۔

### تصور 1: Nodes (سافٹ ویئر پروسیسز)

ایک **node** ایک آزاد process ہے جو robot پر چلتا ہے۔ ہر node کی ایک مخصوص ذمہ داری ہوتی ہے:

  - **Camera driver node**: کیمرے سے خام sensor data پڑھتا ہے اور اسے publish کرتا ہے
  - **Motion planner node**: Sensor data کو subscribe کرتا ہے، راستہ plan کرتا ہے، اور motion commands پبلش کرتا ہے
  - **Motor controller node**: Motion commands کو subscribe کرتا ہے اور motors کو چلاتا ہے (actuates)

Nodes **آزاد** ہوتے ہیں: ایک ناکام ہونے والا node دوسروں کو crash نہیں کرتا۔ اسے **fault isolation** کہا جاتا ہے۔

**مثال**: ROS 2 nodes کو ایک کمپنی کے ملازمین کی طرح سمجھیں۔ Camera driver ریسپشنسٹ ہے (باہر سے معلومات اکٹھی کرتا ہے)، motion planner مینیجر ہے (فیصلے کرتا ہے)، اور motor controller ورکر ہے (کام انجام دیتا ہے)۔

### تصور 2: Topics (مواصلاتی چینلز)

ایک **topic** ایک نامزد communication channel ہے جہاں messages کا تبادلہ ہوتا ہے۔ Topics **غیر مطابقت پذیر (asynchronous)** (non-blocking) ہوتے ہیں اور **many-to-many** کمیونیکیشن کو سپورٹ کرتے ہیں:

  - بہت سے publishers ایک topic پر لکھ سکتے ہیں
  - بہت سے subscribers ایک topic سے پڑھ سکتے ہیں
  - Publishers اور subscribers ایک دوسرے کے بارے میں نہیں جانتے

Topics کو ریڈیو چینلز کی طرح سمجھیں: ایک نیوز اسٹیشن (publisher) FM 101.5 پر نشر کرتا ہے، اور سامعین (subscribers) سنتے ہیں۔ اسٹیشن نہیں جانتا کہ کون سن رہا ہے، اور سامعین ایک دوسرے کو نہیں جانتے۔

**عام topic نام** (ROS 2 کے کنونشنز کے مطابق):

  - `/camera/image_raw` — Camera image stream
  - `/lidar/point_cloud` — 3D laser scanner data
  - `/turtle1/cmd_vel` — Turtle robot کے لیے ولاسٹی کمانڈز
  - `/odom` — Odometry (پوزیشن اور سمت کا تخمینہ)

Topics ہمیشہ `/` سے شروع ہوتے ہیں اور multi-word ناموں کے لیے `snake_case` استعمال کرتے ہیں۔

### تصور 3: Messages (ڈیٹا اسٹرکچرز)

ایک **message** وہ data structure ہے جو کسی topic پر publish کیا جاتا ہے۔ Messages میں مخصوص data types کے ساتھ **fields** ہوتے ہیں:

**مثال: Twist message** (حرکت کی کمانڈز کے لیے استعمال ہوتا ہے)

```yaml
linear:
  x: 1.0       # forward/backward speed (meters/second)
  y: 0.0       # sideways speed
  z: 0.0       # vertical speed
angular:
  x: 0.0       # roll rotation
  y: 0.0       # pitch rotation
  z: 0.5       # yaw rotation (turning left/right)
```

**مثال: Image message** (کیمرے سے)

```yaml
header:
  seq: 42                      # message count
  stamp: [seconds, nanoseconds] # timestamp
  frame_id: "camera_optical_frame"
height: 480                    # pixels
width: 640                     # pixels
encoding: "rgb8"               # RGB color format
data: [binary image data...]   # raw pixel bytes
```

Messages **strongly typed** ہوتے ہیں—publisher اور subscriber کو message کے اسٹرکچر پر متفق ہونا چاہیے، ورنہ communication ناکام ہو جائے گی۔

-----

## مواصلات کے تین طریقے (The Three Communication Patterns)

ROS 2 روبوٹ کمیونیکیشن کے لیے تین الگ الگ پیٹرن فراہم کرتا ہے۔ ہر ایک مختلف مسئلہ حل کرتا ہے۔

### پیٹرن 1: Publish-Subscribe (Pub/Sub) — Asynchronous, One-Way

**استعمال**: Sensor data اور control signals کی اسٹریمنگ

  - **Asynchronous**: Publisher سبسکرائبرز کا انتظار نہیں کرتا؛ subscriber پبلشر کا انتظار نہیں کرتا
  - **One-way**: ڈیٹا publishers → subscribers کی طرف بہتا ہے
  - **Many-to-many**: متعدد publishers اور subscribers سپورٹڈ ہیں

**مثالی ورک فلو**:

```text
Camera node publishes images on /camera/image_raw every 33 ms
    ↓
Two subscriber nodes listen (motion planner + object detector)
Both receive copies of the same data
Planner responds by publishing to /cmd_vel
    ↓
Motor controller subscribes to /cmd_vel and moves robot
```

**حقیقی دنیا کی مثال**: ایک موسمی ریڈیو درجہ حرارت کی اپڈیٹس نشر کرتا ہے۔ آپ کا گھریلو موسمی اسٹیشن سنتا ہے اور اپنی ڈسپلے اپڈیٹ کرتا ہے بغیر ریڈیو سے پوچھے کہ "کیا آپ ڈیٹا بھیجنے والے ہیں؟"

### پیٹرن 2: Services — Synchronous, Request-Response

**استعمال**: One-off کمانڈز جن کے لیے جواب درکار ہو

  - **Synchronous**: جاری رکھنے سے پہلے Client جواب کا انتظار کرتا ہے
  - **Request-response**: Client ڈیٹا بھیجتا ہے، server اسے پروسیس کرتا ہے، اور جواب واپس بھیجتا ہے
  - **One-to-one**: ہر service کال ایک ہی server پر جاتی ہے

**مثالی ورک فلو**:

```text
Motion planner: "Hey, set robot color to red"
    ↓
LED controller receives request
Processes it (sets LED to red)
    ↓
LED controller responds: "Color set successfully"
Motion planner continues executing
```

**حقیقی دنیا کی مثال**: پیزا ریسٹورنٹ کو کال کرنا۔ آپ پیزا کی درخواست کرتے ہیں، وہ آپ کے آرڈر کی تصدیق کرتے ہیں، آپ فون رکھتے ہیں۔ Pub/sub (براڈکاسٹ) کے برعکس، services براہ راست گفتگو ہیں۔

### پیٹرن 3: Actions — Asynchronous, Long-Running Tasks

**استعمال**: وہ کمانڈز جو وقت لیتی ہیں اور فیڈبیک کی ضرورت ہوتی ہے

  - **Asynchronous**: Client تکمیل (completion) کا انتظار نہیں کرتا
  - **Feedback**: کام کے دوران Server پیشرفت کی اپڈیٹس بھیجتا ہے
  - **Cancellable**: Client ایکشن کو جلدی روک سکتا ہے

**مثالی ورک فلو**:

```text
Human: "Navigate to the charging dock"
    ↓
Navigation action starts
    ↓
While navigating, it sends feedback: "25% of the way", "50% complete"
    ↓
Navigation completes and sends final result: "Arrived at dock"
```

**حقیقی دنیا کی مثال**: ڈلیوری آرڈر کرنا۔ آپ پیکج کی ڈلیوری کی درخواست کرتے ہیں، آپ کو لائیو ٹریکنگ اپڈیٹس ملتی ہیں، اور آخر میں اطلاع ملتی ہے جب یہ پہنچ جاتا ہے—یہ ایک سادہ on/off سروس کال سے بہت مختلف ہے۔

**اس باب میں**، ہم pub/sub (سبق 3) اور services (سبق 5) پر توجہ مرکوز کریں گے۔ Actions کا تعارف باب 2 میں کرایا جائے گا۔

-----

## ROS 2 آرکیٹیکچر: بڑی تصویر (The Big Picture)

یہاں بتایا گیا ہے کہ یہ تمام تصورات ایک حقیقی ROS 2 سسٹم میں کیسے فٹ بیٹھتے ہیں:

```text
┌─────────────────────────────────────────────────────────────┐
│                     ROS 2 System                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Node: Camera Driver           Node: Motion Planner         │
│  ┌──────────────────┐         ┌──────────────────┐          │
│  │ Publishes image  │─────→   │ Subscribes image │          │
│  │ at 30 Hz         │         │ Plans trajectory │          │
│  │                  │         │ Publishes cmd    │          │
│  └──────────────────┘         └────────┬─────────┘          │
│                                        │                    │
│         Topic: /camera/image_raw       │                    │
│                                        │                    │
│                    Topic: /turtle1/cmd_vel                  │
│                            ↓                                │
│  Node: Motor Controller                                     │
│  ┌──────────────────┐                                       │
│  │ Subscribes cmd   │                                       │
│  │ Rotates motor    │                                       │
│  └──────────────────┘                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**اہم بات**: Nodes ٹاپکس کے ذریعے آپس میں loosely coupled ہوتے ہیں۔ ہر node کو آزادانہ طور پر develop، test، اور deploy کیا جا سکتا ہے جب تک کہ وہ topic کے ناموں اور message کی اقسام پر متفق ہوں۔

-----

## تصور 4: Message کی اقسام اور Type Safety

ROS 2 میسج کی اقسام `.msg` فائلز میں بیان کی جاتی ہیں۔ یہاں عام پیغامات ہیں جن کا آپ سامنا کریں گے:

### عام Built-in Message کی اقسام

| Message | Package | Common Use |
|---------|---------|-----------|
| `Twist` | `geometry_msgs` | ولاسٹی کمانڈز (linear + angular) |
| `Pose` | `geometry_msgs` | پوزیشن (x,y,z) + اورینٹیشن (roll,pitch,yaw) |
| `LaserScan` | `sensor_msgs` | 2D lidar ڈیٹا (360° فاصلے کی پیمائش) |
| `Image` | `sensor_msgs` | کیمرہ امیج ڈیٹا |
| `PointCloud2` | `sensor_msgs` | 3D point cloud (ڈیپتھ کیمروں سے) |
| `JointState` | `sensor_msgs` | روبوٹ کے جوڑوں کے زاویے اور ولاسٹیز |
| `Odometry` | `nav_msgs` | پوزیشن کا تخمینہ + ولاسٹی |

**Strong typing کیوں اہم ہے**: اگر ایک camera node ایک `Image` میسج publish کرتا ہے لیکن motor controller ایک `Twist` میسج کی توقع کرتا ہے، تو ROS 2 کنکشن کی اجازت نہیں دے گا۔ یہ کیڑے (bugs) کو شروع میں ہی پکڑ لیتا ہے۔

-----

## تصور 5: نام رکھنے کے اصول (ROS 2 اسٹائل)

ROS 2 سسٹمز کو منظم رکھنے کے لیے نام رکھنے کے سخت اصول استعمال کرتا ہے:

### Node کے نام (snake\_case)

```text
camera_driver     ✓ Good
motion_planner    ✓ Good
motorController   ✗ Bad (Python/ROS uses snake_case)
```

### Topic کے نام (leading slash اور snake\_case کے ساتھ)

```text
/camera/image_raw        ✓ Good (hierarchical namespace)
/robot/odom              ✓ Good (logical grouping)
/turtle1/cmd_vel         ✓ Good (robot-specific topics)
/cameraImage             ✗ Bad (missing leading slash)
/camera_image_raw        ✗ Okay but less descriptive
```

### Parameter کے نام (snake\_case)

```text
max_velocity      ✓ Good
linear_gain       ✓ Good
maxLinearVelocity ✗ Bad (use snake_case)
```

**بہترین عمل**: Topics کو سسٹم کے لحاظ سے منظم کرنے کے لیے hierarchical namespaces استعمال کریں (مثال کے طور پر، `/camera/rgb/image_raw`، `/lidar/scan`، `/motor/left_wheel/cmd`)۔

-----

## تصور 6: Simulation-First Development کیوں اہم ہے

**Simulation-first** کا مطلب ہے فزیکل ہارڈ ویئر کو چھونے سے پہلے ورچوئل روبوٹ پر ڈیویلپمنٹ اور ٹیسٹنگ کرنا۔

### Simulation-First طریقہ کار کے فوائد

1.  **حفاظت (Safety)**: مہنگے ہارڈ ویئر کو نقصان پہنچنے کا کوئی خطرہ نہیں
2.  **رفتار (Speed)**: Time-acceleration کا استعمال کرتے ہوئے 1 گھنٹے کا مشن 1 منٹ میں چلائیں
3.  **تکرار کی صلاحیت (Reproducibility)**: ایک جیسے ٹیسٹ 100 بار چلائیں؛ اصلی روبوٹس مختلف ہو سکتے ہیں
4.  **سیکھنا (Learning)**: اخراجات کی فکر کیے بغیر آزادانہ طور پر تجربہ کریں
5.  **ڈیبگنگ (Debugging)**: ورچوئل روبوٹ میں ایسے سینسرز شامل کریں جو اصلی روبوٹس پر موجود نہیں ہیں

### ROS 2 سیمولیشن کا منظرنامہ (جو آپ سیکھیں گے)

**باب 1 (یہ باب)**: Turtlesim

  - سادہ 2D turtle روبوٹ
  - تعلیمی، ہلکا پھلکا (lightweight)
  - Pub/sub اور سروسز سیکھنے کے لیے بہترین

**باب 2 (اگلا)**: Gazebo

  - صنعتی درجے کا فزکس سمیلیٹر
  - URDF روبوٹ کی تفصیل
  - سینسرز اور حقیقت پسندانہ فزکس

**باب 3**: NVIDIA Isaac Sim

  - Photorealistic رینڈرنگ
  - AI/ML کے لیے Synthetic ڈیٹا جنریشن
  - اصلی روبوٹس کے لیے Digital twins

**Deployment** (باب 4+): اصلی ہارڈ ویئر

  - TurtleBot 4، موبائل مینیپولیٹرز، humanoids
  - صرف سیمولیشن میں مہارت کے بعد

-----

## حقیقی دنیا کی مثالیں: ROS 2 استعمال کرنے والی کمپنیاں

ROS 2 پروڈکشن میں روبوٹس کو طاقت فراہم کرتا ہے:

### خود مختار گاڑیاں (Autonomous Vehicles)

  - **Waymo**: Robotaxis جو ROS 2 (اندرونی forks) پر چل رہی ہیں
  - **Aurora**: خود مختار ٹرکنگ فلیٹ
  - **Cruise (GM)**: سان فرانسسکو میں خود چلنے والی کاریں

### صنعتی روبوٹس (Industrial Robots)

  - **Universal Robots (UR)**: Collaborative manipulators
  - **ABB**: ROS 2 کے ساتھ صنعتی بازوؤں کا انضمام
  - **Mobile Industrial Robots (MiR)**: خود مختار گودام کے روبوٹس

### تحقیق اور تعلیم (Research & Education)

  - **MIT CSAIL**: Humanoid روبوٹ ہیرا پھیری (manipulation) کی تحقیق
  - **UC Berkeley**: روبوٹکس لیبز ROS 2 پر معیاری (standardized) ہیں
  - **Open Robotics**: ROS 2 کو برقرار رکھتا ہے، معیارات تیار کرتا ہے

### سروس روبوٹس (Service Robots)

  - **Boston Dynamics**: Spot quadruped جو ROS 2-compatible APIs استعمال کرتا ہے
  - **Agility Robotics**: Digit humanoid لاجسٹکس روبوٹ
  - **Clearpath Robotics**: تحقیقی پلیٹ فارمز جو ROS 2 پر معیاری ہیں

-----

## خود تشخیص چیک لسٹ (Self-Assessment Checklist)

سبق 2 کی طرف بڑھنے سے پہلے، تصدیق کریں کہ آپ سمجھتے ہیں:

  - [ ] میں ROS 2 کی تعریف ایک جملے میں کر سکتا ہوں (روبوٹس کے لیے distributed middleware)
  - [ ] میں nodes، topics، اور messages کے درمیان فرق کی وضاحت کر سکتا ہوں
  - [ ] میں سمجھتا ہوں کہ pub/sub کیوں asynchronous ہے اور services کیوں synchronous ہیں
  - [ ] میں تین کمیونیکیشن پیٹرنز اور ہر ایک کے لیے ایک استعمال جانتا ہوں
  - [ ] میں وضاحت کر سکتا ہوں کہ hardware-first کے مقابلے میں simulation-first ڈیویلپمنٹ کیوں زیادہ محفوظ ہے
  - [ ] میں تسلیم کرتا ہوں کہ ROS 2 بڑی روبوٹکس کمپنیوں (Waymo، Boston Dynamics وغیرہ) کے ذریعہ استعمال ہوتا ہے

-----

## مزید ایکسپلوریشن (اختیاری)

اگلے سبق سے پہلے مزید گہرائی میں جانا چاہتے ہیں؟

1.  **Official ROS 2 Documentation**: https://docs.ros.org/ ملاحظہ کریں اور تصورات کے صفحے کو براؤز کریں
2.  **ROS 2 Architecture Deep Dive**: DDS middleware کے بارے میں پڑھیں جو ROS 2 کو طاقت دیتا ہے (https://www.dds-foundation.org/)
3.  **Real-World ROS Deployments**: اسے ایکشن میں دیکھنے کے لیے YouTube پر "ROS 2 real world robot" تلاش کریں
4.  **Community**: سوالات پوچھنے کے لیے ROS 2 Discourse فورم (https://discourse.ros.org/) میں شامل ہوں

-----

## آگے کیا ہے؟

**سبق 2: اپنا ROS 2 ماحول ترتیب دینا** میں، آپ کریں گے:

  - Ubuntu 22.04 پر ROS 2 Humble انسٹال کریں
  - Turtlesim (آپ کا پہلا سیمولیشن ماحول) ترتیب دیں
  - تصدیق کریں کہ سب کچھ کام کر رہا ہے
  - اپنا پہلا ROS 2 کوڈ لکھنے کے لیے تیار ہو جائیں

**متوقع وقت**: 45 منٹ

**سبق 2 کے لیے ضروریات**: Ubuntu 22.04 (یا VM) والا کمپیوٹر، 2GB خالی ڈسک اسپیس، انٹرنیٹ کنکشن

-----

## Layer 2: AI Collaboration Prompt

**Claude یا کسی دوسرے AI اسسٹنٹ کے ساتھ کام کرنے والے طلباء کے لیے:**

> "I just learned about ROS 2 nodes, topics, and messages. Ask Claude: 'How does the publish-subscribe pattern in ROS 2 differ from function calls in regular Python programs? When would you choose pub/sub instead of calling a function directly?'"

**یہ کیوں اہم ہے**: Sequential function calls اور distributed message passing کے درمیان آرکیٹیکچرل فرق کو سمجھنا ROS 2 کی سوچ کے لیے اہم ہے۔

-----

## ٹربل شوٹنگ (Pre-Lesson سپورٹ)

**سوال: میں روبوٹکس کا ماہر نہیں ہوں۔ کیا یہ کورس میرے لیے ہے؟**
جواب: ہاں\! یہ کورس صرف بنیادی پروگرامنگ کے علم (Python 3 اور ٹرمینل کمانڈز کے ساتھ آرام) فرض کرتا ہے۔ روبوٹکس کے تصورات کی شروع سے وضاحت کی گئی ہے۔

**سوال: اگر میں AI روبوٹس بنانا چاہتا ہوں تو میں ROS 2 کیوں سیکھ رہا ہوں؟**
جواب: ROS 2 روبوٹکس میں ڈی فیکٹو اسٹینڈرڈ ہے۔ یہاں تک کہ جدید ترین AI روبوٹ کمپنیاں (Boston Dynamics، Figure AI، Tesla) بھی ROS 2 یا ROS 2-compatible فریم ورکس استعمال کرتی ہیں۔ یہ ضروری انفراسٹرکچر ہے۔

**سوال: کیا مجھے اس کورس کو مکمل کرنے کے لیے فزیکل روبوٹ کی ضرورت ہے؟**
جواب: نہیں\! تمام اسباق 1-4 Turtlesim (ایک simulation-only کچھوا) استعمال کرتے ہیں۔ فزیکل روبوٹ کے تجربات باب 4+ میں آتے ہیں اور اختیاری ہیں۔

**سوال: اگر میں macOS یا Windows پر ہوں تو کیا ہوگا؟**
جواب: ROS 2 Humble سرکاری طور پر Ubuntu Linux کو سپورٹ کرتا ہے۔ آپ Ubuntu چلانے کے لیے VMware، VirtualBox، یا WSL2 (Windows Subsystem for Linux) استعمال کر سکتے ہیں۔ سبق 2 میں سیٹ اپ کی ہدایات اس کا احاطہ کرتی ہیں۔

-----

## تصور شمار اور CEFR توثیق

**کل A2 تصورات متعارف کرائے گئے**: 6

1.  ROS 2 تعریف اور ایکو سسٹم
2.  Middleware آرکیٹیکچر (DDS، distributed computing)
3.  Nodes بنیادی بلڈنگ بلاکس کے طور پر
4.  Pub/sub کمیونیکیشن کے لیے Topics
5.  Messages اور message کی اقسام (strong typing)
6.  نام رکھنے کے اصول اور ROS 2 اسٹائل

**CEFR A2 الائنمنٹ**: ✓ تصورات بنیادی ہیں، زبان واضح ہے، Gazebo/Isaac Sim کے حوالے نہیں دیے گئے

-----

**باب 1، سبق 1 مکمل**

اگلا: سبق 2 — اپنا ROS 2 ماحول ترتیب دینا (45 منٹ)

</PersonalizedLesson>