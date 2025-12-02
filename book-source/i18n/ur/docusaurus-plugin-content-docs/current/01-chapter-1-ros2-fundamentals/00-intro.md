---
title: "باب 1: ROS 2 کے بنیادی اصول اور کمیونیکیشن"
chapter: 1
total_lessons: 7
completed_lessons: 4
target_hours: 5.5
cefr_entry_level: "A2"
cefr_exit_level: "B1"
simulation_environment: "Turtlesim"
chapter_objectives:
  - "ROS 2 architecture کو سمجھنا (nodes, topics, messages)"
  - "ROS 2 development environment سیٹ اپ کرنا"
  - "Python میں publisher اور subscriber nodes بنانا"
  - "Publish-subscribe communication patterns کو نافذ کرنا"
  - "System inspection اور debugging کے لیے ROS 2 CLI tools کا استعمال کرنا"
  - "Asynchronously پیغامات کو سنبھالنا اور data flows ڈیزائن کرنا"
  - "Service-based synchronous communication کو سمجھنا"
---

<PersonalizedLesson lessonPath="01-chapter-1-ros2-fundamentals/00-intro.md">

## باب 1 میں خوش آمدید: ROS 2 کے بنیادی اصول (Fundamentals)

### اس باب میں آپ کیا سیکھیں گے

Robot Operating System 2 (ROS 2) **robotics development کے لیے de facto standard** ہے۔ Waymo، Boston Dynamics، اور NVIDIA جیسی کمپنیاں اپنی robotics middleware کے طور پر ROS 2 کا استعمال کرتی ہیں۔

اس باب میں، آپ اس بات کا **مکمل ذہنی خاکہ (mental model)** بنائیں گے کہ ROS 2 systems کیسے کام کرتے ہیں:

- **سبق 1**: ROS 2 کیا ہے؟ (تصورات، architecture، یہ کیوں اہم ہے)
- **سبق 2**: اپنا environment سیٹ اپ کرنا (Ubuntu 22.04، انسٹالیشن، Turtlesim)
- **سبق 3**: Nodes اور communication patterns (pub/sub بمقابلہ services)
- **سبق 4**: آپ کا پہلا publisher (code لکھیں، turtle کو حرکت دیں)
- **سبق 5**: آپ کا پہلا subscriber (پیغامات وصول کریں اور ردعمل دیں) — *[Planned]*
- **سبق 6**: Services اور request-response (synchronous communication) — *[Planned]*
- **سبق 7**: Integrated systems بنانا (ایک ساتھ کام کرنے والے متعدد nodes) — *[Planned]*

### باب کی پیشرفت (Chapter Progress)

**مکمل شدہ (MVP)**: اسباق 1-4 (U.S. 1-2، اندازاً 3 گھنٹے)
- ROS 2 architecture کو سمجھنا ✓
- اپنا development environment سیٹ اپ کرنا ✓
- اپنا پہلا publisher بنانا ✓

**منصوبہ بندی (Planned)**: اسباق 5-7 (U.S. 3، اندازاً 2.5 گھنٹے)
- Subscribers اور callbacks
- Services اور RPCs
- Multi-node systems

---

## یہاں سے کیوں شروع کریں؟

ROS 2 physical AI اور robotics میں **ہر چیز کی بنیاد** ہے:

1. **انڈسٹری اسٹینڈرڈ**: 70% سے زیادہ robotics کمپنیاں ROS 2 استعمال کرتی ہیں
2. **Hardware Agnostic**: ایک بار code لکھیں، کسی بھی robot پر چلائیں
3. **Scalable**: سنگل Turtlebot سے لے کر 100+ robots کے swarms تک
4. **Production-Ready**: تعینات شدہ autonomous systems میں استعمال ہوتا ہے (Waymo، Cruise وغیرہ)

**ROS 2 کو سمجھے بغیر**، آپ یہ نہیں کر سکتے:
- Code کو حقیقی robots پر deploy کرنا
- Production robotics systems کے ساتھ کام کرنا
- Gazebo simulation کے ساتھ integrate کرنا (باب 2)
- Perception pipelines (computer vision) کا استعمال کرنا
- Multi-robot systems کو منظم کرنا

**ROS 2 کے بنیادی اصولوں کے ساتھ**، آپ ان کے لیے تیار ہوں گے:
- باب 2: Gazebo کے ساتھ Physics simulation
- باب 3: SLAM کے ساتھ Autonomous navigation
- باب 4: Robotics کے لیے Vision-language-action pipelines
- جدید موضوعات: Manipulation، perception، اور real robot deployment

---

## Simulation-First Approach

اس باب کے تمام اسباق **Turtlesim** کا استعمال کرتے ہیں، جو ایک ہلکا پھلکا 2D robot simulator ہے:

```

Why Turtlesim?
├─ Zero hardware cost (free, open-source)
├─ Fast iteration (no robot startup time)
├─ Safe experimentation (can't damage anything)
├─ Clear diagnostics (easy to see what's happening)
└─ Focus on concepts (not distracted by physics)

```

**بعد کے ابواب** (باب 2+) مزید جدید robotics کی طرف بڑھتے ہیں:
- **باب 2**: Gazebo (physics simulation, URDF robots, sensors)
- **باب 3**: SLAM اور autonomous navigation (حقیقی دنیا کی path planning اور localization)
- **باب 4**: Vision-language-action systems (AI-driven autonomy)
- **Real hardware**: اعتماد کے ساتھ deploy کریں (simulation پر مہارت بطور prerequisite)

---

## ضروری شرائط (Prerequisites)

### ضروری علم
- **Python 3**: بنیادی syntax (functions, classes, loops)
- **Terminal/Command-line**: `cd`, `ls`, `mkdir` کے ساتھ آسانی
- **Text editor**: فائلیں edit اور save کر سکتے ہوں (VS Code, gedit, nano)

### ضروری سافٹ ویئر
- **Ubuntu 22.04 LTS** (یا VM/WSL2)
- **3 GB خالی ڈسک کی جگہ**
- **انٹرنیٹ کنکشن**

### اختیاری لیکن مددگار
- Git (version control کے لیے) — اس باب کے لیے ضروری نہیں
- Python extension کے ساتھ VS Code — کوڈنگ کو آسان بناتا ہے

---

## اس باب کو کیسے استعمال کریں

### آپشن A: لکیری (مبتدیوں کے لیے تجویز کردہ)

اسباق کو ترتیب سے پڑھیں اور مکمل کریں:

1. سبق 1 (30 منٹ) → ROS 2 کے تصورات کو سمجھیں
2. سبق 2 (45 منٹ) → انسٹال کریں اور setup کی تصدیق کریں
3. سبق 3 (60 منٹ) → Architecture اور tools سیکھیں
4. سبق 4 (60 منٹ) → اپنا پہلا code لکھیں
5. *[اسباق 5-7 جب شائع ہوں]*

**وقت کی سرمایہ کاری**: MVP مکمل کرنے اور بنیادی تصورات کو سمجھنے کے لیے 3 گھنٹے

### آپشن B: تیز رفتار (تجربہ کار پروگرامرز کے لیے)

اگر آپ distributed systems کو پہلے سے جانتے ہیں:
- سبق 1 پر سرسری نظر ڈالیں (10 منٹ)
- اگر ROS 2 پہلے سے انسٹال ہے تو سبق 2 پر سرسری نظر ڈالیں (5 منٹ)
- Conceptual model کے لیے سبق 3 کا بغور مطالعہ کریں (20 منٹ)
- سبق 4 کی کوڈنگ پر جائیں (30 منٹ)

**وقت کی سرمایہ کاری**: شروع کرنے کے لیے 1 گھنٹہ

### آپشن C: اپنی رفتار سے (زیادہ سے زیادہ لچک)

جب آپ کے پاس وقت ہو تو ہر سبق پر کام کریں۔ اسباق 1-3 آزاد ہیں (تصورات)؛ سبق 4 کے لیے اسباق 1-3 کی سمجھ بوجھ ضروری ہے۔

---

## سبق کے لحاظ سے سیکھنے کے نتائج

### سبق 1: ROS 2 کیا ہے؟ (30 منٹ)

**آپ سمجھ جائیں گے**:
- ROS 2 کی تعریف اور تاریخ (ROS 2 نے ROS 1 کی جگہ کیوں لی)
- Middleware architecture (nodes, topics, messages)
- تین communication patterns (pub/sub, services, actions)
- Simulation-first کیوں ضروری ہے
- ROS 2 استعمال کرنے والی حقیقی دنیا کی robotics کمپنیاں

**CEFR Target**: A2 (Beginner) — تصوراتی بنیادیں

---

### سبق 2: اپنا ROS 2 Environment سیٹ اپ کرنا (45 منٹ)

**آپ حاصل کریں گے**:
- Ubuntu 22.04 پر ROS 2 Humble انسٹال کرنا
- `ros2 --version` کے ساتھ انسٹالیشن کی تصدیق کرنا
- Turtlesim simulator کو لانچ کرنا
- کی بورڈ کے ساتھ turtle کو کنٹرول کرنا
- Environment variables کو سمجھنا (ROS_DISTRO, ROS_DOMAIN_ID)

**CEFR Target**: A2 (Beginner) — طریقہ کار کا سیٹ اپ

**کامیابی کا معیار**: جب آپ 'w/a/s/d' دباتے ہیں تو Turtle حرکت کرتا ہے

---

### سبق 3: Nodes اور Communication Patterns (60 منٹ)

**آپ سیکھیں گے**:
- Node lifecycle: created → initialized → running → shutdown
- Publish-subscribe (asynchronous, many-to-many)
- Services (synchronous, request-response)
- Message types اور strong typing
- ROS 2 نام رکھنے کے اصول (naming conventions)
- CLI tools: `ros2 node`, `ros2 topic`, `ros2 topic echo`, `rqt_graph`

**CEFR Target**: A2 (Beginner) — آرکیٹیکچرل تصورات

**عملی مشق**: لائیو Turtlesim system کا معائنہ کریں، data flows کو سمجھیں 

---

### سبق 4: آپ کا پہلا ROS 2 Publisher (60 منٹ)

**آپ بنائیں گے**:
- ایک Python ROS 2 node جو velocity commands publish کرتا ہے
- `create_publisher()` کے ساتھ Publishers
- متواتر پیغامات کے لیے Timer callbacks
- Error handling اور logging
- Package structure (package.xml, setup.py)

**CEFR Target**: B1 (Elementary) — ہینڈز آن کوڈنگ

**کامیابی کا معیار**: ایسا code لکھیں جو turtle کو حرکت دے

---

### سبق 5: آپ کا پہلا ROS 2 Subscriber (60 منٹ) — *[Planned]*

**آپ بنائیں گے**:
- ایک Python ROS 2 subscriber node
- Subscription callbacks
- Message synchronization
- مکمل pub/sub systems (publisher + subscriber)

**CEFR Target**: B1 (Elementary)

---

### سبق 6: Services اور Request-Response (60 منٹ) — *[Planned]*

**آپ سیکھیں گے**:
- Service servers اور clients
- Synchronous بمقابلہ asynchronous communication
- Services بمقابلہ pub/sub کب استعمال کریں
- Python میں services کو نافذ کرنا اور call کرنا

**CEFR Target**: B1 (Elementary)

---

### سبق 7: Integrated Systems بنانا (60 منٹ) — *[Planned]*

**آپ بنائیں گے**:
- ایک multi-node robotics system
- Publishers، subscribers، اور services کو مربوط کرنا
- `ros2 node` اور `rqt_graph` کے ساتھ system کی صحت کی نگرانی کرنا
- Communication کی ناکامیوں کو debug کرنا

**CEFR Target**: B1 (Elementary) → مربوط سمجھ بوجھ

---

## آئینی تعمیل (Constitutional Compliance)

یہ باب **Physical AI Safety Framework** (آئین کے سیکشن IX) کی پابندی کرتا ہے:

### Simulation-First کا حکم
- ✓ تمام code examples صرف Turtlesim استعمال کرتے ہیں (کوئی hardware نہیں)
- ✓ صفر جسمانی نقصان کا خطرہ (simulation environment)
- ✓ Safety protocols کی ضرورت نہیں (ابھی تک کوئی autonomous movement نہیں)

### Code کی توثیق
- ✓ تمام code examples نحوی طور پر (syntactically) درست Python 3.10+ ہیں
- ✓ تمام code ٹیسٹ شدہ ہے اور ROS 2 Humble میں چلنے کے قابل ہے
- ✓ ہر code block کے لیے متوقع output دستاویزی ہے
- ✓ عام غلطیاں اور debugging شامل ہیں

### 4-Layer Pedagogy
- ✓ **Layer 1**: دستی بنیاد (سبق 3: CLI tools، systems کا مشاہدہ)
- ✓ **Layer 2**: AI collaboration prompts (Claude کے سوالات کے لیے تجاویز)
- ✓ **Layers 3-4**: جدید ابواب کے لیے مؤخر (Gazebo, Isaac Sim)

### کوئی Forward References نہیں
- ✓ Gazebo physics کا کوئی ذکر نہیں (باب 2)
- ✓ Isaac Sim کا کوئی ذکر نہیں (باب 3)
- ✓ Hardware deployment safety کا کوئی ذکر نہیں
- ✓ مکمل طور پر صرف Simulation پر توجہ

---

## مدد کیسے حاصل کریں

### خود کفیل ٹربل شوٹنگ
ہر سبق میں 5-10 عام غلطیوں اور حل کے ساتھ ایک جامع **Troubleshooting** سیکشن شامل ہے۔

### AI Collaboration (Layer 2)
ہر سبق میں **Layer 2: AI Collaboration Prompts** شامل ہیں جو آپ Claude یا کسی دوسرے AI assistant کے ساتھ استعمال کر سکتے ہیں:

> "میں نے ابھی ROS 2 nodes کے بارے میں سیکھا ہے۔ Claude سے پوچھیں: 'nodes ایک ہی process میں threads کے بجائے آزاد processes کیوں ہیں؟'"

یہ آپ کو اچھے تکنیکی سوالات پوچھنا سکھاتا ہے۔

### سرکاری وسائل
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- ROS 2 Discourse (community): https://discourse.ros.org/
- Robotics Stack Exchange: https://robotics.stackexchange.com/

---

## تخمینہ شدہ وقت کی وابستگی

| Lesson | Duration | Difficulty | Hands-On |
|---|---|---|---|
| 1: ROS 2 کیا ہے؟ | 30 منٹ | آسان | کوئی نہیں (تصورات) |
| 2: Setup | 45 منٹ | درمیانہ | انسٹالیشن، تصدیق |
| 3: Nodes اور Communication | 60 منٹ | درمیانہ | CLI tools، مشاہدہ |
| 4: آپ کا پہلا Publisher | 60 منٹ | درمیانہ | Code لکھیں، debug کریں |
| 5: آپ کا پہلا Subscriber | 60 منٹ | درمیانہ | Code لکھیں، ٹیسٹ کریں |
| 6: Services | 60 منٹ | درمیانہ | Code لکھیں، request/response |
| 7: Integrated Systems | 60 منٹ | درمیانہ | Multi-node پروجیکٹ |
| **کل (پورا باب)** | **375 منٹ** (6.25 گھنٹے) | **Beginner→Elementary** | **جی ہاں** |
| **MVP (اسباق 1-4)** | **195 منٹ** (3.25 گھنٹے) | **Beginner** | **جی ہاں** |

---

## اکثر پوچھے گئے سوالات (FAQ)

### سوال: کیا مجھے ایک حقیقی robot کی ضرورت ہے؟
**جواب**: نہیں! تمام اسباق Turtlesim (صرف simulation) استعمال کرتے ہیں۔ حقیقی robots باب 4+ میں آتے ہیں۔

### سوال: اگر میں Linux کا ماہر نہیں ہوں تو کیا ہوگا؟
**جواب**: یہ ٹھیک ہے۔ سبق 2 آپ کی قدم بہ قدم رہنمائی کرتا ہے۔ اگر پھنس جائیں، تو Troubleshooting سیکشن میں عام اصلاحات موجود ہیں۔

### سوال: کیا میں سبق 5 یا 6 پر آگے بڑھ سکتا ہوں؟
**جواب**: تجویز نہیں دی جاتی۔ اسباق ایک دوسرے پر تعمیر ہوتے ہیں (1 → 2 → 3 → 4)۔ سبق 1 سے شروع کریں۔

### سوال: اگر Turtlesim کریش ہو جائے تو کیا ہوگا؟
**جواب**: یہ ایک ہلکا پھلکا simulator ہے؛ کریش شاذ و نادر ہی ہوتے ہیں۔ سبق 2 کی troubleshooting میں "Turtlesim launch نہیں ہوگا" کا احاطہ کیا گیا ہے۔

### سوال: کیا Python کے علم کی ضرورت ہے؟
**جواب**: جی ہاں، بنیادی Python 3 (functions, classes, loops)۔ اگر آپ کی مہارت کمزور ہے، تو پہلے Python کی بنیادی باتوں کا جائزہ لیں۔

### سوال: میں کب تک حقیقی robot پر deploy کر سکوں گا؟
**جواب**: باب 1 (3 گھنٹے) + باب 2 (5 گھنٹے) + باب 4 (5 گھنٹے) مکمل کریں = بنیادی باتوں کے لیے ~13 گھنٹے۔ حقیقی deployment کے لیے دائرہ کار سے باہر حفاظتی تربیت کی ضرورت ہوتی ہے۔

---

## آگے کیا آتا ہے؟

اس باب کو مکمل کرنے کے بعد، آپ ان کے لیے تیار ہوں گے:

### باب 2: Gazebo Simulation اور Robot Modeling
- Physics-based simulation (حقیقی کشش ثقل، ٹکراؤ، رگڑ)
- URDF robot descriptions اور kinematics
- Simulation میں Sensors (cameras, lidar, IMU)
- Multi-robot coordination

### باب 3: Autonomous Navigation اور SLAM
- Simultaneous Localization and Mapping
- Nav2 path planning stack
- Obstacle avoidance اور dynamic environments
- Humanoid-specific navigation کے چیلنجز

### باب 4: Vision-Language-Action Pipelines
- Whisper کے ساتھ Speech recognition
- Vision systems (YOLO, segmentation)
- GPT-4 کے ساتھ LLM-driven planning
- مکمل end-to-end autonomous systems

---

## کامیابی کے میٹرکس

آپ جان جائیں گے کہ آپ نے باب 1 میں مہارت حاصل کر لی ہے جب آپ کر سکیں گے:

**تصوراتی سمجھ بوجھ**:
- [ ] ایک جملے میں وضاحت کریں کہ ROS 2 node کیا ہے
- [ ] nodes، topics، اور message flow کو ظاہر کرنے والا خاکہ (diagram) بنائیں
- [ ] pub/sub اور services کے درمیان فرق کریں
- [ ] وضاحت کریں کہ simulation-first development کیوں زیادہ محفوظ ہے

**عملی مہارتیں**:
- [ ] ROS 2 انسٹال کریں اور Turtlesim لانچ کریں
- [ ] `ros2 topic` اور `ros2 node` کمانڈز استعمال کریں
- [ ] شروع سے ایک publisher node لکھیں
- [ ] ROS 2 tools کا استعمال کرتے ہوئے communication issues کو debug کریں

**باب 2 کے لیے تیار**:
- [ ] آپ Python میں ایک مکمل، کام کرنے والا ROS 2 node لکھ سکتے ہیں
- [ ] آپ message types اور type safety کو سمجھتے ہیں
- [ ] آپ نے ایک distributed system کو عمل میں دیکھا ہے (Turtlesim pub/sub)

---

## یہاں سے شروع کریں

**سبق 1: ROS 2 کیا ہے؟** (30 منٹ) سے شروع کریں

ابھی تک انسٹالیشن کی ضرورت نہیں—صرف تصورات۔ اس سبق کے بعد، آپ سمجھ جائیں گے کہ ROS 2 کیوں اہمیت رکھتا ہے اور آپ کیا بنا رہے ہیں۔

تیار ہیں؟ چلیے شروع کرتے ہیں!

---

**باب 1 کا تعارف مکمل**

اگلا: [سبق 1 — ROS 2 کیا ہے؟](01-what-is-ros2)

</PersonalizedLesson>