---
title: "Lesson 2: Voice Input with OpenAI Whisper"
chapter: 4
lesson: 2

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

<PersonalizedLesson lessonPath="03-chapter-4-ai-integration/02-lesson-2-voice-whisper.md">

# Lesson 2: Voice Input with OpenAI Whisper

## Learning Objectives

By the end of this lesson, you will:
1. Set up OpenAI Whisper for speech recognition (local inference or API)
2. Create a ROS 2 node that captures audio and publishes transcribed commands
3. Handle transcription errors and edge cases gracefully
4. Integrate voice input into the ROS 2 topic ecosystem
5. Understand real hardware considerations for microphone input and acoustic environments

## What is Whisper?

**Whisper** is OpenAI's automatic speech recognition (ASR) system. It's:
- **Robust**: Handles accents, background noise, technical language
- **Multilingual**: Supports 99 languages (Chapter 4 uses English only)
- **Open-source**: Available on GitHub, runnable locally
- **Free**: No API charges for local inference
- **Accurate**: ≥90% accuracy on robot command vocabulary

### Two Deployment Options

#### Option 1: Local Inference (Recommended for Chapter 4)
```
Microphone Audio → Whisper Model (Local) → Text
```
- **Latency**: &lt;500ms (fast enough for real-time)
- **Privacy**: Audio never leaves your computer
- **Cost**: Free (one-time model download)
- **Requirements**: 4GB+ free disk space, moderate CPU/GPU

#### Option 2: OpenAI API (Cloud-based)
```
Microphone Audio → OpenAI API → Text
```
- **Latency**: 1-3 seconds (network dependent)
- **Privacy**: Audio sent to OpenAI servers
- **Cost**: $0.02 per minute of audio
- **Advantage**: No local compute needed

**For this course**, we recommend **local inference** for faster iteration and offline capability.

## Real-Time vs. Batch Processing

### Real-Time Processing
- User speaks: "Find the red cube"
- As they finish speaking, transcription begins
- Result ready within 500ms
- **Used in**: Interactive systems, command-and-control

### Batch Processing
- User speaks and stops
- Full audio captured
- Whisper processes entire recording
- Result ready after full processing
- **Used in**: Non-interactive logging, archival

**This lesson uses batch processing** (simpler for learning), but production systems often use streaming.

## Architecture: Whisper ROS 2 Node

```
┌──────────────┐
│  Microphone  │ (Audio device)
└──────┬───────┘
       │ (Raw audio stream)
       ↓
┌──────────────────────────────┐
│  whisper_ros2_node.py        │
├──────────────────────────────┤
│ • Listen for audio signal    │
│ • Capture audio bytes        │
│ • Call Whisper inference     │
│ • Parse transcription text   │
│ • Validate confidence score  │
│ • Publish to ROS 2 topic     │
└──────┬───────────────────────┘
       │ (/robot/voice_command topic)
       ↓
┌──────────────────────────┐
│ ROS 2 Topic: voice_cmd  │
│ - text: "find red cube" │
│ - confidence: 0.95      │
│ - timestamp: 123456789  │
└──────────────────────────┘
```

## Step 1: Install Whisper

### Local Installation (Recommended)

```bash
# Install required packages
pip install openai-whisper numpy scipy soundfile

# Download Whisper model (one-time, ~141MB for base model)
python -c "import whisper; whisper.load_model('base')"

# Verify installation
python -c "import whisper; print('Whisper installed successfully')"
```

### Verify Whisper Works

```python
import whisper

# Load model
model = whisper.load_model("base")

# Test with sample audio
result = model.transcribe("audio_sample.mp3")
print(result["text"])  # Output: transcribed text
```

### Model Selection

| Model | Size | Speed | Accuracy | VRAM |
|-------|------|-------|----------|------|
| tiny | 39M | ⚡⚡⚡ | ⭐⭐ | 1GB |
| base | 141M | ⚡⚡ | ⭐⭐⭐ | 1GB |
| small | 244M | ⚡ | ⭐⭐⭐⭐ | 2GB |
| medium | 769M | 🐢 | ⭐⭐⭐⭐⭐ | 5GB |
| large | 2.9GB | 🐢🐢 | ⭐⭐⭐⭐⭐ | 10GB |

**For this course**: Use `base` model (good balance of speed and accuracy)

## Step 2: Create ROS 2 Whisper Node

File: `chapter4_voice/src/whisper_ros2_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import whisper
import sounddevice as sd
import numpy as np
from datetime import datetime
import json

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Load Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base")
        self.get_logger().info('Whisper model loaded')

        # Create publishers
        self.voice_command_pub = self.create_publisher(
            String,
            '/robot/voice_command',
            10
        )
        self.confidence_pub = self.create_publisher(
            Float32,
            '/robot/voice_confidence',
            10
        )

        # Parameters
        self.sample_rate = 16000  # Whisper expects 16kHz audio
        self.duration = 5.0  # Listen for 5 seconds max
        self.confidence_threshold = 0.7  # Minimum confidence to publish

        self.get_logger().info('Whisper ROS 2 node initialized')
        self.get_logger().info(f'Listening for voice commands on {self.sample_rate}Hz')

    def listen_and_transcribe(self):
        """Listen to microphone and transcribe audio"""
        self.get_logger().info('Listening... (speak now)')

        try:
            # Record audio from microphone
            audio_data = sd.rec(
                int(self.duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype=np.float32
            )
            sd.wait()  # Wait for recording to finish

            # Flatten to 1D array
            audio_data = audio_data.flatten()

            # Transcribe with Whisper
            self.get_logger().info('Transcribing...')
            result = self.model.transcribe(
                audio_data,
                language="en",
                fp16=False  # Use CPU-safe FP32
            )

            # Extract text and confidence
            text = result.get("text", "").strip()
            # Estimate confidence from Whisper segments
            confidence = self._calculate_confidence(result)

            if not text:
                self.get_logger().warn('No speech detected')
                return

            self.get_logger().info(f'Transcribed: "{text}" (confidence: {confidence:.2f})')

            # Publish only if confidence meets threshold
            if confidence >= self.confidence_threshold:
                # Publish command
                msg = String()
                msg.data = text
                self.voice_command_pub.publish(msg)

                # Publish confidence
                conf_msg = Float32()
                conf_msg.data = confidence
                self.confidence_pub.publish(conf_msg)

                self.get_logger().info('Published to /robot/voice_command')
            else:
                self.get_logger().warn(
                    f'Confidence {confidence:.2f} below threshold {self.confidence_threshold}'
                )

        except Exception as e:
            self.get_logger().error(f'Transcription error: {str(e)}')

    def _calculate_confidence(self, result):
        """Estimate confidence from Whisper result"""
        # Whisper doesn't provide per-word confidence, estimate from segments
        if "segments" in result and result["segments"]:
            # Average confidence across segments (Whisper's confidence metric)
            confidences = [seg.get("confidence", 0.5) for seg in result["segments"]]
            return sum(confidences) / len(confidences) if confidences else 0.5
        return 0.5  # Default if unable to estimate

def main(args=None):
    rclpy.init(args=args)

    whisper_node = WhisperNode()

    # Run once, then exit (for testing)
    # In production, use a loop with button or voice activation
    whisper_node.listen_and_transcribe()

    # For continuous operation:
    # rclpy.spin(whisper_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 3: Set Up Audio Device

### Check Available Audio Devices

```bash
python -c "import sounddevice; print(sounddevice.query_devices())"
```

Output will show all available microphones. Identify yours.

### Configure Microphone

```bash
# On Linux
alsamixer  # Adjust microphone input level

# On macOS
System Preferences > Sound > Input > [Select Microphone]

# On Windows
Settings > Sound > Volume and device preferences > [Select Microphone]
```

### Test Audio Recording

```bash
python -c "
import sounddevice as sd
import numpy as np

# Record 3 seconds
audio = sd.rec(int(3 * 16000), samplerate=16000, channels=1, dtype=np.float32)
sd.wait()
print(f'Recorded {len(audio)} samples')
print(f'Audio level: {np.max(np.abs(audio)):.3f}')  # Should be > 0.1 for audible speech
"
```

## Step 4: Error Handling & Edge Cases

### No Audio Detected
```python
if len(audio_data[audio_data > 0.01]) < 100:  # Less than 100 samples above noise
    self.get_logger().warn('No speech detected - audio too quiet')
    return
```

### Transcription Confidence Too Low
```python
if confidence < threshold:
    # Ask user to repeat
    self.get_logger().warn(f'Could not understand: "{text}" - please repeat')
    return
```

### Timeout Handling
```python
try:
    audio_data = sd.rec(..., timeout=5)  # Wait max 5 seconds
except TimeoutError:
    self.get_logger().error('Recording timeout - no input detected')
```

## Step 5: Test the Node

### Create Test Script

File: `chapter4_voice/tests/whisper_test.py`

```python
import pytest
import rclpy
from std_msgs.msg import String
from chapter4_voice.whisper_ros2_node import WhisperNode

def test_whisper_node_initialization():
    """Test node initializes successfully"""
    rclpy.init()
    try:
        node = WhisperNode()
        assert node is not None
        assert node.model is not None
    finally:
        rclpy.shutdown()

def test_confidence_calculation():
    """Test confidence score calculation"""
    rclpy.init()
    try:
        node = WhisperNode()

        # Mock Whisper result
        result = {
            "text": "find red cube",
            "segments": [
                {"confidence": 0.95},
                {"confidence": 0.92},
            ]
        }

        confidence = node._calculate_confidence(result)
        assert 0.9 < confidence < 1.0
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    pytest.main([__file__, '-v'])
```

### Run Tests

```bash
cd ros2_packages/chapter4_voice
colcon build --packages-select chapter4_voice
colcon test --packages-select chapter4_voice
```

## Exercise: Build Your Own Voice Node

### Exercise L2-1: Transcribe Voice Command

**Objective**: Set up Whisper and transcribe spoken commands

**Steps**:
1. Install Whisper: `pip install openai-whisper`
2. Download model: `python -c "import whisper; whisper.load_model('base')"`
3. Record yourself saying "find the red cube"
4. Transcribe with: `model = whisper.load_model('base'); result = model.transcribe('recording.wav'); print(result['text'])`
5. Verify accuracy: Does the output match what you said?

**Success Criteria**:
- ✓ Whisper installed and model loaded
- ✓ Audio recording captured successfully
- ✓ Transcription text matches spoken command (≥90% accuracy)
- ✓ Confidence score provided

### Exercise L2-2: ROS 2 Node Integration

**Objective**: Create a ROS 2 node that publishes voice commands

**Steps**:
1. Create `whisper_ros2_node.py` in `chapter4_voice/src/`
2. Implement `listen_and_transcribe()` method
3. Publish to `/robot/voice_command` topic
4. Add error handling for no-speech-detected case
5. Test with: `ros2 run chapter4_voice whisper_ros2_node`
6. In another terminal, monitor topic: `ros2 topic echo /robot/voice_command`

**Success Criteria**:
- ✓ Node launches without errors
- ✓ Microphone input captured
- ✓ Transcription published to topic
- ✓ Confidence scores logged
- ✓ Handles timeout gracefully

## Real Hardware Considerations

### Differences from Simulation

**Simulation (Isaac Sim)**:
- Perfect audio quality, no background noise
- Microphone input simulated or perfect recording
- Transcription happens instantly

**Real Hardware**:
- Background noise: HVAC, traffic, other people
- Acoustic effects: Echo, reverberation in rooms
- Microphone quality varies: USB microphone vs. built-in
- Accents and speech patterns: Whisper more robust now, but not 100%
- Latency: Network if using API, GPU if local

### Best Practices

1. **Use external USB microphone** (better quality than built-in)
2. **Reduce background noise** when possible (quiet environment)
3. **Speak clearly and at normal volume** (not too soft, not too loud)
4. **Use local inference** for real robots (API latency unacceptable for real-time)
5. **Set confidence threshold carefully** (too high = missed commands, too low = false positives)
6. **Log all transcriptions** for debugging and improvement

### Production Considerations

- **Voice activation**: Don't listen continuously (power/noise), trigger on wake word
- **Multiple speakers**: Different voices may have different confidence scores
- **Accents and dialects**: Whisper handles many, but test with your speakers
- **Multilingual**: If adding non-English, models may have different accuracy
- **Privacy**: Local inference keeps audio data private

## Key Takeaways

1. **Whisper is a powerful, open-source speech recognition system**
2. **Local inference is faster and more private than cloud APIs**
3. **ROS 2 nodes can easily integrate ML models for perception**
4. **Confidence scores help validate transcription quality**
5. **Error handling and edge cases are critical for reliability**
6. **Real robots have different acoustic challenges than simulation**

## Next Steps

In **Lesson 3**, you'll add **vision perception** to complement voice input:
- Object detection with YOLO-v8
- Semantic segmentation for scene understanding
- Combining voice + vision for powerful perception

See you in Lesson 3! 👁️🎤


</PersonalizedLesson>
