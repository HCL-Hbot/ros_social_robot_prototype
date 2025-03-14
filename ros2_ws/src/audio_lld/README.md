# 🎵 ROS2 Audio File Player Node

This ROS2 node provides an **audio playback service** using **GStreamer**. It allows playing, pausing, resuming, and stopping audio files via ROS2 services. 

The node supports **custom sample rates** and **audio device selection**, allowing flexible playback options.

This node acts as a ``low level audio driver`` for the ros_social_robot_prototype.

---

## 🚀 Features

- 🎧 **Play audio files** from a given file path.
- ⏸️ **Pause and resume** playback.
- 🚫 **Stop** playback at any time.
- 🔊 **Set volume** when requesting playback.
- 🎧 **Configure sample rate** (default: 48 kHz when choosing a device output).
- 🔹 **Choose an audio output device** (ALSA-supported hardware or auto-detected).

---

## 📊 **Required Installation**
To use this node, ensure the following dependencies are installed:

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

### 📂 **Why These Packages Are Required?**

| **Package**                           | **Function** | **Why Needed?** |
|---------------------------------------|------------|-------------------|
| `libgstreamer1.0-dev`                 | 🛠️ **Core API** | Provides the GStreamer C++ API, essential for integrating GStreamer with ROS2. |
| `libgstreamer-plugins-base1.0-dev`    | 🔧 **Basic Plugins** | Includes key elements such as `decodebin`, `audioconvert`, and `autoaudiosink`, necessary for audio playback. |
| `gstreamer1.0-tools`                  | 🛠️ **CLI Debugging Tools** | Contains `gst-launch-1.0` and `gst-inspect-1.0`, useful for testing and debugging GStreamer pipelines. |
| `gstreamer1.0-alsa`                   | 🔊 **ALSA Audio Output** | Enables playback through **ALSA**, the default Linux sound system. |
| `gstreamer1.0-plugins-ugly`           | 🎵 **MP3 Support** | Provides the **MP3 decoder (`mad`)**, required to play MP3 files. |
| `gstreamer1.0-libav`                  | 🎥 **FFmpeg Integration** | Adds FFmpeg-based decoders, supporting **AAC, MP3, and H.264** for extended format compatibility. |

---

## 📚 **Build Instructions**
Ensure your ROS2 workspace is set up, then build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select audio_lld
source install/setup.bash
```

---

## 🎧 **Understanding Audio Device and Sample Rate Behavior**

### **Audio Device Selection**
- If an **audio device is specified**, the application **automatically searches** for that output device and uses it.
- The format for specifying an **(ALSA) audio device** is:
  ```bash
  hw:<card_number>,<device_number>
  ```
  Example:
  ```bash
  hw:0,0
  ```
- To find available ALSA devices, run:
  ```bash
  aplay -l
  ```
  Example output of my USB speaker:
  ```
  card 1: HID [USB Audio and HID], device 0: USB Audio [USB Audio]
    Subdevices: 0/1
    Subdevice #0: subdevice #0
  ```
  In this case, the correct format would be `hw:1,0`.

### **Sample Rate Behavior**
- **If an audio device is specified**, the node will attempt to apply the **configured sample rate**. The default value is 48kHz.
- **If no audio device is specified**, the application **automatically detects** an output device (`autoaudiosink`).
  - In this case, **the sample rate setting is ignored**.
  - The system selects the **best sample rate** based on the detected output device.
  - **Note**: The automatically detected device is the current Sound output device on your Linux system.
    -   Current system sound output device for Ubuntu can be found at: Settings->Sound->Output->Output Device.
- This means that if you want to **enforce a specific sample rate**, you **must specify an audio device**.

---

## 📝 **Usage**

### **1️⃣ Run the Node**
Start the audio player node:

```bash
ros2 run audio_lld audio_file_player_node
```

By default:
- The **audio output device** is **auto-detected** (`autoaudiosink`).
- The **sample rate is ignored** unless an audio device is manually set.

---

### **2️⃣ Play an Audio File**
To play an audio file, use the following **service call**:

```bash
ros2 service call /play_audio_file audio_lld/srv/PlayAudioFile "{file_path: '/absolute/path/to/audio.mp3', volume: 50}"
```

✅ **Parameters:**
- `file_path`: **Full path** to the audio file.
- `volume`: **0 to 100** (percentage of max volume).

---

### **3️⃣ Pause Playback**
```bash
ros2 service call /pause_audio_file std_srvs/srv/Trigger "{}"
```

---

### **4️⃣ Resume Playback**
```bash
ros2 service call /resume_audio_file std_srvs/srv/Trigger "{}"
```

---

### **5️⃣ Stop Playback**
```bash
ros2 service call /stop_audio_file std_srvs/srv/Trigger "{}"
```

---

## ⚙️ **Configuration Options**

### **Set the Sample Rate**
If an audio device is provided, you can configure the sample rate:

```bash
ros2 param set /audio_file_player sample_rate 44100
```

Verify the change:
```bash
ros2 param get /audio_file_player sample_rate
```

Or set it **when launching the node**:

```bash
ros2 run audio_lld audio_file_player_node --ros-args -p sample_rate:=44100
```

---

### **Set a Specific Audio Output Device**
If you want to enforce a specific **ALSA audio output device**, use:

```bash
ros2 run audio_lld audio_file_player_node --ros-args -p audio_device:=hw:1,0
```

If no device is set, the node defaults to **autoaudiosink**, and the sample rate is determined automatically.

---

## 🔧 **Check Available Parameters**
To see all parameters currently available in the node:

```bash
ros2 param list /audio_file_player
```

---

## 📉 **Summary of Commands**

| Action | Command |
|--------|---------|
| **Start node (default)** | `ros2 run audio_lld audio_file_player_node` |
| **Start node with all parameters** | `ros2 run audio_lld audio_file_player_node --ros-args -p sample_rate:=16000 -p audio_device:=hw:1,0` |
| **Start node with choosen device and default samplerate (48kHz)** | `ros2 run audio_lld audio_file_player_node --ros-args -p audio_device:=hw:1,0`|
| **Play audio file** | `ros2 service call /play_audio_file audio_lld/srv/PlayAudioFile "{file_path: '/path/to/audio.mp3', volume: 50}"` |
| **Pause playback** | `ros2 service call /pause_audio_file std_srvs/srv/Trigger "{}"` |
| **Resume playback** | `ros2 service call /resume_audio_file std_srvs/srv/Trigger "{}"` |
| **Stop playback** | `ros2 service call /stop_audio_file std_srvs/srv/Trigger "{}"` |
| **Set sample rate** | `ros2 param set /audio_file_player sample_rate 48000` |
| **List available parameters** | `ros2 param list /audio_file_player` |

---

