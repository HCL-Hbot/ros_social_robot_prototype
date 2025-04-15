# audio_hld

`audio_hld` is a ROS 2 package that contains the `AudioManagerNode`, a high-level component responsible for triggering predefined audio playback commands through an action interface. It delegates actual audio playback to the `audio_lld` package.

---

## 🧠 Node Functionality

The `AudioManagerNode`:
- Accepts a ROS 2 action request containing a `SoundCommand`, a repeat count, and a delay between repetitions.
- Uses a **compile-time mapping** to associate each sound command with a specific audio file.
- Sends service requests to `audio_lld::PlayAudioFile` to perform the playback.
- Supports playback commands such as:
  - `GREET`
  - `FAREWELL`
- Interrupts any currently playing audio when a **new request** is received.
- Supports stopping audio playback by canceling the current action goal.
- Publishes feedback between each repetition and returns a result with execution details.

---

## 🧱 Dependencies

- ROS 2 Jazzy
- [`audio_lld`](../audio_lld) package (for actual playback)
- Action interface: `audio_hld/action/PlaySound.action`
- Message interface: `audio_hld/msg/SoundCommand.msg`

---

## 🛠️ Building

```bash
cd ~/ros2_ws
colcon build --packages-select audio_hld audio_lld
source install/setup.bash
```

---

## 🚀 Usage

### Start the node(s):
```bash
ros2 launch audio_hld audio_launch.py
```

### Send an action goal:
```bash
ros2 action send_goal /play_sound audio_hld/action/PlaySound "{command: 0, repeat_count: 2, repeat_delay_sec: 1.0}"
```

> Use `0` for `GREET`, `1` for `FAREWELL`. These constants are defined in `SoundCommand.msg`.

> Sending a new action request while another one is active will cancel the current playback.

---

## 🧩 Compile-time Mapping

All sound command-to-file path mappings are defined at compile-time in:
```cpp
include/audio_hld/sound_command_mapping.hpp
```

Example:
```cpp
static const std::map<uint8_t, std::string> sound_command_to_file_path_map = {
  {audio_hld::msg::SoundCommand::GREET, "/path/to/greeting.mp3"},
  {audio_hld::msg::SoundCommand::FAREWELL, "/path/to/farewell.mp3"}
};
```

---

## 📬 Action Result and Feedback

The action server provides:
- `success`: whether the command completed successfully
- `executed_count`: number of repetitions that were completed
- `message`: additional context (e.g., "Goal was canceled")

---