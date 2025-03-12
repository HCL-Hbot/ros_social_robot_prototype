# Required installation

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

# ROS2 Audio Playback Node met GStreamer

Deze ROS2 node maakt gebruik van **GStreamer** om audiobestanden af te spelen en te stoppen via ROS2 topics en services.  
Hieronder staat een overzicht van de GStreamer pakketten die nodig zijn voor audio playback.

## 📦 Vereiste GStreamer Packages

| **Package**                           | **Functie** | **Waarom nodig?** |
|---------------------------------------|------------|-------------------|
| `libgstreamer1.0-dev`                 | 🎛️ **Core API** | Bevat de basis C++ API van GStreamer, nodig om GStreamer in ROS2 te gebruiken. |
| `libgstreamer-plugins-base1.0-dev`    | 🔧 **Basis plugins** | Bevat standaard plugins zoals `decodebin`, `audioconvert` en `autoaudiosink`, die nodig zijn om audio af te spelen. |
| `gstreamer1.0-tools`                  | 🛠️ **Test CLI-tools** | Bevat `gst-launch-1.0` en `gst-inspect-1.0` waarmee je de installatie kunt testen en pipelines kunt debuggen. |
| `gstreamer1.0-alsa`                   | 🔊 **ALSA Audio Output** | Nodig om audio via het **ALSA** geluidsysteem (standaard voor Linux) af te spelen. |
| `gstreamer1.0-plugins-ugly`           | 🎵 **MP3 ondersteuning** | Bevat de **MP3 decoder (`mad`)**, die nodig is om MP3-bestanden af te spelen. |
| `gstreamer1.0-libav`                  | 🎥 **FFmpeg integratie** | Bevat FFmpeg-gebaseerde decoders, zoals **AAC, MP3, en H.264**, voor extra audioformaten. |

## 🛠 Installatie

Gebruik het volgende commando om alle benodigde GStreamer pakketten te installeren:
```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

## Voorbeeld gebruik
Run ros node
```bash
ros2 run audio_lld audio_lld_node
```

Play a file
```bash
ros2 topic pub --once /audio_playback/play std_msgs/msg/String "{data: '/home/hcl/Downloads/hello.mp3'}"
```