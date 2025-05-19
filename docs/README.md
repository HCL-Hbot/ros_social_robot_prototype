# 📄 Docs

> This page is written in DUTCH, because the documentation is written in dutch!!!!

> Deze pagina is geschreven in het **Nederlands**, omdat de documentatie ook in het Nederlands is opgesteld!

Deze map bevat de documentatie van het project en een overzicht van welke requirements zijn gerealiseerd.

## Inhoud

- **SRS**: *Software Requirements Specification* document  
- **SDD**: *Software Design Description* document  
- **`micro_ros_rapport.pdf`**: Een handleiding waarin beschreven staat wat micro_ros is  
- **`old_sdd/`**: Map met eerdere versies van het SDD-document  
- **`diagrammen/`**: Map met alle UML-diagrammen in PlantUML-formaat  
  - Verdeeld in oud/nieuw  
  - Binnen deze mappen ook geordend per component  
- **`sdd_overleaf_workspace/`**: Bevat de LaTeX-workspace voor Overleaf om de SDD te schrijven  
- **`srs_overleaf_workspace/`**: Bevat de LaTeX-workspace voor Overleaf om de SRS te schrijven  

## ✅ Overleaf Let op

Wanneer je in Overleaf de documenten wilt compileren:

- Kies als **compiler**: `XeLaTeX`  
- Stel als **main document** in: `document.tex`

## 🎧 Opmerking over audio-component

De laatste versie van het SDD bevat **geen** gedetailleerde documentatie over het audio-component. Alleen op architectuurniveau wordt dit kort besproken, omdat dit onderdeel pas na de initiële oplevering is gerealiseerd.

Voor de actuele en gedetailleerde documentatie van het audio-design, zie:

- [`audio_lld` package](../ros2_ws/src/audio_lld/README.md)  
- [`audio_hld` package](../ros2_ws/src/audio_hld/README.md)

# Requirements check table

BuddyBot/Mika is een sociale robot die gebruik maakt van radar-, audio- en visuele sensoren om te interacteren met mensen in zijn omgeving. Dit document bevat een overzicht van de requirements voor het systeem, inclusief de implementatiestatus en toelichting.

---

## 📋 Overzicht van Requirements

De requirements zijn onderverdeeld in de volgende categorieën:

- [Functionele Requirements](#functionele-requirements--buddybot)
- [Usability Requirements](#usability-requirements--buddybot)
- [Reliability Requirements](#reliability-requirements--buddybot)
- [Performance Requirements](#performance-requirements--buddybot)
- [Supportability Requirements](#supportability-requirements--buddybot)

---

Todo: Bij toelichting aangeven waar de requirement is gerealiseerd

## ✅ Use cases 
De use cases kunnen gevonden worden in het SRS document.
| Use case nr | Waar is dit gerealiseerd? |
|-------------|---------------------------|
| UC 1  | In het mapje [robot_start_up](../scripts/robot_start_up/)|
| UC 2  | Samen werking van de [radar_lld](../micro_ros_platform_io_ws/src/), [radar_hld](../ros2_ws/src/radar_presence_hld/) en [interaction_controller](../ros2_ws/src/interaction_controller/). |
| UC 3  | Samen werking van de [camera_lld](../ros2_ws/src/camera_lld/), [camera_hld](../ros2_ws/src/camera_hld/), [interaction_controller](../ros2_ws/src/interaction_controller/), [eye_display_hld](../ros2_ws/src/eye_display_hld/) en [eye_display_lld](../ros2_ws/src/eye_display_lld/) |
| UC 4  | Samen werking van de benoemde packages uit UC 2 en 3 plus [audio_hld](../ros2_ws/src/audio_hld/) en [audio_hld](../ros2_ws/src/audio_lld/) |
| UC 5  | Het zelfde als UC 4 |

## ✅ Functionele Requirements – BuddyBot

| Code   | Beschrijving | Prioriteit   | Voldaan | Toelichting |
|--------|--------------|--------------|---------|-------------|
| F-001  | De aanwezigheid detectie moet plaatsvinden via radaraanwezigheidssensoren. | Must-have | Ja | – |
| F-002  | De dichtstbijzijnde afstand-detectie van de radarsensoren wordt beschouwd als een persoon. | Must-have | Ja | – |
| F-003  | De communicatie tussen de ESP32 en de mini-PC moet via USB verlopen. | Must-have | Ja | – |
| F-004  | De gezichtsdetectie moet minimaal tot 1 meter afstand werken. | Must-have | Ja | – |
| F-005  | De robot moet een opgegeven audiobestand (mp3) kunnen afspelen en stoppen. | Must-have | Ja | – |
| F-006  | De robot moet een opgegeven audiobestand (mp3) kunnen pauzeren en hervatten. | Should-have | Ja | Alleen in low level driver |
| F-007  | De robot moet een opgegeven audiobestand (mp3) kunnen pauzeren en hervatten. | Should-have | Ja | Alleen in low level driver |
| F-008  | De ogen van de robot moeten periodiek knipperen. | Should-have | Nee | – |
| F-009  | De pupillen van de ogen moeten afhankelijk van de afstand tot de ogen vernauwen of vergroten. Dichterbij = vergroten, verderaf = vernauwen. | Should-have | Ja | – |
| F-010  | De robot moet zijn nek kunnen bewegen met servo-motoren om de gebruiker fysiek te volgen. | Could-have | Nee | – |
| F-011  | De robot moet emotionele expressies kunnen tonen op de LCD-schermen. | Could-have | Nee | – |
| F-012  | De robot moet een animatie kunnen tonen op de LCD-schermen. | Could-have | Nee | – |
| F-013  | De robot moet voorzien zijn van een microfoon en de opgepikte audio hiervan realtime kunnen laten horen. | Could-have | Nee | – |
| F-014  | De robot kan een opgegeven tekst input afspelen als audio. | Could-have | Nee | – |
---

## 🧪 Usability Requirements – BuddyBot

| Code   | Beschrijving | Prioriteit   | Voldaan | Toelichting |
|--------|--------------|--------------|---------|-------------|
| U-001  | De robot moet voor ontwikkelaars via SSH beschikbaar zijn. | Must-have | Ja | – |
| U-002  | Audio-uitvoer moet verstaanbaar zijn. | Must-have | Ja | – |

---

## 💪 Reliability Requirements – BuddyBot

Er zijn geen reliability requirements vereist voor deze uitvoering van het project.

---

## 🚀 Performance Requirements – BuddyBot

Er zijn geen performance requirements vereist voor deze uitvoering van het project.

---

## 🔧 Supportability Requirements – BuddyBot

| Code   | Beschrijving | Prioriteit   | Voldaan | Toelichting |
|--------|--------------|--------------|---------|-------------|
| S-001  | De software moet modulair gebouwd zijn zodat toekomstige studenten makkelijk uitbreidingen kunnen aanbrengen. | Must-have | Ja | Er zijn uitwerkingen gemaakt met ROS topics, service en action service. Daarnaast ook een micro-ros node. (Zowel voor eindproduct als een proof-of-concept) |
| S-002  | De software moet gedocumenteerd zijn. | Must-have | Ja | Het audio component is na de oplevering gerealiseerd, hiervan zijn alleen README's aanwezig met API beschrijving. |
| S-003  | De software moet gebouwd zijn voor ROS2 Jazzy. | Must-have | Ja | Huidige ontwikkeling is gebaseerd op ROS2 Jazzy. |

---



# Demo

In het volgende [filmpje](./demo_robot.mp4) is een demo van MIKA te zien. Het volgende wordt hier mee aangetoond:

- Software handmatig opstarten via SSH
- Openen en sluiten van ogen.
- Gebruiker volgen met ogen.
- Pupillen vernouwen en verwijden op basis van afstand. Goed de ogen bekijken tijdens de video, ik vertel namelijk niks hierover kom ik achter.
- Groeten (bij aankomst en vertrek)

Wat niet te zien is:
- Automatisch opstarten van de software bij opstarten van robot/pc. In de video start ik alles handmatig op.


