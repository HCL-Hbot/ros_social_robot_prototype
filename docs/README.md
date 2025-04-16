# Docs

SRS = Software Requirements Specification document
SDD = Software Design Description document
micro_ros_rapport.pfd = A guide ``what is micro-ros``

De laatste versie van het SDD bevat geen documentatie voer de audio. Dit is gerealiseerd na de inlevering. Alleen een readme hiervan is te vinden in de packages.

Todo: geupdate SDD nog uploaden (alleen compontent en interface beschrijving van audio is dan verwerkt in SDD)

Compiler setting "XeLatex"

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
| S-001  | De software moet modulair gebouwd zijn zodat toekomstige studenten makkelijk uitbreidingen kunnen aanbrengen. | Must-have | Ja | – |
| S-002  | De software moet gedocumenteerd zijn. | Must-have | Ja | Het audio component is na de oplevering gerealiseerd, hiervan zijn alleen README's aanwezig met API beschrijving. |
| S-003  | De software moet gebouwd zijn voor ROS2 Jazzy. | Must-have | Ja | Huidige ontwikkeling is gebaseerd op ROS2 Jazzy. |

---

