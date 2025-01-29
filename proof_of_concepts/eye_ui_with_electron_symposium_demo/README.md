# Symposium DEMO

This project is a clone of [eye_ui_with_electron](../eye_ui_with_electron/).
Specific parts of the code are edited for the symposium DEMO:
- Other HDMI screens (eye configuration are different: html/css)
- Serving of control panel and app are done on 0.0.0.0 (AND NOT LOCALHOST) this way we can control the eyes remotly. 

TODO.. further explaination why...

Examples:
```bash
#Right screen (robot perspectief) should be primary screen
#Left screen should be secondary screen
#Premade commands
npm start #default right screen right eye left screen left eye
npm run start-left-first #left eye on right screen, right eye on left screen
npm run start-right-first #right eye on right screen, left eye on left screen
npm start-left-only #shows left eye on primary screen
npm start-right-only #shows right eye on primary screen

#Or use manual commands
# value 0 = main screen 
# value 1 = secondary screen
# Note when using secondary screen value, the screen must be connected or else the app will close!
npm start -- --left-eye=0 --right-eye=1 #show left eye on main screen and right eye on secondary
npm start -- --left-eye=1 --right-eye=0 #show right eye on main screen and right eye on secondary
npm start -- --left-eye=0 #show left eye on main screen
npm start -- --right-eye=0 #show right eye on main screen

```

# Server side pc abstract required:
Pre condition (optional): Turn of automatic update.

Pre condition: clean install ubuntu 24.04 with GUI configured on X11 (NOT wayland, this is the default!!)

Pre condition: xrandr (is installed on ubuntu)

Pre condition: Adding configure_screen.sh to startup routing of pc:
```bash
sudo nano /etc/systemd/system/configure-screens.service
```

Content of configure-screens.service should be:
```bash
[Unit]
Description=Configure HDMI screens at boot
After=graphical.target

[Service]
User=hcl
Environment=DISPLAY=:0
Environment=MAIN_ROTATION=left
Environment=SECOND_ROTATION=right
ExecStart=~/ros_social_robot_prototype/proof_of_concepts/eye_ui_with_electron_symposium_demo/configure_screens.sh
Restart=always
RestartSec=5

[Install]
WantedBy=graphical.target
```
Note: 
- "Restart=always → Systemd zal de service altijd opnieuw starten als deze faalt." 
- RestartSec=5 → Systemd wacht 5 seconden voordat het de service opnieuw start.


Activate the serivce
```bash
sudo systemctl daemon-reload #reload
sudo systemctl enable configure-screens.service #add to reboot
sudo systemctl start configure-screens.service #(optional start now)
sudo systemctl status configure-screens.service #(optional check if it is running)
```

When you configure screen from bash (manually) 

```bash
chmod +x configure_screens.sh # So screens can get configured automatically
./configure_screens.sh #default settings (mainscreen = left rotate, secondary screen = right rotate)

#Example with arguments
export MAIN_ROTATION=normal
export SECOND_ROTATION=inverted
./configure_screens.sh

#OR in one line
MAIN_ROTATION=normal SECOND_ROTATION=left ./configure_screens.sh

```

Precondition: Overview of nessecery commands for server side
```bash
sudo apt install openssh-server # install openssh server
sudo systemctl status ssh #check if SSH-service is running
sudo systemctl start ssh # start ssh service
sudo systemctl enable ssh # ensure that SSH starts automatically on a reboot
sudo systemctl restart ssh # restart ssh service
```

# Client side pc abstract required:

Connect to server
```bash
ssh username@<IP-adres> # username=hcl password=<login of pc>

```

When electron is not automatically started we can start it manualy:
```bash
#go to demo directory
cd ros_social_robot_prototype/proof_of_concepts/eye_ui_with_electron_symposium_demo/
#start electon app (as main process so we can abort it via or own terminal)
DISPLAY=:0 npm start #start electron app in main display
# or start electon app as background process
DISPLAY=:0 npm start &
```

To see connected screen remotly:
```bash
DISPLAY=:0 xrandr --query
```