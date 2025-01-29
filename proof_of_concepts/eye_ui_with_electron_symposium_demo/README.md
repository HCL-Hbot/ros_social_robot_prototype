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