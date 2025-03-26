# Brief Summary
colcon build will automatically trigger npm install.

Some npm packages are choosen with a specifically with a version number. 

- `electron` is choosen with v32 because this was last version which was compatible with the latest version of rclcnodejs

- `rclcnodejs` is choosen with v0.29. This was the most recent version at the moment of development (and is ofcourse compatible with ros jazzy)

- `@electron/rebuild` is choosen with v3.7.1. This was the most recent version at the moment of development.

```json
  "devDependencies": {
    "@electron/rebuild": "^3.7.1",
    "concurrently": "^9.1.2",
    "electron": "^32.0.0", //Will install 32.3 (latest of v32)
    "electron-reload": "^2.0.0-alpha.1",
    "sass": "^1.83.1"
  },
  "dependencies": {
    "rclnodejs": "0.29.0" //Lets stick to only this version, don't accept higher version.  
  }
```

## Build manually without colcon build / cmake

```bash
cd src # go to the "app folder" (where package.json is)
npm install #install all the required packages

# Every time you run "npm install" after adding new packages, run this:
./node_modules/.bin/electron-rebuild #recompile packages with the nodejs version of our current electron app. This way all packages will be compatible.
```

## Run from ros cli
```bash

```

## Executing application outside vscode terminal (chrome-sandbox)
Chrome sandbox is needed outside visual studio code
1. Zet het bestand in eigendom van root:
```bash
sudo chown root /home/hcl/Documents/ros_social_robot_prototype/ros2_ws/src/eye_display_lld/src/node_modules/electron/dist/chrome-sandbox
```
2.Stel de juiste permissies in (setuid root):
```bash
sudo chmod 4755 /home/hcl/Documents/ros_social_robot_prototype/ros2_ws/src/eye_display_lld/src/node_modules/electron/dist/chrome-sandbox
```