const { app, screen, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
let rclnodejs = require("rclnodejs");

// Check if the app is running in development mode (for hot reloading)
if (process.env.NODE_ENV === 'development') {
  const electronReload = require('electron-reload');
  electronReload(__dirname, {
    electron: path.join(__dirname, 'node_modules', '.bin', 'electron'),
  });
}
//---------------------------------------------------

const windows = {}; // Object to save windows (left, right, both)

// Listen for log messages from the renderer process
ipcMain.on('log-message', (event, message) => {
  console.log('Renderer Log:', message);
});

// Function to create a BrowserWindow for a specific screen
function createWindowForScreen(display, eye) {
  const window = new BrowserWindow({
    x: display.bounds.x, // Set the window's position to match the display (For good practice we do this always explicitly, but when we use fullscreen it will be done implecitly)
    y: display.bounds.y, //(For good practice we do this always explicitly, but when we use fullscreen it will be done implecitly)
    width: display.bounds.width,
    height: display.bounds.height,
    frame: false, // (False = Remove title bar and window frame)
    fullscreen: true, // (True = Fullscreen mode)
    autoHideMenuBar: true,   // Hide the menu bar
    alwaysOnTop: false,    // Ensure the window stays on top, for now on false. But might be useful to set it to true for production!!!!
    webPreferences: {
      preload: path.join(__dirname, 'renderer.js'), // Optional preload script
      nodeIntegration: true
    },
  })
  window.loadURL(`file://${__dirname}/index.html?eye=${eye}`);
  windows[eye] = window; // Store in the windows object
  console.log(`Window created for ${eye}`);
  return window;
}

function setupScreens() {
  const displays = screen.getAllDisplays();
  const options = parseArguments();
  const argumentCount = Object.keys(options).length;
  console.log('Arguments:', argumentCount);

  if(displays.length === 0)
  {
    console.error('No screens detected. Exiting.');
    process.exit(1); // Beëindig de applicatie onmiddellijk met een foutcode
  }

  if (options.leftEye === undefined && options.rightEye === undefined) {
    console.log("No arguments provided, go with default settings");
    if(displays.length > 1)
    {
      createWindowForScreen(displays[0], 'left');
      createWindowForScreen(displays[1], 'right');
    }
    else
    {
      createWindowForScreen(displays[0], 'both');
    }
    // console.log('Windows object:', windows); // Debugging
    return;
  }
  if (options.leftEye !== undefined) {
    const leftDisplay = displays[options.leftEye];
    if (leftDisplay) {
      createWindowForScreen(leftDisplay, 'left');
    } else {
      console.error(`Invalid leftEye display index: ${options.leftEye}`);
    }
  }

  if (options.rightEye !== undefined) {
    const rightDisplay = displays[options.rightEye];
    if (rightDisplay) {
      createWindowForScreen(rightDisplay, 'right');
    } else {
      console.error(`Invalid rightEye display index: ${options.rightEye}`);
    }
  }

  // console.log('Windows object:', windows); // Debugging
}


// Parse command-line arguments
function parseArguments() {
  const args = process.argv.slice(2);
  const options = {};

  args.forEach(arg => {
    if (arg.startsWith('--left-eye=')) {
      const value = parseInt(arg.split('=')[1], 10);
      if (isNaN(value) || value < 0 || value >= screen.getAllDisplays().length) {
        console.error(`Invalid value for --left-eye: ${value}. Must be a valid display index.`);
        app.quit();
      }
      options.leftEye = value;
    } else if (arg.startsWith('--right-eye=')) {
      const value = parseInt(arg.split('=')[1], 10);
      if (isNaN(value) || value < 0 || value >= screen.getAllDisplays().length) {
        console.error(`Invalid value for --right-eye: ${value}. Must be a valid display index.`);
        app.quit();
      }
      options.rightEye = value;
    }
  });

  return options;
}

// Function to send a message to the renderer process
function sendToRenderer(event_name, msg) {
  for (const eye_window in windows) {
    if (windows[eye_window]) {
      windows[eye_window].webContents.send(event_name, msg);
    }
  }
}

function startRclNodejs() {
  rclnodejs.init().then(() => {

    const node = new rclnodejs.Node('eye_display_lld');

    node.createSubscription('eye_display_lld/msg/PupilControl', 'pupil_control', (msg) => {
      //console.log(`Received message: ${msg.dilation_percentage}`);
      sendToRenderer('pupil-control', msg);
    });
    
    node.createSubscription('eye_display_lld/msg/EyesDirection', 'eyes_direction_control', (msg) => {
      //console.log(`Received message: ${msg.yaw} and ${msg.pitch}`);
      sendToRenderer('eyes_direction_control', msg);
    });

    node.createSubscription('eye_display_lld/msg/EyeLidControl', 'eye_lid_control', (msg) => {
      //console.log(`Received message: ${msg.left_lid} and ${msg.right_lid}`);
      console.log(`Received message with eye_id: ${msg.eye_id}`);
      sendToRenderer('eye_lid_control', msg);
    });

    node.spin();
  });
}
app.whenReady().then(() => {
  setupScreens();
  startRclNodejs();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});