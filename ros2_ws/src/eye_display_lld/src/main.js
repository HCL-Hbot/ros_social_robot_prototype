const { app, screen, BrowserWindow, ipcMain } = require('electron');
const WebSocket = require('ws');
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
    frame: true, // (False = Remove title bar and window frame)
    fullscreen: false, // (True = Fullscreen mode)
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

// Function to start the WebSocket server
function startWebSocketServer() {
  const wss = new WebSocket.Server({ host: '0.0.0.0', port: 8080 });

  wss.on('connection', (ws) => {
    console.log('Controller connected');

    ws.on('message', (message) => {
      console.log('Received:', message);

      try {
        const command = JSON.parse(message);
        console.log('Command:', command);
        
        for (const eye_window in windows) {
          if (windows[eye_window]) {
            windows[eye_window].webContents.send('websocket-message', command);
          }
        }
          
      } catch (error) {
        console.error('Invalid message format', error);
        ws.send(JSON.stringify({ status: 'error', message: 'Invalid format' }));
      }
    });

    ws.on('close', () => {
      console.log('Controller disconnected');
    });
  });

  console.log('WebSocket server is running on ws://0.0.0.0:8080');
}

function setupScreens() {
  const displays = screen.getAllDisplays();
  const options = parseArguments();
  const argumentCount = Object.keys(options).length;
  console.log('Arguments:', argumentCount);
  let leftEyeScreenIndex = null;
  let rightEyeScreenIndex = null;

  if(displays.length === 0)
  {
    console.error('No screens detected. Exiting.');
    process.exit(1); // Beëindig de applicatie onmiddellijk met een foutcode
  }

  if (options.leftEye === undefined && options.rightEye === undefined) {
    console.log("No arguments provided, go with default settings");
    createWindowForScreen(displays[0], 'both');
    console.log('Windows object:', windows); // Debugging
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

  console.log('Windows object:', windows); // Debugging
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

function startRclNodejs() {
  // Initialize ROS2 node
  // rclnodejs.init().then(() => {
  //   const node = new rclnodejs.Node('eye_display_lld');
  //   const pupilControlSub = node.createSubscription(
  //     'eye_display_lld/msg/PupilControl',
  //     'pupil_control',
  //     (msg) => {
  //       console.log(`Received PupilControl message: ${msg.dilation_percentage}`);
  //       // Send the message to the renderer process
  //       for (const eye_window in windows) {
  //         if (windows[eye_window]) {
  //           windows[eye_window].webContents.send('pupil-control', msg);
  //         }
  //       }
  //     }
  //   );
  //   rclnodejs.spin(node);
  // });
}

app.whenReady().then(() => {
  setupScreens();
  //startWebSocketServer();
  //startRclNodejs();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

// Serve a simple UI for testing
const { createServer } = require('http');
const fs = require('fs');
const { start } = require('repl');

createServer((req, res) => {
  if (req.url === '/') {
    res.writeHead(200, { 'Content-Type': 'text/html' });
    fs.createReadStream(`${__dirname}/control-panel.html`).pipe(res);
  } else {
    res.writeHead(404);
    res.end('Not Found');
  }
}).listen(8081, '0.0.0.0', () => {
  console.log('Control Panel running at http://0.0.0.0:8081');
});