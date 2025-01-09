const { app, screen, BrowserWindow, ipcMain } = require('electron');
const WebSocket = require('ws');
const path = require('path');

const windows = {}; // Object om vensters op te slaan (left, right, both)

// Function to create a BrowserWindow for a specific screen
function createWindowForScreen(display, eye) {
  const window = new BrowserWindow({
    x: display.bounds.x, // Set the window's position to match the display (For good practice we do this always explicitly, but when we use fullscreen it will be done implecitly)
    y: display.bounds.y, //(For good practice we do this always explicitly, but when we use fullscreen it will be done implecitly)
    width: display.bounds.width,
    height: display.bounds.height,
    frame: false, // Remove title bar and window frame
    fullscreen: true, // Fullscreen mode
    autoHideMenuBar: true,   // Hide the menu bar
    alwaysOnTop: false,    // Ensure the window stays on top, for now on false. But might be useful to set it to true for production!!!!
    webPreferences: {
      preload: path.join(__dirname, 'renderer.js'), // Optional preload script
      nodeIntegration: true,
    },
  })
  window.loadURL(`file://${__dirname}/index.html?eye=${eye}`);
  windows[eye] = window; // Store in the windows object
  console.log(`Window created for ${eye}`);
  return window;
}

// Function to start the WebSocket server
function startWebSocketServer() {
  const wss = new WebSocket.Server({ port: 8020 });

  wss.on('connection', (ws) => {
    console.log('Controller connected');

    ws.on('message', (message) => {
      console.log('Received:', message);

      try {
        const data = JSON.parse(message);
        const { animation, duration, repeat, target } = data;

        // Stuur het commando naar specifieke vensters of alle vensters
        if (target === 'left' && windows.left) {
          windows.left.webContents.send('play-animation', { animation, duration, repeat });
        } else if (target === 'right' && windows.right) {
          windows.right.webContents.send('play-animation', { animation, duration, repeat });
        } else if (target === 'both') {
          console.log("BOTH!");
          windows.both.webContents.send('play-animation', { animation, duration, repeat });
        }

        // Bevestiging terugsturen naar de controller
        ws.send(JSON.stringify({ status: 'success', message: 'Animation started' }));
      } catch (error) {
        console.error('Invalid message format', error);
        ws.send(JSON.stringify({ status: 'error', message: 'Invalid format' }));
      }
    });

    ws.on('close', () => {
      console.log('Controller disconnected');
    });
  });

  console.log('WebSocket server is running on ws://localhost:8020');
}

function setupScreens() {
  const displays = screen.getAllDisplays();

  if (displays.length > 1) {
      createWindowForScreen(displays[0], 'left');
      createWindowForScreen(displays[1], 'right');
  } else if (displays.length === 1) {
      createWindowForScreen(displays[0], 'both');
  } else {
      console.error('No screens detected. Exiting.');
      app.quit();
  }

  console.log('Windows object:', windows); // Debugging
}
// function setupScreens() {
//   const displays = screen.getAllDisplays();

//   if (displays.length > 1) {
//     // Assign displays to the left and right eyes
//     const left_eye_screen = displays[0];
//     const right_eye_screen = displays[1];

//     // Create windows for each eye
//     const leftEyeWindow = createWindowForScreen(left_eye_screen);
//     leftEyeWindow.loadURL(`file://${__dirname}/index.html?eye=left`);
//     windows['left'] = leftEyeWindow;


//     const rightEyeWindow = createWindowForScreen(right_eye_screen);
//     rightEyeWindow.loadURL(`file://${__dirname}/index.html?eye=right`);
//     windows['right'] = rightEyeWindow;

//   } else if (displays.length === 1) {
//     // Single screen (simulation mode)
//     const simulationWindow = createWindowForScreen(displays[0], 'both');
//     simulationWindow.loadURL(`file://${__dirname}/index.html?eye=both`);
//     windows['both'] = simulationWindow;
//   }
//   else {
//     console.error('No screens detected. Ensure at least one display is connected.');
//     console.error('Abort application');
//     app.quit();
//   }
// }

app.whenReady().then(() => {
  setupScreens();
  startWebSocketServer();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
