const { app, screen, BrowserWindow, ipcMain } = require('electron');
const WebSocket = require('ws');
const path = require('path');

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
        
        if(windows.left && windows.right) {
          windows.left.webContents.send('websocket-message', command);
          windows.right.webContents.send('websocket-message', command);  
        } else if(windows.both) {
          windows.both.webContents.send('websocket-message', command);
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

  if (displays.length > 1) {
      createWindowForScreen(displays[0], 'right');
      createWindowForScreen(displays[1], 'left');
  } else if (displays.length === 1) {
      createWindowForScreen(displays[0], 'both');
  } else {
      console.error('No screens detected. Exiting.');
      app.quit();
  }
  console.log('Windows object:', windows); // Debugging
}

app.whenReady().then(() => {
  setupScreens();
  startWebSocketServer();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

// Serve a simple UI for testing
const { createServer } = require('http');
const fs = require('fs');

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
