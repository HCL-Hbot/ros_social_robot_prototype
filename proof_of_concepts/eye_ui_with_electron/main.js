const { app, screen, BrowserWindow } = require('electron');
const path = require('path');

// Function to create a BrowserWindow for a specific screen
function createWindowForScreen(display) {
  return new BrowserWindow({
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
}

function setupScreens() {
  const displays = screen.getAllDisplays();

  if (displays.length > 1) {
    // Assign displays to the left and right eyes
    const left_eye_screen = displays[0];
    const right_eye_screen = displays[1];

    // Create windows for each eye
    const leftEyeWindow = createWindowForScreen(left_eye_screen);
    leftEyeWindow.loadURL(`file://${__dirname}/index.html?eye=left`);

    const rightEyeWindow = createWindowForScreen(right_eye_screen);
    rightEyeWindow.loadURL(`file://${__dirname}/index.html?eye=right`);

  } else if (displays.length === 1) {
    // Single screen (simulation mode)
    const simulationWindow = createWindowForScreen(displays[0], 'both');
    simulationWindow.loadURL(`file://${__dirname}/index.html?eye=both`);
  }
  else {
    console.error('No screens detected. Ensure at least one display is connected.');
    console.error('Abort application');
    app.quit();
  }

}

app.whenReady().then(setupScreens);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
