const { app, BrowserWindow } = require('electron');

let mainWindow;

app.on('ready', () => {
  mainWindow = new BrowserWindow({
    fullscreen: true,        // Fullscreen mode
    frame: false,            // Remove window buttons
    autoHideMenuBar: true,   // Hide the menu bar
    alwaysOnTop: false,      // Ensure the window stays on top, for now on false. But might be useful to set it to true for production!!!!
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
    },
  });

  mainWindow.loadFile('index.html');

  // Disable scrolling
//   mainWindow.webContents.on('did-finish-load', () => {
//     mainWindow.webContents.insertCSS('body { overflow: hidden; }');
//   });
});
