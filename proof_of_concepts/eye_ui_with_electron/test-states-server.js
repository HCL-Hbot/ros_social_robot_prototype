const WebSocket = require('ws');

// Start a WebSocket server on port 9002
const wss = new WebSocket.Server({ port: 9002 });

console.log('WebSocket server running on ws://localhost:9002');

// Possible emotions
const emotions = ['grin', 'sad', 'up', 'down', 'blink', 'double-blink'];

wss.on('connection', (ws) => {
  console.log('Client connected');

  let currentIndex = 0;

  // Periodically send an emotion
  setInterval(() => {
    const emotion = emotions[currentIndex];
    ws.send(JSON.stringify({ type: 'emotion', value: emotion }));
    console.log(`Sent emotion: ${emotion}`);

    currentIndex = (currentIndex + 1) % emotions.length;
  }, 5000); // Send every 5 seconds
});
