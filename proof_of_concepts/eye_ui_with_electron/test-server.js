const WebSocket = require('ws');

// Start a WebSocket server on port 9002
const wss = new WebSocket.Server({ port: 9002 });

console.log("WebSocket server running on ws://localhost:9002");

wss.on('connection', (ws) => {
  console.log('A new client has connected.');

  // Periodically send random state updates to the client
  setInterval(() => {
    const states = ['AWAKE', 'SLEEP', 'SLEEPY'];
    const randomState = states[Math.floor(Math.random() * states.length)];

    ws.send(JSON.stringify({ type: 'state', value: randomState }));
    console.log(`Sent: State=${randomState}`);
  }, 5000); // Every 5 seconds

  // Periodically send random position updates to the client
  setInterval(() => {
    const randomX = Math.random() * 40 - 20; // Random between -20 and +20
    const randomY = Math.random() * 40 - 20;

    ws.send(JSON.stringify({ type: 'position', x: randomX, y: randomY }));
    console.log(`Sent: Position x=${randomX.toFixed(2)}, y=${randomY.toFixed(2)}`);
  }, 1000); // Every 1 second
});
