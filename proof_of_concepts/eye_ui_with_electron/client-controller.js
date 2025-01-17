const WebSocket = require('ws');

// Connect to the WebSocket server
const ws = new WebSocket('ws://localhost:8020');

// blink, double-blink, grin, sad, up, down
// blink, double-blink, grinb, grina, sadb, up, down

// Define a queue of animations
const animationQueue = [
  { animation: 'blink', duration: 5, repeat: 1, target: 'both' },
  { animation: 'double-blink', duration: 5, repeat: 1, target: 'both' },
  // { animation: 'grin', duration: 5, repeat: 1, target: 'both' },
  // { animation: 'sad', duration: 5, repeat: 1, target: 'both' },
  // { animation: 'sad', duration: 5, repeat: 1, target: 'both' },
  // { animation: 'up', duration: 5, repeat: 1, target: 'both' },
  // { animation: 'down', duration: 5, repeat: 1, target: 'both' }
  // { animation: 'grin', duration: 3, repeat: 2, target: 'both' },
  //{ animation: 'sad', duration: 4, repeat: 1, target: 'both' },
  //{ animation: 'blink', duration: 1, repeat: 5, target: 'both' }
];

// Function to send animations sequentially
function sendAnimations(queue) {
  if (queue.length === 0) {
    console.log('All animations completed');
    return;
  }

  const currentAnimation = queue.shift(); // Get the first animation in the queue

  console.log('Sending animation:', currentAnimation);
  ws.send(JSON.stringify(currentAnimation));

  // Wait for the duration of the current animation before sending the next one
  setTimeout(() => {
    sendAnimations(queue);
  }, currentAnimation.duration * currentAnimation.repeat * 1000);
}

ws.on('open', () => {
  console.log('Connected to WebSocket server');
  const start = Date.now();
  const seconds = 2;
  while (Date.now() - start < seconds * 1000) {
  }
  sendAnimations([...animationQueue]); // Use a copy of the queue to avoid mutating the original
});

ws.on('message', (data) => {
  const response = JSON.parse(data);
  console.log('Response from server:', response.message);
});

ws.on('close', () => {
  console.log('Disconnected from WebSocket server');
});

ws.on('error', (error) => {
  console.error('WebSocket error:', error);
});
