const WebSocket = require('ws');

// Connect to the WebSocket server
const ws = new WebSocket('ws://localhost:8020');

// Define a queue of animations
const animationQueue = [
  { animation: 'blink', duration: 2, repeat: 3, target: 'both' },
  { animation: 'grin', duration: 3, repeat: 2, target: 'both' },
  { animation: 'sad', duration: 4, repeat: 1, target: 'both' },
  { animation: 'blink', duration: 1, repeat: 5, target: 'both' }
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
