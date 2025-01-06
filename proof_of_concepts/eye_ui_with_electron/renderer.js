const WebSocket = require('websocket').w3cwebsocket;

// Create a WebSocket connection to the server
const ws = new WebSocket('ws://localhost:9002');

// Select the eye elements
const leftEye = document.querySelector('.eye.left');
const rightEye = document.querySelector('.eye.right');

// Set default state to blinking
setState('blink');

// Handle incoming WebSocket messages
ws.onmessage = (message) => {
  const data = JSON.parse(message.data);

  if (data.type === 'emotion') {
    setState(data.value); // Update the state based on received emotion
  }
};

// Set the state of the eyes
function setState(state) {
  const allStates = ['blink', 'double-blink', 'grin', 'sad', 'up', 'down'];

  // Remove all states from the eyes
  allStates.forEach((s) => {
    leftEye.classList.remove(s);
    rightEye.classList.remove(s);
  });

  // Add the new state
  if (allStates.includes(state)) {
    leftEye.classList.add(state);
    rightEye.classList.add(state);
  } else {
    // Default to blinking if an invalid state is received
    leftEye.classList.add('blink');
    rightEye.classList.add('blink');
  }
}

// Log WebSocket connection status
ws.onopen = () => console.log('WebSocket connected');
ws.onclose = () => console.log('WebSocket disconnected');
