const WebSocket = require('websocket').w3cwebsocket; // Todo maybe replace with 'ws'??

// Create a WebSocket connection to the server
const ws = new WebSocket('ws://localhost:9002'); // Adjust this URL if needed

// Select the eyes in the DOM
const leftEye = document.getElementById('left-eye');
const rightEye = document.getElementById('right-eye');

// Process messages from the WebSocket server
ws.onmessage = (message) => {
  const data = JSON.parse(message.data);

  // Handle state updates (AWAKE, SLEEP, SLEEPY)
  if (data.type === 'state') {
    if (data.value === 'AWAKE') {
      leftEye.classList.remove('closed');
      rightEye.classList.remove('closed');
    } else if (data.value === 'SLEEP' || data.value === 'SLEEPY') {
      leftEye.classList.add('closed');
      rightEye.classList.add('closed');
    }
  }

  // Handle position updates for the pupils
  if (data.type === 'position') {
    const pupilX = data.x;
    const pupilY = data.y;

    leftEye.querySelector('.pupil').style.transform = `translate(${pupilX}px, ${pupilY}px)`;
    rightEye.querySelector('.pupil').style.transform = `translate(${pupilX}px, ${pupilY}px)`;
  }
};

// Log WebSocket status
ws.onopen = () => {
  console.log('WebSocket connected');
};

ws.onerror = (error) => {
  console.error('WebSocket error:', error);
};

ws.onclose = () => {
  console.log('WebSocket connection closed');
};
