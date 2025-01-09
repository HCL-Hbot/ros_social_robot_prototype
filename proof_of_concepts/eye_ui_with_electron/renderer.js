const { ipcRenderer } = require('electron');

window.onload = () => {
  const queryParams = new URLSearchParams(window.location.search);
  const eye = queryParams.get('eye'); // Determine if it's left or right eye or both

  const leftEye = document.getElementById('left-eye');
  const rightEye = document.getElementById('right-eye');

  if (eye === 'left') {
    rightEye.style.display = 'none';
  } else if (eye === 'right') {
    leftEye.style.display = 'none';
  }

  ipcRenderer.on('play-animation', (event, { animation, duration, repeat }) => {
    console.log(`Playing animation: ${animation}, Duration: ${duration}s, Repeat: ${repeat}`);
    playAnimation(animation, duration, repeat);
  });

  function playAnimation(animation, duration, repeat) {
    const allEyes = document.querySelectorAll('.eye');
    allEyes.forEach((eyeElement) => {
      eyeElement.className = 'eye';
      eyeElement.style.animationDuration = `${duration}s`;
      eyeElement.style.animationIterationCount = repeat;
      eyeElement.classList.add(animation);
    });
  }
};
