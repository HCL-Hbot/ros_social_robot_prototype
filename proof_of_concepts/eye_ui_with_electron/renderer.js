// renderer.js
const { ipcRenderer } = require('electron');

window.onload = () => {

  const queryParams = new URLSearchParams(window.location.search);
  const eyeParameter = queryParams.get('eye'); // Determine if it's left or right eye or both

  const leftEye = document.getElementById('left-eye');
  const rightEye = document.getElementById('right-eye');

  if (eyeParameter === 'left') {
    rightEye.remove();
  } else if (eyeParameter === 'right') {
    leftEye.remove();
  }

  const eyes = document.querySelectorAll('.eye');
  const irises = document.querySelectorAll('.iris');
  const topLids = document.querySelectorAll('.eyelid.top');
  const bottomLids = document.querySelectorAll('.eyelid.bottom');
  let isAnimationActive = false;

  window.addEventListener('mousemove', (e) => {
    if (isAnimationActive) return;

    const { clientX, clientY } = e;
    const { innerWidth, innerHeight } = window;

    irises.forEach((eye) => {
      const rect = eye.getBoundingClientRect();
      const eyeX = rect.left + rect.width / 2;
      const eyeY = rect.top + rect.height / 2;
      const deltaX = clientX - eyeX;
      const deltaY = clientY - eyeY;
      const angle = Math.atan2(deltaY, deltaX);
      const distance = Math.min(rect.width / 4, Math.hypot(deltaX, deltaY));
      const x = distance * Math.cos(angle);
      const y = distance * Math.sin(angle);

      eye.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
    });
  });

  function blink() {
    topLids.forEach(lid => lid.style.height = '50%');
    bottomLids.forEach(lid => lid.style.height = '50%');
    setTimeout(() => {
      if(!isAnimationActive)
      {
        topLids.forEach(lid => lid.style.height = '0%');
        bottomLids.forEach(lid => lid.style.height = '0%');
      }
    }, 200);
  }

  let blinkInterval = setInterval(() => {
    if (!isAnimationActive) blink();
  }, 2000);

  ipcRenderer.on('websocket-message', (event, command) => {
    console.log('Command received:', command);
    
    if (command.command === 'moveEye') {
      moveEye(command.eye, command.x, command.y);
    } else if (command.command === 'moveLid') {
      moveLid(command.lid, command.position);
    } else if (command.command === 'startAnimation') {
      startAnimation(command.animation);
    } else if (command.command === 'autoBlink') {
      setAutoBlink(command.value);
    } else if (command.command === 'reset') {
      reset();
    } else {
      console.error('Unknown command:', command);
    }
  });

  // Example: Move the eye to a specific position. Also supports 'both' as eye parameter which moves both iris
  function moveEye(eye, x, y) {
    console.log("moveEye", eye, x, y);
    if(x === null || y === null) return;

    let searchId = null;
    if(eye === "left") searchId = "left-eye";
    else if(eye === "right") searchId = "right-eye";

    if(eye === "both"){
      console.log("Lets move both iris");
      irises.forEach((iris) => {
        console.log('Setting eye position:', x, y);
        iris.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
      });
    }
    else if(searchId !== null) {
      console.log("Lets move one eye");
      const irisElement = document.querySelector(`#${searchId} .iris`);
      if (irisElement) {
        console.log('Setting eye position:', x, y);
        irisElement.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
      }
    }
  }

  // Example: Move a eyelid to a specific position. Also supports 'all' as lid parameter which moves all eyelids
  function moveLid(lid, position) {

    const searchClass = lid === "all" ? "eyelid" : null;

    if(searchClass === null) {
      let searchId = null;

      const lidElement = document.getElementById(lid);
      console.log(lidElement);

      if (lidElement) {
        console.log('Setting lid position:', position);
        lidElement.style.height = `${position}%`;
      }
      else
      {
        console.log('Lid not found:', searchId);
      }
    }
    else
    {
      const lids = document.querySelectorAll(`.${searchClass}`);
      lids.forEach((lidElement) => {
        console.log('Setting lid position:', position);
        lidElement.style.height = `${position}%`;
      });
    }
  }

  function startAnimation(animation) {
    const animations = ['blink-active', 'sad', 'grin', 'look-random'];
  
    reset();
    isAnimationActive = true;
    if(animations.includes(animation))
    {
      eyes.forEach((eye) => {
        console.log("adding animation");
        eye.classList.remove(`${animation}`);
        void eye.offsetWidth; // Trigger reflow
        eye.classList.add(`${animation}`);

        // Remove the class once the animation is completed
        eye.addEventListener('animationend', (event) => {
          eye.classList.remove(`${animation}`);
          console.log(`Animatie '${event.animationName}' is done`);
          isAnimationActive = false;
        }, { once: true }); // `once: true` removes the event listener after it has been triggered
      });
    }
  }

  function setAutoBlink(blinkValue) {
    if(blinkInterval === null && blinkValue !==null && blinkValue === true) // Start the interval
    {
      console.log("Auto blink enabled");
      blinkInterval = setInterval(() => {
        if (!isAnimationActive) blink();
      }, 2000);
    }
    else if(blinkInterval !== null && blinkValue === false) // Stop the interval
    {
      clearInterval(blinkInterval);
      blinkInterval = null;
      reset();
      console.log("Auto blink disabled");
    }
  }
  
  function reset()
  {
    console.log("reset");

    eyes.forEach((eye) => {
      eye.classList = "eye";
    });

    irises.forEach((eye) => {
      eye.style.transform = 'translate(-50%, -50%)';
    });
    topLids.forEach(lid => lid.style.height = '0%');
    bottomLids.forEach(lid => lid.style.height = '0%');
  }
}