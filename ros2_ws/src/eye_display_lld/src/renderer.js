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
  const pupiles = document.querySelectorAll('.pupil'); 
  const topLids = document.querySelectorAll('.eyelid.top'); 
  const bottomLids = document.querySelectorAll('.eyelid.bottom');

  /* features taken from proof of concept, but not used yet */
  // let isAnimationActive = false; //TODO make animation controlable?

  // function blink() {
  //   topLids.forEach(lid => lid.style.height = '50%');
  //   bottomLids.forEach(lid => lid.style.height = '50%');
  //   setTimeout(() => {
  //     if (!isAnimationActive) {
  //       topLids.forEach(lid => lid.style.height = '0%');
  //       bottomLids.forEach(lid => lid.style.height = '0%');
  //     }
  //   }, 200);
  // }

  // let blinkInterval = setInterval(() => {
  //   if (!isAnimationActive) blink();
  // }, 2000);
  /************************************************** */

  ipcRenderer.on('pupil-control', (event, msg) => {
    updatePupilDilation(msg.dilation_percentage);
  });

  function updatePupilDilation(dilationPercentage) {
    pupiles.forEach((pupil) => {
      pupil.style.width = `${dilationPercentage}%`;
      pupil.style.height = `${dilationPercentage}%`;
    });
  }

  ipcRenderer.on('eye_lid_control', (event, eye_lid_control_msg) => {
    //for now only support one value for both eyes
    if(eye_lid_control_msg.eye_id===2)
    {
      moveLid('all', eye_lid_control_msg.top_lid_position); //TODO create conrol for each eye lid seperatly? For now everything is the same
    }
    //TODO each eye seperatly?
  });

  ipcRenderer.on('eyes_direction_control', (event, eyes_direction) => {
    // console.log('Eyes direction yaw:', eyes_direction.yaw);
    // console.log('Eyes direction pitch:', eyes_direction.pitch);
    moveEyes(eyes_direction.yaw, eyes_direction.pitch);
  });

  // Function to move the eyes based on yaw and pitch
  function moveEyes(yaw, pitch) {
    const x = yawToX(yaw);
    const y = pitchToY(pitch);

    // Move both eyes to the calculated position
    moveEye('both', x, y);
  }

  // Helper function to map yaw to x position
  function yawToX(yaw) {
    // Assuming yaw ranges from -90 to 90 degrees
    // Map this range to a suitable x position range
    const minYaw = 30; // left
    const maxYaw = -30; // right
    const minX = 0; //left (robot perspective)
    const maxX = 100; //right (robot persepective)
    const x = ((yaw - minYaw) / (maxYaw - minYaw)) * (maxX - minX) + minX;
    return Math.max(minX, Math.min(maxX, x));
  }

  // Helper function to map pitch to y position
  function pitchToY(pitch) {
    // Assuming pitch ranges from -90 to 90 degrees
    // Map this range to a suitable y position range
    const minPitch = -20; //down
    const maxPitch = 20; //up
    const minY = 0; //down
    const maxY = 100; //up  
    const y = ((pitch - minPitch) / (maxPitch - minPitch)) * (maxY - minY) + minY;
    return Math.max(minY, Math.min(maxY, y));
  }

  // Example: Move the eye to a specific position. Also supports 'both' as eye parameter which moves both iris
  function moveEye(eye, x, y) {
    console.log("moveEye", eye, x, y);
    if (x === null || y === null) return;

    let searchId = null;
    if (eye === "left") searchId = "left-eye";
    else if (eye === "right") searchId = "right-eye";

    if (eye === "both") {
      console.log("Lets move both iris");
      irises.forEach((iris) => {
        //translate(left-right, up-down) (0= full left, 0= full down) robot perspective
        iris.style.transform = `translate(-${x}%, -${y}%)`;
      });
    }
    else if (searchId !== null) {
      console.log("Lets move one eye");
      const irisElement = document.querySelector(`#${searchId} .iris`);
      if (irisElement) {
        irisElement.style.transform = `translate(-${x}%, -${y}%)`;
      }
    }
  }

  // FROME HERE ONWARDS ALL THE FUNCTIONS ARE NOT USED YET. OLD CODE FROM PROOF OF CONCEPT WHICH SHOULD BE REFACTORED AND REVIEWED HOW TO USE IT IN MY CASE
  // Currently ONLY using the moveLid function from PROOF OF CONCEPT verssion, only the case where we move both lids at the same time
  // The rest of the code in the function is not used yet, but can be used to control the eyes in a more advanced way. Should be tested and reviewed how to use it in my case
  function moveLid(lid, position) {

    const searchClass = lid === "all" ? "eyelid" : null;

    if (searchClass === null) {
      let searchId = null;

      const lidElement = document.getElementById(lid);
      console.log(lidElement);

      if (lidElement) {
        console.log('Setting lid position:', position);
        lidElement.style.height = `${position}%`;
      }
      else {
        console.log('Lid not found:', searchId);
      }
    }
    else {
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
    if (animations.includes(animation)) {
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
    if (blinkInterval === null && blinkValue !== null && blinkValue === true) // Start the interval
    {
      console.log("Auto blink enabled");
      blinkInterval = setInterval(() => {
        if (!isAnimationActive) blink();
      }, 2000);
    }
    else if (blinkInterval !== null && blinkValue === false) // Stop the interval
    {
      clearInterval(blinkInterval);
      blinkInterval = null;
      reset();
      console.log("Auto blink disabled");
    }
  }

  function reset() {
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