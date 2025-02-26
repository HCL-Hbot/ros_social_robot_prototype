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

  //TODO might need to calculate iris size based on eye size
  //const eyeSize = document.querySelector('.eye').getBoundingClientRect().width; //width and height are the same for both eyes
  const eyes = document.querySelectorAll('.eye');
  const irises = document.querySelectorAll('.iris');
  const pupiles = document.querySelectorAll('.pupil'); //TODO make this controlable
  const topLids = document.querySelectorAll('.eyelid.top'); //TODO make this controlable
  const bottomLids = document.querySelectorAll('.eyelid.bottom'); //TODO make this controlable
  let isAnimationActive = false; //TODO make animation controlable?


  function blink() {
    topLids.forEach(lid => lid.style.height = '50%');
    bottomLids.forEach(lid => lid.style.height = '50%');
    setTimeout(() => {
      if (!isAnimationActive) {
        topLids.forEach(lid => lid.style.height = '0%');
        bottomLids.forEach(lid => lid.style.height = '0%');
      }
    }, 200);
  }

  // let blinkInterval = setInterval(() => {
  //   if (!isAnimationActive) blink();
  // }, 2000);

  ipcRenderer.on('pupil-control', (event, msg) => {
    //console.log(`Received PupilControl message in renderer: ${msg.dilation_percentage}`);
    // Hier kun je de logica toevoegen om de pupil dilatatie te verwerken
    
    //TODO this does not seem to work.
    updatePupilDilation(msg.dilation_percentage);
  });

  function updatePupilDilation(dilationPercentage) {
    // Add logic to handle the pupil dilation
    // For example, update the size of an HTML element representing the pupil
    pupiles.forEach((pupil) => {
      pupil.style.width = `${dilationPercentage}%`;
      pupil.style.height = `${dilationPercentage}%`;
    });
  }

  ipcRenderer.on('eye_lid_control', (event, eye_lid_control_msg) => {
    //for now only support one value for both eyes
    if(eye_lid_control_msg.eye_id===2)
    {
      moveLid('all', eye_lid_control_msg.top_lid_position); //TODO each eye lid seperatly. For now everything is the same
    }
    //TODO each eye seperatly
  });

  ipcRenderer.on('eyes_direction_control', (event, eyes_direction) => {
    console.log('Eyes direction yaw:', eyes_direction.yaw);
    console.log('Eyes direction pitch:', eyes_direction.pitch);
    moveEyes(eyes_direction.yaw, eyes_direction.pitch);
  });

  // Function to move the eyes based on yaw and pitch
  function moveEyes(yaw, pitch) {
    // Map yaw and pitch to x and y positions
    // yaw links-rechts, pitch omhoog-omlaag
    //Recht voor camera yaw 75 pitch 55 (voornu dus hiermee compenseneren)
    //wat er nu fout gebeurd als input. Ik krijg een lage pitch waarde voor omhoog en hoge pitch warde voor omlaag 

    // yaw = yaw-75; //temp fix for offset 

    // pitch = pitch-55; //temp fix for offset

    // pitch = pitch * -1; //for mirror effect, because currenty we get the input mirrored

    const x = yawToX(yaw);
    const y = pitchToY(pitch);

    // Move both eyes to the calculated position
    moveEye('both', x, y);
  }

  // Helper function to map yaw to x position
  function yawToX(yaw) {
    // Assuming yaw ranges from -90 to 90 degrees
    // Map this range to a suitable x position range (e.g., -50 to 50)
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
    // Map this range to a suitable y position range (e.g., -50 to 50)
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
        iris.style.transform = `translate(-${x}%, -${y}%)`; //translate(${x}px, ${y}px)`;
      });
    }
    else if (searchId !== null) {
      console.log("Lets move one eye");
      const irisElement = document.querySelector(`#${searchId} .iris`);
      if (irisElement) {
        irisElement.style.transform = `translate(-${x}%, -${y}%)`;// translate(${x}px, ${y}px)`;
      }
    }
  }

  // FROME HERE ONWARDS IS NOT USED YET. OLD CODE FROM PROOF OF CONCEPT WHICH SHOULD BE REFACTORED AND REVIEWED HOW TO USE IT IN MY CASE
  // Example: Move a eyelid to a specific position. Also supports 'all' as lid parameter which moves all eyelids
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