window.onload = () => {
  const queryParams = new URLSearchParams(window.location.search);
  const eye = queryParams.get('eye'); // Determine if it's left or right eye

  const leftEye = document.getElementById('left-eye');
  const rightEye = document.getElementById('right-eye');

  if (eye === 'left') {
    rightEye.style.display = 'none';
  } else if (eye === 'right') {
    leftEye.style.display = 'none';
  }

  // List of expressions
  const expressions = ['blink', 'double-blink', 'grin', 'sad'];
  //const expressions = ['blink'];

  let currentExpressionIndex = 0;

  function changeExpression() {
    const allEyes = document.querySelectorAll('.eye');
    allEyes.forEach((eyeElement) => {
      eyeElement.className = 'eye'; // Reset all classes
      eyeElement.classList.add(expressions[currentExpressionIndex]); // Add new expression
    });

    currentExpressionIndex = (currentExpressionIndex + 1) % expressions.length; // Cycle through expressions
  }
  // Change expression every 5 seconds
  setInterval(changeExpression, 5000);
  changeExpression(); // Initialize with the first expression
};
