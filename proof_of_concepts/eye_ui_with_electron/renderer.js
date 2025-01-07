window.onload = () => {
  // Parse the query parameter from the URL
  const queryParams = new URLSearchParams(window.location.search);
  const eye = queryParams.get('eye'); // Get the "eye" parameter (left or right)

  if (eye === 'left') {
    // Show only the left eye
    document.getElementById('right-eye').style.display = 'none';
  } else if (eye === 'right') {
    // Show only the right eye
    document.getElementById('left-eye').style.display = 'none';
  }
};
