setInterval(unwatermark, 100);

function unwatermark() {
  const referralElements = document.getElementsByClassName("referral");
  const juicerElements = document.getElementsByClassName("juicer-about");
  while (referralElements.length > 0) referralElements[0].remove();
  while (juicerElements.length > 0) juicerElements[0].remove();
}
//Get the button:
mybutton = document.getElementById("btt");

// When the user clicks on the button, scroll to the top of the document
function topFunction() {
  document.body.scrollTop = 0; // For Safari
  document.documentElement.scrollTop = 0; // For Chrome, Firefox, IE and Opera
}
