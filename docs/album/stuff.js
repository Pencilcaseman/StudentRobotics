//Get the button:
mybutton = document.getElementById("btt");

// When the user scrolls down 20px from the top of the document, show the button
window.onscroll = function() {scrollFunction()};

function scrollFunction() {
  if (document.body.scrollTop > 20 || document.documentElement.scrollTop > 20) {
    mybutton.style.display = "block";
  } else {
    mybutton.style.display = "none";
  }
}

// When the user clicks on the button, scroll to the top of the document
function topFunction() {
  document.body.scrollTop = 0; // For Safari
  document.documentElement.scrollTop = 0; // For Chrome, Firefox, IE and Opera
}
function grab(){
	let caption = [];
	let thumbnail = [];
	let posts = [];
	
	//Pull information from HAM social media webpages
	//Use instafeed jquery plugin here
	//Twitter integration tool of choice TBD

	//Write to HTML
	caption = document.getElementsByClassName("card-text");
	thumbnail = document.getElementsByClassName("bd-placeholder-img card-img-top");
	for (let i=0; i<caption.length; i++){
		caption[i].innerHTML = i.toString();
	}
}
