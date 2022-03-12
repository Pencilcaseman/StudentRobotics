//setTimeout(grab,1000);
//document.getElementById('twitter').contentWindow.location.reload();
setInterval(unwatermark,100);

function unwatermark(){
	const chungus = document.getElementsByClassName("referral");
	const blungus = document.getElementsByClassName("juicer-about");
	while (chungus.length > 0) chungus[0].remove();
	while (blungus.length > 0) blungus[0].remove();
}
/*
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
	let postText = [];
	let postImages = [];
	
	//Pull information from HAM social media webpages
	postText = (document.getElementById('twitter-widget-0').contentWindow.document.getElementsByClassName("timeline-Tweet-text"));
	//postText.push(document.getElementById('twitter-widget-0').contentWindow.document.getElementsByTagName("p"));
	postImages = (document.getElementById('twitter-widget-0').contentWindow.document.getElementsByClassName("NaturalImage-image"));
	
	//Write to HTML
	caption = document.getElementsByClassName("card-text");
	thumbnail = document.getElementsByClassName("bd-placeholder-img card-img-top");
	console.log(postText);
	console.log(postImages);
	console.log(caption);
	console.log(thumbnail);

	for (let i=0; i<caption.length; i++){
		caption[i].innerHTML = "";
	}

	//write to HTML
	let ph = [];
	let img = [];
	for (let i=0; i<thumbnail.length; i++){
		ph.push(thumbnail[i].getElementsByTagName("text")[0]);
		img.push(thumbnail[i].getElementsByTagName("image")[0]);
		ph[i].innerHTML = "Big Chungus";
		img[i].href = postImages[i].href;
		console.log(ph[i])
			thumbnail.appendChild
	}
	for (let i=0; i<caption.length; i++){
		caption[i].innerHTML = postText[i].innerHTML;
	}

}*/
