
// Check if two arrays are equal, with option to specify index interval to check
function arraysEqual(arr1, arr2, lower = 0, upper = 0) {
    if(arr1.length !== arr2.length)
        return false;

    var upperL = (upper == 0 ? arr1.length : upper);

    for(var i = upperL; i >= lower; i--) {
        if(arr1[i] !== arr2[i])
            return false;
    }
    return true;
}


// Function to return sum of array values
function sumArray(a) {
    function add(a, b) {
        return a + b;
    }
    return a.reduce(add, 0);
}


// Function for short-hand access to querySelector
function select(query) {
    return document.querySelector(query);
}


// Function for short-hand access to adding eventListener to element
function addListener(selector, eventType, func) {
    return document.querySelector(selector).addEventListener(eventType, func);
}


// XHR wrapped in a promise
function loadFile(url) {

    return new Promise(function(resolve, reject) {
        var request = new XMLHttpRequest();
        request.open('GET', url);
        request.responseType = 'document';

        // When the request loads, check whether it was successful
        request.onload = function() {
        if (request.status === 200) {

        // If successful, resolve the promise by passing back the request response
            resolve(request.response);
        } else {

        // If it fails, reject the promise with a error message
            reject(Error('File didn\'t load successfully; error code:' + request.statusText));
        }
      };

      request.onerror = function() {

      // Also deal with the case when the entire request fails to begin with
      // This is probably a network error, so reject the promise with an appropriate message
          reject(Error('There was a network error.'));
      };

      // Send the request
      request.send();
    });
  }

  // jQuery like object for fadeIn and fadeOut
function _(el) {
   if (!(this instanceof _)) {
     return new _(el);
   }
   this.el = document.getElementById(el);
 }

 _.prototype.fade = function fade(type, ms) {
   var isIn = type === 'in',
     opacity = isIn ? 0 : 1,
     interval = 50,
     duration = ms,
     gap = interval / duration,
     self = this;

   if(isIn) {
     self.el.style.display = 'inline';
     self.el.style.opacity = opacity;
   }

   function func() {
     opacity = isIn ? opacity + gap : opacity - gap;
     self.el.style.opacity = opacity;

     if(opacity <= 0) self.el.style.display = 'none'
     if(opacity <= 0 || opacity >= 1) window.clearInterval(fading);
   }

   var fading = window.setInterval(func, interval);
 }
