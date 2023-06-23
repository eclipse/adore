// COPY TO CLIPBOARD
// Text in an element
function copyToClipboard(textSelector) {
  const textToCopy = document.querySelector(textSelector);
  const selection = window.getSelection();
  const range = document.createRange();
  
  range.selectNodeContents(textToCopy);
  selection.removeAllRanges();
  selection.addRange(range);
  
  document.execCommand('copy');
  selection.removeAllRanges();

  // Custom feedback
  alert('Text copied: ' + textToCopy.textContent);
}

// Text as string
// function copyToClipboard(text) {
//   const element = document.createElement('textarea');
//   element.value = text;

//   document.body.appendChild(element);
//   element.select();
//   document.execCommand('copy');
//   document.body.removeChild(element);

//   // Custom feedback
//   alert('Text copied: ' + text);
// };


// USAGE
document.querySelector('.button').addEventListener('click', function() {
  copyToClipboard('.text');
});
