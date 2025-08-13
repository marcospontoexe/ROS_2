// When the notebook is resized, adjust the scrollbar position
let notebookLocation = window.location.href;
let currentScroll = 0;
let currentHeight = 0;
function windowScrolled(event) {
    currentScroll = Math.round(window.scrollY);
    // console.log(`Scrolled to ${currentScroll}`);
}


/**
 * clickCallback handles clicks on links to prevent default browser navigation.
 * It checks if the clicked link points to the current page or a different one.
 * For same page links, it updates window.location.href to navigate.
 * For external links, it opens the URL in a new tab/window.
 */
function clickCallback(event) {
    // console.log(`Notebook location ${notebookLocation}`);
    if (event.target.tagName !== 'A') return;
    event.preventDefault();
    const link = event.target.href;

    if (link.startsWith(notebookLocation)) {
        // console.log("same page");
        window.location.href = link;
    } else {
        // console.log("different page");
        window.open(link, '_blank');
    }
}

/**
 * windowResized is called when the browser window is resized.
 * It adjusts the scroll position to maintain approximately the same visible portion of the page.
*/
function windowResized(event) {
    let newHeight = document.body.scrollHeight;
    // console.log(`Scroll height changed from ${currentHeight} to ${newHeight}`);
    let changeRatio = newHeight / currentHeight;
    let correctionFactor = currentHeight > newHeight ? 350 : -450;
    let resizeValue = Math.round(currentScroll * changeRatio) + correctionFactor;
    // console.log(`Auto-Scrolling to ${resizeValue}`);
    window.scrollTo(0, resizeValue);
    currentScroll = Math.round(window.scrollY);
    currentHeight = newHeight;
}


/**
 * Adds a copy button to code cells to allow copying cell contents.
 * Searches for code cells with selectors and inserts a copy button.
 */
function addCopyThisButton() {
    // Add a 'copy this' button to every code cell
    // The bottons are added just before the code cells with the classes specified in 'selectors'
    let selectors = [".hl-ipython2", ".hl-ipython3"];
    let html = "<button title='Copy this code' class='copy-code-btn' style='position: absolute; top: 5px; right: 5px;"
    html += "color: lightseagreen; font-weight: bold;'><i class='fa fa-clipboard' aria-hidden='true'></i></button>";
    let cells;
    for (let selector of selectors) {
        cells = [];
        cells = document.querySelectorAll(selector);
        cells.forEach(cell => cell.insertAdjacentHTML("beforebegin", html));
    }
}

/**
 * domLoaded is called when the DOM has loaded.
 * It sets up event listeners and calls addCopyThisButton to add copy buttons.
*/
function domLoaded() {
    addCopyThisButton();
    notebookLocation = window.location.href;
    currentHeight = document.body.scrollHeight;
    window.addEventListener('scroll', windowScrolled);
    window.addEventListener('resize', windowResized);
    if (document.addEventListener)
        document.addEventListener('click', clickCallback, false);
    else
        document.attachEvent('onclick', clickCallback);
}

// what to do when the DOM has loaded
document.addEventListener("DOMContentLoaded", domLoaded);

// add the logic for copying the content of the code cells to the clipboard
// courtesy of https://clipboardjs.com/
let clipboard = new ClipboardJS('.copy-code-btn', {
    target: function(trigger) {
        return trigger.nextElementSibling;
    }
});

// what do do after the item has been copied to the clipboard
clipboard.on('success', function(e) {
    let trigger = e.trigger;
    e.clearSelection();
    const btnTextBackup = trigger.innerHTML;
    trigger.disabled = true;
    trigger.innerText = 'Copied!';
    setTimeout(() => {
        trigger.innerHTML = btnTextBackup;
        trigger.disabled = false;
    }, 2000);
});
