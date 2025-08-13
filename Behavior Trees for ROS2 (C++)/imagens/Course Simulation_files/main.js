let BASE_URL = '/xterm';
// original_url = 'i-000864de7d5e94740.robotigniteacademy.com/f1083ea6-566f-4d74-b9d9-30da72b1674d/xterm/pid/583'
// remote_server = 'i-000864de7d5e94740.robotigniteacademy.com/xterm/f1083ea6-566f-4d74-b9d9-30da72b1674d/pid/583'
// http://localhost:3000/xterm/pid/584?remote_server=${remote_server}

try {
  let slot_prefix = location.pathname.split('/')[1]
  if (slot_prefix !== 'xterm'){
    BASE_URL = '/' + slot_prefix + '/xterm';
  }
} catch (e) {
  console.warn(e);
}

const queryString = window.location.search;
const urlParams = new URLSearchParams(queryString);

// -----------------------------
// Auto run ~/course_install.sh?
// -----------------------------
const isCourseInstall = urlParams.get('isCourseInstall')
let isCourseInstallQuery = ''
if (isCourseInstall){
  isCourseInstallQuery = '?isCourseInstall=1'
}

// -----------------------------
// Auto run ~/launch-sim.sh?
// -----------------------------
const isLaunchSim = urlParams.get('isLaunchSim')
let isLaunchSimQuery = ''
if (isLaunchSim){
  isLaunchSimQuery = '?isLaunchSim=1'
}

// Remote server
const remote_server = urlParams.get('remote_server');
if (remote_server) {
  // e.g: i-06bf505e754542da8.robotigniteacademy.com/138dcc4c-bc31-45d7-b346-cfa0a0a7c6a9/xterm/pid/584
  BASE_URL = remote_server.split('/').slice(0, 3).join('/')
}

// Log
let log_user_id = urlParams.get('log_user_id');
const LOG_SERVER_DOMAIN = 'https://app.theconstruct.ai'
const frontendLog = (level, text) => {
  console.log(level, text);
  try {
    if( ! log_user_id)
      return

    // Removing the last "/" from "username/"
    if(log_user_id.endsWith('/')) {
      log_user_id = log_user_id.split('/')[0]
    }

    if (log_user_id.length == 37) log_user_id = log_user_id.substring(0, 36)
    const data = {
      user_id: log_user_id,
      text: `[xterm] ${text}`,
      level: level
    };
    const body = JSON.stringify(data);
    fetch(`${LOG_SERVER_DOMAIN}/front-logs/`, {
      method: 'post',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body
    });
  } catch (ex) {
    console.error(ex)
  }
}

console.info('xterm.BASE_URL: ' + BASE_URL);

var term,
    protocol,
    socketURL,
    queryPid,
    fetchURL,
    socket,
    pid,
    charWidth,
    charHeight;
var pingInterval = null;
var reloadTimeout = null;

var terminalContainer = document.getElementById('terminal-container'),
    optionElements = {
      cursorBlink: document.querySelector('#option-cursor-blink'),
      cursorStyle: document.querySelector('#option-cursor-style'),
      scrollback: document.querySelector('#option-scrollback'),
      tabstopwidth: document.querySelector('#option-tabstopwidth'),
      bellStyle: document.querySelector('#option-bell-style')
    };

function setTerminalSize () {
  // Automatically resize using the fit add on...
  term.fit();

  // Then, leave one extra row at the bottom
  var geometry = term.proposeGeometry();
  term.resize(geometry.cols, geometry.rows - 1)
}

var addEvent = function(object, type, callback) {
  if (object == null || typeof(object) == 'undefined') return;
  if (object.addEventListener) {
      object.addEventListener(type, callback, false);
  } else if (object.attachEvent) {
      object.attachEvent("on" + type, callback);
  } else {
      object["on"+type] = callback;
  }
};
addEvent(window, "resize", setTerminalSize);

createTerminal();

window.addEventListener("message", receiveMessage, false);

document.addEventListener("click", request_focus);

function request_focus(e) {
  parent.postMessage(JSON.stringify({type: 'shell', data: 'request_focus'}), '*');
}

var terminalCmds = {
  focus: 0
};

function receiveMessage(event) {
  if(event.data.cmd != undefined && event.data.cmd == terminalCmds.focus) {
    term.focus();
    if (term.sendFocus) {
      term.send(EscapeSequences_1.C0.ESC + '[I');
    }
    term.element.classList.add('focus');
    term.showCursor();
    term.restartCursorBlinking.apply(term);
    term.emit('focus');
  }
}

function createTerminal() {
  // Clean terminal
  while (terminalContainer.children.length) {
    terminalContainer.removeChild(terminalContainer.children[0]);
  }

  term = new Terminal({
    cursorBlink: true,
    // Setting scrollback to infinite:
    //  https://github.com/xtermjs/xterm.js/issues/518
    // When setting to infinity, the browser was getting frozen for one of
    // the Master's student. Therefore, I'm now limitting to 7000 lines.
    scrollback: 1000, // Original was 1000.
    tabStopWidth: 8
  });

  term.on('resize', function (size) {
    if (!pid) {
      return;
    }
    var cols = size.cols,
        rows = size.rows,
        url = fetchURL + '/terminals/' + pid + '/size?cols=' + cols + '&rows=' + rows;

    fetch(url, {method: 'POST'});
  });

  var usage_already_logged = false;

  // to check every 30s
  var interval_to_check_data = 30 * 1000; // (ms)

  // clear interval before setting up - it may happen it is not cleared on close event
  if (pingInterval) clearInterval(pingInterval)
  pingInterval = setInterval(function() {
    try {
      socket.send('');
      var ts_now = Date.now();
      var diff = ts_now - ts_last_empty_msg;
      if (diff > (2 * interval_to_check_data)) {
        console.info(`${new Date()}: close terminal manually due to inactivity`);
        socket.close();
        onCloseHandler();
      }
    }
    catch (ex) {
      console.error(ex)
    }
  }, interval_to_check_data);

  var ts_last_empty_msg = Date.now();

  function socketOnMessage(ev) {
    if(ev.data == '') {
      ts_last_empty_msg = Date.now();
    }
  }

  term.on('data', function(data) {
    if (!usage_already_logged) {
      usage_already_logged = true;
      var check_sh_used = location.search.split('check_sh_used=')[1];
      if (check_sh_used == 'true') {
        parent.postMessage(JSON.stringify({type : 'shell', data : 'used'}), '*');
      }
    }
  });

  protocol = (location.protocol === 'https:') ? 'wss://' : 'wss://';

  if (remote_server) {
    console.info('Terminal: setup remote_server')
    socketURL = `${protocol}${BASE_URL}/terminals/`
    fetchURL = `https://${BASE_URL}`
  } else {
    socketURL = protocol + location.hostname + ((location.port) ? (':' + location.port) : '') + BASE_URL + '/terminals/';
    fetchURL = `${BASE_URL}`
  }
  console.info('Terminal Socket URL: ' + socketURL)
  console.info('Terminal Fetch URL: ' + fetchURL)

  term.open(terminalContainer);
  term.fit();

  term.attachCustomKeyEventHandler(function (e) {
    if ((e.ctrlKey) && (e.key == 'c')) {
      if(term.getSelection() != '') {
        return false;
      } else {
        try {
          if (parent)
            parent.postMessage(JSON.stringify({ TC: true, app: 'xterm', data: { event: 'ctrl+c' } }), '*');
        } catch (ex) {
          console.error(ex)
        }
      }
    }
  });

  var initialGeometry = term.proposeGeometry(),
      cols = initialGeometry.cols,
      rows = initialGeometry.rows;

  const pathArgs = remote_server.split('/');
  let queryString = 'terminals';
  if (pathArgs.length > 3 && pathArgs[3] === 'pid') {
    window.queryPid = pathArgs[4]
    queryString += '/pid'
    queryString += `?cols=${term.cols}&rows=${term.rows}`
    queryString += `&pid=${queryPid}`;
    console.info(queryString)
  } else {
    console.error('Terminal query string was not setup properly')
  }

  fetch(`${fetchURL}/${queryString}`, {method: 'POST'}).then(function (res) {
    document.getElementById("terminal-reload").style["display"] = "none";
    document.getElementById("terminal-countdown").style["display"] = "none";
    document.getElementById("terminal-loading").style["display"] = "none";
    document.getElementById("terminal-container").style["display"] = "block";

    charWidth = Math.ceil(term.element.offsetWidth / cols);
    charHeight = Math.ceil(term.element.offsetHeight / rows);

    res.text().then(function (pid) {
      if (pid != queryPid) {
        sendToNewPIDPage(pid, queryPid)
        return
      }
      // We do 'isCourseInstallQuery + isLaunchSimQuery' becauase only one of
      // them will be set, not both at the same time.
      window.pid = pid;
      socketURL += pid + isCourseInstallQuery + isLaunchSimQuery;
      try {
        if(reconnectionStatus?.interval) clearInterval(reconnectionStatus.interval);
      } catch (ex) {
        console.error(ex)
      }
      socket = new WebSocket(socketURL);
      socket.onopen = runRealTerminal;
      socket.onclose = onCloseHandler;
      socket.onerror = onErrorHandler;
      socket.addEventListener('message', socketOnMessage);
    }).catch(function (err) {
      console.error('Something wrong when reading terminal PID connection')
      console.error(err)

      frontendLog(30, 'Failed reading terminal PID connection')
      frontendLog(30, JSON.stringify(err))
    });

    setTimeout(() => {
      term.resize(term.geometry[0] - 4, term.geometry[1] - 1)
    }, 1000)

    setTimeout(() => {
      term.resize(term.geometry[0] - 4, term.geometry[1])
    }, 2000)
  }).catch((err) => {
    onCloseHandler()
    console.error('Error when trying to create terminal', err)
  });
}

/*
0 - Idle
1 - Busy
*/
var reloadButtonState = 0;
function setReloadButtonIdle() {
  reloadButtonState = 0;
}
function setReloadButtonBusy() {
  reloadButtonState = 1;
}
function isReloadButtonIdle() {
  return reloadButtonState == 0;
}
function isReloadButtonBusy() {
  return reloadButtonState == 1;
}

function sendToNewPIDPage(pid, queryPid) {
  frontendLog(40, `pid and queryPid were different: ${pid} != ${queryPid}`)

  console.warn(`Terminal pid and queryPid were different: ${pid} != ${queryPid}`)
  console.info('send to new page ' + pid)
  console.log(window.location.href)
  let remote_server = urlParams.get('remote_server')
  let remoteServerArr = remote_server.split('/').slice(0, -1)
  remoteServerArr.push(pid)
  urlParams.set('remote_server', remoteServerArr.join('/'))
  const [address, search] = window.location.href.split('?')
  const newAddress = `${window.location.origin}${window.location.pathname}?${urlParams.toString()}`
  console.info('New address: ', newAddress)
  window.location.href = newAddress
}

function reloadTerminal() {
  // window.location.reload();
  if (isReloadButtonIdle()) {
    setReloadButtonBusy();
    checkConnection(
      function() {
        if (reloadTimeout) clearTimeout(reloadTimeout);
        window.location.reload();
      }, function() {
        var innerHTML = 'Cannot connect to terminal. Please check your internet connection and ';
        innerHTML += '<a href="#" onclick="reloadTerminal()" style="color:blue;">click to reload the terminal</a>.';
        document.getElementById("terminal-loading").innerHTML = innerHTML;
        document.getElementById("terminal-loading").style["display"] = "block";
        document.getElementById("terminal-countdown").style["display"] = "none";
        document.getElementById("terminal-reload").style["display"] = "none";
        reloadTimeout = setTimeout(reloadTerminal, 30 * 1000);
        // alert('Please check your internet connection and try again. If the problem persists, please try to reload the page.');
      }, function() {
        setReloadButtonIdle();
      }
    );
  }
}

function checkConnection(okClbk, errorClbk, completeClbk) {
  $.ajax({
    type: "GET",
    url: window.location.href,
    success: function(data) {
      okClbk();
    }, error: function(e) {
      errorClbk();
    }, complete: function(e) {
      completeClbk();
    }
  });
}

const modeRetryConnectionAutomatically = false;
const retryMaxTrials = 15
const retryTimeBetween = 3
const reconnectionStatus = {
  trials: 0,
  maxTrials: retryMaxTrials,
  timeBetween: retryTimeBetween, // seconds
  interval: null,
};
function retryConnectionAutomatically() {
  // auto-reconnect mode or static button mode
  if (modeRetryConnectionAutomatically) {
    console.info('Terminal: start auto-reconnection mode')
    // max trials done
    if (reconnectionStatus.trials >= reconnectionStatus.maxTrials) {
      document.getElementById("terminal-reload").style["display"] = "block";
      document.getElementById("terminal-countdown").style["display"] = "none";
      return
    }
    // countdown and re-trying
    document.getElementById("terminal-container").style["display"] = "none";
    document.getElementById("terminal-loading").style["display"] = "none";
    document.getElementById("terminal-reload").style["display"] = "none";
    document.getElementById("terminal-countdown").style["display"] = "block";
    const el = document.getElementById("terminal-countdown-text");
    reconnectionStatus.interval = setInterval(() => {
      var innerHTML = `Terminal is disconnected. Trying to reconnect in ${reconnectionStatus.timeBetween} seconds `;
      innerHTML += '<a href="#" onclick="reloadTerminal()" style="color:blue;">or click to reload</a>';
      el.innerHTML = innerHTML;
      reconnectionStatus.timeBetween--;
      if (reconnectionStatus.timeBetween <= 0) {
        reconnectionStatus.trials++;
        reconnectionStatus.timeBetween = retryTimeBetween;
        clearInterval(reconnectionStatus.interval);
        createTerminal();
      }
    }, 1000);
  } else {
    document.getElementById("terminal-reload").style["display"] = "block";
  }
}

function onCloseHandler() {
  clearInterval(pingInterval);
  setTimeout(reloadTerminal(), 3000);
  // retryConnectionAutomatically();
}

function onErrorHandler(err) {
  console.error('Error on connecting terminal');
  frontendLog(40, 'Error on connecting terminal');
  if (err) {
    console.error(JSON.stringify(err));
    frontendLog(40, JSON.stringify(err));
  }
  else {
    console.error('No error messages to log');
    frontendLog(40, 'No error messages to log');
  }
  onCloseHandler();
}

function runRealTerminal() {
  console.info('Terminal connected to websocket');
  frontendLog(20, 'Connected to websocket');
  if (parent) {
    parent.postMessage(JSON.stringify({ TC: true, app: 'xterm', data: { event: 'connected', pid } }), '*');
  } else {
    console.error('Could not find window.parent object');
    frontendLog(40, 'Could not find window.parent object');
  }
  document.getElementById("terminal-countdown").style["display"] = "none";
  document.getElementById("terminal-loading").style["display"] = "none";
  document.getElementById("terminal-reload").style["display"] = "none";
  term.attach(socket);
  term.on('key', function (key, ev) {
    if (ev.keyCode == 13) {
      try {
        if(parent) {
          parent.postMessage(JSON.stringify({ TC: true, app: 'xterm', data: { event: 'enter' } }), '*');
        }
      } catch (ex) {
        console.error(ex)
      }
    }
  })
  term._initialized = true;
}

function runFakeTerminal() {
  if (term._initialized) {
    return;
  }

  term._initialized = true;

  var shellprompt = '$ ';

  term.prompt = function () {
    term.write('\r\n' + shellprompt);
  };

  term.writeln('Sorry about this');
  term.writeln('Something went wrong, it was not possible to connect to the remote terminal');
  term.writeln('');
  term.prompt();

  term.on('key', function (key, ev) {
    var printable = (
      !ev.altKey && !ev.altGraphKey && !ev.ctrlKey && !ev.metaKey
    );

    if (ev.keyCode == 13) {
      term.prompt();
    } else if (ev.keyCode == 8) {
     // Do not delete the prompt
      if (term.x > 2) {
        term.write('\b \b');
      }
    } else if (printable) {
      term.write(key);
    }
  });

  term.on('paste', function (data, ev) {
    term.write(data);
  });
}
