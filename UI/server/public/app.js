const socket = io();

const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const btnConnect = document.getElementById('btn-connect');
const btnDisconnect = document.getElementById('btn-disconnect');
const btnClear = document.getElementById('btn-clear');
const toggleAutoscroll = document.getElementById('toggle-autoscroll');
const logContainer = document.getElementById('log-container');
const logList = document.getElementById('log-list');

// Giới hạn bộ nhớ UI: Tối đa 1000 dòng log
const MAX_LOGS = 1000;
let logCount = 0;

function updateStatus(state) {
    statusDot.className = 'dot';
    if (state === 'listening') {
        statusDot.classList.add('green');
        statusText.textContent = 'Listening (Port 8080)';
    } else if (state === 'disconnected') {
        statusDot.classList.add('red');
        statusText.textContent = 'Disconnected';
    } else if (state === 'reconnecting') {
        statusDot.classList.add('yellow');
        statusText.textContent = 'Reconnecting...';
    }
}

// Lắng nghe trạng thái từ Backend
socket.on('status', (state) => {
    updateStatus(state);
});

// Lắng nghe log mới đẩy xuống từ Backend (Tách biệt luồng UI)
socket.on('log', (logEntry) => {
    const li = document.createElement('li');
    li.innerHTML = `
        <div class="log-meta">
            <span class="timestamp">${logEntry.timestamp}</span> 
            <span class="source">[${logEntry.from}]</span>
        </div>
        <div class="log-content">${formatLogData(logEntry.data)}</div>
    `;
    logList.appendChild(li);
    logCount++;

    // Cơ chế FIFO để tránh treo trình duyệt
    if (logCount > MAX_LOGS) {
        logList.removeChild(logList.firstChild);
        logCount--;
    }

    // Auto scroll logic
    if (toggleAutoscroll.checked) {
        logContainer.scrollTop = logContainer.scrollHeight;
    }
});

btnConnect.addEventListener('click', () => {
    socket.emit('command', 'connect');
});

btnDisconnect.addEventListener('click', () => {
    socket.emit('command', 'disconnect');
});

btnClear.addEventListener('click', () => {
    logList.innerHTML = '';
    logCount = 0;
});

// Xử lý XSS an toàn
function escapeHTML(str) {
    return str.replace(/[&<>'"]/g, 
        tag => ({
            '&': '&amp;',
            '<': '&lt;',
            '>': '&gt;',
            "'": '&#39;',
            '"': '&quot;'
        }[tag] || tag)
    );
}

// Format dữ liệu log, nếu là JSON thì parse và tô màu
function formatLogData(data) {
    try {
        const obj = JSON.parse(data);
        const jsonStr = JSON.stringify(obj, null, 2);
        return `<pre class="json-block">${syntaxHighlight(jsonStr)}</pre>`;
    } catch (e) {
        return `<span class="plain-text">${escapeHTML(data)}</span>`;
    }
}

// Syntax highlighter cho JSON
function syntaxHighlight(json) {
    json = escapeHTML(json);
    return json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
        let cls = 'json-number';
        if (/^"/.test(match)) {
            if (/:$/.test(match)) {
                cls = 'json-key';
            } else {
                cls = 'json-string';
            }
        } else if (/true|false/.test(match)) {
            cls = 'json-boolean';
        } else if (/null/.test(match)) {
            cls = 'json-null';
        }
        return '<span class="' + cls + '">' + match + '</span>';
    });
}
