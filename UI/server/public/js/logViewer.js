// logViewer.js

export class LogViewer {
    constructor() {
        this.logContainer = document.getElementById('log-container');
        this.logList = document.getElementById('log-list');
        this.toggleAutoscroll = document.getElementById('toggle-autoscroll');
        
        this.MAX_LOGS = 1000;
        this.logCount = 0;
        this.logBatch = [];
        this.isRendering = false;
        this.isActive = true; // Only render when the tab is active
        
        // Bind the render loop to this instance
        this.renderLogs = this.renderLogs.bind(this);
    }
    
    setActive(active) {
        this.isActive = active;
        if (active && this.logBatch.length > 0 && !this.isRendering) {
            this.isRendering = true;
            requestAnimationFrame(this.renderLogs);
        }
    }

    addLog(logEntry) {
        // We always buffer, but only render if active to save DOM operations
        this.logBatch.push(logEntry);
        
        // Prevent batch from growing infinitely if tab is backgrounded for a long time
        if (this.logBatch.length > this.MAX_LOGS) {
             this.logBatch.splice(0, this.logBatch.length - this.MAX_LOGS);
        }

        if (this.isActive && !this.isRendering) {
            this.isRendering = true;
            requestAnimationFrame(this.renderLogs);
        }
    }

    renderLogs() {
        if (this.logBatch.length === 0 || !this.isActive) {
            this.isRendering = false;
            return;
        }

        const fragment = document.createDocumentFragment();
        const logsToAdd = this.logBatch.splice(0, this.logBatch.length);

        logsToAdd.forEach(logEntry => {
            const li = document.createElement('li');
            li.innerHTML = `
                <div class="log-meta">
                    <span class="timestamp">${logEntry.timestamp}</span> 
                    <span class="source">[${logEntry.from}]</span>
                </div>
                <div class="log-content">${this.formatLogData(logEntry.data)}</div>
            `;
            fragment.appendChild(li);
            this.logCount++;
        });

        this.logList.appendChild(fragment);

        // FIFO mechanism
        while (this.logCount > this.MAX_LOGS && this.logList.firstChild) {
            this.logList.removeChild(this.logList.firstChild);
            this.logCount--;
        }

        // Auto scroll
        if (this.toggleAutoscroll.checked) {
            this.logContainer.scrollTop = this.logContainer.scrollHeight;
        }

        // Continue rendering if more logs arrived
        if (this.logBatch.length > 0 && this.isActive) {
             requestAnimationFrame(this.renderLogs);
        } else {
             this.isRendering = false;
        }
    }

    clear() {
        this.logList.innerHTML = '';
        this.logCount = 0;
        this.logBatch = [];
    }

    // XSS Prevention
    escapeHTML(str) {
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

    formatLogData(data) {
        try {
            const obj = JSON.parse(data);
            const jsonStr = JSON.stringify(obj, null, 2);
            return `<pre class="json-block">${this.syntaxHighlight(jsonStr)}</pre>`;
        } catch (e) {
            return `<span class="plain-text">${this.escapeHTML(data)}</span>`;
        }
    }

    syntaxHighlight(json) {
        json = this.escapeHTML(json);
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
}
