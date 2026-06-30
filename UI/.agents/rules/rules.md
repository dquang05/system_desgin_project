---
trigger: always_on
---

# UI Project Agent Rules

## Tech Stack

- Backend: Node.js, Express.js, Socket.io, `dgram` (UDP server).
- Frontend: Vanilla HTML, CSS, JavaScript. No frontend frameworks.
- Backend receives UDP datagrams from the MCU and forwards them to the frontend via WebSockets.

## Backend

- Keep the UDP-to-WebSocket bridge architecture.
- Implement UDP socket self-recovery on errors using automatic restart (e.g., `setTimeout`).
- Clean up all sockets and timers on `SIGINT` and `SIGTERM`.

## Frontend

- Use only native DOM APIs.
- Batch high-frequency updates with `requestAnimationFrame` and `DocumentFragment`.
- Limit log elements with a FIFO queue (maximum 1000).
- Sanitize all incoming data before inserting it into the DOM.
- Preserve the dark theme using `--bg-color`, `--panel-bg`, and `--accent`.
- Provide real-time status indicators and controls (auto-scroll, clear logs).
- Automatically parse and syntax-highlight JSON payloads in logs.

```

```
