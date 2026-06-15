const connectionStatus = document.getElementById('connectionStatus');
const updatedAt = document.getElementById('updatedAt');
const sourceValue = document.getElementById('sourceValue');
const modeValue = document.getElementById('modeValue');
const commandValue = document.getElementById('commandValue');
const targetColorValue = document.getElementById('targetColorValue');
const currentTargetLabel = document.getElementById('currentTargetLabel');
const targetStatus = document.getElementById('targetStatus');
const errorValue = document.getElementById('errorValue');
const fpsValue = document.getElementById('fpsValue');
const targetVisibleValue = document.getElementById('targetVisibleValue');
const targetXValue = document.getElementById('targetXValue');
const targetYValue = document.getElementById('targetYValue');
const targetAreaValue = document.getElementById('targetAreaValue');
const frameValue = document.getElementById('frameValue');
const logList = document.getElementById('logList');
const avoidanceFill = document.getElementById('avoidanceFill');
const visionFill = document.getElementById('visionFill');
const finishFill = document.getElementById('finishFill');
const targetButtons = Array.from(document.querySelectorAll('[data-target-color]'));

let activeTargetColor = 'HIJAU';

function formatValue(value, fallback = '-') {
    if (value === null || value === undefined || value === '') {
        return fallback;
    }
    return String(value);
}

function normalizeTargetColor(value) {
    const targetColor = String(value || '').trim().toUpperCase();
    return ['MERAH', 'HIJAU', 'BIRU'].includes(targetColor) ? targetColor : null;
}

function setActiveTargetColor(targetColor) {
    const normalizedTargetColor = normalizeTargetColor(targetColor);
    if (!normalizedTargetColor) {
        return;
    }

    activeTargetColor = normalizedTargetColor;
    currentTargetLabel.textContent = `Current target: ${normalizedTargetColor}`;
    targetStatus.textContent = `Target aktif: ${normalizedTargetColor}`;

    targetButtons.forEach((button) => {
        const isActive = button.dataset.targetColor === normalizedTargetColor;
        button.classList.toggle('is-active', isActive);
        button.setAttribute('aria-pressed', String(isActive));
    });
}

function renderTelemetry(payload) {
    sourceValue.textContent = formatValue(payload.source);
    modeValue.textContent = formatValue(payload.mode);
    commandValue.textContent = formatValue(payload.command);
    targetColorValue.textContent = formatValue(payload.target_color);
    errorValue.textContent = formatValue(payload.error);
    fpsValue.textContent = payload.fps ? Number(payload.fps).toFixed(1) : '-';
    targetVisibleValue.textContent = payload.target_visible ? 'Target Found' : 'Searching';
    targetXValue.textContent = formatValue(payload.target_x);
    targetYValue.textContent = formatValue(payload.target_y);
    targetAreaValue.textContent = formatValue(payload.target_area);
    frameValue.textContent = payload.frame_width && payload.frame_height
        ? `${payload.frame_width} x ${payload.frame_height}`
        : '-';
    updatedAt.textContent = payload.receivedAt ? `Updated ${new Date(payload.receivedAt).toLocaleTimeString()}` : 'Updated now';

    if (payload.target_color) {
        setActiveTargetColor(payload.target_color);
    }

    const command = String(payload.command || '').toUpperCase();
    avoidanceFill.style.width = command === 'STOP' ? '45%' : '80%';
    visionFill.style.width = payload.target_visible ? '90%' : '25%';
    finishFill.style.width = command === 'F' ? '95%' : '20%';
}

function renderLogs(history = []) {
    logList.innerHTML = '';
    if (!history.length) {
        logList.innerHTML = '<div class="log-item"><strong>No telemetry yet</strong><span>Waiting for data from Python or other robot programs.</span></div>';
        return;
    }

    history.slice(0, 8).forEach((item) => {
        const element = document.createElement('div');
        element.className = 'log-item';
        element.innerHTML = `
      <strong>${item.source || 'unknown'} • ${item.command || '-'}</strong>
      <span>${item.mode || '-'} | target ${item.target_color || '-'} | error ${item.error ?? '-'} | ${item.receivedAt ? new Date(item.receivedAt).toLocaleTimeString() : ''}</span>
    `;
        logList.appendChild(element);
    });
}

async function loadState() {
    try {
        const response = await fetch('/api/state');
        const state = await response.json();
        if (state.targetColor) {
            setActiveTargetColor(state.targetColor);
        }
        if (state.latest) {
            renderTelemetry(state.latest);
        }
        renderLogs(state.history || []);
    } catch (error) {
        connectionStatus.textContent = 'Offline';
    }
}

function connectEvents() {
    const source = new EventSource('/events');

    source.addEventListener('open', () => {
        connectionStatus.textContent = 'Connected';
    });

    source.addEventListener('message', (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.payload) {
                renderTelemetry(data.payload);
            }
            if (data.state && data.state.targetColor) {
                setActiveTargetColor(data.state.targetColor);
            }
            if (data.state && data.state.history) {
                renderLogs(data.state.history);
            }
        } catch (error) {
            connectionStatus.textContent = 'Data error';
        }
    });

    source.addEventListener('error', () => {
        connectionStatus.textContent = 'Reconnecting';
    });
}

async function setTargetColor(targetColor) {
    try {
        connectionStatus.textContent = 'Updating target';
        const response = await fetch('/api/target', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ target_color: targetColor }),
        });

        const data = await response.json();
        if (!response.ok || !data.ok) {
            throw new Error(data.error || 'Failed to update target');
        }

        if (data.targetColor) {
            setActiveTargetColor(data.targetColor);
        }
        connectionStatus.textContent = 'Connected';
    } catch (error) {
        connectionStatus.textContent = 'Target update failed';
    }
}

targetButtons.forEach((button) => {
    button.addEventListener('click', () => {
        const targetColor = button.dataset.targetColor;
        if (targetColor) {
            setTargetColor(targetColor);
        }
    });
});

loadState();
connectEvents();