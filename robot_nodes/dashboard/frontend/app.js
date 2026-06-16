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

const imuPitchValue = document.getElementById('imuPitchValue');
const imuRollValue = document.getElementById('imuRollValue');

let activeTargetColor = 'HIJAU';

// ==============================================================
// VARIABEL & SETUP DRAWING BOARD CANVAS
// ==============================================================
const canvas = document.getElementById('trackingCanvas');
const ctx = canvas ? canvas.getContext('2d') : null;

if (canvas) {
    // Resolusi internal kertas (HD 720p aspect ratio supaya tajam)
    canvas.width = 1280;
    canvas.height = 720;
}

// Data memori untuk Simulasi Posisi
let globalCommand = 'STOP'; // Diambil dari Command Python
let globalHeading = 0.0;    // Diambil dari Yaw IMU
let globalSpeed = 0.0;      // Estimasi Speed
let totalDistance = 0.0;

// Titik mulai gambar (Tengah Kertas)
let rX = 640;
let rY = 360;
const pathHistory = [{ x: rX, y: rY }];

function updateDrawingBoard() {
    if (!ctx) return;

    // 1. Logika Kecepatan Simulasi berdasar Command yang aktif
    if (globalCommand === 'F' || globalCommand === 'FORWARD') {
        globalSpeed = 1.0; // Anggap kecepatan lurus penuh (Multiplier)
    } else if (globalCommand === 'BACKUP') {
        globalSpeed = -0.6; // Mundur pelan
    } else if (globalCommand === 'L' || globalCommand === 'R' || globalCommand.includes('STEER')) {
        globalSpeed = 0.3; // Maju sangat pelan sambil belok
    } else {
        globalSpeed = 0.0; // STOP
    }

    // 2. Kalkulasi Vektor Gerak (Berdasarkan IMU Yaw)
    if (globalSpeed !== 0) {
        // Konversi derajat ke radian. Asumsi 0 derajat adalah Utara/Atas.
        // Formula standar navigasi: x += sin(heading), y -= cos(heading)
        let rad = (globalHeading) * Math.PI / 180;

        let dx = Math.sin(rad) * (globalSpeed * 2.5); // 2.5 adalah pengali visual agar rute terlihat
        let dy = -Math.cos(rad) * (globalSpeed * 2.5);

        rX += dx;
        rY += dy;

        totalDistance += Math.abs(globalSpeed) * 0.02; // Tambah kalkulasi jarak tempuh

        // Cegah robot keluar/menghilang dari kertas (Clamp di border)
        rX = Math.max(20, Math.min(canvas.width - 20, rX));
        rY = Math.max(20, Math.min(canvas.height - 20, rY));

        // Simpan titik ke history rute
        pathHistory.push({ x: rX, y: rY });
    }

    // 3. Bersihkan & Gambar Ulang Kertas Putih
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Gambar Garis Grid ala Buku Tulis
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 2;
    for (let i = 0; i < canvas.width; i += 80) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();
    }
    for (let i = 0; i < canvas.height; i += 80) {
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();
    }

    // Gambar Rute Masa Lalu (Tracking Path)
    if (pathHistory.length > 0) {
        ctx.beginPath();
        ctx.strokeStyle = '#0284c7'; // Warna tinta jalur (Biru langit tua)
        ctx.lineWidth = 5;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        for (let i = 1; i < pathHistory.length; i++) {
            ctx.lineTo(pathHistory[i].x, pathHistory[i].y);
        }
        ctx.stroke();
    }

    // Gambar Indikator Robot Sekarang (Titik Merah)
    ctx.beginPath();
    ctx.arc(rX, rY, 12, 0, 2 * Math.PI);
    ctx.fillStyle = '#ef4444';
    ctx.fill();
    ctx.strokeStyle = '#7f1d1d';
    ctx.lineWidth = 4;
    ctx.stroke();

    // Gambar Garis Panah Arah Hadap (Heading)
    let arrowRad = (globalHeading) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(rX, rY);
    ctx.lineTo(rX + Math.sin(arrowRad) * 25, rY - Math.cos(arrowRad) * 25);
    ctx.strokeStyle = '#10b981'; // Panah Hijau
    ctx.lineWidth = 6;
    ctx.stroke();

    // 4. Update Angka Teks di Dashboard
    const elPos = document.getElementById('canvasPosValue');
    const elHead = document.getElementById('canvasHeadingValue');
    const elDist = document.getElementById('canvasDistValue');
    const elSpeed = document.getElementById('canvasSpeedValue');

    // Koordinat virtual, rX dan rY dikurangi pusat agar mulai dari 0,0
    if (elPos) elPos.textContent = `${((rX - 640) / 10).toFixed(1)}, ${((360 - rY) / 10).toFixed(1)}`;
    if (elHead) elHead.textContent = `${globalHeading.toFixed(1)}°`;
    if (elDist) elDist.textContent = `${totalDistance.toFixed(1)} m`;
    if (elSpeed) elSpeed.textContent = `${globalSpeed.toFixed(1)} m/s`;

    // Loop Animasi terus menerus
    requestAnimationFrame(updateDrawingBoard);
}

// Panggil loop pertama kali
requestAnimationFrame(updateDrawingBoard);
// ==============================================================

function formatValue(value, fallback = '-') {
    if (value === null || value === undefined || value === '') return fallback;
    return String(value);
}

function normalizeTargetColor(value) {
    const targetColor = String(value || '').trim().toUpperCase();
    return ['MERAH', 'HIJAU', 'BIRU'].includes(targetColor) ? targetColor : null;
}

function setActiveTargetColor(targetColor) {
    const normalizedTargetColor = normalizeTargetColor(targetColor);
    if (!normalizedTargetColor) return;

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
    updatedAt.textContent = payload.receivedAt ? `Updated ${new Date(payload.receivedAt).toLocaleTimeString()}` : 'Updated now';
    sourceValue.textContent = formatValue(payload.source);
    modeValue.textContent = formatValue(payload.mode);

    if (payload.source === 'deteksi_warna' || payload.source === 'obstacle_avoidance_node') {
        const rawCmd = String(payload.command || '').toUpperCase();
        commandValue.textContent = formatValue(rawCmd);

        // Simpan command ke global untuk dipakai menggambar di canvas
        globalCommand = rawCmd;

        if (payload.target_color) {
            targetColorValue.textContent = formatValue(payload.target_color);
            setActiveTargetColor(payload.target_color);
        }

        if (payload.error !== undefined) errorValue.textContent = formatValue(payload.error);
        if (payload.fps !== undefined) fpsValue.textContent = Number(payload.fps).toFixed(1);
        if (payload.target_visible !== undefined) targetVisibleValue.textContent = payload.target_visible ? 'Target Found' : 'Searching';
        if (payload.target_x !== undefined) targetXValue.textContent = formatValue(payload.target_x);
        if (payload.target_y !== undefined) targetYValue.textContent = formatValue(payload.target_y);

        avoidanceFill.style.width = rawCmd === 'STOP' ? '45%' : '80%';
        visionFill.style.width = payload.target_visible ? '90%' : '25%';
        finishFill.style.width = rawCmd === 'F' ? '95%' : '20%';
    }
    else if (payload.source === 'imu_sensor') {
        if (payload.yaw !== undefined) globalHeading = parseFloat(payload.yaw); // Set YAW untuk kompas gambar
        if (payload.pitch !== undefined && imuPitchValue) imuPitchValue.textContent = formatValue(payload.pitch) + '°';
        if (payload.roll !== undefined && imuRollValue) imuRollValue.textContent = formatValue(payload.roll) + '°';
    }
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
      <span>${item.mode || '-'} | target ${item.target_color || '-'} | ${item.receivedAt ? new Date(item.receivedAt).toLocaleTimeString() : ''}</span>
    `;
        logList.appendChild(element);
    });
}

async function loadState() {
    try {
        const response = await fetch('/api/state');
        const state = await response.json();
        if (state.targetColor) setActiveTargetColor(state.targetColor);
        if (state.latest) renderTelemetry(state.latest);
        renderLogs(state.history || []);
    } catch (error) {
        connectionStatus.textContent = 'Offline';
    }
}

function connectEvents() {
    const source = new EventSource('/events');

    source.addEventListener('open', () => { connectionStatus.textContent = 'Connected'; });

    source.addEventListener('message', (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.payload) renderTelemetry(data.payload);
            if (data.state && data.state.targetColor) setActiveTargetColor(data.state.targetColor);
            if (data.state && data.state.history) renderLogs(data.state.history);
        } catch (error) {
            connectionStatus.textContent = 'Data error';
        }
    });

    source.addEventListener('error', () => { connectionStatus.textContent = 'Reconnecting'; });
}

async function setTargetColor(targetColor) {
    try {
        connectionStatus.textContent = 'Updating target';
        const response = await fetch('/api/target', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ target_color: targetColor }),
        });

        const data = await response.json();
        if (!response.ok || !data.ok) throw new Error(data.error || 'Failed to update target');
        if (data.targetColor) setActiveTargetColor(data.targetColor);

        connectionStatus.textContent = 'Connected';
    } catch (error) {
        connectionStatus.textContent = 'Target update failed';
    }
}

targetButtons.forEach((button) => {
    button.addEventListener('click', () => {
        const targetColor = button.dataset.targetColor;
        if (targetColor) setTargetColor(targetColor);
    });
});

loadState();
connectEvents();