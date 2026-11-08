(() => {
    'use strict';

    const API_BASE = (window.RADAR_API_BASE || 'http://127.0.0.1:5000').replace(/\/$/, '');
    const STREAM_URL = `${API_BASE}/api/radar/stream`;
    const CONTROL_URL = `${API_BASE}/api/radar/control`;
    const LOCK_URL = `${API_BASE}/api/lock`;

    const canvas = document.getElementById('radarCanvas');
    const ctx = canvas.getContext('2d');
    const logBox = document.getElementById('output-container');
    const connEl = document.getElementById('conn-status');
    const modeEl = document.getElementById('lbl-mode');
    const rangeSlider = document.getElementById('range-slider');
    const rangeLabel = document.getElementById('range-label');
    const durationSelect = document.getElementById('scan-duration');
    const startScanButton = document.getElementById('btn-start-scan');
    const stopScanButton = document.getElementById('btn-stop-scan');
    const scanStatusEl = document.getElementById('scan-status');
    const soundToggle = document.getElementById('chk-sound');

    const lblAngle = document.getElementById('lbl-angle');
    const lblDistance = document.getElementById('lbl-distance');
    const lblXY = document.getElementById('lbl-xy');
    const lblTargetId = document.getElementById('lbl-target-id');
    const lblTargetCoords = document.getElementById('lbl-target-coords');
    const lblTargetVel = document.getElementById('lbl-target-vel');
    const lblTargetSize = document.getElementById('lbl-target-size');

    let maxRangeCm = Number(rangeSlider.value);
    let sweepAngle = 0;
    let blips = new Map();
    let lockedTrackId = null;
    let lastPacketAt = 0;
    let lastSequence = -1;
    let eventSource = null;
    let pendingState = null;
    let streamConnected = false;
    let latestScan = { running: true, duration_s: 0, remaining_s: null };
    let audioContext = null;
    let backgroundReady = false;
    const backgroundCanvas = document.createElement('canvas');
    const backgroundContext = backgroundCanvas.getContext('2d');

    const MAX_LOG_LINES = 120;

    function appendLog(message) {
        const rows = logBox.textContent ? logBox.textContent.split('\n') : [];
        rows.push(`[${new Date().toLocaleTimeString()}] ${message}`);
        if (rows.length > MAX_LOG_LINES) rows.splice(0, rows.length - MAX_LOG_LINES);
        logBox.textContent = rows.join('\n');
        logBox.scrollTop = logBox.scrollHeight;
    }

    function beep(frequency = 880, durationMs = 65, volume = 0.035) {
        if (!soundToggle.checked) return;
        try {
            audioContext = audioContext || new (window.AudioContext || window.webkitAudioContext)();
            if (audioContext.state === 'suspended') audioContext.resume();
            const oscillator = audioContext.createOscillator();
            const gain = audioContext.createGain();
            oscillator.type = 'square';
            oscillator.frequency.value = frequency;
            gain.gain.value = volume;
            oscillator.connect(gain).connect(audioContext.destination);
            oscillator.start();
            oscillator.stop(audioContext.currentTime + durationMs / 1000);
        } catch (_) {
            // Audio is optional; unsupported browsers can still use the dashboard.
        }
    }

    function geometry() {
        const width = canvas.width;
        const height = canvas.height;
        const cx = width / 2;
        const cy = height / 2;
        const radius = Math.min(width, height) / 2 - 20;
        return { width, height, cx, cy, radius };
    }

    // 0° is straight ahead (top of the display), +90° is to the right.
    function polarToCanvas(bearingDeg, distanceCm) {
        const { cx, cy, radius } = geometry();
        const radians = (90 - bearingDeg) * Math.PI / 180;
        const distance = (distanceCm / maxRangeCm) * radius;
        return {
            x: cx + distance * Math.cos(radians),
            y: cy - distance * Math.sin(radians),
            r: distance,
            maxRadius: radius,
        };
    }

    function sizeColor(sizeClass, selected) {
        if (selected) return '#ff6262';
        if (sizeClass === 'large') return '#ffc15c';
        if (sizeClass === 'small') return '#00ccff';
        return '#00ed72';
    }

    function drawStaticBackground() {
        const { width, height, cx, cy, radius } = geometry();
        if (backgroundCanvas.width !== width || backgroundCanvas.height !== height) {
            backgroundCanvas.width = width;
            backgroundCanvas.height = height;
        }
        backgroundContext.clearRect(0, 0, width, height);
        backgroundContext.strokeStyle = 'rgba(0, 237, 114, 0.15)';
        backgroundContext.lineWidth = 1;
        for (let ring = 1; ring <= 3; ring++) {
            backgroundContext.beginPath();
            backgroundContext.arc(cx, cy, radius * ring / 3, 0, Math.PI * 2);
            backgroundContext.stroke();
        }

        backgroundContext.strokeStyle = 'rgba(0, 237, 114, 0.10)';
        for (let bearing = 0; bearing < 360; bearing += 30) {
            const radians = (90 - bearing) * Math.PI / 180;
            backgroundContext.beginPath();
            backgroundContext.moveTo(cx, cy);
            backgroundContext.lineTo(cx + radius * Math.cos(radians), cy - radius * Math.sin(radians));
            backgroundContext.stroke();
        }

        // Mark the physical ESP32 scan limits (±80° from the forward direction).
        backgroundContext.strokeStyle = 'rgba(255, 193, 92, 0.42)';
        backgroundContext.setLineDash([5, 5]);
        for (const bearing of [-80, 80]) {
            const radians = (90 - bearing) * Math.PI / 180;
            backgroundContext.beginPath();
            backgroundContext.moveTo(cx, cy);
            backgroundContext.lineTo(cx + radius * Math.cos(radians), cy - radius * Math.sin(radians));
            backgroundContext.stroke();
        }
        backgroundContext.setLineDash([]);

        backgroundContext.fillStyle = 'rgba(0, 237, 114, 0.55)';
        backgroundContext.font = '10px ui-monospace, monospace';
        backgroundContext.textAlign = 'center';
        backgroundContext.textBaseline = 'middle';
        for (let bearing = -90; bearing <= 90; bearing += 30) {
            const radians = (90 - bearing) * Math.PI / 180;
            backgroundContext.fillText(
                `${bearing}°`,
                cx + (radius + 12) * Math.cos(radians),
                cy - (radius + 12) * Math.sin(radians)
            );
        }
        backgroundReady = true;
    }

    function drawRadar() {
        const { width, height, cx, cy, radius } = geometry();
        if (!backgroundReady || backgroundCanvas.width !== width || backgroundCanvas.height !== height) {
            drawStaticBackground();
        }
        ctx.clearRect(0, 0, width, height);
        ctx.drawImage(backgroundCanvas, 0, 0);

        const sweepRadians = (90 - sweepAngle) * Math.PI / 180;
        const endX = cx + radius * Math.cos(sweepRadians);
        const endY = cy - radius * Math.sin(sweepRadians);
        const gradient = ctx.createLinearGradient(cx, cy, endX, endY);
        gradient.addColorStop(0, 'rgba(0, 237, 114, 0)');
        gradient.addColorStop(1, 'rgba(0, 237, 114, 0.72)');
        ctx.strokeStyle = gradient;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(endX, endY);
        ctx.stroke();

        const now = Date.now();
        for (const [id, track] of blips) {
            if (!Number.isFinite(track.distance_cm) || track.distance_cm <= 0) continue;
            const point = polarToCanvas(track.angle_deg, track.distance_cm);
            if (point.r > point.maxRadius) continue;

            const selected = id === lockedTrackId;
            const color = sizeColor(track.size_class, selected);
            const baseRadius = selected || track.size_class === 'large' ? 6 : 4.5;
            const amplitude = Number.isFinite(track.amplitude) ? track.amplitude : 1;
            const dotRadius = baseRadius * (0.75 + amplitude * 0.5);

            ctx.beginPath();
            ctx.arc(point.x, point.y, dotRadius, 0, Math.PI * 2);
            ctx.fillStyle = color;
            ctx.fill();

            if (selected) {
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 1.5;
                ctx.stroke();
                const pulse = (Math.sin(now / 180) + 1) / 2;
                ctx.beginPath();
                ctx.arc(point.x, point.y, 10 + pulse * 9, 0, Math.PI * 2);
                ctx.strokeStyle = `rgba(255, 98, 98, ${0.7 - pulse * 0.45})`;
                ctx.stroke();
            }
        }

        if (lockedTrackId !== null) {
            const selectedTrack = blips.get(lockedTrackId);
            if (selectedTrack) {
                const point = polarToCanvas(selectedTrack.angle_deg, selectedTrack.distance_cm);
                ctx.beginPath();
                ctx.moveTo(cx, cy);
                ctx.lineTo(point.x, point.y);
                ctx.setLineDash([4, 4]);
                ctx.strokeStyle = 'rgba(255, 98, 98, 0.55)';
                ctx.stroke();
                ctx.setLineDash([]);
            }
        }
    }

    function updateConnectionStatus() {
        if (!streamConnected) {
            connEl.textContent = '○ DISCONNECTED';
            connEl.className = 'conn-status disconnected';
            return;
        }
        const age = lastPacketAt ? Date.now() - lastPacketAt : Infinity;
        if (latestScan.running && age < 1800) {
            connEl.textContent = '● LIVE';
            connEl.className = 'conn-status live';
        } else if (!latestScan.running) {
            connEl.textContent = '● CONNECTED · PAUSED';
            connEl.className = 'conn-status live';
        } else {
            connEl.textContent = '○ WAITING FOR SENSOR';
            connEl.className = 'conn-status disconnected';
        }
    }

    function updateScanStatus(scan) {
        if (!scan) return;
        latestScan = scan;
        if (!scan.running) {
            scanStatusEl.textContent = scan.reason === 'duration_elapsed' ? 'Finished' : 'Stopped';
            startScanButton.disabled = false;
            stopScanButton.disabled = true;
            modeEl.textContent = 'SCAN PAUSED';
        } else {
            if (scan.duration_s === 0) {
                scanStatusEl.textContent = 'Running · continuous';
            } else {
                const remaining = scan.remaining_s == null ? scan.duration_s : Math.ceil(scan.remaining_s);
                scanStatusEl.textContent = `Running · ${remaining}s left`;
            }
            startScanButton.disabled = true;
            stopScanButton.disabled = false;
            modeEl.textContent = `SCANNING · ${scan.sector || 'RADAR'}`;
        }
    }

    function updateLockDisplay() {
        const track = blips.get(lockedTrackId);
        if (!track) return;
        lblTargetId.textContent = `TRACK #${track.id}`;
        lblTargetCoords.textContent = `${track.angle_deg.toFixed(1)}° / ${track.distance_cm.toFixed(1)}cm`;
        lblTargetVel.textContent = `${(track.v_angle || 0).toFixed(1)}°/s · ${(track.v_distance || 0).toFixed(1)}cm/s`;
        lblTargetSize.textContent = track.width_cm != null
            ? `${track.width_cm.toFixed(1)}cm · ${(track.size_class || 'unknown').toUpperCase()}`
            : 'measuring…';
    }

    function resetLockDisplay() {
        lblTargetId.textContent = 'NONE';
        lblTargetCoords.textContent = '—';
        lblTargetVel.textContent = '—';
        lblTargetSize.textContent = '—';
    }

    async function postJson(url, body) {
        const response = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
        });
        const result = await response.json().catch(() => ({}));
        if (!response.ok || result.status === 'error') {
            throw new Error(result.message || `HTTP ${response.status}`);
        }
        return result;
    }

    async function startScan() {
        const durationSeconds = Number(durationSelect.value);
        startScanButton.disabled = true;
        try {
            const result = await postJson(CONTROL_URL, { action: 'start', duration_s: durationSeconds });
            updateScanStatus(result.scan);
            appendLog(durationSeconds === 0
                ? '[SCAN] Started continuous scan.'
                : `[SCAN] Started ${durationSeconds}s scan.`);
            beep(1050, 65, 0.035);
        } catch (error) {
            appendLog(`[ERROR] Could not start scan: ${error.message}`);
            startScanButton.disabled = false;
        }
    }

    async function stopScan() {
        stopScanButton.disabled = true;
        try {
            const result = await postJson(CONTROL_URL, { action: 'stop' });
            updateScanStatus(result.scan);
            appendLog('[SCAN] Stopped by dashboard.');
            beep(350, 90, 0.035);
        } catch (error) {
            appendLog(`[ERROR] Could not stop scan: ${error.message}`);
            stopScanButton.disabled = false;
        }
    }

    async function setVisualLock(targetId) {
        try {
            await postJson(LOCK_URL, { mode: 'lock', target_id: targetId });
        } catch (error) {
            appendLog(`[WARN] Could not sync selected track: ${error.message}`);
        }
    }

    async function clearVisualLock() {
        try {
            await postJson(LOCK_URL, { mode: 'scan' });
        } catch (error) {
            appendLog(`[WARN] Could not clear selected track: ${error.message}`);
        }
    }

    function clearSelection(reason) {
        if (lockedTrackId === null) return;
        lockedTrackId = null;
        resetLockDisplay();
        clearVisualLock();
        if (reason) appendLog(`[SELECTION CLEARED] ${reason}`);
    }

    function selectTrack(id) {
        const track = blips.get(id);
        if (!track) return;
        lockedTrackId = id;
        updateLockDisplay();
        setVisualLock(id);
        appendLog(`[TRACK SELECTED] #${id} at ${track.angle_deg.toFixed(1)}° / ${track.distance_cm.toFixed(1)}cm`);
        beep(1200, 60, 0.04);
    }

    function processMeasurements(state) {
        const measurement = state.last_measurement;
        if (measurement) {
            lblAngle.textContent = `${Number(measurement.angle_deg).toFixed(1)}°`;
            const distance = Number(measurement.distance_cm);
            lblDistance.textContent = Number.isFinite(distance) && distance > 0 ? `${distance.toFixed(1)} cm` : 'No echo';
            lblXY.textContent = `${measurement.x}, ${measurement.y}`;
            sweepAngle = Number(measurement.angle_deg) || 0;
        }

        const tracks = Array.isArray(state.tracked_objects) ? state.tracked_objects : [];
        const nextTracks = new Map();
        for (const track of tracks) {
            nextTracks.set(track.id, track);
            const previous = blips.get(track.id);
            if (!previous && track.hits >= 3) {
                appendLog(`[NEW TRACK #${track.id}] bearing=${track.angle_deg.toFixed(1)}° distance=${track.distance_cm.toFixed(1)}cm`);
                beep(1200, 55, 0.025);
            }
            if (track.width_cm != null && previous && previous.width_cm == null) {
                appendLog(`[SIZE] Track #${track.id}: estimated width ${track.width_cm.toFixed(1)}cm (${track.size_class})`);
            }
        }
        blips = nextTracks;

        if (lockedTrackId !== null && !blips.has(lockedTrackId)) {
            clearSelection('selected track was lost');
        } else if (lockedTrackId !== null) {
            updateLockDisplay();
        }
    }

    function processPendingState() {
        if (!pendingState) return;
        const state = pendingState;
        pendingState = null;
        updateScanStatus(state.scan);

        const sequence = Number(state.seq);
        if (Number.isFinite(sequence) && sequence !== lastSequence) {
            lastSequence = sequence;
            lastPacketAt = Date.now();
            processMeasurements(state);
        }

        if (state.error) {
            scanStatusEl.textContent = `Sensor error: ${state.error}`;
        }
    }

    function connectStream() {
        if (eventSource) eventSource.close();
        appendLog(`Connecting to ${STREAM_URL}`);
        eventSource = new EventSource(STREAM_URL);
        eventSource.onopen = () => {
            streamConnected = true;
            appendLog('Backend event stream connected.');
        };
        eventSource.onmessage = (event) => {
            try {
                pendingState = JSON.parse(event.data);
            } catch (_) {
                appendLog('[WARN] Received an invalid stream packet.');
            }
        };
        eventSource.onerror = () => {
            streamConnected = false;
            connEl.textContent = '○ RECONNECTING…';
            connEl.className = 'conn-status disconnected';
            // EventSource retries automatically.
        };
    }

    canvas.addEventListener('click', (event) => {
        const rect = canvas.getBoundingClientRect();
        const mouseX = (event.clientX - rect.left) * canvas.width / rect.width;
        const mouseY = (event.clientY - rect.top) * canvas.height / rect.height;
        let nearestId = null;
        let nearestDistance = 20;

        for (const [id, track] of blips) {
            if (!Number.isFinite(track.distance_cm) || track.distance_cm <= 0) continue;
            const point = polarToCanvas(track.angle_deg, track.distance_cm);
            const distance = Math.hypot(mouseX - point.x, mouseY - point.y);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestId = id;
            }
        }

        if (nearestId !== null) selectTrack(nearestId);
        else clearSelection('manual deselection');
    });

    rangeSlider.addEventListener('input', () => {
        maxRangeCm = Number(rangeSlider.value);
        rangeLabel.textContent = String(maxRangeCm);
        drawRadar();
    });
    startScanButton.addEventListener('click', startScan);
    stopScanButton.addEventListener('click', stopScan);

    function animationLoop() {
        processPendingState();
        drawRadar();
        updateConnectionStatus();
        requestAnimationFrame(animationLoop);
    }

    appendLog('Dashboard initialized.');
    connectStream();
    requestAnimationFrame(animationLoop);
})();
