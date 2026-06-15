const http = require('http');
const dgram = require('dgram');
const fs = require('fs');
const path = require('path');
const url = require('url');

const HTTP_PORT = process.env.PORT ? Number(process.env.PORT) : 3000;
const UDP_PORT = process.env.TELEMETRY_UDP_PORT ? Number(process.env.TELEMETRY_UDP_PORT) : 5006;
const UDP_HOST = process.env.TELEMETRY_UDP_HOST || '0.0.0.0';
const TARGET_COMMAND_PORT = process.env.TARGET_COMMAND_PORT ? Number(process.env.TARGET_COMMAND_PORT) : 5007;
const TARGET_COMMAND_HOST = process.env.TARGET_COMMAND_HOST || '127.0.0.1';

const TARGET_COLORS = new Set(['MERAH', 'HIJAU', 'BIRU']);

const frontendDir = path.join(__dirname, '..', 'frontend');

const state = {
    latest: null,
    sources: {},
    history: [],
    updatedAt: null,
    targetColor: 'HIJAU',
};

const sseClients = new Set();

function normalizeTelemetry(payload) {
    const source = typeof payload.source === 'string' && payload.source.trim() ? payload.source.trim() : 'unknown';
    return {
        ...payload,
        source,
        receivedAt: new Date().toISOString(),
    };
}

function normalizeTargetColor(value) {
    const targetColor = typeof value === 'string' ? value.trim().toUpperCase() : '';
    return TARGET_COLORS.has(targetColor) ? targetColor : null;
}

function pushState(payload) {
    const telemetry = normalizeTelemetry(payload);

    state.latest = telemetry;
    state.sources[telemetry.source] = telemetry;
    state.updatedAt = telemetry.receivedAt;
    state.history.unshift(telemetry);
    state.history = state.history.slice(0, 50);

    const message = `data: ${JSON.stringify({ type: 'telemetry', payload: telemetry, state })}\n\n`;
    for (const client of sseClients) {
        client.write(message);
    }

    return telemetry;
}

function broadcast(message) {
    const serialized = `data: ${JSON.stringify(message)}\n\n`;
    for (const client of sseClients) {
        client.write(serialized);
    }
}

function appendHistory(entry) {
    state.history.unshift(entry);
    state.history = state.history.slice(0, 50);
}

function publishTargetColor(targetColor, source = 'dashboard') {
    const normalizedTargetColor = normalizeTargetColor(targetColor);
    if (!normalizedTargetColor) {
        return null;
    }

    state.targetColor = normalizedTargetColor;

    const event = {
        source,
        mode: 'control',
        command: `TARGET ${normalizedTargetColor}`,
        target_color: normalizedTargetColor,
        receivedAt: new Date().toISOString(),
    };

    appendHistory(event);
    broadcast({ type: 'target-updated', targetColor: normalizedTargetColor, state });

    const commandSocket = dgram.createSocket('udp4');
    const commandPayload = Buffer.from(JSON.stringify({
        type: 'set_target',
        target_color: normalizedTargetColor,
    }));

    commandSocket.send(commandPayload, TARGET_COMMAND_PORT, TARGET_COMMAND_HOST, (error) => {
        commandSocket.close();
        if (error) {
            console.error(`Failed to send target command: ${error.message}`);
        }
    });

    return normalizedTargetColor;
}

function sendJson(res, statusCode, data) {
    res.writeHead(statusCode, {
        'Content-Type': 'application/json; charset=utf-8',
        'Access-Control-Allow-Origin': '*',
    });
    res.end(JSON.stringify(data, null, 2));
}

function serveFile(res, filePath) {
    fs.readFile(filePath, (error, content) => {
        if (error) {
            res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
            res.end('Not found');
            return;
        }

        const ext = path.extname(filePath).toLowerCase();
        const contentType = {
            '.html': 'text/html; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
            '.json': 'application/json; charset=utf-8',
        }[ext] || 'application/octet-stream';

        res.writeHead(200, { 'Content-Type': contentType });
        res.end(content);
    });
}

function handleRequest(req, res) {
    const parsedUrl = url.parse(req.url, true);
    const pathname = parsedUrl.pathname || '/';

    if (pathname === '/api/health') {
        return sendJson(res, 200, {
            ok: true,
            udpPort: UDP_PORT,
            httpPort: HTTP_PORT,
            updatedAt: state.updatedAt,
        });
    }

    if (pathname === '/api/state') {
        return sendJson(res, 200, state);
    }

    if (pathname === '/api/history') {
        return sendJson(res, 200, { history: state.history });
    }

    if (pathname === '/api/target' && req.method === 'POST') {
        let body = '';
        req.on('data', (chunk) => {
            body += chunk;
        });
        req.on('end', () => {
            try {
                const payload = JSON.parse(body || '{}');
                const targetColor = normalizeTargetColor(payload.target_color || payload.targetColor);
                if (!targetColor) {
                    return sendJson(res, 400, { ok: false, error: 'Target color must be MERAH, HIJAU, or BIRU.' });
                }

                const updatedTarget = publishTargetColor(targetColor, 'dashboard');
                return sendJson(res, 200, {
                    ok: true,
                    targetColor: updatedTarget,
                    state,
                });
            } catch (error) {
                return sendJson(res, 400, { ok: false, error: 'Invalid JSON' });
            }
        });
        return;
    }

    if (pathname === '/api/telemetry' && req.method === 'POST') {
        let body = '';
        req.on('data', (chunk) => {
            body += chunk;
        });
        req.on('end', () => {
            try {
                const payload = JSON.parse(body || '{}');
                const telemetry = pushState(payload);
                return sendJson(res, 200, { ok: true, telemetry });
            } catch (error) {
                return sendJson(res, 400, { ok: false, error: 'Invalid JSON' });
            }
        });
        return;
    }

    if (pathname === '/events') {
        res.writeHead(200, {
            'Content-Type': 'text/event-stream; charset=utf-8',
            'Cache-Control': 'no-cache, no-transform',
            Connection: 'keep-alive',
            'Access-Control-Allow-Origin': '*',
        });

        res.write(`data: ${JSON.stringify({ type: 'snapshot', state })}\n\n`);
        sseClients.add(res);

        req.on('close', () => {
            sseClients.delete(res);
        });
        return;
    }

    const requestPath = pathname === '/' ? '/index.html' : pathname;
    const filePath = path.join(frontendDir, requestPath);

    if (!filePath.startsWith(frontendDir)) {
        res.writeHead(403, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('Forbidden');
        return;
    }

    serveFile(res, filePath);
}

const server = http.createServer(handleRequest);

const udpServer = dgram.createSocket('udp4');

udpServer.on('message', (message) => {
    try {
        const payload = JSON.parse(message.toString('utf8'));
        pushState(payload);
    } catch (error) {
        // Ignore invalid packets so other programs can keep sending.
    }
});

udpServer.bind(UDP_PORT, UDP_HOST, () => {
    console.log(`Telemetry UDP listening on udp://${UDP_HOST}:${UDP_PORT}`);
});

server.listen(HTTP_PORT, () => {
    console.log(`Dashboard running at http://localhost:${HTTP_PORT}`);
});