// server.js - Versión SIMPLE del multiplexor para testing inicial
const WebSocket = require('ws');
const http = require('http');
const express = require('express');
const { v4: uuidv4 } = require('uuid');

// Configuración básica
const PORT = 8080;
const ROSBRIDGE_URL = process.env.ROSBRIDGE_URL || 'ws://localhost:9090';

console.log('🚀 Iniciando Servidor Multiplexor Simple...');
console.log(`📡 Puerto: ${PORT}`);
console.log(`🤖 ROS Bridge: ${ROSBRIDGE_URL}`);

// Express app para API básica
const app = express();
app.use(express.json());

// CORS para desarrollo
app.use((req, res, next) => {
    res.header('Access-Control-Allow-Origin', '*');
    res.header('Access-Control-Allow-Headers', '*');
    next();
});

// Estado del servidor
const state = {
    clients: new Map(),
    rosbridgeWs: null,
    rosbridgeConnected: false
};

// Endpoint básico de status
app.get('/api/status', (req, res) => {
    res.json({
        ok: true,
        clients: state.clients.size,
        rosbridgeConnected: state.rosbridgeWs?.readyState === WebSocket.OPEN,
        timestamp: Date.now()
    });
});

// Crear servidor HTTP
const server = http.createServer(app);

// WebSocket Server para clientes
const wss = new WebSocket.Server({ server });

// Conectar a rosbridge
function connectToRosbridge() {
    console.log(`🔌 Conectando a rosbridge: ${ROSBRIDGE_URL}`);
    
    state.rosbridgeWs = new WebSocket(ROSBRIDGE_URL);

    state.rosbridgeWs.on('open', () => {
        console.log('✅ Conectado a rosbridge');
        state.rosbridgeConnected = true;
        
        // Suscribirse a topics básicos del simulador
        const subscriptions = [
            { op: 'subscribe', topic: '/robot_state', type: 'std_msgs/String' },
            { op: 'subscribe', topic: '/joint_states', type: 'sensor_msgs/JointState' }
        ];
        
        subscriptions.forEach(sub => {
            state.rosbridgeWs.send(JSON.stringify(sub));
            console.log(`📡 Suscrito a ${sub.topic}`);
        });
    });

    state.rosbridgeWs.on('message', (data) => {
        // Reenviar TODOS los mensajes de ROS a TODOS los clientes
        const message = {
            type: 'ros_data',
            data: JSON.parse(data)
        };
        
        broadcastToAll(message);
    });

    state.rosbridgeWs.on('close', () => {
        console.log('⚠️  Desconectado de rosbridge. Reintentando en 3s...');
        state.rosbridgeConnected = false;
        setTimeout(connectToRosbridge, 3000);
    });

    state.rosbridgeWs.on('error', (error) => {
        console.error('❌ Error en rosbridge:', error.message);
    });
}

// Manejar conexiones de clientes
wss.on('connection', (ws, req) => {
    const clientId = uuidv4();
    const clientIp = req.socket.remoteAddress;
    
    console.log(`👤 Nuevo cliente: ${clientId} desde ${clientIp}`);
    
    // Guardar cliente
    state.clients.set(clientId, {
        id: clientId,
        ws: ws,
        ip: clientIp,
        connectedAt: new Date()
    });
    
    // Enviar mensaje de bienvenida
    ws.send(JSON.stringify({
        type: 'welcome',
        clientId: clientId,
        rosbridgeConnected: state.rosbridgeConnected,
        timestamp: Date.now()
    }));
    
    // Manejar mensajes del cliente
    ws.on('message', (data) => {
        try {
            const message = JSON.parse(data);
            console.log(`📨 Cliente ${clientId} envió:`, message.type || 'unknown');
            
            // Por ahora, simplemente reenviar a rosbridge
            if (state.rosbridgeWs && state.rosbridgeWs.readyState === WebSocket.OPEN) {
                state.rosbridgeWs.send(JSON.stringify(message));
            } else {
                ws.send(JSON.stringify({
                    type: 'error',
                    message: 'ROS bridge no conectado'
                }));
            }
        } catch (error) {
            console.error(`❌ Error procesando mensaje de ${clientId}:`, error);
        }
    });
    
    // Manejar desconexión
    ws.on('close', () => {
        console.log(`👋 Cliente desconectado: ${clientId}`);
        state.clients.delete(clientId);
    });
    
    ws.on('error', (error) => {
        console.error(`❌ Error en cliente ${clientId}:`, error.message);
    });
});

// Broadcast a todos los clientes
function broadcastToAll(message) {
    const data = JSON.stringify(message);
    state.clients.forEach((client) => {
        if (client.ws.readyState === WebSocket.OPEN) {
            client.ws.send(data);
        }
    });
}

// Iniciar servidor
server.listen(PORT, () => {
    console.log(`\n${'='.repeat(50)}`);
    console.log('✅ Servidor Multiplexor Simple está ACTIVO');
    console.log(`${'='.repeat(50)}`);
    console.log(`📍 API HTTP: http://localhost:${PORT}/api/status`);
    console.log(`📍 WebSocket: ws://localhost:${PORT}`);
    console.log(`${'='.repeat(50)}\n`);
    
    // Conectar a rosbridge
    connectToRosbridge();
});

// Manejo de cierre limpio
process.on('SIGINT', () => {
    console.log('\n🛑 Cerrando servidor...');
    
    // Cerrar todas las conexiones
    state.clients.forEach(client => client.ws.close());
    if (state.rosbridgeWs) state.rosbridgeWs.close();
    
    process.exit(0);
});
