// IoT Dashboard - Orange LTE-M Guadeloupe
// Configuration MQTT
const BROKER_URL = 'wss://test.mosquitto.org:8081/mqtt';
const TOPICS = {
    temperature: 'dev/data/guadeloupe/temperature',
    pression: 'dev/data/guadeloupe/pression',
    humidite: 'dev/data/guadeloupe/humidite',
    position: 'dev/data/guadeloupe/position'
};

// State
let client = null;
let isConnected = false;
const dataHistory = {
    temperature: [],
    pression: [],
    humidite: [],
    position: []
};
const MAX_HISTORY = 20;

// Orientation state - pour la visualisation 3D
let deviceOrientation = { x: 0, y: 0, z: 0 }; // Angles en degrés (pitch, roll, yaw)
let lastUpdateTime = Date.now();
const GYRO_TO_ANGLE_FACTOR = 0.001; // Conversion mdps vers deg/ms (pour ancien format)

// DOM Elements
const statusBadge = document.getElementById('statusBadge');
const statusText = document.getElementById('statusText');
const connectBtn = document.getElementById('connectBtn');
const logContainer = document.getElementById('logContainer');
const deviceContainer = document.getElementById('deviceContainer');

// Initialize mini charts
function initCharts() {
    ['Temperature', 'Pressure', 'Humidity'].forEach(type => {
        const chart = document.getElementById(`chart${type}`);
        if (chart) {
            for (let i = 0; i < MAX_HISTORY; i++) {
                const bar = document.createElement('div');
                bar.className = 'chart-bar';
                bar.style.height = '10%';
                chart.appendChild(bar);
            }
        }
    });
}

// Update mini chart
function updateChart(type, values) {
    const chartId = type === 'pression' ? 'chartPressure' : 
                   type === 'humidite' ? 'chartHumidity' : 
                   type === 'temperature' ? 'chartTemperature' : null;
    
    if (!chartId) return;
    
    const chart = document.getElementById(chartId);
    if (!chart) return;
    
    const bars = chart.querySelectorAll('.chart-bar');
    
    if (values.length === 0) return;

    const min = Math.min(...values);
    const max = Math.max(...values);
    const range = max - min || 1;

    values.forEach((val, i) => {
        if (bars[i]) {
            const height = ((val - min) / range * 80) + 20;
            bars[i].style.height = `${height}%`;
        }
    });
}

// Add log entry
function addLog(message, type = '') {
    const time = new Date().toLocaleTimeString('fr-FR');
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    entry.innerHTML = `
        <span class="log-time">${time}</span>
        <span class="log-message ${type}">${message}</span>
    `;
    logContainer.insertBefore(entry, logContainer.firstChild);
    
    // Keep only last 50 entries
    while (logContainer.children.length > 50) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

// Update connection status
function setStatus(status) {
    statusBadge.className = `status-badge ${status}`;
    switch(status) {
        case 'connected':
            statusText.textContent = 'Connecté';
            connectBtn.textContent = 'Se déconnecter';
            connectBtn.classList.add('disconnect');
            connectBtn.disabled = false;
            break;
        case 'connecting':
            statusText.textContent = 'Connexion...';
            connectBtn.disabled = true;
            break;
        case 'error':
            statusText.textContent = 'Erreur';
            connectBtn.disabled = false;
            connectBtn.classList.remove('disconnect');
            break;
        default:
            statusText.textContent = 'Déconnecté';
            connectBtn.textContent = 'Se connecter au broker';
            connectBtn.disabled = false;
            connectBtn.classList.remove('disconnect');
    }
}

// Update topic subscription status
function setTopicSubscribed(topic, subscribed) {
    const topicMap = {
        [TOPICS.temperature]: 'topicTemp',
        [TOPICS.pression]: 'topicPressure',
        [TOPICS.humidite]: 'topicHumidity',
        [TOPICS.position]: 'topicPosition'
    };
    const el = document.getElementById(topicMap[topic]);
    if (el) {
        el.classList.toggle('subscribed', subscribed);
    }
}

// Update data display
function updateValue(type, value) {
    const idMap = {
        temperature: { value: 'valueTemperature', time: 'timeTemperature', card: 'cardTemperature' },
        pression: { value: 'valuePressure', time: 'timePressure', card: 'cardPressure' },
        humidite: { value: 'valueHumidity', time: 'timeHumidity', card: 'cardHumidity' },
        position: { time: 'timePosition', card: 'cardPosition' }
    };
    
    const ids = idMap[type];
    if (!ids) return;

    // Handle position data (gyroscope + accelerometer + orientation)
    if (type === 'position') {
        updatePositionValue(value, ids);
        return;
    }

    const numValue = parseFloat(value);
    if (isNaN(numValue)) return;

    // Update value - format selon le type
    const valueEl = document.getElementById(ids.value);
    if (type === 'pression') {
        // Pression en Pa, afficher en entier
        valueEl.textContent = Math.round(numValue);
    } else {
        valueEl.textContent = numValue.toFixed(1);
    }

    // Update timestamp
    const timeEl = document.getElementById(ids.time);
    timeEl.textContent = new Date().toLocaleTimeString('fr-FR');

    // Flash animation
    const card = document.getElementById(ids.card);
    card.classList.add('updated');
    setTimeout(() => card.classList.remove('updated'), 600);

    // Update history
    dataHistory[type].push(numValue);
    if (dataHistory[type].length > MAX_HISTORY) {
        dataHistory[type].shift();
    }
    updateChart(type, dataHistory[type]);
}

// Update position (orientation 3D absolue - nécessite gyroscope + accéléromètre)
function updatePositionValue(value, ids) {
    const statusMessage = document.getElementById('statusMessage3D');
    
    // NOUVEAU FORMAT REQUIS: {gyro: {x,y,z}, accel: {x,y,z}, orientation: {pitch,roll,yaw}}
    if (typeof value === 'object' && value.gyro && value.accel && value.orientation) {
        // ✅ FORMAT CORRECT - Afficher l'orientation absolue
        const pitch = value.orientation.pitch || 0;
        const roll = value.orientation.roll || 0;
        const yaw = value.orientation.yaw || 0;
        
        // Mettre à jour l'orientation 3D
        deviceOrientation.x = pitch;
        deviceOrientation.y = roll;
        deviceOrientation.z = yaw;
        
        applyDeviceOrientation();
        
        // Cacher le message de statut (la visualisation 3D est active)
        if (statusMessage) {
            statusMessage.classList.add('hidden');
        }
        
        addLog(`✅ Position 3D: Pitch=${pitch.toFixed(1)}°, Roll=${roll.toFixed(1)}°, Yaw=${yaw.toFixed(1)}°`, 'success');
    } 
    // MODE COMPATIBILITÉ : Gyroscope seul - Visualisation proportionnelle
    else if (typeof value === 'object' && (value.x !== undefined || value.X !== undefined)) {
        // ⚠️ Format gyroscope seul détecté - MODE COMPATIBLE
        const gyroX = value.x || value.X || 0;
        const gyroY = value.y || value.Y || 0;
        const gyroZ = value.z || value.Z || 0;
        
        // Afficher une visualisation proportionnelle (pas l'orientation réelle)
        const DISPLAY_SCALE = 0.08; // Facteur d'échelle
        deviceOrientation.x = gyroX * DISPLAY_SCALE;
        deviceOrientation.y = gyroY * DISPLAY_SCALE;
        deviceOrientation.z = gyroZ * DISPLAY_SCALE;
        
        // Limiter pour éviter rotations extrêmes
        deviceOrientation.x = Math.max(-90, Math.min(90, deviceOrientation.x));
        deviceOrientation.y = Math.max(-90, Math.min(90, deviceOrientation.y));
        deviceOrientation.z = Math.max(-90, Math.min(90, deviceOrientation.z));
        
        applyDeviceOrientation();
        
        // Message d'avertissement mais fonctionnel
        if (statusMessage) {
            statusMessage.className = 'status-message error';
            statusMessage.innerHTML = `
                <span class="status-icon">⚠️</span>
                <span class="status-text">
                    <strong>Mode compatibilité</strong><br>
                    Gyroscope seul: ${Math.round(gyroX)}, ${Math.round(gyroY)}, ${Math.round(gyroZ)} mdps<br>
                    Visualisation proportionnelle active<br><br>
                    <small>Pour l'orientation 3D réelle, implémentez le code Arduino complet</small>
                </span>
            `;
        }
        
        addLog(`⚠️ Gyro seul (${Math.round(gyroX)}, ${Math.round(gyroY)}, ${Math.round(gyroZ)} mdps) - Mode proportionnel`, 'data');
    }
    // Format string - essayer de parser
    else if (typeof value === 'string') {
        try {
            const parsed = JSON.parse(value);
            updatePositionValue(parsed, ids);
            return;
        } catch (e) {
            // Format invalide
            if (statusMessage) {
                statusMessage.className = 'status-message error';
                statusMessage.innerHTML = `
                    <span class="status-icon">❌</span>
                    <span class="status-text">
                        <strong>Format de données invalide</strong><br>
                        Consultez GUIDE_IMPLEMENTATION.md
                    </span>
                `;
            }
            addLog(`❌ Format invalide: ${value}`, 'error');
        }
    }
    // Format complètement invalide
    else {
        if (statusMessage) {
            statusMessage.className = 'status-message';
            statusMessage.innerHTML = `
                <span class="status-icon">⏳</span>
                <span class="status-text">
                    En attente des données d'orientation 3D...<br><br>
                    Format attendu: {gyro, accel, orientation}
                </span>
            `;
        }
        return;
    }

    // Update timestamp
    const timeEl = document.getElementById(ids.time);
    timeEl.textContent = new Date().toLocaleTimeString('fr-FR');

    // Flash animation
    const card = document.getElementById(ids.card);
    card.classList.add('updated');
    setTimeout(() => card.classList.remove('updated'), 600);
}

// Apply device orientation directly (for accelerometer-based orientation data)
function applyDeviceOrientation() {
    if (deviceContainer) {
        deviceContainer.style.transform = `
            translate(-50%, -50%)
            rotateX(${deviceOrientation.x}deg)
            rotateY(${deviceOrientation.y}deg)
            rotateZ(${deviceOrientation.z}deg)
        `;
    }
    updateAngleDisplay();
}

// Update device orientation from gyroscope data (integration mode - for old format)
// [OBSOLÈTE] Cette fonction n'est plus utilisée
// Le dashboard nécessite maintenant le format complet avec orientation calculée
function updateDeviceOrientationFromGyro(gyroX, gyroY, gyroZ) {
    // NOUVELLE APPROCHE : Affichage direct proportionnel aux valeurs du gyroscope
    // Au lieu d'intégrer, on affiche l'inclinaison proportionnelle à la vitesse
    
    // Facteur d'échelle pour visualisation (ajustable)
    const DISPLAY_SCALE = 0.1; // 100 mdps = 10° de rotation visuelle
    
    // Appliquer directement les valeurs (pas d'intégration dans le temps)
    deviceOrientation.x = gyroX * DISPLAY_SCALE;
    deviceOrientation.y = gyroY * DISPLAY_SCALE;
    deviceOrientation.z = gyroZ * DISPLAY_SCALE;
    
    // Limiter les angles pour éviter des rotations extrêmes
    deviceOrientation.x = Math.max(-90, Math.min(90, deviceOrientation.x));
    deviceOrientation.y = Math.max(-90, Math.min(90, deviceOrientation.y));
    deviceOrientation.z = Math.max(-90, Math.min(90, deviceOrientation.z));
    
    applyDeviceOrientation();
}

// Normalize angle to -180 to 180 range
function normalizeAngle(angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Update angle display
function updateAngleDisplay() {
    const angleX = document.getElementById('angleX');
    const angleY = document.getElementById('angleY');
    const angleZ = document.getElementById('angleZ');
    
    if (angleX) angleX.textContent = Math.round(deviceOrientation.x);
    if (angleY) angleY.textContent = Math.round(deviceOrientation.y);
    if (angleZ) angleZ.textContent = Math.round(deviceOrientation.z);
}

// Reset device orientation
function resetOrientation() {
    deviceOrientation = { x: 0, y: 0, z: 0 };
    lastUpdateTime = Date.now();
    applyDeviceOrientation();
    addLog('Orientation réinitialisée');
}

// Connect to MQTT broker
function connect() {
    setStatus('connecting');
    addLog('Connexion au broker MQTT...');

    const options = {
        keepalive: 60,
        clientId: 'webClient_' + Math.random().toString(16).substr(2, 8),
        clean: true,
        reconnectPeriod: 5000,
        connectTimeout: 30 * 1000
    };

    client = mqtt.connect(BROKER_URL, options);

    client.on('connect', () => {
        isConnected = true;
        setStatus('connected');
        addLog('Connecté au broker Mosquitto', 'success');

        // Subscribe to topics
        Object.entries(TOPICS).forEach(([name, topic]) => {
            client.subscribe(topic, { qos: 0 }, (err) => {
                if (err) {
                    addLog(`Erreur subscription ${name}: ${err.message}`, 'error');
                } else {
                    addLog(`Souscrit à ${topic}`, 'success');
                    setTopicSubscribed(topic, true);
                }
            });
        });
    });

    client.on('message', (topic, message) => {
        const rawMessage = message.toString();
        const type = Object.entries(TOPICS).find(([_, t]) => t === topic)?.[0];
        
        if (type) {
            try {
                // Parse le format JSON du capteur iotContinuum
                // Format standard: {"s":"urn:lo:nsid:SampleData","v":25.5,"tags":["temperature"]}
                // Format position: {"s":"...","v":{"gyro":{...},"accel":{...},"orientation":{...}},"tags":["position"]}
                const data = JSON.parse(rawMessage);
                const value = data.v;
                
                if (type === 'position') {
                    addLog(`Position reçue (format ${value.orientation ? 'nouveau' : 'ancien'})`, 'data');
                } else {
                    addLog(`${type}: ${value}`, 'data');
                }
                updateValue(type, value);
            } catch (e) {
                // Si ce n'est pas du JSON, essayer comme valeur brute
                addLog(`${type}: ${rawMessage}`, 'data');
                updateValue(type, rawMessage);
            }
        }
    });

    client.on('error', (err) => {
        addLog(`Erreur: ${err.message}`, 'error');
        setStatus('error');
    });

    client.on('close', () => {
        if (isConnected) {
            addLog('Connexion fermée');
            isConnected = false;
            setStatus('');
            Object.values(TOPICS).forEach(topic => setTopicSubscribed(topic, false));
        }
    });

    client.on('offline', () => {
        addLog('Client hors ligne', 'error');
    });

    client.on('reconnect', () => {
        addLog('Tentative de reconnexion...');
        setStatus('connecting');
    });
}

// Disconnect from MQTT broker
function disconnect() {
    if (client) {
        client.end(true);
        client = null;
        isConnected = false;
        setStatus('');
        addLog('Déconnexion du broker');
        Object.values(TOPICS).forEach(topic => setTopicSubscribed(topic, false));
    }
}

// Toggle connection
function toggleConnection() {
    if (isConnected) {
        disconnect();
    } else {
        connect();
    }
}

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', () => {
    initCharts();
    addLog('Interface prête. Cliquez pour vous connecter.');
    
    // Initialiser la scène 3D
    setTimeout(() => {
        initialize3DVisualization();
        addLog('Visualisation 3D initialisée', 'success');
    }, 100);
});
// ===== VISUALISATION 3D AVEC THREE.JS =====
// À ajouter dans iot-dashboard.js

// Variables Three.js
let scene, camera, renderer, gyroscopeDevice;
let axisHelpers = { x: null, y: null, z: null };
let grid;

// État de la caméra
let cameraAngle = { theta: 0.5, phi: 0.3 };
let isDragging = false;
let previousMousePosition = { x: 0, y: 0 };

// Initialiser la scène 3D
function init3DScene() {
    const container = document.getElementById('canvasContainer');
    if (!container) {
        console.error('Container canvas non trouvé');
        return;
    }

    // Créer la scène
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0A0A0F);
    scene.fog = new THREE.Fog(0x0A0A0F, 10, 50);

    // Créer la caméra
    const width = container.clientWidth;
    const height = container.clientHeight;
    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    camera.position.set(4, 3, 5);
    camera.lookAt(0, 0, 0);

    // Créer le renderer
    renderer = new THREE.WebGLRenderer({ 
        antialias: true,
        alpha: true
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    
    // Créer le canvas avec un ID
    renderer.domElement.id = 'threejsCanvas';
    container.appendChild(renderer.domElement);

    // Ajouter les lumières
    addLights();

    // Créer la grille de référence
    createReferenceGrid();

    // Créer les axes de référence
    createAxisHelpers();

    // Créer le gyroscope (smartphone)
    gyroscopeDevice = createGyroscopeDevice();
    scene.add(gyroscopeDevice);

    // Gérer les interactions souris
    setupMouseControls(renderer.domElement);

    // Gérer le redimensionnement
    window.addEventListener('resize', onWindowResize);

    // Démarrer l'animation
    animate3DScene();
}

// Ajouter les lumières
function addLights() {
    // Lumière ambiante
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);

    // Lumière directionnelle principale
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 5, 5);
    directionalLight.castShadow = true;
    directionalLight.shadow.camera.near = 0.1;
    directionalLight.shadow.camera.far = 50;
    directionalLight.shadow.camera.left = -10;
    directionalLight.shadow.camera.right = 10;
    directionalLight.shadow.camera.top = 10;
    directionalLight.shadow.camera.bottom = -10;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    scene.add(directionalLight);

    // Lumière Orange (signature)
    const orangeLight = new THREE.PointLight(0xFF7900, 0.5, 20);
    orangeLight.position.set(-3, 2, 3);
    scene.add(orangeLight);

    // Lumière de remplissage
    const fillLight = new THREE.DirectionalLight(0x4ECDC4, 0.3);
    fillLight.position.set(-5, 0, -5);
    scene.add(fillLight);
}

// Créer la grille de référence
function createReferenceGrid() {
    // Grille au sol
    const gridHelper = new THREE.GridHelper(10, 20, 0xFF7900, 0x333344);
    gridHelper.material.opacity = 0.2;
    gridHelper.material.transparent = true;
    scene.add(gridHelper);

    // Plan de référence transparent
    const planeGeometry = new THREE.PlaneGeometry(10, 10);
    const planeMaterial = new THREE.MeshStandardMaterial({
        color: 0x1a1a2e,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.1
    });
    const plane = new THREE.Mesh(planeGeometry, planeMaterial);
    plane.rotation.x = Math.PI / 2;
    plane.position.y = -0.05;
    plane.receiveShadow = true;
    scene.add(plane);
}

// Créer les axes de référence
function createAxisHelpers() {
    const axisLength = 2.5;
    const axisWidth = 0.02;

    // Axe X (Rouge)
    const xGeometry = new THREE.CylinderGeometry(axisWidth, axisWidth, axisLength, 8);
    const xMaterial = new THREE.MeshBasicMaterial({ color: 0xFF6B6B });
    const xAxis = new THREE.Mesh(xGeometry, xMaterial);
    xAxis.rotation.z = Math.PI / 2;
    scene.add(xAxis);

    // Flèche X
    const xConeGeometry = new THREE.ConeGeometry(axisWidth * 3, axisWidth * 10, 8);
    const xCone = new THREE.Mesh(xConeGeometry, xMaterial);
    xCone.position.x = axisLength / 2 + 0.1;
    xCone.rotation.z = -Math.PI / 2;
    scene.add(xCone);

    // Label X
    addAxisLabel('X', axisLength / 2 + 0.3, 0, 0, 0xFF6B6B);

    // Axe Y (Vert)
    const yGeometry = new THREE.CylinderGeometry(axisWidth, axisWidth, axisLength, 8);
    const yMaterial = new THREE.MeshBasicMaterial({ color: 0x4ECDC4 });
    const yAxis = new THREE.Mesh(yGeometry, yMaterial);
    scene.add(yAxis);

    // Flèche Y
    const yConeGeometry = new THREE.ConeGeometry(axisWidth * 3, axisWidth * 10, 8);
    const yCone = new THREE.Mesh(yConeGeometry, yMaterial);
    yCone.position.y = axisLength / 2 + 0.1;
    scene.add(yCone);

    // Label Y
    addAxisLabel('Y', 0, axisLength / 2 + 0.3, 0, 0x4ECDC4);

    // Axe Z (Bleu)
    const zGeometry = new THREE.CylinderGeometry(axisWidth, axisWidth, axisLength, 8);
    const zMaterial = new THREE.MeshBasicMaterial({ color: 0xA855F7 });
    const zAxis = new THREE.Mesh(zGeometry, zMaterial);
    zAxis.rotation.x = Math.PI / 2;
    scene.add(zAxis);

    // Flèche Z
    const zConeGeometry = new THREE.ConeGeometry(axisWidth * 3, axisWidth * 10, 8);
    const zCone = new THREE.Mesh(zConeGeometry, zMaterial);
    zCone.position.z = axisLength / 2 + 0.1;
    zCone.rotation.x = Math.PI / 2;
    scene.add(zCone);

    // Label Z
    addAxisLabel('Z', 0, 0, axisLength / 2 + 0.3, 0xA855F7);
}

// Ajouter un label d'axe (simplifié - utilise des sphères)
function addAxisLabel(text, x, y, z, color) {
    const sphereGeometry = new THREE.SphereGeometry(0.05, 16, 16);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: color });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    sphere.position.set(x, y, z);
    scene.add(sphere);
}

// Créer le gyroscope (smartphone)
function createGyroscopeDevice() {
    const group = new THREE.Group();

    // Corps principal (forme de smartphone)
    const bodyGeometry = new THREE.BoxGeometry(0.8, 1.6, 0.1);
    const bodyMaterial = new THREE.MeshPhongMaterial({
        color: 0x1a1a2e,
        shininess: 100,
        specular: 0xFF7900,
        emissive: 0x0a0a0f
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.castShadow = true;
    body.receiveShadow = true;
    group.add(body);

    // Écran
    const screenGeometry = new THREE.BoxGeometry(0.7, 1.45, 0.11);
    const screenMaterial = new THREE.MeshPhongMaterial({
        color: 0xFF7900,
        emissive: 0xFF7900,
        emissiveIntensity: 0.3,
        shininess: 80,
        opacity: 0.9,
        transparent: true
    });
    const screen = new THREE.Mesh(screenGeometry, screenMaterial);
    screen.position.z = 0.05;
    group.add(screen);

    // Bordure Orange
    const borderGeometry = new THREE.BoxGeometry(0.82, 1.62, 0.12);
    const borderMaterial = new THREE.MeshPhongMaterial({
        color: 0xFF7900,
        emissive: 0xFF7900,
        emissiveIntensity: 0.5,
        transparent: true,
        opacity: 0.3,
        side: THREE.BackSide
    });
    const border = new THREE.Mesh(borderGeometry, borderMaterial);
    group.add(border);

    // Encoche (notch) en haut
    const notchGeometry = new THREE.BoxGeometry(0.3, 0.05, 0.12);
    const notchMaterial = new THREE.MeshPhongMaterial({ color: 0x0a0a0f });
    const notch = new THREE.Mesh(notchGeometry, notchMaterial);
    notch.position.set(0, 0.75, 0.05);
    group.add(notch);

    // Bouton home en bas
    const buttonGeometry = new THREE.CylinderGeometry(0.08, 0.08, 0.02, 16);
    const buttonMaterial = new THREE.MeshPhongMaterial({
        color: 0xFF7900,
        emissive: 0xFF7900,
        emissiveIntensity: 0.3
    });
    const button = new THREE.Mesh(buttonGeometry, buttonMaterial);
    button.rotation.x = Math.PI / 2;
    button.position.set(0, -0.7, 0.06);
    group.add(button);

    // Indicateur de direction (flèche sur le dessus)
    const arrowGeometry = new THREE.ConeGeometry(0.1, 0.2, 3);
    const arrowMaterial = new THREE.MeshPhongMaterial({
        color: 0x4ECDC4,
        emissive: 0x4ECDC4,
        emissiveIntensity: 0.5
    });
    const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
    arrow.position.set(0, 0.9, 0);
    arrow.rotation.x = Math.PI;
    group.add(arrow);

    return group;
}

// Configuration des contrôles souris
function setupMouseControls(canvas) {
    canvas.addEventListener('mousedown', (e) => {
        isDragging = true;
        previousMousePosition = { x: e.clientX, y: e.clientY };
    });

    canvas.addEventListener('mousemove', (e) => {
        if (!isDragging) return;

        const deltaX = e.clientX - previousMousePosition.x;
        const deltaY = e.clientY - previousMousePosition.y;

        cameraAngle.theta += deltaX * 0.01;
        cameraAngle.phi += deltaY * 0.01;

        // Limiter phi pour éviter le retournement
        cameraAngle.phi = Math.max(-Math.PI / 2 + 0.1, Math.min(Math.PI / 2 - 0.1, cameraAngle.phi));

        updateCameraPosition();

        previousMousePosition = { x: e.clientX, y: e.clientY };
    });

    canvas.addEventListener('mouseup', () => {
        isDragging = false;
    });

    canvas.addEventListener('mouseleave', () => {
        isDragging = false;
    });

    // Support tactile
    canvas.addEventListener('touchstart', (e) => {
        if (e.touches.length === 1) {
            isDragging = true;
            previousMousePosition = { 
                x: e.touches[0].clientX, 
                y: e.touches[0].clientY 
            };
        }
    });

    canvas.addEventListener('touchmove', (e) => {
        if (!isDragging || e.touches.length !== 1) return;
        e.preventDefault();

        const deltaX = e.touches[0].clientX - previousMousePosition.x;
        const deltaY = e.touches[0].clientY - previousMousePosition.y;

        cameraAngle.theta += deltaX * 0.01;
        cameraAngle.phi += deltaY * 0.01;

        cameraAngle.phi = Math.max(-Math.PI / 2 + 0.1, Math.min(Math.PI / 2 - 0.1, cameraAngle.phi));

        updateCameraPosition();

        previousMousePosition = { 
            x: e.touches[0].clientX, 
            y: e.touches[0].clientY 
        };
    });

    canvas.addEventListener('touchend', () => {
        isDragging = false;
    });
}

// Mettre à jour la position de la caméra
function updateCameraPosition() {
    const radius = 6;
    camera.position.x = radius * Math.sin(cameraAngle.theta) * Math.cos(cameraAngle.phi);
    camera.position.y = radius * Math.sin(cameraAngle.phi);
    camera.position.z = radius * Math.cos(cameraAngle.theta) * Math.cos(cameraAngle.phi);
    camera.lookAt(0, 0, 0);
}

// Gérer le redimensionnement
function onWindowResize() {
    const container = document.getElementById('canvasContainer');
    if (!container) return;

    const width = container.clientWidth;
    const height = container.clientHeight;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

// Boucle d'animation
function animate3DScene() {
    requestAnimationFrame(animate3DScene);
    
    // Rotation du gyroscope selon deviceOrientation
    if (gyroscopeDevice && deviceOrientation) {
        // Convertir degrés en radians et appliquer
        gyroscopeDevice.rotation.x = THREE.MathUtils.degToRad(deviceOrientation.x);
        gyroscopeDevice.rotation.y = THREE.MathUtils.degToRad(deviceOrientation.y);
        gyroscopeDevice.rotation.z = THREE.MathUtils.degToRad(deviceOrientation.z);
    }

    renderer.render(scene, camera);
}

// Fonction à appeler dans l'initialisation du dashboard
function initialize3DVisualization() {
    init3DScene();
}
