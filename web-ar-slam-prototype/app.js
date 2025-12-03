/**
 * Web AR SLAM Application
 * Main application controller with Three.js rendering
 */

// ============================================================================
// GLOBAL STATE
// ============================================================================

let slamEngine = null;
let threeScene = null;
let isARActive = false;
let cvReady = false;
let placedObjects = [];

// ============================================================================
// OPENCV READY CALLBACK
// ============================================================================

function onOpenCvReady() {
    console.log('OpenCV.js loaded');
    cvReady = true;
    updateLoadingStatus('OpenCV ready');
    
    // If SLAM engine already exists, initialize visual SLAM
    if (slamEngine && slamEngine.videoElement) {
        slamEngine.initializeSLAM();
    }
    
    checkInitializationComplete();
}

// ============================================================================
// THREE.JS SCENE
// ============================================================================

class ARScene {
    constructor(canvas) {
        this.canvas = canvas;
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ 
            canvas: canvas, 
            alpha: true,
            antialias: true 
        });
        
        this.objects = [];
        this.gridHelper = null;
        
        this.init();
    }
    
    init() {
        // Setup renderer
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.setClearColor(0x000000, 0);
        
        // Setup camera
        this.camera.position.set(0, 1.6, 0);  // Approximate eye height
        
        // Add lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 5);
        directionalLight.castShadow = true;
        this.scene.add(directionalLight);
        
        // Add grid helper (ground plane visualization)
        this.gridHelper = new THREE.GridHelper(20, 20, 0x00f5d4, 0x1a1a2e);
        this.gridHelper.material.opacity = 0.3;
        this.gridHelper.material.transparent = true;
        this.scene.add(this.gridHelper);
        
        // Add origin marker
        this.addOriginMarker();
        
        // Handle resize
        window.addEventListener('resize', () => this.onResize());
    }
    
    addOriginMarker() {
        // Create axis helper at origin
        const axisHelper = new THREE.AxesHelper(1);
        this.scene.add(axisHelper);
        
        // Origin sphere
        const geometry = new THREE.SphereGeometry(0.1, 16, 16);
        const material = new THREE.MeshStandardMaterial({ 
            color: 0x00f5d4,
            emissive: 0x00f5d4,
            emissiveIntensity: 0.3
        });
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.set(0, 0.1, 0);
        this.scene.add(sphere);
    }
    
    /**
     * Place an AR object at the current camera position
     */
    placeObject(position, type = 'cube') {
        let object;
        
        switch (type) {
            case 'cube':
                const cubeGeo = new THREE.BoxGeometry(0.3, 0.3, 0.3);
                const cubeMat = new THREE.MeshStandardMaterial({
                    color: 0xf72585,
                    metalness: 0.3,
                    roughness: 0.4
                });
                object = new THREE.Mesh(cubeGeo, cubeMat);
                break;
                
            case 'sphere':
                const sphereGeo = new THREE.SphereGeometry(0.2, 32, 32);
                const sphereMat = new THREE.MeshStandardMaterial({
                    color: 0x4361ee,
                    metalness: 0.5,
                    roughness: 0.2
                });
                object = new THREE.Mesh(sphereGeo, sphereMat);
                break;
                
            case 'cylinder':
                const cylGeo = new THREE.CylinderGeometry(0.15, 0.15, 0.4, 32);
                const cylMat = new THREE.MeshStandardMaterial({
                    color: 0xfee440,
                    metalness: 0.3,
                    roughness: 0.5
                });
                object = new THREE.Mesh(cylGeo, cylMat);
                break;
                
            case 'house':
                object = this.createHouseModel();
                break;
                
            default:
                const defaultGeo = new THREE.BoxGeometry(0.3, 0.3, 0.3);
                const defaultMat = new THREE.MeshStandardMaterial({ color: 0x00f5d4 });
                object = new THREE.Mesh(defaultGeo, defaultMat);
        }
        
        // Position the object
        object.position.copy(position);
        object.userData.worldPosition = position.clone();
        object.userData.placedAt = Date.now();
        
        // Add shadow
        object.castShadow = true;
        object.receiveShadow = true;
        
        this.scene.add(object);
        this.objects.push(object);
        
        return object;
    }
    
    /**
     * Create a simple house model
     */
    createHouseModel() {
        const group = new THREE.Group();
        
        // House body
        const bodyGeo = new THREE.BoxGeometry(1, 0.8, 1);
        const bodyMat = new THREE.MeshStandardMaterial({ 
            color: 0xf5f5f5,
            roughness: 0.8 
        });
        const body = new THREE.Mesh(bodyGeo, bodyMat);
        body.position.y = 0.4;
        group.add(body);
        
        // Roof
        const roofGeo = new THREE.ConeGeometry(0.85, 0.5, 4);
        const roofMat = new THREE.MeshStandardMaterial({ 
            color: 0xb5651d,
            roughness: 0.9 
        });
        const roof = new THREE.Mesh(roofGeo, roofMat);
        roof.position.y = 1.05;
        roof.rotation.y = Math.PI / 4;
        group.add(roof);
        
        // Door
        const doorGeo = new THREE.BoxGeometry(0.2, 0.4, 0.05);
        const doorMat = new THREE.MeshStandardMaterial({ color: 0x4a3728 });
        const door = new THREE.Mesh(doorGeo, doorMat);
        door.position.set(0, 0.2, 0.52);
        group.add(door);
        
        // Windows
        const windowGeo = new THREE.BoxGeometry(0.15, 0.15, 0.05);
        const windowMat = new THREE.MeshStandardMaterial({ 
            color: 0x87ceeb,
            emissive: 0x87ceeb,
            emissiveIntensity: 0.2
        });
        
        const window1 = new THREE.Mesh(windowGeo, windowMat);
        window1.position.set(-0.25, 0.5, 0.52);
        group.add(window1);
        
        const window2 = new THREE.Mesh(windowGeo, windowMat);
        window2.position.set(0.25, 0.5, 0.52);
        group.add(window2);
        
        // Scale down
        group.scale.set(0.5, 0.5, 0.5);
        
        return group;
    }
    
    /**
     * Update camera pose from SLAM
     */
    updateCameraPose(position, orientation) {
        // Update camera position (inverted - we move the scene, not the camera)
        // This keeps objects stable while camera view changes
        
        // Apply orientation (Euler angles to quaternion)
        const euler = new THREE.Euler(
            orientation.x,
            orientation.z,  // Yaw as Y rotation
            orientation.y,
            'YXZ'
        );
        this.camera.quaternion.setFromEuler(euler);
        
        // For position, we offset objects rather than moving camera
        // This prevents floating point precision issues at large distances
    }
    
    /**
     * Update placed objects based on world anchor changes
     */
    updateObjectPositions(anchorOffset) {
        for (const obj of this.objects) {
            if (obj.userData.worldPosition) {
                obj.position.x = obj.userData.worldPosition.x - anchorOffset.x;
                obj.position.z = obj.userData.worldPosition.z - anchorOffset.y;
            }
        }
    }
    
    onResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }
    
    render() {
        // Animate objects
        const time = Date.now() * 0.001;
        for (const obj of this.objects) {
            // Gentle floating animation
            if (obj.userData.placedAt) {
                const age = (Date.now() - obj.userData.placedAt) * 0.001;
                obj.position.y = (obj.userData.worldPosition?.y || 0.3) + Math.sin(time + age) * 0.05;
                obj.rotation.y += 0.005;
            }
        }
        
        this.renderer.render(this.scene, this.camera);
    }
    
    clear() {
        for (const obj of this.objects) {
            this.scene.remove(obj);
        }
        this.objects = [];
    }
}

// ============================================================================
// FEATURE VISUALIZATION
// ============================================================================

class FeatureVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.resize();
        
        window.addEventListener('resize', () => this.resize());
    }
    
    resize() {
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
    }
    
    draw(features, tracked) {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Scale factors (features are from a smaller resolution)
        const scaleX = this.canvas.width / 640;
        const scaleY = this.canvas.height / 480;
        
        // Draw detected features
        this.ctx.fillStyle = 'rgba(247, 37, 133, 0.7)';
        for (const feature of features.slice(0, 100)) {  // Limit for performance
            const x = feature.x * scaleX;
            const y = feature.y * scaleY;
            
            this.ctx.beginPath();
            this.ctx.arc(x, y, 3, 0, Math.PI * 2);
            this.ctx.fill();
        }
        
        // Draw tracked features with motion vectors
        this.ctx.strokeStyle = 'rgba(0, 245, 212, 0.8)';
        this.ctx.lineWidth = 1;
        
        for (const track of tracked.slice(0, 50)) {
            const x1 = track.prevX * scaleX;
            const y1 = track.prevY * scaleY;
            const x2 = track.x * scaleX;
            const y2 = track.y * scaleY;
            
            // Draw line
            this.ctx.beginPath();
            this.ctx.moveTo(x1, y1);
            this.ctx.lineTo(x2, y2);
            this.ctx.stroke();
            
            // Draw endpoint
            this.ctx.fillStyle = 'rgba(0, 245, 212, 1)';
            this.ctx.beginPath();
            this.ctx.arc(x2, y2, 4, 0, Math.PI * 2);
            this.ctx.fill();
        }
    }
    
    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }
}

// ============================================================================
// UI UPDATES
// ============================================================================

function updateLoadingStatus(status) {
    const statusEl = document.getElementById('loading-status');
    if (statusEl) {
        statusEl.textContent = status;
    }
}

function hideLoading() {
    const overlay = document.getElementById('loading-overlay');
    if (overlay) {
        overlay.classList.add('hidden');
    }
}

function showPermissionModal() {
    const modal = document.getElementById('permission-modal');
    if (modal) {
        modal.classList.remove('hidden');
    }
}

function hidePermissionModal() {
    const modal = document.getElementById('permission-modal');
    if (modal) {
        modal.classList.add('hidden');
    }
}

function updateStatus(id, state) {
    const dot = document.getElementById(`status-${id}`);
    if (dot) {
        dot.classList.remove('active', 'warning', 'error');
        if (state === 'active') {
            dot.classList.add('active');
        } else if (state === 'warning') {
            dot.classList.add('warning');
        } else if (state === 'error') {
            dot.classList.add('error');
        }
    }
}

function updateTelemetry(fusion) {
    document.getElementById('pos-x').textContent = fusion.position.x.toFixed(2) + ' m';
    document.getElementById('pos-y').textContent = fusion.position.y.toFixed(2) + ' m';
    
    // Convert heading to degrees
    const heading = MathUtils.toDeg(fusion.orientation.z);
    const normalizedHeading = ((heading % 360) + 360) % 360;
    document.getElementById('heading').textContent = normalizedHeading.toFixed(0) + '°';
    
    document.getElementById('confidence').textContent = (fusion.confidence * 100).toFixed(0) + '%';
}

function updateDebugPanel(state) {
    // GPS
    if (state.gps) {
        if (state.gps.position) {
            document.getElementById('debug-lat').textContent = state.gps.position.latitude.toFixed(6);
            document.getElementById('debug-lon').textContent = state.gps.position.longitude.toFixed(6);
        }
        document.getElementById('debug-accuracy').textContent = 
            (state.gps.accuracy !== Infinity ? state.gps.accuracy.toFixed(1) + ' m' : 'N/A');
        if (state.gps.enu) {
            document.getElementById('debug-enu').textContent = 
                `${state.gps.enu.x.toFixed(1)}, ${state.gps.enu.y.toFixed(1)}`;
        }
    }
    
    // Orientation
    if (state.imu && state.imu.orientation) {
        document.getElementById('debug-alpha').textContent = state.imu.orientation.alpha.toFixed(1) + '°';
        document.getElementById('debug-beta').textContent = state.imu.orientation.beta.toFixed(1) + '°';
        document.getElementById('debug-gamma').textContent = state.imu.orientation.gamma.toFixed(1) + '°';
    }
    
    // SLAM
    if (state.slam) {
        document.getElementById('debug-features').textContent = state.slam.featureCount || 0;
        document.getElementById('debug-keyframes').textContent = state.slam.keyframeCount || 0;
        document.getElementById('debug-mappoints').textContent = state.slam.mapPointCount || 0;
        
        // Show mode info
        const modeText = state.slam.useSimpleMode ? 'Simple' : 
                        (state.slam.cvReady ? 'OpenCV' : 'Init...');
        document.getElementById('debug-trackstatus').textContent = 
            state.fusion ? `${state.fusion.trackingMode} (${modeText})` : modeText;
    }
    
    // Performance
    if (state.performance) {
        document.getElementById('debug-frametime').textContent = state.performance.frameTime.toFixed(1) + ' ms';
        document.getElementById('debug-fps').textContent = state.performance.fps;
    }
}

function toggleDebugPanel() {
    const panel = document.getElementById('debug-panel');
    panel.classList.toggle('collapsed');
}

// ============================================================================
// CAMERA ACCESS
// ============================================================================

async function startCamera() {
    const video = document.getElementById('camera-feed');
    
    const constraints = {
        video: {
            facingMode: 'environment',
            width: { ideal: 1280 },
            height: { ideal: 720 }
        },
        audio: false
    };
    
    try {
        const stream = await navigator.mediaDevices.getUserMedia(constraints);
        video.srcObject = stream;
        await video.play();
        
        console.log('Camera started:', video.videoWidth, 'x', video.videoHeight);
        return true;
    } catch (e) {
        console.error('Camera error:', e);
        return false;
    }
}

// ============================================================================
// PERMISSION HANDLING
// ============================================================================

async function requestPermissions() {
    hidePermissionModal();
    updateLoadingStatus('Requesting permissions...');
    
    let cameraOk = false;
    let sensorsOk = false;
    
    // Request camera
    try {
        cameraOk = await startCamera();
        if (cameraOk) {
            updateStatus('camera', 'active');
            updateLoadingStatus('Camera ready');
        }
    } catch (e) {
        console.error('Camera permission denied:', e);
        updateStatus('camera', 'error');
    }
    
    // Request motion sensors (iOS)
    if (typeof DeviceMotionEvent !== 'undefined' && 
        typeof DeviceMotionEvent.requestPermission === 'function') {
        try {
            const permission = await DeviceMotionEvent.requestPermission();
            sensorsOk = permission === 'granted';
        } catch (e) {
            console.warn('Motion permission error:', e);
        }
    } else {
        sensorsOk = true;  // Android doesn't require explicit permission
    }
    
    if (sensorsOk) {
        updateStatus('imu', 'active');
    }
    
    if (cameraOk) {
        initializeApp();
    } else {
        alert('Camera access is required for AR. Please enable camera permissions and refresh.');
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

let initChecks = {
    dom: false,
    opencv: false,
    camera: false
};

function checkInitializationComplete() {
    if (cvReady) {
        initChecks.opencv = true;
    }
    
    // Check if we can proceed
    if (initChecks.dom) {
        hideLoading();
        showPermissionModal();
    }
}

async function initializeApp() {
    updateLoadingStatus('Initializing AR system...');
    
    const video = document.getElementById('camera-feed');
    const arCanvas = document.getElementById('ar-canvas');
    const featureCanvas = document.getElementById('feature-canvas');
    
    // Initialize Three.js scene first (independent of SLAM)
    threeScene = new ARScene(arCanvas);
    console.log('Three.js scene initialized');
    
    // Initialize feature visualizer
    const featureViz = new FeatureVisualizer(featureCanvas);
    
    // Start render loop immediately
    function renderLoop() {
        if (threeScene) {
            threeScene.render();
        }
        requestAnimationFrame(renderLoop);
    }
    renderLoop();
    
    // Initialize SLAM engine
    slamEngine = new ARSLAMEngine();
    
    try {
        await slamEngine.initialize(video);
        console.log('SLAM engine initialized');
    } catch (e) {
        console.warn('SLAM initialization had issues, continuing anyway:', e);
    }
    
    // Set up update callback
    slamEngine.onUpdate = (state) => {
        // Update UI
        if (state.fusion) {
            updateTelemetry(state.fusion);
        }
        updateDebugPanel(state);
        
        // Update status indicators
        updateStatus('gps', state.gps && state.gps.isActive ? 'active' : 'warning');
        updateStatus('imu', state.imu && state.imu.isActive ? 'active' : 'warning');
        updateStatus('slam', state.slam && state.slam.isActive ? 'active' : 'warning');
        
        // Update feature visualization
        if (state.slam && state.slam.isActive && slamEngine.slam) {
            const slamState = slamEngine.slam;
            if (slamState.currentFeatures && slamState.trackedFeatures) {
                featureViz.draw(slamState.currentFeatures, slamState.trackedFeatures);
            }
        }
        
        // Update 3D scene camera
        if (state.fusion && threeScene) {
            threeScene.updateCameraPose(state.fusion.position, state.fusion.orientation);
        }
    };
    
    // Enable all buttons
    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-place').disabled = false;
    document.getElementById('btn-reset').disabled = false;
    
    updateLoadingStatus('Ready');
    console.log('AR system initialized - all buttons enabled');
}

// ============================================================================
// USER ACTIONS
// ============================================================================

function startAR() {
    if (!slamEngine) return;
    
    const btn = document.getElementById('btn-start');
    
    if (!isARActive) {
        slamEngine.start();
        isARActive = true;
        btn.innerHTML = `
            <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
                <rect x="6" y="4" width="4" height="16"/>
                <rect x="14" y="4" width="4" height="16"/>
            </svg>
            Pause
        `;
        btn.classList.remove('primary');
        btn.classList.add('danger');
        
        document.getElementById('btn-place').disabled = false;
        document.getElementById('btn-reset').disabled = false;
    } else {
        slamEngine.stop();
        isARActive = false;
        btn.innerHTML = `
            <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
                <polygon points="5,3 19,12 5,21"/>
            </svg>
            Start AR
        `;
        btn.classList.remove('danger');
        btn.classList.add('primary');
    }
}

function placeObject() {
    if (!threeScene) {
        console.warn('Three.js scene not initialized');
        return;
    }
    
    // Get position from fusion state or use default
    let posX = 0, posY = 0;
    
    if (slamEngine) {
        const state = slamEngine.getState();
        if (state && state.fusion) {
            posX = state.fusion.position.x || 0;
            posY = state.fusion.position.y || 0;
        }
    }
    
    // Place object in front of camera (2 meters away)
    // Use a slight offset based on placed objects count to spread them out
    const offsetAngle = (placedObjects.length * 45) * Math.PI / 180;
    const distance = 2 + (placedObjects.length * 0.5);
    
    const pos = new THREE.Vector3(
        posX + Math.sin(offsetAngle) * distance,
        0.3,  // Height above ground
        -(posY + Math.cos(offsetAngle) * distance)  // Invert for Three.js coordinate system
    );
    
    // Cycle through object types
    const types = ['cube', 'sphere', 'cylinder', 'house'];
    const typeIndex = placedObjects.length % types.length;
    
    const obj = threeScene.placeObject(pos, types[typeIndex]);
    placedObjects.push(obj);
    
    console.log('Placed object:', types[typeIndex], 'at:', pos.x.toFixed(2), pos.y.toFixed(2), pos.z.toFixed(2));
}

function resetAnchor() {
    if (!slamEngine) return;
    
    slamEngine.resetAnchor();
    
    // Clear placed objects
    if (threeScene) {
        threeScene.clear();
    }
    placedObjects = [];
    
    console.log('Anchor reset');
}

// ============================================================================
// DOM READY
// ============================================================================

document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM ready');
    initChecks.dom = true;
    
    // Check if OpenCV is already loaded
    if (typeof cv !== 'undefined') {
        onOpenCvReady();
    }
    
    checkInitializationComplete();
});

// Handle visibility change (pause when tab hidden)
document.addEventListener('visibilitychange', () => {
    if (document.hidden && isARActive && slamEngine) {
        // Pause processing when hidden
        slamEngine.stop();
    } else if (!document.hidden && isARActive && slamEngine) {
        // Resume when visible
        slamEngine.start();
    }
});

// Cleanup on unload
window.addEventListener('beforeunload', () => {
    if (slamEngine) {
        slamEngine.cleanup();
    }
});
