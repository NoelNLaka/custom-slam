/**
 * Web AR SLAM Engine
 * Hybrid GPS + Visual SLAM for Web-based AR
 * 
 * Architecture:
 * - GPS Module: World-scale positioning with Kalman filtering
 * - IMU Module: High-frequency motion prediction
 * - Visual SLAM: Feature detection and tracking (when OpenCV.js loads)
 * - Sensor Fusion: Extended Kalman Filter combining all inputs
 */

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

const MathUtils = {
    // Degrees to radians
    toRad: (deg) => deg * Math.PI / 180,
    
    // Radians to degrees
    toDeg: (rad) => rad * 180 / Math.PI,
    
    // Clamp value between min and max
    clamp: (val, min, max) => Math.min(Math.max(val, min), max),
    
    // Linear interpolation
    lerp: (a, b, t) => a + (b - a) * t,
    
    // Smooth step interpolation
    smoothstep: (a, b, t) => {
        t = MathUtils.clamp((t - a) / (b - a), 0, 1);
        return t * t * (3 - 2 * t);
    },
    
    // Matrix multiplication (4x4)
    multiplyMatrices: (a, b) => {
        const result = new Float32Array(16);
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                result[i * 4 + j] = 
                    a[i * 4 + 0] * b[0 * 4 + j] +
                    a[i * 4 + 1] * b[1 * 4 + j] +
                    a[i * 4 + 2] * b[2 * 4 + j] +
                    a[i * 4 + 3] * b[3 * 4 + j];
            }
        }
        return result;
    }
};

// ============================================================================
// GPS MODULE - World-scale positioning
// ============================================================================

class GPSModule {
    constructor() {
        this.origin = null;          // First GPS reading as origin
        this.currentPosition = null;  // Current GPS position
        this.enuPosition = { x: 0, y: 0, z: 0 };  // ENU coordinates
        this.accuracy = Infinity;
        this.isActive = false;
        this.watchId = null;
        this.kalmanFilter = new GPSKalmanFilter();
        this.lastUpdate = 0;
        
        // Earth parameters for coordinate conversion
        this.EARTH_RADIUS = 6378137.0;  // meters
        this.FLATTENING = 1 / 298.257223563;
    }
    
    async start() {
        return new Promise((resolve, reject) => {
            if (!navigator.geolocation) {
                reject(new Error('Geolocation not supported'));
                return;
            }
            
            const options = {
                enableHighAccuracy: true,
                timeout: 10000,
                maximumAge: 0
            };
            
            this.watchId = navigator.geolocation.watchPosition(
                (position) => this.handlePosition(position),
                (error) => console.warn('GPS error:', error.message),
                options
            );
            
            // Get initial position
            navigator.geolocation.getCurrentPosition(
                (position) => {
                    this.handlePosition(position);
                    this.isActive = true;
                    resolve(this.currentPosition);
                },
                (error) => {
                    console.warn('Initial GPS failed:', error.message);
                    // Still resolve - we can work without GPS
                    this.isActive = false;
                    resolve(null);
                },
                options
            );
        });
    }
    
    stop() {
        if (this.watchId !== null) {
            navigator.geolocation.clearWatch(this.watchId);
            this.watchId = null;
        }
        this.isActive = false;
    }
    
    handlePosition(position) {
        const { latitude, longitude, altitude, accuracy } = position.coords;
        const timestamp = position.timestamp;
        
        // Set origin on first reading
        if (!this.origin) {
            this.origin = { latitude, longitude, altitude: altitude || 0 };
            console.log('GPS origin set:', this.origin);
        }
        
        this.currentPosition = { latitude, longitude, altitude: altitude || 0 };
        this.accuracy = accuracy;
        this.lastUpdate = timestamp;
        
        // Convert to ENU coordinates
        const enu = this.gpsToENU(latitude, longitude, altitude || 0);
        
        // Apply Kalman filter
        const dt = (timestamp - this.kalmanFilter.lastUpdate) / 1000;
        if (dt > 0) {
            this.kalmanFilter.predict(dt);
            const filtered = this.kalmanFilter.update([enu.x, enu.y], accuracy);
            this.enuPosition = { x: filtered[0], y: filtered[1], z: enu.z };
        }
        
        this.isActive = true;
    }
    
    /**
     * Convert GPS coordinates to local ENU (East-North-Up)
     */
    gpsToENU(lat, lon, alt) {
        if (!this.origin) return { x: 0, y: 0, z: 0 };
        
        const latRad = MathUtils.toRad(lat);
        const lonRad = MathUtils.toRad(lon);
        const originLatRad = MathUtils.toRad(this.origin.latitude);
        const originLonRad = MathUtils.toRad(this.origin.longitude);
        
        const dlat = latRad - originLatRad;
        const dlon = lonRad - originLonRad;
        const dalt = alt - this.origin.altitude;
        
        // Radius of curvature calculations
        const sinLat = Math.sin(originLatRad);
        const RLat = this.EARTH_RADIUS * (1 - this.FLATTENING) / 
                     Math.pow(1 - this.FLATTENING * sinLat * sinLat, 1.5);
        const RLon = this.EARTH_RADIUS / 
                     Math.sqrt(1 - this.FLATTENING * sinLat * sinLat);
        
        // ENU coordinates
        const east = dlon * RLon * Math.cos(originLatRad);
        const north = dlat * RLat;
        const up = dalt;
        
        return { x: east, y: north, z: up };
    }
    
    getConfidence() {
        if (!this.isActive || this.accuracy === Infinity) return 0;
        // Map accuracy (meters) to confidence (0-1)
        // <5m = 1.0, >50m = 0.0
        return MathUtils.clamp(1 - (this.accuracy - 5) / 45, 0, 1);
    }
    
    getState() {
        return {
            position: this.currentPosition,
            enu: this.enuPosition,
            accuracy: this.accuracy,
            confidence: this.getConfidence(),
            isActive: this.isActive
        };
    }
}

// ============================================================================
// GPS KALMAN FILTER
// ============================================================================

class GPSKalmanFilter {
    constructor() {
        // State: [x, y, vx, vy]
        this.state = new Float32Array([0, 0, 0, 0]);
        
        // Covariance matrix (4x4)
        this.P = new Float32Array([
            10, 0, 0, 0,
            0, 10, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ]);
        
        // Process noise
        this.processNoise = 0.1;
        
        // Measurement noise base
        this.measurementNoiseBase = 5.0;
        
        this.lastUpdate = Date.now();
    }
    
    predict(dt) {
        // State transition: x' = x + vx*dt, y' = y + vy*dt
        this.state[0] += this.state[2] * dt;
        this.state[1] += this.state[3] * dt;
        
        // Update covariance (simplified)
        const q = this.processNoise * dt;
        this.P[0] += q + this.P[8] * dt;
        this.P[5] += q + this.P[13] * dt;
        
        this.lastUpdate = Date.now();
    }
    
    update(measurement, accuracy) {
        // Adjust measurement noise based on GPS accuracy
        const R = accuracy * accuracy;
        
        // Kalman gain (simplified 2D measurement)
        const K0 = this.P[0] / (this.P[0] + R);
        const K1 = this.P[5] / (this.P[5] + R);
        
        // Innovation
        const y0 = measurement[0] - this.state[0];
        const y1 = measurement[1] - this.state[1];
        
        // Update state
        this.state[0] += K0 * y0;
        this.state[1] += K1 * y1;
        this.state[2] += K0 * y0 * 0.1;  // Velocity update
        this.state[3] += K1 * y1 * 0.1;
        
        // Update covariance
        this.P[0] *= (1 - K0);
        this.P[5] *= (1 - K1);
        
        return [this.state[0], this.state[1]];
    }
}

// ============================================================================
// IMU MODULE - Motion sensors
// ============================================================================

class IMUModule {
    constructor() {
        this.orientation = { alpha: 0, beta: 0, gamma: 0 };  // Device orientation
        this.acceleration = { x: 0, y: 0, z: 0 };            // Linear acceleration
        this.rotationRate = { alpha: 0, beta: 0, gamma: 0 }; // Gyroscope
        this.isActive = false;
        this.calibrated = false;
        this.calibrationSamples = [];
        this.bias = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 };
        
        // Complementary filter parameters
        this.filterAlpha = 0.98;  // Weight for gyroscope
        
        // Integrated pose (for dead reckoning)
        this.integratedRotation = { x: 0, y: 0, z: 0 };
        this.lastTimestamp = 0;
    }
    
    async start() {
        // Check for DeviceOrientation support
        if (typeof DeviceOrientationEvent !== 'undefined') {
            // iOS 13+ requires permission
            if (typeof DeviceOrientationEvent.requestPermission === 'function') {
                try {
                    const permission = await DeviceOrientationEvent.requestPermission();
                    if (permission !== 'granted') {
                        console.warn('DeviceOrientation permission denied');
                        return false;
                    }
                } catch (e) {
                    console.warn('DeviceOrientation permission error:', e);
                    return false;
                }
            }
            
            window.addEventListener('deviceorientation', (e) => this.handleOrientation(e));
        }
        
        // DeviceMotion for acceleration and rotation rate
        if (typeof DeviceMotionEvent !== 'undefined') {
            if (typeof DeviceMotionEvent.requestPermission === 'function') {
                try {
                    const permission = await DeviceMotionEvent.requestPermission();
                    if (permission !== 'granted') {
                        console.warn('DeviceMotion permission denied');
                        return false;
                    }
                } catch (e) {
                    console.warn('DeviceMotion permission error:', e);
                    return false;
                }
            }
            
            window.addEventListener('devicemotion', (e) => this.handleMotion(e));
        }
        
        this.isActive = true;
        return true;
    }
    
    stop() {
        window.removeEventListener('deviceorientation', this.handleOrientation);
        window.removeEventListener('devicemotion', this.handleMotion);
        this.isActive = false;
    }
    
    handleOrientation(event) {
        if (event.alpha !== null) {
            // Apply low-pass filter for smooth orientation
            const alpha = 0.8;
            this.orientation.alpha = alpha * this.orientation.alpha + (1 - alpha) * event.alpha;
            this.orientation.beta = alpha * this.orientation.beta + (1 - alpha) * event.beta;
            this.orientation.gamma = alpha * this.orientation.gamma + (1 - alpha) * event.gamma;
            this.isActive = true;
        }
    }
    
    handleMotion(event) {
        const now = performance.now();
        const dt = this.lastTimestamp > 0 ? (now - this.lastTimestamp) / 1000 : 0;
        this.lastTimestamp = now;
        
        // Linear acceleration (gravity removed)
        if (event.accelerationIncludingGravity) {
            const acc = event.accelerationIncludingGravity;
            this.acceleration = {
                x: (acc.x || 0) - this.bias.ax,
                y: (acc.y || 0) - this.bias.ay,
                z: (acc.z || 0) - this.bias.az
            };
        }
        
        // Rotation rate (gyroscope)
        if (event.rotationRate) {
            const rot = event.rotationRate;
            this.rotationRate = {
                alpha: (rot.alpha || 0) - this.bias.gx,
                beta: (rot.beta || 0) - this.bias.gy,
                gamma: (rot.gamma || 0) - this.bias.gz
            };
            
            // Integrate rotation for dead reckoning
            if (dt > 0 && dt < 0.1) {
                this.integratedRotation.x += this.rotationRate.beta * dt;
                this.integratedRotation.y += this.rotationRate.gamma * dt;
                this.integratedRotation.z += this.rotationRate.alpha * dt;
            }
        }
        
        // Collect calibration samples
        if (!this.calibrated && this.calibrationSamples.length < 50) {
            this.calibrationSamples.push({
                acc: { ...this.acceleration },
                rot: { ...this.rotationRate }
            });
            
            if (this.calibrationSamples.length === 50) {
                this.calibrate();
            }
        }
        
        this.isActive = true;
    }
    
    calibrate() {
        // Calculate bias from stationary samples
        const n = this.calibrationSamples.length;
        let sumAx = 0, sumAy = 0, sumAz = 0;
        let sumGx = 0, sumGy = 0, sumGz = 0;
        
        for (const sample of this.calibrationSamples) {
            sumAx += sample.acc.x;
            sumAy += sample.acc.y;
            sumAz += sample.acc.z;
            sumGx += sample.rot.alpha;
            sumGy += sample.rot.beta;
            sumGz += sample.rot.gamma;
        }
        
        this.bias = {
            ax: sumAx / n,
            ay: sumAy / n,
            az: sumAz / n - 9.81,  // Remove gravity
            gx: sumGx / n,
            gy: sumGy / n,
            gz: sumGz / n
        };
        
        this.calibrated = true;
        console.log('IMU calibrated:', this.bias);
    }
    
    /**
     * Get compass heading (0-360 degrees, 0 = North)
     */
    getHeading() {
        let heading = this.orientation.alpha;
        
        // Adjust for device orientation
        if (window.screen.orientation) {
            const angle = window.screen.orientation.angle || 0;
            heading -= angle;
        }
        
        // Normalize to 0-360
        heading = ((heading % 360) + 360) % 360;
        
        return heading;
    }
    
    getState() {
        return {
            orientation: this.orientation,
            acceleration: this.acceleration,
            rotationRate: this.rotationRate,
            heading: this.getHeading(),
            isActive: this.isActive,
            calibrated: this.calibrated
        };
    }
}

// ============================================================================
// VISUAL SLAM MODULE - Feature detection and tracking
// ============================================================================

class VisualSLAMModule {
    constructor() {
        this.isActive = false;
        this.cvReady = false;
        this.initAttempted = false;
        this.useSimpleMode = false;  // Fallback mode without OpenCV
        
        // Feature tracking state
        this.prevFrame = null;
        this.prevGray = null;
        this.prevFeatures = null;
        this.currentFeatures = [];
        this.trackedFeatures = [];
        
        // Keyframe management
        this.keyframes = [];
        this.maxKeyframes = 30;
        this.keyframeInterval = 500;  // ms
        this.lastKeyframeTime = 0;
        
        // Map points (sparse 3D reconstruction)
        this.mapPoints = [];
        this.maxMapPoints = 5000;
        
        // Camera parameters (will be estimated)
        this.cameraMatrix = null;
        this.frameWidth = 640;
        this.frameHeight = 480;
        
        // Pose estimation
        this.pose = {
            rotation: [0, 0, 0],
            translation: [0, 0, 0]
        };
        
        // Performance metrics
        this.featureCount = 0;
        this.trackingQuality = 0;
        this.processingTime = 0;
        
        // Canvas for frame capture
        this.captureCanvas = null;
        this.captureCtx = null;
    }
    
    async initialize(videoElement) {
        this.initAttempted = true;
        this.videoElement = videoElement;
        
        // Get video dimensions
        this.frameWidth = videoElement.videoWidth || 640;
        this.frameHeight = videoElement.videoHeight || 480;
        
        // Create capture canvas
        this.captureCanvas = document.createElement('canvas');
        this.captureCanvas.width = this.frameWidth;
        this.captureCanvas.height = this.frameHeight;
        this.captureCtx = this.captureCanvas.getContext('2d', { willReadFrequently: true });
        
        // Check if OpenCV is available and working
        if (typeof cv === 'undefined') {
            console.warn('OpenCV.js not loaded - using simple mode');
            this.useSimpleMode = true;
            this.isActive = true;
            return true;
        }
        
        // Wait for OpenCV to be fully ready
        if (cv.Mat === undefined) {
            console.warn('OpenCV.js not fully initialized - using simple mode');
            this.useSimpleMode = true;
            this.isActive = true;
            return true;
        }
        
        try {
            // Initialize camera matrix (assuming typical smartphone FOV ~60°)
            const focalLength = this.frameWidth / (2 * Math.tan(MathUtils.toRad(30)));
            this.cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, [
                focalLength, 0, this.frameWidth / 2,
                0, focalLength, this.frameHeight / 2,
                0, 0, 1
            ]);
            
            // Test if ORB works
            this.orbDetector = new cv.ORB(300);  // Reduced features for stability
            
            this.cvReady = true;
            this.isActive = true;
            console.log('Visual SLAM initialized with OpenCV');
            return true;
        } catch (e) {
            console.warn('OpenCV initialization failed, using simple mode:', e);
            this.useSimpleMode = true;
            this.isActive = true;
            return true;
        }
    }
    
    /**
     * Process a video frame
     */
    processFrame(videoElement) {
        if (!this.isActive) return null;
        
        const startTime = performance.now();
        
        // Use simple corner detection if OpenCV isn't working
        if (this.useSimpleMode || !this.cvReady) {
            return this.processFrameSimple(videoElement, startTime);
        }
        
        return this.processFrameOpenCV(videoElement, startTime);
    }
    
    /**
     * Simple frame processing without OpenCV
     * Uses basic corner detection via canvas
     */
    processFrameSimple(videoElement, startTime) {
        try {
            // Capture frame to canvas
            this.captureCtx.drawImage(videoElement, 0, 0, this.frameWidth, this.frameHeight);
            const imageData = this.captureCtx.getImageData(0, 0, this.frameWidth, this.frameHeight);
            
            // Simple corner detection using FAST-like approach
            this.currentFeatures = this.detectCornersSimple(imageData);
            this.featureCount = this.currentFeatures.length;
            
            // Track features if we have previous frame data
            if (this.prevFeatures && this.prevFeatures.length > 0) {
                this.trackedFeatures = this.trackFeaturesSimple(this.currentFeatures);
            }
            
            // Store for next frame
            this.prevFeatures = this.currentFeatures.slice();
            
            // Calculate tracking quality
            this.trackingQuality = Math.min(this.featureCount / 200, 1);
            
            this.processingTime = performance.now() - startTime;
            
            return {
                features: this.currentFeatures,
                tracked: this.trackedFeatures,
                pose: this.pose,
                quality: this.trackingQuality
            };
        } catch (e) {
            console.warn('Simple frame processing error:', e);
            return null;
        }
    }
    
    /**
     * Simple corner detection
     */
    detectCornersSimple(imageData) {
        const width = imageData.width;
        const height = imageData.height;
        const data = imageData.data;
        const corners = [];
        
        // Convert to grayscale and detect corners
        const threshold = 30;
        const step = 8;  // Sample every 8 pixels for performance
        
        for (let y = step; y < height - step; y += step) {
            for (let x = step; x < width - step; x += step) {
                const idx = (y * width + x) * 4;
                const gray = (data[idx] + data[idx + 1] + data[idx + 2]) / 3;
                
                // Simple corner score (difference from neighbors)
                let score = 0;
                const offsets = [
                    [-step, 0], [step, 0], [0, -step], [0, step],
                    [-step, -step], [step, -step], [-step, step], [step, step]
                ];
                
                for (const [dx, dy] of offsets) {
                    const nIdx = ((y + dy) * width + (x + dx)) * 4;
                    const nGray = (data[nIdx] + data[nIdx + 1] + data[nIdx + 2]) / 3;
                    score += Math.abs(gray - nGray);
                }
                
                if (score > threshold * 8) {
                    corners.push({
                        x: x,
                        y: y,
                        size: 5,
                        response: score
                    });
                }
            }
        }
        
        // Sort by response and take top features
        corners.sort((a, b) => b.response - a.response);
        return corners.slice(0, 200);
    }
    
    /**
     * Simple feature tracking (nearest neighbor matching)
     */
    trackFeaturesSimple(currentFeatures) {
        const tracked = [];
        const maxDist = 30;  // Maximum tracking distance
        
        for (const prev of this.prevFeatures.slice(0, 50)) {
            let bestMatch = null;
            let bestDist = maxDist;
            
            for (const curr of currentFeatures) {
                const dist = Math.sqrt(
                    Math.pow(curr.x - prev.x, 2) + 
                    Math.pow(curr.y - prev.y, 2)
                );
                
                if (dist < bestDist) {
                    bestDist = dist;
                    bestMatch = curr;
                }
            }
            
            if (bestMatch) {
                tracked.push({
                    prevX: prev.x,
                    prevY: prev.y,
                    x: bestMatch.x,
                    y: bestMatch.y
                });
            }
        }
        
        return tracked;
    }
    
    /**
     * OpenCV-based frame processing
     */
    processFrameOpenCV(videoElement, startTime) {
        let frame = null;
        let gray = null;
        let keypoints = null;
        let descriptors = null;
        
        try {
            // Capture frame to canvas first (more reliable than VideoCapture)
            this.captureCtx.drawImage(videoElement, 0, 0, this.frameWidth, this.frameHeight);
            
            // Create Mat from canvas
            frame = cv.imread(this.captureCanvas);
            
            // Convert to grayscale
            gray = new cv.Mat();
            cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);
            
            // Detect ORB features
            keypoints = new cv.KeyPointVector();
            descriptors = new cv.Mat();
            const mask = new cv.Mat();
            
            this.orbDetector.detectAndCompute(gray, mask, keypoints, descriptors);
            mask.delete();
            
            this.featureCount = keypoints.size();
            
            // Extract feature positions
            this.currentFeatures = [];
            for (let i = 0; i < keypoints.size(); i++) {
                const kp = keypoints.get(i);
                this.currentFeatures.push({
                    x: kp.pt.x,
                    y: kp.pt.y,
                    size: kp.size,
                    response: kp.response
                });
            }
            
            // Track features from previous frame
            if (this.prevGray && this.prevFeatures && this.prevFeatures.length > 0) {
                this.trackFeaturesOpenCV(gray);
            }
            
            // Check if we should create a keyframe
            const now = performance.now();
            if (now - this.lastKeyframeTime > this.keyframeInterval) {
                this.createKeyframe(keypoints);
                this.lastKeyframeTime = now;
            }
            
            // Store for next frame
            if (this.prevGray) this.prevGray.delete();
            this.prevGray = gray.clone();
            this.prevFeatures = this.currentFeatures.slice();
            
            // Calculate tracking quality
            this.trackingQuality = Math.min(this.featureCount / 200, 1);
            
            this.processingTime = performance.now() - startTime;
            
            return {
                features: this.currentFeatures,
                tracked: this.trackedFeatures,
                pose: this.pose,
                quality: this.trackingQuality
            };
            
        } catch (e) {
            console.warn('OpenCV processing error, switching to simple mode:', e.message || e);
            this.useSimpleMode = true;
            return this.processFrameSimple(videoElement, startTime);
        } finally {
            // Cleanup
            if (frame) frame.delete();
            if (gray) gray.delete();
            if (keypoints) keypoints.delete();
            if (descriptors) descriptors.delete();
        }
    }
    
    /**
     * Track features using optical flow (OpenCV)
     */
    trackFeaturesOpenCV(currentGray) {
        if (!this.prevGray || this.prevFeatures.length === 0) return;
        
        let prevPts = null;
        let nextPts = null;
        let status = null;
        let err = null;
        let winSize = null;
        
        try {
            // Convert previous features to cv.Mat
            const pts = [];
            for (const f of this.prevFeatures.slice(0, 100)) {
                pts.push(f.x, f.y);
            }
            prevPts = cv.matFromArray(pts.length / 2, 1, cv.CV_32FC2, pts);
            
            // Lucas-Kanade optical flow
            nextPts = new cv.Mat();
            status = new cv.Mat();
            err = new cv.Mat();
            winSize = new cv.Size(21, 21);
            
            cv.calcOpticalFlowPyrLK(
                this.prevGray, 
                currentGray, 
                prevPts, 
                nextPts,
                status, 
                err,
                winSize,
                2
            );
            
            // Collect tracked features
            this.trackedFeatures = [];
            
            for (let i = 0; i < status.rows; i++) {
                if (status.data[i] === 1) {
                    const x = nextPts.floatAt(i, 0);
                    const y = nextPts.floatAt(i, 1);
                    
                    // Check if within frame bounds
                    if (x >= 0 && x < this.frameWidth && y >= 0 && y < this.frameHeight) {
                        this.trackedFeatures.push({
                            prevX: this.prevFeatures[i].x,
                            prevY: this.prevFeatures[i].y,
                            x: x,
                            y: y
                        });
                    }
                }
            }
            
        } catch (e) {
            console.warn('Optical flow error:', e.message || e);
        } finally {
            if (prevPts) prevPts.delete();
            if (nextPts) nextPts.delete();
            if (status) status.delete();
            if (err) err.delete();
        }
    }
    
    /**
     * Create a keyframe
     */
    createKeyframe(keypoints) {
        const keyframe = {
            id: this.keyframes.length,
            timestamp: Date.now(),
            pose: { ...this.pose },
            featureCount: keypoints ? keypoints.size() : this.featureCount
        };
        
        this.keyframes.push(keyframe);
        
        // Remove old keyframes
        if (this.keyframes.length > this.maxKeyframes) {
            this.keyframes.shift();
        }
    }
    
    getState() {
        return {
            isActive: this.isActive,
            cvReady: this.cvReady,
            useSimpleMode: this.useSimpleMode,
            featureCount: this.featureCount,
            trackedCount: this.trackedFeatures.length,
            keyframeCount: this.keyframes.length,
            mapPointCount: this.mapPoints.length,
            trackingQuality: this.trackingQuality,
            pose: this.pose,
            processingTime: this.processingTime
        };
    }
    
    cleanup() {
        try {
            if (this.prevGray) this.prevGray.delete();
            if (this.cameraMatrix) this.cameraMatrix.delete();
            if (this.orbDetector) this.orbDetector.delete();
        } catch (e) {
            // Ignore cleanup errors
        }
    }
}

// ============================================================================
// SENSOR FUSION - Extended Kalman Filter
// ============================================================================

class SensorFusion {
    constructor() {
        // Fused state
        this.position = { x: 0, y: 0, z: 0 };     // World position (meters)
        this.orientation = { x: 0, y: 0, z: 0 };  // Euler angles (radians)
        this.velocity = { x: 0, y: 0, z: 0 };     // Velocity (m/s)
        
        // Confidence and mode
        this.confidence = 0;
        this.trackingMode = 'initializing';  // 'gps', 'slam', 'hybrid', 'lost'
        
        // Sensor weights
        this.gpsWeight = 0.7;
        this.slamWeight = 0.3;
        this.imuWeight = 0.5;
        
        // Drift correction
        this.driftOffset = { x: 0, y: 0, z: 0 };
        this.lastDriftCorrection = 0;
        this.driftCorrectionInterval = 30000;  // 30 seconds
        
        // World anchor
        this.worldAnchor = null;
        
        // Update timing
        this.lastUpdate = performance.now();
    }
    
    /**
     * Update fused state with all sensor inputs
     */
    update(gpsState, imuState, slamState) {
        const now = performance.now();
        const dt = (now - this.lastUpdate) / 1000;
        this.lastUpdate = now;
        
        if (dt <= 0 || dt > 1) return;  // Skip invalid intervals
        
        // Determine tracking mode based on sensor availability
        this.updateTrackingMode(gpsState, imuState, slamState);
        
        // Fuse position
        this.fusePosition(gpsState, slamState, dt);
        
        // Fuse orientation
        this.fuseOrientation(imuState, slamState);
        
        // Apply drift correction
        this.applyDriftCorrection(gpsState);
        
        // Calculate confidence
        this.calculateConfidence(gpsState, imuState, slamState);
    }
    
    updateTrackingMode(gpsState, imuState, slamState) {
        const hasGPS = gpsState && gpsState.isActive && gpsState.confidence > 0.3;
        const hasSLAM = slamState && slamState.isActive && slamState.trackingQuality > 0.3;
        const hasIMU = imuState && imuState.isActive;
        
        if (hasGPS && hasSLAM) {
            this.trackingMode = 'hybrid';
            this.gpsWeight = 0.6;
            this.slamWeight = 0.4;
        } else if (hasGPS) {
            this.trackingMode = 'gps';
            this.gpsWeight = 0.9;
            this.slamWeight = 0.1;
        } else if (hasSLAM) {
            this.trackingMode = 'slam';
            this.gpsWeight = 0.1;
            this.slamWeight = 0.9;
        } else if (hasIMU) {
            this.trackingMode = 'imu_only';
        } else {
            this.trackingMode = 'lost';
        }
    }
    
    fusePosition(gpsState, slamState, dt) {
        let gpsPos = { x: 0, y: 0, z: 0 };
        let slamPos = { x: 0, y: 0, z: 0 };
        
        if (gpsState && gpsState.enu) {
            gpsPos = gpsState.enu;
        }
        
        if (slamState && slamState.pose) {
            // Scale SLAM translation (arbitrary scale without depth sensor)
            const scale = 0.1;  // Adjust based on testing
            slamPos = {
                x: slamState.pose.translation[0] * scale,
                y: slamState.pose.translation[1] * scale,
                z: slamState.pose.translation[2] * scale
            };
        }
        
        // Weighted fusion
        const totalWeight = this.gpsWeight + this.slamWeight;
        this.position = {
            x: (gpsPos.x * this.gpsWeight + slamPos.x * this.slamWeight) / totalWeight,
            y: (gpsPos.y * this.gpsWeight + slamPos.y * this.slamWeight) / totalWeight,
            z: (gpsPos.z * this.gpsWeight + slamPos.z * this.slamWeight) / totalWeight
        };
        
        // Apply drift correction
        this.position.x -= this.driftOffset.x;
        this.position.y -= this.driftOffset.y;
        this.position.z -= this.driftOffset.z;
    }
    
    fuseOrientation(imuState, slamState) {
        if (imuState && imuState.isActive) {
            // Primary orientation from IMU
            this.orientation = {
                x: MathUtils.toRad(imuState.orientation.beta),   // Pitch
                y: MathUtils.toRad(imuState.orientation.gamma),  // Roll
                z: MathUtils.toRad(imuState.orientation.alpha)   // Yaw
            };
        }
        
        // Can refine with SLAM rotation if available
        if (slamState && slamState.pose && slamState.trackingQuality > 0.5) {
            // Complementary filter for rotation refinement
            const alpha = 0.9;  // Trust IMU more for orientation
            this.orientation = {
                x: alpha * this.orientation.x + (1 - alpha) * slamState.pose.rotation[0],
                y: alpha * this.orientation.y + (1 - alpha) * slamState.pose.rotation[1],
                z: alpha * this.orientation.z + (1 - alpha) * slamState.pose.rotation[2]
            };
        }
    }
    
    applyDriftCorrection(gpsState) {
        const now = performance.now();
        
        if (now - this.lastDriftCorrection > this.driftCorrectionInterval) {
            if (gpsState && gpsState.isActive && gpsState.confidence > 0.5) {
                // Calculate drift
                const drift = {
                    x: this.position.x - gpsState.enu.x,
                    y: this.position.y - gpsState.enu.y,
                    z: this.position.z - gpsState.enu.z
                };
                
                // Gradually apply correction
                const correctionRate = 0.1;
                this.driftOffset.x += drift.x * correctionRate;
                this.driftOffset.y += drift.y * correctionRate;
                this.driftOffset.z += drift.z * correctionRate;
                
                this.lastDriftCorrection = now;
            }
        }
    }
    
    calculateConfidence(gpsState, imuState, slamState) {
        let confidence = 0;
        let weights = 0;
        
        if (gpsState && gpsState.isActive) {
            confidence += gpsState.confidence * 0.4;
            weights += 0.4;
        }
        
        if (imuState && imuState.isActive) {
            confidence += (imuState.calibrated ? 0.8 : 0.5) * 0.2;
            weights += 0.2;
        }
        
        if (slamState && slamState.isActive) {
            confidence += slamState.trackingQuality * 0.4;
            weights += 0.4;
        }
        
        this.confidence = weights > 0 ? confidence / weights : 0;
    }
    
    /**
     * Set world anchor (origin point)
     */
    setWorldAnchor(gpsPosition) {
        this.worldAnchor = {
            gps: { ...gpsPosition },
            localPosition: { ...this.position },
            timestamp: Date.now()
        };
        
        // Reset drift
        this.driftOffset = { x: 0, y: 0, z: 0 };
    }
    
    /**
     * Reset anchor to current position
     */
    resetAnchor() {
        this.position = { x: 0, y: 0, z: 0 };
        this.driftOffset = { x: 0, y: 0, z: 0 };
        this.worldAnchor = null;
    }
    
    getState() {
        return {
            position: this.position,
            orientation: this.orientation,
            velocity: this.velocity,
            confidence: this.confidence,
            trackingMode: this.trackingMode,
            worldAnchor: this.worldAnchor
        };
    }
}

// ============================================================================
// AR SLAM ENGINE - Main controller
// ============================================================================

class ARSLAMEngine {
    constructor() {
        this.gps = new GPSModule();
        this.imu = new IMUModule();
        this.slam = new VisualSLAMModule();
        this.fusion = new SensorFusion();
        
        this.isRunning = false;
        this.videoElement = null;
        this.animationFrame = null;
        
        // Performance tracking
        this.fps = 0;
        this.frameCount = 0;
        this.lastFPSUpdate = 0;
        this.frameTime = 0;
        
        // Callbacks
        this.onUpdate = null;
        this.onStatusChange = null;
    }
    
    async initialize(videoElement) {
        this.videoElement = videoElement;
        
        console.log('Initializing AR SLAM Engine...');
        
        // Start GPS
        try {
            await this.gps.start();
            console.log('GPS initialized');
        } catch (e) {
            console.warn('GPS initialization failed:', e);
        }
        
        // Start IMU
        try {
            await this.imu.start();
            console.log('IMU initialized');
        } catch (e) {
            console.warn('IMU initialization failed:', e);
        }
        
        // Initialize Visual SLAM (after video is ready)
        if (this.videoElement.readyState >= 2) {
            await this.initializeSLAM();
        } else {
            this.videoElement.addEventListener('loadeddata', () => this.initializeSLAM());
        }
        
        return true;
    }
    
    async initializeSLAM() {
        if (typeof cv !== 'undefined') {
            await this.slam.initialize(this.videoElement);
        } else {
            console.warn('OpenCV.js not ready');
        }
    }
    
    start() {
        if (this.isRunning) return;
        
        this.isRunning = true;
        this.lastFPSUpdate = performance.now();
        this.frameCount = 0;
        
        // Set initial world anchor
        if (this.gps.currentPosition) {
            this.fusion.setWorldAnchor(this.gps.currentPosition);
        }
        
        this.update();
    }
    
    stop() {
        this.isRunning = false;
        
        if (this.animationFrame) {
            cancelAnimationFrame(this.animationFrame);
            this.animationFrame = null;
        }
    }
    
    update() {
        if (!this.isRunning) return;
        
        const startTime = performance.now();
        
        // Process visual SLAM
        let slamResult = null;
        if (this.slam.cvReady && this.videoElement) {
            slamResult = this.slam.processFrame(this.videoElement);
        }
        
        // Get sensor states
        const gpsState = this.gps.getState();
        const imuState = this.imu.getState();
        const slamState = this.slam.getState();
        
        // Fuse sensors
        this.fusion.update(gpsState, imuState, slamState);
        
        // Calculate FPS
        this.frameCount++;
        const now = performance.now();
        if (now - this.lastFPSUpdate >= 1000) {
            this.fps = this.frameCount;
            this.frameCount = 0;
            this.lastFPSUpdate = now;
        }
        
        this.frameTime = now - startTime;
        
        // Callback with full state
        if (this.onUpdate) {
            this.onUpdate({
                gps: gpsState,
                imu: imuState,
                slam: slamState,
                fusion: this.fusion.getState(),
                performance: {
                    fps: this.fps,
                    frameTime: this.frameTime
                }
            });
        }
        
        // Schedule next frame
        this.animationFrame = requestAnimationFrame(() => this.update());
    }
    
    resetAnchor() {
        this.fusion.resetAnchor();
        if (this.gps.currentPosition) {
            this.fusion.setWorldAnchor(this.gps.currentPosition);
        }
    }
    
    cleanup() {
        this.stop();
        this.gps.stop();
        this.imu.stop();
        this.slam.cleanup();
    }
    
    getState() {
        return {
            gps: this.gps.getState(),
            imu: this.imu.getState(),
            slam: this.slam.getState(),
            fusion: this.fusion.getState(),
            performance: {
                fps: this.fps,
                frameTime: this.frameTime
            }
        };
    }
}

// Export for use
window.ARSLAMEngine = ARSLAMEngine;
window.MathUtils = MathUtils;
