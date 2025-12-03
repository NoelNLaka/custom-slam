# Web AR SLAM Prototype

A browser-based Augmented Reality system implementing a hybrid GPS + Visual SLAM architecture for outdoor AR experiences without native ARCore/ARKit dependencies.

![Architecture](https://img.shields.io/badge/Architecture-Hybrid%20SLAM-00f5d4)
![Platform](https://img.shields.io/badge/Platform-Web-4361ee)
![Status](https://img.shields.io/badge/Status-Prototype-f72585)

## 🎯 Overview

This prototype demonstrates the core concepts from the Custom AR SLAM architecture document, adapted for web deployment:

- **GPS Module**: World-scale positioning with Kalman filtering
- **IMU Module**: Device orientation and motion tracking
- **Visual SLAM**: ORB feature detection and optical flow tracking via OpenCV.js
- **Sensor Fusion**: Extended Kalman Filter combining all sensor inputs
- **AR Rendering**: Three.js-based 3D object placement and rendering

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Web AR SLAM System                        │
├─────────────────────────────────────────────────────────────┤
│  SENSOR LAYER (Browser APIs)                                │
│  ├── getUserMedia()          → Camera (30fps)               │
│  ├── Geolocation API         → GPS                          │
│  ├── DeviceMotion API        → IMU (~60Hz)                  │
│  └── DeviceOrientation API   → Compass                      │
├─────────────────────────────────────────────────────────────┤
│  PROCESSING LAYER                                           │
│  ├── GPSModule               → Kalman-filtered positioning  │
│  ├── IMUModule               → Orientation tracking         │
│  ├── VisualSLAMModule        → Feature detection/tracking   │
│  └── SensorFusion            → EKF state estimation         │
├─────────────────────────────────────────────────────────────┤
│  RENDERING LAYER                                            │
│  ├── Three.js                → 3D AR rendering              │
│  ├── Camera feed background  → Video texture                │
│  └── Feature visualization   → Debug overlay                │
└─────────────────────────────────────────────────────────────┘
```

## 📁 File Structure

```
ar-slam-prototype/
├── index.html          # Main HTML with UI components
├── slam-engine.js      # Core SLAM engine (GPS, IMU, Visual SLAM, Fusion)
├── app.js              # Application controller and Three.js scene
└── README.md           # This file
```

## 🚀 Quick Start

### Local Development

1. **Start a local HTTPS server** (required for sensor APIs):

   ```bash
   # Using Python
   python -m http.server 8000
   
   # Or using Node.js
   npx serve .
   
   # Or using PHP
   php -S localhost:8000
   ```

2. **For HTTPS (required on mobile)**, use ngrok or similar:
   
   ```bash
   ngrok http 8000
   ```

3. **Open in browser** and grant permissions for:
   - Camera access
   - Location access
   - Motion sensors (iOS will prompt)

### Deployment Options

#### Option 1: Vercel (Recommended)

```bash
# Install Vercel CLI
npm i -g vercel

# Deploy
cd ar-slam-prototype
vercel
```

#### Option 2: Netlify

```bash
# Drag and drop the folder to netlify.com/drop
# Or use CLI:
npm i -g netlify-cli
netlify deploy --prod
```

#### Option 3: GitHub Pages

1. Create a repository
2. Push the files
3. Enable GitHub Pages in repository settings
4. Access at `https://username.github.io/repo-name`

#### Option 4: Cloudflare Pages

```bash
# Connect your GitHub repo at pages.cloudflare.com
# Or use Wrangler CLI
npm i -g wrangler
wrangler pages publish .
```

## 📱 Device Requirements

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| Browser | Chrome 80+, Safari 14+ | Chrome 100+, Safari 16+ |
| HTTPS | Required | Required |
| Camera | 720p | 1080p |
| GPS | Any | High accuracy |
| IMU | Accelerometer + Gyro | + Magnetometer |

## 🎮 Usage

1. **Start AR**: Click the "Start AR" button to begin tracking
2. **Place Objects**: Tap "Place Object" to add 3D objects at your position
3. **Reset**: Use "Reset" to clear objects and re-anchor the world origin
4. **Debug Panel**: Toggle the debug panel to see sensor data

## 🔧 Configuration

### Adjusting Visual SLAM Parameters

In `slam-engine.js`, modify the `VisualSLAMModule` class:

```javascript
// Feature detection
this.orbDetector = new cv.ORB(500);  // Max features (reduce for performance)

// Keyframe timing
this.keyframeInterval = 500;  // ms between keyframes
this.maxKeyframes = 30;       // Maximum stored keyframes
```

### Adjusting Sensor Fusion

In `slam-engine.js`, modify the `SensorFusion` class:

```javascript
// Sensor weights
this.gpsWeight = 0.7;   // Higher = trust GPS more
this.slamWeight = 0.3;  // Higher = trust visual SLAM more

// Drift correction
this.driftCorrectionInterval = 30000;  // ms between corrections
```

## ⚡ Performance Optimization

### For Mobile Devices

1. **Reduce feature count**: Change ORB detector to 300 features
2. **Lower camera resolution**: Request 640x480 instead of 1280x720
3. **Skip frames**: Process every 2nd or 3rd frame for visual SLAM
4. **Disable feature visualization**: Comment out `featureViz.draw()` call

### Battery Optimization

- The system automatically pauses when the tab is hidden
- GPS polling reduces when stationary
- Consider adding a "power save" mode that reduces processing rate

## 🐛 Troubleshooting

### "OpenCV.js not loaded"

The OpenCV.js library loads asynchronously. Wait for the loading indicator or check console for errors. If persistent, try a different CDN:

```html
<script src="https://docs.opencv.org/4.5.5/opencv.js"></script>
```

### "Camera access denied"

- Ensure you're using HTTPS
- Check browser permissions
- On iOS, permissions must be requested from a user gesture

### "GPS not available"

- Ensure location services are enabled
- Check if the site has location permission
- GPS may not work indoors; the system will fall back to visual SLAM

### "Low tracking quality"

- Ensure good lighting
- Point camera at feature-rich areas (buildings, objects with texture)
- Avoid rapid movements

## 🗺️ Roadmap

### Phase 1 (Current)
- [x] Basic GPS positioning with Kalman filter
- [x] IMU orientation tracking
- [x] ORB feature detection
- [x] Optical flow tracking
- [x] Three.js AR rendering
- [x] Basic sensor fusion

### Phase 2 (Planned)
- [ ] Loop closure detection
- [ ] Improved pose estimation with RANSAC
- [ ] WebXR integration (where available)
- [ ] Persistent anchors with localStorage

### Phase 3 (Future)
- [ ] WebGPU acceleration
- [ ] Neural feature descriptors (TensorFlow.js)
- [ ] Multi-user synchronization
- [ ] Cloud anchor support

## 📚 References

Based on the architecture document concepts from:

- ORB-SLAM: "ORB-SLAM: A Versatile and Accurate Monocular SLAM System" (Mur-Artal et al., 2015)
- Visual-Inertial: "Visual-Inertial Monocular SLAM with Map Reuse" (Mur-Artal & Tardós, 2017)

Libraries used:
- [Three.js](https://threejs.org/) - 3D rendering
- [OpenCV.js](https://docs.opencv.org/4.x/d5/d10/tutorial_js_root.html) - Computer vision

## 📄 License

MIT License - Feel free to use and modify for your projects.

---

## 🔗 Integration with Your AR Architecture App

This prototype can be integrated with your existing AR architecture visualization app:

1. **Replace the simple house model** with your GLTF building models
2. **Use the GPS anchoring** for geo-located building placement
3. **Leverage the sensor fusion** for stable AR overlays during investor walkthroughs
4. **Add your Supabase backend** for storing anchor positions and building data

Example integration point in `app.js`:

```javascript
// Load your building model instead of simple shapes
async function loadBuildingModel(url) {
    const loader = new THREE.GLTFLoader();
    const gltf = await loader.loadAsync(url);
    return gltf.scene;
}

// Place building at GPS coordinates
async function placeBuilding(lat, lon, modelUrl) {
    const enu = slamEngine.gps.gpsToENU(lat, lon, 0);
    const model = await loadBuildingModel(modelUrl);
    model.position.set(enu.x, 0, -enu.y);
    threeScene.scene.add(model);
}
```
