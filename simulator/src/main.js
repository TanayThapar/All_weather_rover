import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ============================================================
// 1. RENDERER / SCENE / CAMERA
// ============================================================

const canvas = document.getElementById('sim-canvas');

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.15;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a0e05);
scene.fog = new THREE.FogExp2(0x1a0e05, 0.022);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.05, 300);
camera.position.set(-3, 2.5, -5);
camera.lookAt(0, 0, 0);

let cameraMode = 'follow';
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 2;
controls.maxDistance = 40;
controls.maxPolarAngle = Math.PI / 2.05;
controls.target.set(0, 0.3, 0);

// ============================================================
// 2. LIGHTING
// ============================================================

scene.add(new THREE.AmbientLight(0x6b4c22, 1.2));

const sun = new THREE.DirectionalLight(0xffd080, 3.5);
sun.position.set(15, 30, 10);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.near = 0.5;
sun.shadow.camera.far = 120;
sun.shadow.camera.left = -30;
sun.shadow.camera.right = 30;
sun.shadow.camera.top = 30;
sun.shadow.camera.bottom = -30;
sun.shadow.bias = -0.0005;
scene.add(sun);

scene.add(new THREE.HemisphereLight(0xff8c00, 0x3d1f00, 0.7));

const roverLight = new THREE.PointLight(0xfff0aa, 3.5, 6, 2);
scene.add(roverLight);

// ============================================================
// 3. TERRAIN – sandy desert + rocky outcrops
// ============================================================

const sandMat = new THREE.MeshStandardMaterial({ color: 0x9c6e3a, roughness: 0.98, metalness: 0.0 });
const floorMesh = new THREE.Mesh(new THREE.PlaneGeometry(120, 120, 60, 60), sandMat);
floorMesh.rotation.x = -Math.PI / 2;
floorMesh.receiveShadow = true;
scene.add(floorMesh);

const grid = new THREE.GridHelper(120, 60, 0x7a4a1a, 0x7a4a1a);
grid.material.opacity = 0.15;
grid.material.transparent = true;
scene.add(grid);

function makeRock(x, z, scale) {
  const geo = new THREE.DodecahedronGeometry(scale, 0);
  const mat = new THREE.MeshStandardMaterial({
    color: new THREE.Color().setHSL(0.07, 0.3, 0.25 + Math.random() * 0.15),
    roughness: 0.95, metalness: 0.05,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(x, scale * 0.6, z);
  mesh.rotation.set(Math.random() * 2, Math.random() * 2, Math.random() * 2);
  mesh.scale.y = 0.6 + Math.random() * 0.5;
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);
}

const rockSeeds = [
  [6,4,0.5],[-5,7,0.4],[4,-6,0.6],[-6,-4,0.35],[9,2,0.8],
  [-9,3,0.45],[2,10,0.55],[4,-11,0.7],[-4,9,0.4],[8,-5,0.5],
  [12,8,0.9],[-12,5,0.6],[-3,-12,0.5],[11,-9,0.7],[-10,-8,0.6],
];
rockSeeds.forEach(([x,z,s]) => makeRock(x,z,s));


// ============================================================
// 3B. DYNAMIC ALL-WEATHER SIMULATION SYSTEM
//     Implements atmospheric particulate scattering, optical degradation,
//     LiDAR beam attenuation, wheel traction drop, thermal contrast shifts
// ============================================================

let currentWeather = 'clear'; // 'clear' | 'fog' | 'sandstorm' | 'rain' | 'night'

const WEATHER_PROFILES = {
  clear: {
    name: '☀️ Clear',
    badgeClass: 'badge-blue',
    skyColor: 0x1a0e05,
    fogDensity: 0.022,
    fogColor: 0x1a0e05,
    sunIntensity: 3.5,
    sunColor: 0xffd080,
    ambientColor: 0x6b4c22,
    ambientIntensity: 1.2,
    traction: 0.85,
    lidarMaxRange: 30.0,
    lidarDropoutRate: 0.0,
    lidarNoiseSigma: 0.02,
    camFilter: 'sepia(1) saturate(3.5) hue-rotate(195deg) contrast(1.35)',
    camContrastBase: 0.65,
    radarRcsLoss: 1.0,
    rainActive: false,
    dustActive: false,
    lidarDesc: 'Optimal (30m, 0% dropout)',
    camDesc: 'Optimal visual + thermal',
    radarDesc: 'Primary sensor nominal',
    tractionDesc: 'High (μ = 0.85)',
  },
  fog: {
    name: '🌫️ Dense Fog',
    badgeClass: 'badge-purple',
    skyColor: 0x606a75,
    fogDensity: 0.095,
    fogColor: 0x788490,
    sunIntensity: 0.8,
    sunColor: 0xdde5ed,
    ambientColor: 0x7a8b9e,
    ambientIntensity: 1.8,
    traction: 0.65,
    lidarMaxRange: 7.0,          // Severe laser backscatter in dense aerosol
    lidarDropoutRate: 0.65,       // 65% optical beams absorbed / scattered
    lidarNoiseSigma: 0.12,
    camFilter: 'sepia(1) saturate(1.8) hue-rotate(200deg) contrast(0.7) blur(1.5px)',
    camContrastBase: 0.25,
    radarRcsLoss: 0.98,          // mmWave penetrates fog completely! (Key paper result)
    rainActive: false,
    dustActive: false,
    lidarDesc: 'CRITICAL (7m max, 65% dropout)',
    camDesc: 'Optical blackout, thermal degraded',
    radarDesc: 'Optimal penetration (77GHz)',
    tractionDesc: 'Damp ground (μ = 0.65)',
  },
  sandstorm: {
    name: '🌪️ Dust Storm',
    badgeClass: 'badge-orange',
    skyColor: 0x5a2d0c,
    fogDensity: 0.075,
    fogColor: 0x8a4515,
    sunIntensity: 0.9,
    sunColor: 0xffaa44,
    ambientColor: 0xaa5522,
    ambientIntensity: 2.0,
    traction: 0.45,             // Loose sandy slip
    lidarMaxRange: 9.0,         // Severe particulate occlusion
    lidarDropoutRate: 0.55,
    lidarNoiseSigma: 0.18,
    camFilter: 'sepia(1) saturate(4.0) hue-rotate(170deg) contrast(0.9) brightness(0.8)',
    camContrastBase: 0.35,
    radarRcsLoss: 0.95,         // Radar penetrates atmospheric dust
    rainActive: false,
    dustActive: true,
    lidarDesc: 'Severe clutter (9m max, 55% dropout)',
    camDesc: 'Aerosol scattering, low SNR',
    radarDesc: 'Optimal penetration (77GHz)',
    tractionDesc: 'Loose sand slip (μ = 0.45)',
  },
  rain: {
    name: '🌧️ Heavy Rain',
    badgeClass: 'badge-blue',
    skyColor: 0x101a26,
    fogDensity: 0.045,
    fogColor: 0x1c2b3a,
    sunIntensity: 1.4,
    sunColor: 0x99bbdd,
    ambientColor: 0x334a60,
    ambientIntensity: 1.5,
    traction: 0.38,             // Wet surface low friction
    lidarMaxRange: 14.0,        // Water droplet refraction
    lidarDropoutRate: 0.35,
    lidarNoiseSigma: 0.08,
    camFilter: 'sepia(1) saturate(3.0) hue-rotate(210deg) contrast(1.1) brightness(0.9)',
    camContrastBase: 0.50,
    radarRcsLoss: 0.88,         // Slight attenuation from precipitation
    rainActive: true,
    dustActive: false,
    lidarDesc: 'Droplet refraction (14m, 35% dropout)',
    camDesc: 'Specular noise, wet thermal blur',
    radarDesc: 'Minor rain clutter (Doppler valid)',
    tractionDesc: 'Low wet friction (μ = 0.38)',
  },
  night: {
    name: '🌑 Zero Light',
    badgeClass: 'badge-purple',
    skyColor: 0x020306,
    fogDensity: 0.025,
    fogColor: 0x020306,
    sunIntensity: 0.05,         // Near total darkness
    sunColor: 0x334466,
    ambientColor: 0x050810,
    ambientIntensity: 0.15,
    traction: 0.85,
    lidarMaxRange: 30.0,        // LiDAR active illumination unaffected by dark
    lidarDropoutRate: 0.0,
    lidarNoiseSigma: 0.02,
    camFilter: 'sepia(1) saturate(4.0) hue-rotate(195deg) contrast(1.6) brightness(1.2)', // Thermal infrared shines here!
    camContrastBase: 0.85,
    radarRcsLoss: 1.0,
    rainActive: false,
    dustActive: false,
    lidarDesc: 'Optimal (Active IR laser, 30m)',
    camDesc: 'Optical dark, Thermal IR EXCELLENT',
    radarDesc: 'Optimal (Independent of illumination)',
    tractionDesc: 'High (μ = 0.85)',
  }
};

// Particle Systems (Rain drops & Dust storm particles)
const RAIN_COUNT = 2500;
const rainGeo = new THREE.BufferGeometry();
const rainPositions = new Float32Array(RAIN_COUNT * 3);
for (let i = 0; i < RAIN_COUNT * 3; i += 3) {
  rainPositions[i]     = (Math.random() - 0.5) * 60;
  rainPositions[i + 1] = Math.random() * 25;
  rainPositions[i + 2] = (Math.random() - 0.5) * 60;
}
rainGeo.setAttribute('position', new THREE.BufferAttribute(rainPositions, 3));
const rainMat = new THREE.PointsMaterial({
  color: 0x88ccff,
  size: 0.15,
  transparent: true,
  opacity: 0.65,
});
const rainParticles = new THREE.Points(rainGeo, rainMat);
rainParticles.visible = false;
scene.add(rainParticles);

const DUST_COUNT = 1800;
const dustGeo = new THREE.BufferGeometry();
const dustPositions = new Float32Array(DUST_COUNT * 3);
for (let i = 0; i < DUST_COUNT * 3; i += 3) {
  dustPositions[i]     = (Math.random() - 0.5) * 50;
  dustPositions[i + 1] = Math.random() * 12;
  dustPositions[i + 2] = (Math.random() - 0.5) * 50;
}
dustGeo.setAttribute('position', new THREE.BufferAttribute(dustPositions, 3));
const dustMat = new THREE.PointsMaterial({
  color: 0xd47a2a,
  size: 0.35,
  transparent: true,
  opacity: 0.5,
});
const dustParticles = new THREE.Points(dustGeo, dustMat);
dustParticles.visible = false;
scene.add(dustParticles);

let contactMatRef = null;

function setWeather(weatherKey) {
  if (!WEATHER_PROFILES[weatherKey]) return;
  currentWeather = weatherKey;
  const p = WEATHER_PROFILES[weatherKey];

  // 1. Visual atmosphere
  scene.background.setHex(p.skyColor);
  scene.fog.color.setHex(p.fogColor);
  scene.fog.density = p.fogDensity;
  sun.intensity = p.sunIntensity;
  sun.color.setHex(p.sunColor);

  // 2. Weather particles
  rainParticles.visible = p.rainActive;
  dustParticles.visible = p.dustActive;

  // 3. Camera filter & contrast
  if (camCanvas) {
    camCanvas.style.filter = p.camFilter;
  }

  // 4. Update wheel-ground physics friction
  if (contactMatRef) {
    contactMatRef.friction = p.traction;
  }

  // 5. Update UI badge and buttons
  const badge = document.getElementById('weather-badge');
  if (badge) {
    badge.textContent = p.name;
    badge.className = 'badge ' + p.badgeClass;
  }
  document.querySelectorAll('.wbtn').forEach(b => {
    b.classList.toggle('active', b.dataset.weather === weatherKey);
  });

  // Update Degradation Info table
  const lEl = document.getElementById('w-lidar-impact');
  const cEl = document.getElementById('w-cam-impact');
  const rEl = document.getElementById('w-radar-impact');
  const tEl = document.getElementById('w-traction-impact');

  if (lEl) {
    lEl.textContent = p.lidarDesc;
    lEl.className = 'w-val ' + (p.lidarDropoutRate > 0.5 ? 'degraded' : p.lidarDropoutRate > 0.2 ? 'warning' : 'optimal');
  }
  if (cEl) {
    cEl.textContent = p.camDesc;
    cEl.className = 'w-val ' + (weatherKey === 'night' ? 'optimal' : weatherKey === 'fog' ? 'degraded' : 'warning');
  }
  if (rEl) {
    rEl.textContent = p.radarDesc;
    rEl.className = 'w-val optimal';
  }
  if (tEl) {
    tEl.textContent = p.tractionDesc;
    tEl.className = 'w-val ' + (p.traction < 0.5 ? 'degraded' : p.traction < 0.7 ? 'warning' : 'optimal');
  }
}

// ============================================================
// 4. PHYSICS WORLD
// ============================================================

const world = new CANNON.World({ gravity: new CANNON.Vec3(0, -9.82, 0) });
world.broadphase = new CANNON.SAPBroadphase(world);
world.allowSleep = true;

const groundMat = new CANNON.Material('ground');
const wheelMat  = new CANNON.Material('wheel');
contactMatRef = new CANNON.ContactMaterial(groundMat, wheelMat, {
  friction: 0.85,
  restitution: 0.01,
  contactEquationStiffness: 1e8,
  contactEquationRelaxation: 3,
  frictionEquationStiffness: 1e8,
});
world.addContactMaterial(contactMatRef);

const groundBody = new CANNON.Body({ type: CANNON.Body.STATIC, material: groundMat });
groundBody.addShape(new CANNON.Plane());
groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
world.addBody(groundBody);

// ============================================================
// 5. ROVER PHYSICS
//    All_weather_rover URDF exact dimensions:
//    base.xacro:   chassis  0.8 x 0.5 x 0.3 m (L x W x H)
//    wheels.xacro: radius=0.12  width=0.08
//                  wheelbase=0.60  track_width=0.56
//    Ackermann steering: front steer / rear drive
// ============================================================

const CHASSIS_L = 0.8;
const CHASSIS_W = 0.5;
const CHASSIS_H = 0.3;
const WHEEL_R   = 0.12;
const WHEEL_W   = 0.08;
const WHEELBASE = 0.60;
const TRACK     = 0.56;
const ROVER_MASS = 20;

const chassisBody = new CANNON.Body({ mass: ROVER_MASS });
chassisBody.addShape(new CANNON.Box(
  new CANNON.Vec3(CHASSIS_W / 2, CHASSIS_H / 2, CHASSIS_L / 2)
));
chassisBody.position.set(0, WHEEL_R + CHASSIS_H / 2 + 0.04, 0);
chassisBody.linearDamping  = 0.3;
chassisBody.angularDamping = 0.7;

const vehicle = new CANNON.RaycastVehicle({
  chassisBody,
  indexRightAxis:   0,
  indexUpAxis:      1,
  indexForwardAxis: 2,
});

const suspRestLen   = 0.08;
const suspStiffness = 120;
const suspDamping   = 8;
const suspMaxTravel = 0.12;
const rollInfluence = 0.01;

const HALF_TRACK = TRACK / 2;
const HALF_WB    = WHEELBASE / 2;

const wheelPositions = [
  { x:  HALF_TRACK, y: -CHASSIS_H / 2, z:  HALF_WB },
  { x: -HALF_TRACK, y: -CHASSIS_H / 2, z:  HALF_WB },
  { x:  HALF_TRACK, y: -CHASSIS_H / 2, z: -HALF_WB },
  { x: -HALF_TRACK, y: -CHASSIS_H / 2, z: -HALF_WB },
];

wheelPositions.forEach(pos => {
  vehicle.addWheel({
    radius: WHEEL_R,
    directionLocal:              new CANNON.Vec3(0, -1, 0),
    axleLocal:                   new CANNON.Vec3(-1, 0, 0),
    chassisConnectionPointLocal: new CANNON.Vec3(pos.x, pos.y, pos.z),
    suspensionRestLength:  suspRestLen,
    suspensionStiffness:   suspStiffness,
    dampingRelaxation:     suspDamping,
    dampingCompression:    suspDamping * 0.8,
    maxSuspensionTravel:   suspMaxTravel,
    frictionSlip:          2.8,
    rollInfluence,
    maxSuspensionForce:    1e4,
    customSlidingRotationalSpeed:     -30,
    useCustomSlidingRotationalSpeed:  true,
  });
});

vehicle.addToWorld(world);

const FRONT_RIGHT = 0;
const FRONT_LEFT  = 1;
const REAR_RIGHT  = 2;
const REAR_LEFT   = 3;

// ============================================================
// 6. THREE.JS ROVER MESHES
//    Accurate All_weather_rover visual:
//    dark-grey armoured chassis + all 5 sensor systems
// ============================================================

const roverGroup = new THREE.Group();

// Chassis body
const bodyMat = new THREE.MeshStandardMaterial({ color: 0x2b2e35, roughness: 0.6, metalness: 0.4 });
const bodyMesh = new THREE.Mesh(
  new THREE.BoxGeometry(CHASSIS_W, CHASSIS_H, CHASSIS_L),
  bodyMat
);
bodyMesh.castShadow = true;
roverGroup.add(bodyMesh);

// Top electronics plate
const topPlate = new THREE.Mesh(
  new THREE.BoxGeometry(CHASSIS_W * 0.85, 0.025, CHASSIS_L * 0.6),
  new THREE.MeshStandardMaterial({ color: 0x1e2126, roughness: 0.7, metalness: 0.5 })
);
topPlate.position.set(0, CHASSIS_H / 2 + 0.013, 0);
roverGroup.add(topPlate);

// Side skirts
[-1, 1].forEach(side => {
  const skirt = new THREE.Mesh(
    new THREE.BoxGeometry(0.015, CHASSIS_H * 0.5, CHASSIS_L * 0.95),
    new THREE.MeshStandardMaterial({ color: 0x1a1d22, roughness: 0.8, metalness: 0.5 })
  );
  skirt.position.set(side * (CHASSIS_W / 2 + 0.008), -CHASSIS_H * 0.12, 0);
  roverGroup.add(skirt);
});

// Front bumper bar
const bumperBar = new THREE.Mesh(
  new THREE.BoxGeometry(CHASSIS_W + 0.05, 0.04, 0.04),
  new THREE.MeshStandardMaterial({ color: 0x444851, roughness: 0.5, metalness: 0.6 })
);
bumperBar.position.set(0, -CHASSIS_H * 0.2, CHASSIS_L / 2 + 0.02);
roverGroup.add(bumperBar);

// SENSOR 1: Front mmWave Radar pod
const radarPod = new THREE.Mesh(
  new THREE.BoxGeometry(0.12, 0.07, 0.04),
  new THREE.MeshStandardMaterial({ color: 0x0d1c3a, roughness: 0.3, metalness: 0.9 })
);
radarPod.position.set(0, -CHASSIS_H * 0.1, CHASSIS_L / 2 + 0.04);
roverGroup.add(radarPod);

const radarGlow = new THREE.Mesh(
  new THREE.PlaneGeometry(0.10, 0.05),
  new THREE.MeshStandardMaterial({ color: 0x00aaff, emissive: 0x0066ff, emissiveIntensity: 2.5, side: THREE.FrontSide })
);
radarGlow.position.set(0, -CHASSIS_H * 0.1, CHASSIS_L / 2 + 0.062);
roverGroup.add(radarGlow);

// SENSOR 2: LiDAR mast (centre top)
const mastPole = new THREE.Mesh(
  new THREE.CylinderGeometry(0.012, 0.012, 0.25, 8),
  new THREE.MeshStandardMaterial({ color: 0x3a3d44, roughness: 0.7, metalness: 0.5 })
);
mastPole.position.set(0, CHASSIS_H / 2 + 0.125, 0.05);
roverGroup.add(mastPole);

const lidarHead = new THREE.Mesh(
  new THREE.CylinderGeometry(0.05, 0.04, 0.06, 16),
  new THREE.MeshStandardMaterial({ color: 0x111316, roughness: 0.4, metalness: 0.7 })
);
lidarHead.position.set(0, CHASSIS_H / 2 + 0.265, 0.05);
roverGroup.add(lidarHead);

const lidarRing = new THREE.Mesh(
  new THREE.TorusGeometry(0.045, 0.006, 6, 24),
  new THREE.MeshStandardMaterial({ color: 0x00ff44, emissive: 0x00ff44, emissiveIntensity: 3 })
);
lidarRing.position.copy(lidarHead.position);
lidarRing.rotation.x = Math.PI / 2;
roverGroup.add(lidarRing);

// SENSOR 3: Thermal IR camera (front-left)
const thermalBox = new THREE.Mesh(
  new THREE.BoxGeometry(0.06, 0.04, 0.05),
  new THREE.MeshStandardMaterial({ color: 0x222529, roughness: 0.5, metalness: 0.6 })
);
thermalBox.position.set(-CHASSIS_W * 0.3, CHASSIS_H / 2 + 0.02, CHASSIS_L * 0.3);
roverGroup.add(thermalBox);
const thermalLens = new THREE.Mesh(
  new THREE.CylinderGeometry(0.014, 0.014, 0.02, 12),
  new THREE.MeshStandardMaterial({ color: 0xff6600, emissive: 0xff3300, emissiveIntensity: 1.5, roughness: 0.2, metalness: 0.8 })
);
thermalLens.rotation.x = Math.PI / 2;
thermalLens.position.set(-CHASSIS_W * 0.3, CHASSIS_H / 2 + 0.02, CHASSIS_L * 0.3 + 0.035);
roverGroup.add(thermalLens);

// SENSOR 4: GPS mast (rear-right)
const gpsPole = new THREE.Mesh(
  new THREE.CylinderGeometry(0.008, 0.008, 0.20, 6),
  new THREE.MeshStandardMaterial({ color: 0x888888, roughness: 0.6, metalness: 0.6 })
);
gpsPole.position.set(CHASSIS_W * 0.35, CHASSIS_H / 2 + 0.10, -CHASSIS_L * 0.35);
roverGroup.add(gpsPole);
const gpsAntenna = new THREE.Mesh(
  new THREE.SphereGeometry(0.018, 8, 8),
  new THREE.MeshStandardMaterial({ color: 0xffcc00, emissive: 0xffaa00, emissiveIntensity: 1.0 })
);
gpsAntenna.position.set(CHASSIS_W * 0.35, CHASSIS_H / 2 + 0.21, -CHASSIS_L * 0.35);
roverGroup.add(gpsAntenna);

// SENSOR 5: IMU (centre chassis top)
const imuBox = new THREE.Mesh(
  new THREE.BoxGeometry(0.04, 0.02, 0.04),
  new THREE.MeshStandardMaterial({ color: 0x1e88e5, roughness: 0.5, metalness: 0.7 })
);
imuBox.position.set(0, CHASSIS_H / 2 + 0.01, 0);
roverGroup.add(imuBox);

// Headlamps (front emissive)
[-1, 1].forEach(side => {
  const lamp = new THREE.Mesh(
    new THREE.BoxGeometry(0.05, 0.03, 0.02),
    new THREE.MeshStandardMaterial({ color: 0xfff8e1, emissive: 0xfff8e1, emissiveIntensity: 3 })
  );
  lamp.position.set(side * CHASSIS_W * 0.35, -CHASSIS_H * 0.1, CHASSIS_L / 2 + 0.01);
  roverGroup.add(lamp);
});

scene.add(roverGroup);

// Wheel meshes
const wheelMeshes = vehicle.wheelInfos.map((_, i) => {
  const g = new THREE.Group();

  const tyre = new THREE.Mesh(
    new THREE.CylinderGeometry(WHEEL_R, WHEEL_R, WHEEL_W, 24),
    new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.97, metalness: 0.0 })
  );
  tyre.rotation.z = Math.PI / 2;
  tyre.castShadow = true;
  g.add(tyre);

  const tread = new THREE.Mesh(
    new THREE.TorusGeometry(WHEEL_R, 0.012, 6, 18),
    new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.95 })
  );
  tread.rotation.y = Math.PI / 2;
  g.add(tread);

  const rim = new THREE.Mesh(
    new THREE.CylinderGeometry(WHEEL_R * 0.55, WHEEL_R * 0.55, WHEEL_W + 0.004, 12),
    new THREE.MeshStandardMaterial({ color: 0x8a8f98, roughness: 0.3, metalness: 0.9 })
  );
  rim.rotation.z = Math.PI / 2;
  g.add(rim);

  const hub = new THREE.Mesh(
    new THREE.CylinderGeometry(0.018, 0.018, WHEEL_W + 0.01, 6),
    new THREE.MeshStandardMaterial({ color: 0xff6d00, roughness: 0.4, metalness: 0.7 })
  );
  hub.rotation.z = Math.PI / 2;
  g.add(hub);

  scene.add(g);
  return g;
});

// ============================================================
// 7. RICH MULTI-OBJECT MAP ENVIRONMENT & OBSTACLES
//    - Concrete barriers & perimeter walls
//    - Industrial cargo containers / crates
//    - Weathered desert boulder clusters
//    - Slanted climbable rubble ramps
//    - Road pylons / caution markers
// ============================================================

const obstacles = [];

// Helper: Add Generic Box Obstacle
function addBoxObstacle(x, z, w, h, l, mass, colorHex, roughness = 0.7, metalness = 0.3, rotY = 0) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(w, h, l),
    new THREE.MeshStandardMaterial({ color: colorHex, roughness, metalness })
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);

  const body = new CANNON.Body({
    mass: mass,
    shape: new CANNON.Box(new CANNON.Vec3(w / 2, h / 2, l / 2)),
    position: new CANNON.Vec3(x, h / 2 + 0.02, z),
    material: groundMat,
  });
  if (rotY !== 0) {
    body.quaternion.setFromEuler(0, rotY, 0);
  }
  world.addBody(body);
  obstacles.push({ mesh, body });
  return { mesh, body };
}

// Helper: Add Boulder / Rock Obstacle
function addRockObstacle(x, z, r, mass = 30, rotY = 0) {
  const geo = new THREE.DodecahedronGeometry(r, 1);
  const hue = 0.06 + (Math.random() - 0.5) * 0.03;
  const mat = new THREE.MeshStandardMaterial({
    color: new THREE.Color().setHSL(hue, 0.3, 0.28 + Math.random() * 0.1),
    roughness: 0.95,
    metalness: 0.05
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.scale.set(1.0 + Math.random() * 0.3, 0.7 + Math.random() * 0.4, 1.0 + Math.random() * 0.3);
  scene.add(mesh);

  const body = new CANNON.Body({
    mass: mass,
    shape: new CANNON.Sphere(r * 0.85),
    position: new CANNON.Vec3(x, r * 0.65 + 0.01, z),
    material: groundMat,
  });
  if (rotY !== 0) body.quaternion.setFromEuler(0, rotY, 0);
  world.addBody(body);
  obstacles.push({ mesh, body });
  return { mesh, body };
}

// Helper: Add Traffic / Hazard Pylon
function addPylonObstacle(x, z) {
  const group = new THREE.Group();
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(0.16, 0.55, 12),
    new THREE.MeshStandardMaterial({ color: 0xff5500, roughness: 0.4, metalness: 0.1 })
  );
  cone.position.y = 0.275;
  cone.castShadow = true;
  group.add(cone);

  const stripe = new THREE.Mesh(
    new THREE.CylinderGeometry(0.09, 0.12, 0.14, 12),
    new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.3 })
  );
  stripe.position.y = 0.28;
  group.add(stripe);
  scene.add(group);

  const body = new CANNON.Body({
    mass: 3.5,
    shape: new CANNON.Cylinder(0.05, 0.16, 0.55, 8),
    position: new CANNON.Vec3(x, 0.28, z),
    material: groundMat,
  });
  world.addBody(body);
  obstacles.push({ mesh: group, body });
}

// Helper: Add Cargo Shipping Container
function addShippingContainer(x, z, rotY = 0, colorHex = 0xb71c1c) {
  const w = 1.6, h = 1.4, l = 3.8;
  const container = addBoxObstacle(x, z, w, h, l, 0, colorHex, 0.6, 0.4, rotY); // Static mass = 0
  
  // Add container corrugated ribs visually
  [-1, 1].forEach(side => {
    for (let r = -l / 2 + 0.3; r < l / 2 - 0.2; r += 0.4) {
      const rib = new THREE.Mesh(
        new THREE.BoxGeometry(0.04, h * 0.9, 0.08),
        new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.8 })
      );
      rib.position.set(side * (w / 2 + 0.01), 0, r);
      container.mesh.add(rib);
    }
  });
}

// Helper: Add Slanted Terrain Ramp (Climbable elevation)
function addRampObstacle(x, z, w, length, height, rotY = 0) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(w, 0.15, length),
    new THREE.MeshStandardMaterial({ color: 0x546e7a, roughness: 0.9, metalness: 0.2 })
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);

  const body = new CANNON.Body({
    mass: 0, // Static ramp
    shape: new CANNON.Box(new CANNON.Vec3(w / 2, 0.08, length / 2)),
    position: new CANNON.Vec3(x, height / 2 + 0.02, z),
    material: groundMat,
  });
  
  const pitchAngle = Math.atan2(height, length);
  const qPitch = new CANNON.Quaternion();
  qPitch.setFromAxisAngle(new CANNON.Vec3(1, 0, 0), pitchAngle);
  const qYaw = new CANNON.Quaternion();
  qYaw.setFromAxisAngle(new CANNON.Vec3(0, 1, 0), rotY);
  body.quaternion = qYaw.mult(qPitch);

  world.addBody(body);
  obstacles.push({ mesh, body });
}

// ============================================================
// MAP GENERATION: CURATED ALL-WEATHER TESTING GROUND
// ============================================================

function buildRichTestMap() {
  // 1. Perimeter Boundary Wall (40m x 40m area bounds)
  const WALL_THICK = 0.5;
  const WALL_H = 1.2;
  const B_SIZE = 36;
  addBoxObstacle(0, -B_SIZE / 2, B_SIZE, WALL_H, WALL_THICK, 0, 0x37474f, 0.9); // North
  addBoxObstacle(0,  B_SIZE / 2, B_SIZE, WALL_H, WALL_THICK, 0, 0x37474f, 0.9); // South
  addBoxObstacle(-B_SIZE / 2, 0, WALL_THICK, WALL_H, B_SIZE, 0, 0x37474f, 0.9); // West
  addBoxObstacle( B_SIZE / 2, 0, WALL_THICK, WALL_H, B_SIZE, 0, 0x37474f, 0.9); // East

  // 2. Industrial Freight Depot (North-East Zone)
  addShippingContainer(7, 8, 0, 0x1565c0);      // Blue Container
  addShippingContainer(11, 8, 0, 0xc62828);     // Red Container
  addShippingContainer(9, 13, Math.PI / 2, 0x2e7d32); // Green Container
  // Heavy wooden crates
  addBoxObstacle(6.5, 4.5, 0.8, 0.7, 0.8, 25, 0x8d6e63, 0.85);
  addBoxObstacle(8.2, 4.2, 0.6, 0.6, 0.6, 18, 0x6d4c41, 0.85);
  addBoxObstacle(7.2, 4.2, 0.6, 1.1, 0.6, 32, 0x795548, 0.85);

  // 3. Rocky Canyon Field & Boulder Mazes (North-West Zone)
  const rockPlacements = [
    [-6, 6, 0.65], [-9, 7, 0.85], [-7, 10, 0.55], [-11, 11, 1.1],
    [-5, 12, 0.75], [-12, 5, 0.8], [-8, 3.5, 0.6], [-13, 8.5, 0.9],
    [-4, 8.5, 0.45], [-10, 2.5, 0.7], [-7, 14, 0.95], [-12, 13, 0.85]
  ];
  rockPlacements.forEach(([rx, rz, rad]) => addRockObstacle(rx, rz, rad, 45));

  // 4. Urban Alleyway / Barrier Slalom (South-East Zone)
  for (let i = -1; i <= 3; i++) {
    addBoxObstacle(4 + (i % 2) * 3.5, -4 - i * 2.8, 2.6, 0.75, 0.4, 0, 0x546e7a, 0.8, 0.2, (i % 2) * 0.3);
  }
  // Slalom Hazard Pylons
  [
    [3.5, -3.2], [7.5, -5.8], [4.0, -8.5], [8.0, -11.2], [5.5, -13.5]
  ].forEach(([px, pz]) => addPylonObstacle(px, pz));

  // 5. Rubble Climb & Elevated Ramps (South-West Zone)
  addRampObstacle(-7, -7, 2.2, 4.5, 0.65, 0);              // Incline ramp
  addBoxObstacle(-7, -10.5, 2.5, 0.65, 2.5, 0, 0x455a64); // Platform
  addRampObstacle(-7, -14, 2.2, 4.5, 0.65, Math.PI);       // Decline ramp
  // Scatter rocks near ramp
  addRockObstacle(-4.5, -6.5, 0.5, 30);
  addRockObstacle(-9.5, -8.5, 0.6, 35);
  addRockObstacle(-4.5, -12.5, 0.55, 30);

  // 6. Central Arena Scattered Debris (Near Rover Spawn with safe clearing)
  addBoxObstacle(3.0, 2.2, 0.5, 0.5, 0.5, 15, 0xd7ccc8);
  addBoxObstacle(-2.8, 2.8, 0.6, 0.45, 0.6, 12, 0x8d6e63);
  addRockObstacle(-2.5, -2.8, 0.45, 20);
  addRockObstacle(3.2, -2.5, 0.4, 18);
  addPylonObstacle(1.8, 3.5);
  addPylonObstacle(-1.8, -3.5);
}

buildRichTestMap();

// Interactive dynamic obstacle spawn (when pressing 'C')
function addObstacle(x, z) {
  const type = Math.random();
  if (type < 0.35) {
    addBoxObstacle(x, z, 0.4 + Math.random() * 0.4, 0.4 + Math.random() * 0.4, 0.4 + Math.random() * 0.4, 12, 0x8d6e63);
  } else if (type < 0.7) {
    addRockObstacle(x, z, 0.25 + Math.random() * 0.35, 25);
  } else {
    addPylonObstacle(x, z);
  }
}

// ============================================================
// 8. AUTONOMOUS FRONTIER EXPLORATION STATE & INPUT
// ============================================================

let isAutoNav = false;
let autoNavState = 'IDLE'; // 'SEARCHING', 'NAVIGATING', 'OBSTACLE_AVOID', 'REVERSING', 'REACHED'
let currentGoal = null; // { x, z }
let currentPath = [];   // Array of { x, z }
let navStateTimer = 0;
let reverseTimer = 0;
let reverseSteer = 0;
let autoThrottle = 0;
let autoSteer = 0;
let exploredPercent = 0;
let frontiersCount = 0;

// SLAM Occupancy Grid Map Settings
const MAP_SIZE_M = 40;        // 40m x 40m
const MAP_RES = 0.5;          // 0.5m per cell
const MAP_DIM = Math.round(MAP_SIZE_M / MAP_RES); // 80 x 80 cells
// Cells: 0 = UNKNOWN (gray), 1 = FREE (black/dark green), 2 = OCCUPIED (bright red/orange), 3 = FRONTIER (cyan)
const slamGrid = new Uint8Array(MAP_DIM * MAP_DIM); // default 0

function worldToGrid(x, z) {
  const gx = Math.floor((x + MAP_SIZE_M / 2) / MAP_RES);
  const gz = Math.floor((z + MAP_SIZE_M / 2) / MAP_RES);
  if (gx >= 0 && gx < MAP_DIM && gz >= 0 && gz < MAP_DIM) {
    return { gx, gz };
  }
  return null;
}

function gridToWorld(gx, gz) {
  return {
    x: (gx + 0.5) * MAP_RES - MAP_SIZE_M / 2,
    z: (gz + 0.5) * MAP_RES - MAP_SIZE_M / 2,
  };
}

function setAutoMode(enabled) {
  isAutoNav = enabled;
  const badge = document.getElementById('auto-mode-badge');
  const btn = document.getElementById('btn-toggle-auto');
  if (badge) {
    badge.textContent = isAutoNav ? '⚡ AUTONOMOUS' : '🕹️ MANUAL';
    badge.className = isAutoNav ? 'badge badge-green' : 'badge badge-purple';
  }
  if (btn) {
    btn.classList.toggle('active', isAutoNav);
    btn.textContent = isAutoNav ? '🛑 STOP AUTONOMOUS EXPLORATION' : '⚡ TOGGLE AUTONOMOUS EXPLORATION';
  }
}

const keys = {
  ArrowUp: false, ArrowDown: false, ArrowLeft: false, ArrowRight: false,
  w: false, s: false, a: false, d: false,
};

document.addEventListener('keydown', e => {
  if (e.key in keys) { keys[e.key] = true; e.preventDefault(); }
  if (e.key === 'c' || e.key === 'C') {
    const p = chassisBody.position;
    addObstacle(p.x + (Math.random() - 0.5) * 8, p.z + (Math.random() - 0.5) * 8);
  }
  if (e.key === 'r' || e.key === 'R') resetRover();
  if (e.key === 'e' || e.key === 'E') setAutoMode(!isAutoNav);
  if (e.key === '1') setCameraMode('orbit');
  if (e.key === '2') setCameraMode('follow');
  if (e.key === '3') setCameraMode('topdown');
  if (e.key === '4') setCameraMode('hood');
});
document.addEventListener('keyup', e => {
  if (e.key in keys) { keys[e.key] = false; e.preventDefault(); }
});

function resetRover() {
  chassisBody.position.set(0, WHEEL_R + CHASSIS_H / 2 + 0.08, 0);
  chassisBody.velocity.set(0, 0, 0);
  chassisBody.angularVelocity.set(0, 0, 0);
  chassisBody.quaternion.setFromEuler(0, 0, 0);
}

// ============================================================
// 9. HUD
// ============================================================

const statFps   = document.getElementById('stat-fps');
const statSpeed = document.getElementById('stat-speed');
const statHead  = document.getElementById('stat-heading');
const statX     = document.getElementById('stat-x');
const statZ     = document.getElementById('stat-z');
const mL = document.getElementById('motor-left');
const mR = document.getElementById('motor-right');
const keyEls = {
  ArrowUp:    document.getElementById('key-up'),
  ArrowDown:  document.getElementById('key-down'),
  ArrowLeft:  document.getElementById('key-left'),
  ArrowRight: document.getElementById('key-right'),
};

function setMotorBar(el, v, max) {
  const pct = Math.abs(v) / max;
  el.style.width = (pct * 46).toFixed(1) + '%';
  if (v > 0) {
    el.style.left = 'calc(50% - ' + (pct * 46).toFixed(1) + '%)';
    el.style.background = 'linear-gradient(90deg,#4a9eff,#00e676)';
  } else if (v < 0) {
    el.style.left = '50%';
    el.style.background = 'linear-gradient(90deg,#ff5252,#ff9800)';
  } else {
    el.style.width = '2px';
    el.style.left = 'calc(50% - 1px)';
    el.style.background = '#4a9eff';
  }
}

// ============================================================
// 10. ANIMATION LOOP
// ============================================================

// Engine force scaled to rover mass (20 kg) and wheel radius (0.12 m).
// 18 N per rear wheel * 2 = 36 N total -> 36/20 = 1.8 m/s^2 max accel (realistic)
const MAX_FORCE   = 18;
const MAX_STEER   = 0.38;
const BRAKE_FORCE = 6;
const fixedStep   = 1 / 60;
const clock       = new THREE.Clock();
let lastFpsTime = 0, frameCount = 0;
let throttle = 0;

const camOffset = {
  follow: new THREE.Vector3(0, 0.5, -1.4),
  hood:   new THREE.Vector3(0, 0.22, 0.38),
};
const camModeBadge = document.getElementById('cam-mode-badge');

function setCameraMode(mode) {
  cameraMode = mode;
  controls.enabled = (mode === 'orbit');
  if (camModeBadge) {
    camModeBadge.textContent = {
      orbit: '🔭 Orbit',
      follow: '📷 Follow',
      topdown: '🗺 Top-Down',
      hood: '👀 Hood-Cam',
    }[mode] || mode;
  }
}
setCameraMode('follow');

function animate(timestamp) {
  requestAnimationFrame(animate);
  const delta = Math.min(clock.getDelta(), 0.1);

  // Keyboard manual inputs
  const manualFwd = keys.ArrowUp   || keys.w;
  const manualBwd = keys.ArrowDown || keys.s;
  const manualLft = keys.ArrowLeft || keys.a;
  const manualRgt = keys.ArrowRight|| keys.d;
  const isManualActive = manualFwd || manualBwd || manualLft || manualRgt;

  // Cancel autonomy if user manually drives
  if (isManualActive && isAutoNav) {
    setAutoMode(false);
  }

  let fwd = manualFwd;
  let bwd = manualBwd;
  let lft = manualLft;
  let rgt = manualRgt;
  let steer = 0;
  let targetThrottle = 0;
  let braking = false;

  if (isAutoNav) {
    updateAutonomousController(delta);
    targetThrottle = autoThrottle;
    steer = autoSteer;
    braking = (Math.abs(autoThrottle) < 0.05);
    fwd = autoThrottle > 0.1;
    bwd = autoThrottle < -0.1;
    lft = autoSteer > 0.05;
    rgt = autoSteer < -0.05;
  } else {
    targetThrottle = fwd ? 1 : bwd ? -0.6 : 0;
    if (lft) steer =  MAX_STEER;
    if (rgt) steer = -MAX_STEER;
    braking = !fwd && !bwd;
  }

  throttle += (targetThrottle - throttle) * 0.12;
  if (Math.abs(throttle) < 0.005) throttle = 0;

  const engineForce = -throttle * MAX_FORCE;

  vehicle.setSteeringValue(steer, FRONT_RIGHT);
  vehicle.setSteeringValue(steer, FRONT_LEFT);
  vehicle.applyEngineForce(engineForce, REAR_RIGHT);
  vehicle.applyEngineForce(engineForce, REAR_LEFT);
  vehicle.applyEngineForce(0, FRONT_RIGHT);
  vehicle.applyEngineForce(0, FRONT_LEFT);

  if (braking) {
    [0,1,2,3].forEach(i => vehicle.setBrake(BRAKE_FORCE, i));
  } else {
    [0,1,2,3].forEach(i => vehicle.setBrake(0, i));
  }

  world.step(fixedStep, delta, 3);

  roverGroup.position.copy(chassisBody.position);
  roverGroup.quaternion.set(
    chassisBody.quaternion.x, chassisBody.quaternion.y,
    chassisBody.quaternion.z, chassisBody.quaternion.w
  );

  vehicle.wheelInfos.forEach((wheel, i) => {
    vehicle.updateWheelTransform(i);
    const t = wheel.worldTransform;
    wheelMeshes[i].position.copy(t.position);
    wheelMeshes[i].quaternion.set(t.quaternion.x, t.quaternion.y, t.quaternion.z, t.quaternion.w);
  });

  obstacles.forEach(({ mesh, body }) => {
    mesh.position.copy(body.position);
    mesh.quaternion.set(body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w);
  });

  const rp = chassisBody.position;
  roverLight.position.set(rp.x, rp.y + 0.8, rp.z);
  updateWeatherParticles(delta, rp);

  const chassisQuat = new THREE.Quaternion(
    chassisBody.quaternion.x, chassisBody.quaternion.y,
    chassisBody.quaternion.z, chassisBody.quaternion.w
  );

  if (cameraMode === 'orbit') {
    controls.target.lerp(new THREE.Vector3(rp.x, rp.y + 0.3, rp.z), 0.06);
    controls.update();
  } else if (cameraMode === 'follow') {
    const offset = camOffset.follow.clone().applyQuaternion(chassisQuat);
    camera.position.lerp(new THREE.Vector3(rp.x + offset.x, rp.y + offset.y, rp.z + offset.z), 0.08);
    camera.lookAt(rp.x, rp.y + 0.15, rp.z);
  } else if (cameraMode === 'topdown') {
    camera.position.lerp(new THREE.Vector3(rp.x, rp.y + 6, rp.z), 0.08);
    camera.lookAt(rp.x, rp.y, rp.z);
  } else if (cameraMode === 'hood') {
    const offset = camOffset.hood.clone().applyQuaternion(chassisQuat);
    camera.position.lerp(new THREE.Vector3(rp.x + offset.x, rp.y + offset.y, rp.z + offset.z), 0.25);
    const fwdOff = new THREE.Vector3(0, 0.1, 4).applyQuaternion(chassisQuat);
    camera.lookAt(rp.x + fwdOff.x, rp.y + fwdOff.y, rp.z + fwdOff.z);
  }

  // Animate sensor effects
  const pulse = 0.5 + 0.5 * Math.sin(timestamp * 0.008);
  lidarRing.material.emissiveIntensity = 2 + pulse * 2;
  radarGlow.material.emissiveIntensity = 1.5 + Math.sin(timestamp * 0.025) * 1.2;

  // HUD
  frameCount++;
  if (timestamp - lastFpsTime >= 500) {
    const fps = Math.round(frameCount / ((timestamp - lastFpsTime) / 1000));
    statFps.textContent = fps;
    lastFpsTime = timestamp;
    frameCount = 0;
  }

  const vel = chassisBody.velocity;
  const speed = Math.sqrt(vel.x * vel.x + vel.z * vel.z);
  statSpeed.textContent = speed.toFixed(2) + ' m/s';

  const euler = new THREE.Euler().setFromQuaternion(chassisQuat, 'YXZ');
  statHead.textContent = (((THREE.MathUtils.radToDeg(euler.y) % 360) + 360) % 360).toFixed(1) + '°';
  statX.textContent = rp.x.toFixed(2);
  statZ.textContent = rp.z.toFixed(2);

  setMotorBar(mL, -throttle, 1);
  setMotorBar(mR, -throttle, 1);

  keyEls.ArrowUp.classList.toggle('active', fwd);
  keyEls.ArrowDown.classList.toggle('active', bwd);
  keyEls.ArrowLeft.classList.toggle('active', lft);
  keyEls.ArrowRight.classList.toggle('active', rgt);

  renderer.render(scene, camera);
}

// ============================================================
// 11. RESIZE
// ============================================================

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
});

animate(0);

// ============================================================
// 12. SENSOR PANEL – tab switching
// ============================================================

document.querySelectorAll('.stab').forEach(btn => {
  btn.addEventListener('click', () => {
    const tab = btn.dataset.tab;
    document.querySelectorAll('.stab').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.sensor-tab-content').forEach(c => c.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById('tab-' + tab).classList.add('active');
  });
});

// ============================================================
// 13. SENSOR VISUALIZATIONS
// ============================================================

// --- Shared obstacle list for sensors (maintained separately from scene.traverse) ---
const sceneObstacles = obstacles; // already declared above

// --- GPS reference origin (Dubai desert approx) ---
const GPS_LAT0 = 25.2048;
const GPS_LON0 = 55.2708;
const M_PER_DEG_LAT = 111320;
const M_PER_DEG_LON = 111320 * Math.cos(GPS_LAT0 * Math.PI / 180);

// ================================================================
// 13A. LIDAR – 2D polar occupancy canvas
//      72 azimuth rays × 4 elevation channels = 288 rays / frame
//      Rendered as a top-down scan with rover at centre.
// ================================================================

const lidarCanvas = document.getElementById('lidar-canvas');
const lidarCtx    = lidarCanvas.getContext('2d');
const LW = lidarCanvas.width;   // 322
const LH = lidarCanvas.height;  // 280
const LCX = LW / 2, LCY = LH / 2;
const LIDAR_MAX_R = 30;         // metres
const LIDAR_PX_PER_M = (Math.min(LW, LH) / 2 - 14) / LIDAR_MAX_R;
const lidarRaycaster = new THREE.Raycaster();
lidarRaycaster.far = LIDAR_MAX_R;

// Accumulate points across frames for persistence (like a real spinning LiDAR)
const LIDAR_ACCUMULATE_FRAMES = 4;
let lidarAccum = []; // [{ lx, ly, intensity }]

// LiDAR noise sigma (metres) – 0.02m like VLP-16
const LIDAR_SIGMA = 0.02;

const LIDAR_ELEVATIONS = [-0.20, -0.08, 0.08, 0.20]; // radians (~4 vertical channels)
const LIDAR_NUM_AZ = 72; // 360 / 72 = 5° horizontal resolution

function drawLidar() {
  // ---- Raycast this frame ----
  const rp = chassisBody.position;
  const rq = chassisBody.quaternion;
  const mastH = rp.y + (CHASSIS_H / 2) + 0.25; // mast height in world
  const mastPos = new THREE.Vector3(rp.x, mastH, rp.z);

  // Rover's yaw (heading) to rotate scan into rover-local frame for the map
  const chassisQ = new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w);
  const chassisYaw = new THREE.Euler().setFromQuaternion(chassisQ, 'YXZ').y;

  // All scene meshes that aren't part of the rover (include rocks + obstacles)
  const targets = [];
  scene.traverse(obj => {
    if (obj.isMesh && obj !== floorMesh) {
      // Check it's not part of rover or wheel groups
      let inRover = false;
      let p = obj.parent;
      while (p) { if (p === roverGroup) { inRover = true; break; } p = p.parent; }
      if (!inRover) {
        let inWheel = false;
        for (const wg of wheelMeshes) {
          let p2 = obj.parent;
          while (p2) { if (p2 === wg) { inWheel = true; break; } p2 = p2.parent; }
        }
        if (!inWheel) targets.push(obj);
      }
    }
  });

  const newPts = [];
  for (let ei = 0; ei < LIDAR_ELEVATIONS.length; ei++) {
    const elev = LIDAR_ELEVATIONS[ei];
    for (let ai = 0; ai < LIDAR_NUM_AZ; ai++) {
      const worldAz = (ai / LIDAR_NUM_AZ) * Math.PI * 2;
      const cosEl = Math.cos(elev);
      const dir = new THREE.Vector3(
        Math.cos(worldAz) * cosEl,
        Math.sin(elev),
        Math.sin(worldAz) * cosEl
      ).normalize();

      lidarRaycaster.set(mastPos, dir);
      const hits = lidarRaycaster.intersectObjects(targets, true);
      if (hits.length > 0) {
        const wp = WEATHER_PROFILES[currentWeather] || WEATHER_PROFILES.clear;
        // Weather degradation: dropout & reduced effective range
        if (Math.random() < wp.lidarDropoutRate) continue; // Beam absorbed/scattered by aerosol/rain

        let dist = hits[0].distance + (Math.random() + Math.random() - 1) * wp.lidarNoiseSigma;
        if (dist < 0.1 || dist > wp.lidarMaxRange) continue;

        // Convert to rover-local coordinates
        const localAz = worldAz - chassisYaw - Math.PI * 0.5;
        const lx = LCX + Math.cos(localAz) * dist * LIDAR_PX_PER_M;
        const ly = LCY - Math.sin(localAz) * dist * LIDAR_PX_PER_M;
        newPts.push({ lx, ly, intensity: Math.max(0.2, (1.0 - ei * 0.15) * (1.0 - dist / wp.lidarMaxRange)) });
      }
    }
  }

  // Accumulate with decay
  lidarAccum = lidarAccum
    .map(p => ({ ...p, intensity: p.intensity * 0.55 }))
    .filter(p => p.intensity > 0.08)
    .concat(newPts);

  // Update SLAM Global Occupancy Grid with new scan returns
  if (typeof updateSlamGrid === 'function') {
    updateSlamGrid(mastPos, newPts.map(p => ({
      dist: Math.hypot((p.lx - LCX) / LIDAR_PX_PER_M, (LCY - p.ly) / LIDAR_PX_PER_M),
      dir: new THREE.Vector3(
        Math.cos(Math.atan2(LCX - p.lx, LCY - p.ly) + chassisYaw + Math.PI * 0.5),
        0,
        Math.sin(Math.atan2(LCX - p.lx, LCY - p.ly) + chassisYaw + Math.PI * 0.5)
      ).normalize()
    })));
  }

  // ---- Draw ----
  // Black background
  lidarCtx.fillStyle = '#020d04';
  lidarCtx.fillRect(0, 0, LW, LH);

  // Clip to circle
  lidarCtx.save();
  lidarCtx.beginPath();
  lidarCtx.arc(LCX, LCY, Math.min(LW, LH) / 2 - 2, 0, Math.PI * 2);
  lidarCtx.clip();

  // Range rings
  for (let r = 5; r <= LIDAR_MAX_R; r += 5) {
    const pr = r * LIDAR_PX_PER_M;
    lidarCtx.beginPath();
    lidarCtx.arc(LCX, LCY, pr, 0, Math.PI * 2);
    lidarCtx.strokeStyle = r % 10 === 0 ? 'rgba(0,255,100,0.15)' : 'rgba(0,255,100,0.07)';
    lidarCtx.lineWidth = 1;
    lidarCtx.stroke();
    if (r % 10 === 0) {
      lidarCtx.fillStyle = 'rgba(0,255,100,0.35)';
      lidarCtx.font = '7px JetBrains Mono, monospace';
      lidarCtx.fillText(r + 'm', LCX + pr + 2, LCY - 2);
    }
  }

  // Crosshairs
  lidarCtx.strokeStyle = 'rgba(0,255,100,0.08)';
  lidarCtx.lineWidth = 1;
  lidarCtx.beginPath();
  lidarCtx.moveTo(LCX, 2); lidarCtx.lineTo(LCX, LH - 2);
  lidarCtx.moveTo(2, LCY); lidarCtx.lineTo(LW - 2, LCY);
  lidarCtx.stroke();

  // FWD direction line (dashed)
  lidarCtx.setLineDash([3, 4]);
  lidarCtx.strokeStyle = 'rgba(74,158,255,0.25)';
  lidarCtx.beginPath();
  lidarCtx.moveTo(LCX, LCY);
  lidarCtx.lineTo(LCX, 8);
  lidarCtx.stroke();
  lidarCtx.setLineDash([]);

  // Draw accumulated point cloud
  for (const pt of lidarAccum) {
    const g = Math.round(80 + pt.intensity * 175);
    lidarCtx.fillStyle = `rgba(0,${g},${Math.round(g * 0.5)},${pt.intensity.toFixed(2)})`;
    lidarCtx.fillRect(pt.lx - 1, pt.ly - 1, 2, 2);
  }

  // Rover icon (triangle pointing up)
  lidarCtx.fillStyle = '#4a9eff';
  lidarCtx.strokeStyle = 'rgba(74,158,255,0.5)';
  lidarCtx.lineWidth = 1;
  lidarCtx.beginPath();
  lidarCtx.moveTo(LCX, LCY - 7);
  lidarCtx.lineTo(LCX + 5, LCY + 4);
  lidarCtx.lineTo(LCX - 5, LCY + 4);
  lidarCtx.closePath();
  lidarCtx.fill();
  lidarCtx.stroke();

  lidarCtx.restore();

  // FWD label
  lidarCtx.fillStyle = 'rgba(74,158,255,0.6)';
  lidarCtx.font = '8px JetBrains Mono, monospace';
  lidarCtx.fillText('FWD', LCX - 10, 12);

  // Update HUD stats
  const nearest = lidarAccum.length > 0
    ? lidarAccum.reduce((min, p) => {
        const d = Math.hypot(p.lx - LCX, p.ly - LCY) / LIDAR_PX_PER_M;
        return d < min ? d : min;
      }, Infinity)
    : Infinity;

  const nearTxt = nearest < Infinity ? nearest.toFixed(2) + ' m' : '> 30 m';
  document.getElementById('lidar-near').textContent = nearTxt;
  document.getElementById('lidar-near-ov').textContent = nearTxt;
  document.getElementById('lidar-pts-ov').textContent = lidarAccum.length + ' pts';

  const bearing = nearest < Infinity
    ? (() => {
        const closest = lidarAccum.reduce((best, p) => {
          const d = Math.hypot(p.lx - LCX, p.ly - LCY) / LIDAR_PX_PER_M;
          return d < best.d ? { d, p } : best;
        }, { d: Infinity, p: null });
        if (!closest.p) return '--';
        const ang = Math.atan2(LCX - closest.p.lx, LCY - closest.p.ly);
        return (((THREE.MathUtils.radToDeg(ang) + 360) % 360)).toFixed(1) + '°';
      })()
    : '--';
  document.getElementById('lidar-bearing').textContent = bearing;
  document.getElementById('lidar-rpm').textContent = (600 + Math.round(Math.sin(Date.now() * 0.002) * 3)).toString();
}

// ================================================================
// 13B. RADAR – PPI Plan Position Indicator canvas
//      Classic rotating sweep with blip persistence & Doppler colour
// ================================================================

const radarCanvas = document.getElementById('radar-canvas');
const radarCtx    = radarCanvas.getContext('2d');
const RW = radarCanvas.width;
const RH = radarCanvas.height;
const RCX = RW / 2, RCY = RH / 2;
const RADAR_MAX_RANGE = 100;
const RADAR_RADIUS = Math.min(RW, RH) / 2 - 4;
const RADAR_SCALE  = RADAR_RADIUS / RADAR_MAX_RANGE;
const RADAR_RPM = 20; // rotations per minute
const RADAR_RPS = RADAR_RPM / 60; // rotations per second

let radarSweepAngle = -Math.PI / 2; // start pointing up (forward)
let radarBlips = []; // { bx, by, age, doppler, range }

function drawRadar(dt) {
  // Advance sweep
  radarSweepAngle += RADAR_RPS * Math.PI * 2 * dt;
  if (radarSweepAngle > Math.PI * 2 - Math.PI / 2) radarSweepAngle -= Math.PI * 2;

  // Fade the previous frame (persistence phosphor effect)
  radarCtx.fillStyle = 'rgba(0, 10, 2, 0.18)';
  radarCtx.fillRect(0, 0, RW, RH);

  radarCtx.save();
  radarCtx.beginPath();
  radarCtx.arc(RCX, RCY, RADAR_RADIUS + 1, 0, Math.PI * 2);
  radarCtx.clip();

  // Sweep trail (sector glow)
  const TRAIL_RAD = 0.55; // radians
  for (let t = 0; t < 12; t++) {
    const a0 = radarSweepAngle - TRAIL_RAD * (1 - t / 12);
    const a1 = radarSweepAngle - TRAIL_RAD * (1 - (t + 1) / 12);
    radarCtx.beginPath();
    radarCtx.moveTo(RCX, RCY);
    radarCtx.arc(RCX, RCY, RADAR_RADIUS, a0, a1);
    radarCtx.closePath();
    radarCtx.fillStyle = `rgba(0,255,80,${(t / 12) * 0.07})`;
    radarCtx.fill();
  }

  // Sweep line
  radarCtx.beginPath();
  radarCtx.moveTo(RCX, RCY);
  radarCtx.lineTo(
    RCX + Math.cos(radarSweepAngle) * RADAR_RADIUS,
    RCY + Math.sin(radarSweepAngle) * RADAR_RADIUS
  );
  radarCtx.strokeStyle = 'rgba(0,255,80,0.95)';
  radarCtx.lineWidth = 1.5;
  radarCtx.stroke();

  radarCtx.restore();

  // Range rings
  const RING_RANGES = [25, 50, 75, 100];
  RING_RANGES.forEach(r => {
    radarCtx.beginPath();
    radarCtx.arc(RCX, RCY, r * RADAR_SCALE, 0, Math.PI * 2);
    radarCtx.strokeStyle = 'rgba(0,160,50,0.2)';
    radarCtx.lineWidth = 1;
    radarCtx.stroke();
    radarCtx.fillStyle = 'rgba(0,200,60,0.38)';
    radarCtx.font = '7px JetBrains Mono, monospace';
    radarCtx.fillText(r + 'm', RCX + r * RADAR_SCALE + 2, RCY - 2);
  });

  // Azimuth spokes (every 30°)
  for (let a = 0; a < 12; a++) {
    const ang = (a / 12) * Math.PI * 2 - Math.PI / 2;
    radarCtx.beginPath();
    radarCtx.moveTo(RCX, RCY);
    radarCtx.lineTo(RCX + Math.cos(ang) * RADAR_RADIUS, RCY + Math.sin(ang) * RADAR_RADIUS);
    radarCtx.strokeStyle = 'rgba(0,160,50,0.08)';
    radarCtx.lineWidth = 1;
    radarCtx.stroke();
  }

  // Outer ring border
  radarCtx.beginPath();
  radarCtx.arc(RCX, RCY, RADAR_RADIUS, 0, Math.PI * 2);
  radarCtx.strokeStyle = 'rgba(0,200,60,0.4)';
  radarCtx.lineWidth = 2;
  radarCtx.stroke();

  // ---- Compute targets, add blips when sweep passes ----
  const rp = chassisBody.position;
  const rv = chassisBody.velocity;
  const rq = chassisBody.quaternion;
  const roverFwd = new THREE.Vector3(0, 0, 1).applyQuaternion(
    new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w)
  );

  let maxDoppler = 0;
  let targetCount = 0;

  sceneObstacles.forEach(({ body }) => {
    const dx = body.position.x - rp.x;
    const dz = body.position.z - rp.z;
    const range = Math.hypot(dx, dz);
    if (range > RADAR_MAX_RANGE) return;

    targetCount++;

    // Azimuth of target in world space (0 = east, CCW)
    const worldAzimuth = Math.atan2(dx, dz); // atan2(x,z) → angle from north/fwd

    // Canvas angle: radar top = forward. We map worldAzimuth to canvas angle.
    // canvas: angle -PI/2 = up, so canvasAngle = worldAzimuth - PI/2
    const canvasAngle = worldAzimuth - Math.PI / 2;

    // Check if sweep just crossed this target azimuth
    const sweepDiff = ((canvasAngle - radarSweepAngle + Math.PI * 3) % (Math.PI * 2)) - Math.PI;

    if (Math.abs(sweepDiff) < RADAR_RPS * Math.PI * 2 * dt * 2.5) {
      // Compute Doppler (relative velocity towards rover along LOS)
      const toTarget = new THREE.Vector3(dx, 0, dz).normalize();
      const bv = body.velocity;
      const relVx = (bv ? bv.x : 0) - rv.x;
      const relVz = (bv ? bv.z : 0) - rv.z;
      const doppler = -(toTarget.x * relVx + toTarget.z * relVz); // + = approaching
      const noise = (Math.random() - 0.5) * 0.03;
      if (Math.abs(doppler) > Math.abs(maxDoppler)) maxDoppler = doppler;

      const bx = RCX + Math.cos(canvasAngle) * range * RADAR_SCALE;
      const by = RCY + Math.sin(canvasAngle) * range * RADAR_SCALE;
      radarBlips.push({ bx, by, age: 1.0, doppler: doppler + noise, range });
    }
  });

  // Draw and decay blips
  radarBlips = radarBlips.filter(b => b.age > 0.03);
  radarBlips.forEach(b => {
    const alpha = Math.min(1, b.age * 1.2);
    let r, g, bl;
    if (b.doppler > 0.15) {
      // Approaching: bright green
      r = 0; g = 255; bl = 80;
    } else if (b.doppler < -0.15) {
      // Receding: orange/red
      r = 255; g = 120; bl = 0;
    } else {
      // Stationary: cyan
      r = 0; g = 200; bl = 255;
    }
    // Glow
    const grd = radarCtx.createRadialGradient(b.bx, b.by, 0, b.bx, b.by, 7);
    grd.addColorStop(0, `rgba(${r},${g},${bl},${alpha})`);
    grd.addColorStop(1, `rgba(${r},${g},${bl},0)`);
    radarCtx.beginPath();
    radarCtx.arc(b.bx, b.by, 7, 0, Math.PI * 2);
    radarCtx.fillStyle = grd;
    radarCtx.fill();
    // Core dot
    radarCtx.beginPath();
    radarCtx.arc(b.bx, b.by, 2.5, 0, Math.PI * 2);
    radarCtx.fillStyle = `rgba(${r},${g},${bl},${Math.min(1, alpha * 1.5)})`;
    radarCtx.fill();
    b.age -= dt * 0.45;
  });

  // Rover at centre
  radarCtx.beginPath();
  radarCtx.arc(RCX, RCY, 4, 0, Math.PI * 2);
  radarCtx.fillStyle = '#4a9eff';
  radarCtx.fill();

  // FWD indicator
  radarCtx.fillStyle = 'rgba(74,158,255,0.5)';
  radarCtx.font = '8px JetBrains Mono, monospace';
  radarCtx.fillText('N/FWD', RCX - 14, 12);

  // Legend
  radarCtx.fillStyle = '#00ff50';
  radarCtx.fillRect(RW - 68, 8, 6, 6);
  radarCtx.fillStyle = 'rgba(0,200,80,0.6)';
  radarCtx.font = '7px JetBrains Mono, monospace';
  radarCtx.fillText('approach', RW - 60, 14);

  radarCtx.fillStyle = '#ff7800';
  radarCtx.fillRect(RW - 68, 18, 6, 6);
  radarCtx.fillStyle = 'rgba(255,150,0,0.6)';
  radarCtx.fillText('recede', RW - 60, 24);

  radarCtx.fillStyle = '#00c8ff';
  radarCtx.fillRect(RW - 68, 28, 6, 6);
  radarCtx.fillStyle = 'rgba(0,200,255,0.6)';
  radarCtx.fillText('static', RW - 60, 34);

  // HUD stats
  document.getElementById('radar-targets').textContent = targetCount;
  document.getElementById('radar-targets-ov').textContent = targetCount + ' tgts';
  document.getElementById('radar-sweep-ov').textContent = 'RPM ' + RADAR_RPM;
  document.getElementById('radar-max-dop').textContent = Math.abs(maxDoppler) > 0.01
    ? (maxDoppler > 0 ? '+' : '') + maxDoppler.toFixed(3) + ' m/s'
    : '0.000 m/s';
}

// ================================================================
// 13C. THERMAL CAMERA – separate WebGL renderer (teleop FPV)
//      Camera mounts at thermal cam position on rover
//      CSS filter approximates LWIR thermal colorization
// ================================================================

const camCanvas  = document.getElementById('cam-canvas');
const camRenderer = new THREE.WebGLRenderer({ canvas: camCanvas, antialias: false });
camRenderer.setPixelRatio(1);
camRenderer.setSize(camCanvas.width, camCanvas.height);
camRenderer.shadowMap.enabled = false; // no shadows in cam feed for perf
camRenderer.toneMapping = THREE.ACESFilmicToneMapping;
camRenderer.toneMappingExposure = 1.0;
// Thermal IR colorization via CSS filter (sepia→hue-rotate→saturate approximates LWIR palette)
camCanvas.style.filter = 'sepia(1) saturate(3.5) hue-rotate(195deg) contrast(1.35)';

const thermalCam = new THREE.PerspectiveCamera(62, camCanvas.width / camCanvas.height, 0.02, 80);

// Thermal cam local offset from chassis centre (from URDF sensors.xacro approximation)
// front-left, just above chassis top
const THERMAL_LOCAL = new THREE.Vector3(
  -CHASSIS_W * 0.3,
  CHASSIS_H / 2 + 0.04,
  CHASSIS_L * 0.32
);

// The cam points in the +Z direction (rover forward) with a slight downward tilt
const THERMAL_CAM_PITCH = -0.06; // radians

let camFrameCount = 0;
let lastCamFpsTime = 0;

function updateThermalCam() {
  const rp = chassisBody.position;
  const rq = chassisBody.quaternion;
  const chassisQ = new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w);

  // World position of thermal camera
  const worldOffset = THERMAL_LOCAL.clone().applyQuaternion(chassisQ);
  thermalCam.position.set(rp.x + worldOffset.x, rp.y + worldOffset.y, rp.z + worldOffset.z);

  // Camera orientation: rover's heading + slight downward tilt
  const pitchQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(THERMAL_CAM_PITCH, 0, 0));
  thermalCam.quaternion.copy(chassisQ).multiply(pitchQ);

  camRenderer.render(scene, thermalCam);

  // FPS counter
  camFrameCount++;
  const now = performance.now();
  if (now - lastCamFpsTime >= 1000) {
    const fps = Math.round(camFrameCount / ((now - lastCamFpsTime) / 1000));
    const fpsTxt = fps + ' FPS';
    const fpsEl = document.getElementById('cam-fps-chip');
    if (fpsEl) fpsEl.textContent = fpsTxt;
    lastCamFpsTime = now;
    camFrameCount = 0;
  }
}

// ================================================================
// 13D. GPS + IMU – text readouts (10 Hz, physics-derived)
// ================================================================

let prevVelX = 0, prevVelY = 0, prevVelZ = 0;
let sensorTick = 0;

function n(v, sigma) {
  return v + (Math.random() + Math.random() - 1) * sigma;
}

function updateGpsImu(dt) {
  sensorTick++;
  const rp = chassisBody.position;
  const rv = chassisBody.velocity;
  const rav = chassisBody.angularVelocity;
  const rq = chassisBody.quaternion;

  const euler = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w), 'XYZ'
  );
  const rollDeg  = THREE.MathUtils.radToDeg(euler.x);
  const pitchDeg = THREE.MathUtils.radToDeg(euler.y);
  const yawDeg   = ((THREE.MathUtils.radToDeg(euler.z) % 360) + 360) % 360;

  // IMU
  const accX = n((rv.x - prevVelX) / Math.max(dt, 0.001), 0.05);
  const accY = n((rv.y - prevVelY) / Math.max(dt, 0.001) + 9.82, 0.05);
  const accZ = n((rv.z - prevVelZ) / Math.max(dt, 0.001), 0.05);
  prevVelX = rv.x; prevVelY = rv.y; prevVelZ = rv.z;

  document.getElementById('imu-ax').textContent = accX.toFixed(3) + ' m/s²';
  document.getElementById('imu-ay').textContent = accY.toFixed(3) + ' m/s²';
  document.getElementById('imu-az').textContent = accZ.toFixed(3) + ' m/s²';
  document.getElementById('imu-gx').textContent = n(rav.x, 0.002).toFixed(4) + ' rad/s';
  document.getElementById('imu-gy').textContent = n(rav.y, 0.002).toFixed(4) + ' rad/s';
  document.getElementById('imu-gz').textContent = n(rav.z, 0.002).toFixed(4) + ' rad/s';
  document.getElementById('imu-roll').textContent  = n(rollDeg,  0.02).toFixed(2) + '°';
  document.getElementById('imu-pitch').textContent = n(pitchDeg, 0.02).toFixed(2) + '°';
  document.getElementById('imu-yaw').textContent   = n(yawDeg,   0.02).toFixed(2) + '°';
  document.getElementById('imu-qw').textContent = rq.w.toFixed(5);
  document.getElementById('imu-qx').textContent = rq.x.toFixed(5);
  document.getElementById('imu-qy').textContent = rq.y.toFixed(5);
  document.getElementById('imu-qz').textContent = rq.z.toFixed(5);

  // GPS
  const speed2d = Math.hypot(rv.x, rv.z);
  const lat = GPS_LAT0 + rp.z / M_PER_DEG_LAT;
  const lon = GPS_LON0 + rp.x / M_PER_DEG_LON;
  const alt = n(180 + rp.y, 0.08);
  const sats = 12 + Math.round(Math.sin(sensorTick * 0.003) * 2);

  document.getElementById('gps-lat').textContent  = lat.toFixed(7) + '°';
  document.getElementById('gps-lon').textContent  = lon.toFixed(7) + '°';
  document.getElementById('gps-alt').textContent  = alt.toFixed(2) + ' m';
  document.getElementById('gps-sats').textContent = sats;
  document.getElementById('gps-hdop').textContent = n(0.8, 0.02).toFixed(2);
  document.getElementById('gps-speed').textContent = n(speed2d, 0.01).toFixed(3);
  document.getElementById('gps-course').textContent = ((yawDeg + n(0, 0.1)) % 360).toFixed(2) + '°';

  const fixEl = document.getElementById('gps-fix');
  if (speed2d < 0.02) {
    fixEl.textContent = 'RTK FLOAT';
    fixEl.classList.replace('sval-green', 'sval-orange');
  } else {
    fixEl.textContent = 'RTK FIXED';
    fixEl.className = fixEl.className.replace('sval-orange', 'sval-green');
    if (!fixEl.classList.contains('sval-green')) fixEl.classList.add('sval-green');
  }

  // Thermal cam HUD stats
  const speed = speed2d;
  const baseTemp = 38 + speed * 3;
  const tempTxt = n(baseTemp, 0.3).toFixed(1) + ' °C';
  const contrastVal = Math.min(1.0, 0.55 + speed * 0.08).toFixed(2);
  document.getElementById('cam-temp').textContent = tempTxt;
  document.getElementById('cam-contrast').textContent = contrastVal;
  const chipTemp = document.getElementById('cam-temp-chip');
  const chipContrast = document.getElementById('cam-contrast-chip');
  if (chipTemp) chipTemp.textContent = tempTxt;
  if (chipContrast) chipContrast.textContent = 'AGC: ' + contrastVal;

  // Count thermal blobs in FOV
  const roverFwdCam = new THREE.Vector3(0, 0, 1).applyQuaternion(
    new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w)
  );
  let blobCount = 0;
  sceneObstacles.forEach(({ body }) => {
    const dx = body.position.x - rp.x;
    const dz = body.position.z - rp.z;
    const range = Math.hypot(dx, dz);
    if (range > 15) return;
    const toObj = new THREE.Vector3(dx, 0, dz).normalize();
    const az = THREE.MathUtils.radToDeg(Math.acos(Math.max(-1, Math.min(1, roverFwdCam.dot(toObj)))));
    if (az < 45) blobCount++;
  });
  const bcEl = document.getElementById('cam-blob-count');
  if (bcEl) bcEl.textContent = blobCount;
}

// ================================================================
// 14. SENSOR LOOP – runs at different rates per sensor type
//     LiDAR: 10 Hz (real VLP-16 = 10-20 Hz)
//     Radar: every rAF (driven by sweep animation continuity)
//     Camera: 30 Hz (set in camRenderer render calls via main loop)
//     GPS/IMU: 10 Hz
// ================================================================

let lastLidarTime = 0;
let lastGpsImuTime = 0;

function sensorLoop(ts) {
  requestAnimationFrame(sensorLoop);
  const dt = Math.min((ts - (lastLidarTime || ts)) / 1000, 0.2);

  // Radar runs every frame (sweep must be continuous for PPI animation)
  const radarDt = Math.min((ts - (sensorLoop._lastRadarTs || ts)) / 1000, 0.1);
  sensorLoop._lastRadarTs = ts;
  drawRadar(radarDt);

  // LiDAR & SLAM Map at ~10 Hz
  if (ts - lastLidarTime >= 100) {
    drawLidar();
    if (typeof drawSlamMap === 'function') drawSlamMap();
    lastLidarTime = ts;
  }

  // GPS + IMU at ~10 Hz
  if (ts - lastGpsImuTime >= 100) {
    const gdt = (ts - lastGpsImuTime) / 1000;
    lastGpsImuTime = ts;
    updateGpsImu(gdt);
  }

  // Thermal camera at 30 Hz
  if (ts - (sensorLoop._lastCamTs || 0) >= 33) {
    sensorLoop._lastCamTs = ts;
    updateThermalCam();
  }
}

requestAnimationFrame(sensorLoop);


// ============================================================
// 15. FRONTIER EXPLORATION & SLAM MAP PIPELINE
// ============================================================

const slamCanvas = document.getElementById('slam-canvas');
const slamCtx = slamCanvas ? slamCanvas.getContext('2d') : null;

// Initialize Auto toggle button
const autoBtn = document.getElementById('btn-toggle-auto');
if (autoBtn) {
  autoBtn.addEventListener('click', () => {
    setAutoMode(!isAutoNav);
  });
}

// Update Occupancy Grid from LiDAR raycast hits and free space
function updateSlamGrid(mastPos, scanHits) {
  const gRover = worldToGrid(mastPos.x, mastPos.z);
  if (!gRover) return;

  // Bresenham line to mark free rays, and end cells as occupied
  scanHits.forEach(hit => {
    // End point in world
    const endX = mastPos.x + hit.dir.x * hit.dist;
    const endZ = mastPos.z + hit.dir.z * hit.dist;
    const gEnd = worldToGrid(endX, endZ);
    if (!gEnd) return;

    // Ray trace grid cells (Free space = 1)
    let x0 = gRover.gx, y0 = gRover.gz;
    let x1 = gEnd.gx, y1 = gEnd.gz;
    let dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
    let sx = (x0 < x1) ? 1 : -1;
    let sy = (y0 < y1) ? 1 : -1;
    let err = dx - dy;

    while (true) {
      if (x0 === x1 && y0 === y1) break;
      if (x0 >= 0 && x0 < MAP_DIM && y0 >= 0 && y0 < MAP_DIM) {
        const idx = y0 * MAP_DIM + x0;
        if (slamGrid[idx] === 0 || slamGrid[idx] === 3) {
          slamGrid[idx] = 1; // Mark Free
        }
      }
      let e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
    }

    // Mark obstacle endpoint if within max range (Occupied = 2)
    if (hit.dist < LIDAR_MAX_R - 0.5) {
      // Dilate obstacle slightly for safety margin
      for (let ox = -1; ox <= 1; ox++) {
        for (let oz = -1; oz <= 1; oz++) {
          const nx = gEnd.gx + ox;
          const nz = gEnd.gz + oz;
          if (nx >= 0 && nx < MAP_DIM && nz >= 0 && nz < MAP_DIM) {
            slamGrid[nz * MAP_DIM + nx] = 2; // Occupied
          }
        }
      }

      // Automatically register obstacle in landmark database if map recording or SLAM is active
      if (typeof registerDiscoveredObstacle === 'function') {
        registerDiscoveredObstacle(endX, endZ, hit.dist, 0);
      }
    }
  });

  // Rover immediate surroundings are always free
  for (let ox = -1; ox <= 1; ox++) {
    for (let oz = -1; oz <= 1; oz++) {
      const nx = gRover.gx + ox;
      const nz = gRover.gz + oz;
      if (nx >= 0 && nx < MAP_DIM && nz >= 0 && nz < MAP_DIM) {
        if (slamGrid[nz * MAP_DIM + nx] !== 2) {
          slamGrid[nz * MAP_DIM + nx] = 1;
        }
      }
    }
  }
}

// Find Frontier Cells (Free cells adjacent to Unknown cells)
function extractFrontiers() {
  const frontiers = [];
  let freeCount = 0;
  let occupiedCount = 0;

  for (let gz = 1; gz < MAP_DIM - 1; gz++) {
    for (let gx = 1; gx < MAP_DIM - 1; gx++) {
      const idx = gz * MAP_DIM + gx;
      const val = slamGrid[idx];
      if (val === 1) {
        freeCount++;
        // Check 8-neighbors for UNKNOWN (0)
        let hasUnknownNeighbor = false;
        let hasOccupiedNeighbor = false;
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            if (dx === 0 && dy === 0) continue;
            const nVal = slamGrid[(gz + dy) * MAP_DIM + (gx + dx)];
            if (nVal === 0) hasUnknownNeighbor = true;
            if (nVal === 2) hasOccupiedNeighbor = true;
          }
        }
        // Valid frontier candidate: free, borders unknown, not bordering obstacles
        if (hasUnknownNeighbor && !hasOccupiedNeighbor) {
          const worldPos = gridToWorld(gx, gz);
          frontiers.push({ gx, gz, x: worldPos.x, z: worldPos.z });
        }
      } else if (val === 2) {
        occupiedCount++;
      }
    }
  }

  exploredPercent = ((freeCount + occupiedCount) / (MAP_DIM * MAP_DIM) * 100).toFixed(1);
  frontiersCount = frontiers.length;

  return frontiers;
}

// Advanced Artificial Potential Field (APF) & Pure Pursuit Steering Controller
let stuckTimer = 0;
let lastProgressPos = new THREE.Vector3();
let lastProgressTime = 0;

function updateAutonomousController(dt) {
  const rp = chassisBody.position;
  const rq = chassisBody.quaternion;
  const chassisQ = new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w);
  const roverFwd = new THREE.Vector3(0, 0, 1).applyQuaternion(chassisQ);

  navStateTimer += dt;

  // 1. Stuck Detection (Check if rover hasn't moved > 0.4m in 3.0s while trying to navigate)
  const now = performance.now();
  if (now - lastProgressTime > 3000) {
    const movedDist = rp.distanceTo(lastProgressPos);
    if (movedDist < 0.35 && autoNavState !== 'REVERSING') {
      // Trigger recovery un-stick maneuver
      autoNavState = 'REVERSING';
      reverseTimer = 1.6;
      reverseSteer = Math.random() > 0.5 ? MAX_STEER : -MAX_STEER;
      currentGoal = null; // Re-evaluate path
    }
    lastProgressPos.copy(rp);
    lastProgressTime = now;
  }

  // 2. State Machine Execution
  if (autoNavState === 'REVERSING') {
    reverseTimer -= dt;
    autoThrottle = -0.65;
    autoSteer = reverseSteer;
    if (reverseTimer <= 0) {
      autoNavState = 'SEARCHING';
      navStateTimer = 0;
    }
  } else {
    // Frontier Goal Selection
    if (!currentGoal || navStateTimer > 12.0) {
      autoNavState = 'SEARCHING';
      const frontiers = extractFrontiers();
      if (frontiers.length > 0) {
        let bestScore = -Infinity;
        let bestF = null;

        frontiers.forEach(f => {
          const dist = Math.hypot(f.x - rp.x, f.z - rp.z);
          // Only pick goals that are in open space (not inside obstacles)
          if (dist > 2.0 && dist < 22.0) {
            const toGoal = new THREE.Vector2(f.x - rp.x, f.z - rp.z).normalize();
            const fwd2D = new THREE.Vector2(roverFwd.x, roverFwd.z).normalize();
            const alignment = toGoal.dot(fwd2D);
            const score = -dist * 0.5 + alignment * 5.0;
            if (score > bestScore) {
              bestScore = score;
              bestF = f;
            }
          }
        });

        currentGoal = bestF ? { x: bestF.x, z: bestF.z } : frontiers[Math.floor(Math.random() * frontiers.length)];
        navStateTimer = 0;
      } else {
        // Fallback exploration waypoint
        const randomAngle = Math.random() * Math.PI * 2;
        currentGoal = { x: Math.cos(randomAngle) * 14, z: Math.sin(randomAngle) * 14 };
        navStateTimer = 0;
      }
    }

    if (currentGoal) {
      const toGoal = new THREE.Vector3(currentGoal.x - rp.x, 0, currentGoal.z - rp.z);
      const distToGoal = toGoal.length();

      if (distToGoal < 1.4) {
        autoNavState = 'REACHED';
        currentGoal = null;
        navStateTimer = 0;
        autoThrottle = 0;
        autoSteer = 0;
      } else {
        autoNavState = 'NAVIGATING';
        toGoal.normalize();

        // 3. ARTIFICIAL POTENTIAL FIELD (APF) FORCE COMPUTATION
        // Attractive force towards goal (in rover local frame: +Z fwd, -X left, +X right)
        const toGoalLocal = toGoal.clone().applyQuaternion(chassisQ.clone().invert());
        let forceX = toGoalLocal.x * 1.5;
        let forceZ = toGoalLocal.z * 1.5;

        // Repulsive force from nearby LiDAR obstacles (Laser Scan Vector Field)
        let minFrontDist = 999;
        let leftRepulsion = 0;
        let rightRepulsion = 0;

        lidarAccum.forEach(p => {
          const lx = (p.lx - LCX) / LIDAR_PX_PER_M; // rover frame X (+ right, - left)
          const lz = (LCY - p.ly) / LIDAR_PX_PER_M; // rover frame Z (+ forward)
          const dist = Math.hypot(lx, lz);

          // Danger corridor in front of rover
          if (lz > 0.1 && lz < 3.8 && Math.abs(lx) < 1.4) {
            if (lz < minFrontDist) minFrontDist = lz;
            const repStrength = Math.pow((3.8 - lz) / 3.8, 2) * 2.2;
            if (lx < 0) leftRepulsion += repStrength;
            if (lx >= 0) rightRepulsion += repStrength;

            // Push vector away from obstacle
            forceX -= (lx / dist) * repStrength;
            forceZ -= (lz / dist) * repStrength * 0.5;
          }
        });

        // Emergency braking / reversing if too close
        if (minFrontDist < 0.85) {
          autoNavState = 'REVERSING';
          reverseTimer = 1.4;
          reverseSteer = leftRepulsion > rightRepulsion ? -MAX_STEER : MAX_STEER;
        } else {
          if (minFrontDist < 2.0) {
            autoNavState = 'OBSTACLE_AVOID';
          }

          // Compute desired heading from Net APF Vector
          // In rover frame: Left steering = +MAX_STEER, Right steering = -MAX_STEER
          // Angle positive when target vector is to the LEFT (-forceX)
          const desiredHeading = Math.atan2(-forceX, Math.max(0.1, forceZ));

          // Proportional-Derivative (PD) Steering Control
          const targetSteer = Math.max(-MAX_STEER, Math.min(MAX_STEER, desiredHeading * 1.8));
          autoSteer += (targetSteer - autoSteer) * 0.35; // Smooth steering response

          // Speed profiling based on turn sharpness and front clearance
          const turnSlowdown = Math.max(0.35, 1.0 - (Math.abs(desiredHeading) / Math.PI) * 0.8);
          const clearanceSlowdown = Math.min(1.0, Math.max(0.3, minFrontDist / 3.0));
          autoThrottle = 0.85 * turnSlowdown * clearanceSlowdown;
        }
      }
    }
  }

  // Update UI Elements
  const stateEl = document.getElementById('nav-state');
  const goalEl = document.getElementById('nav-goal');
  const distEl = document.getElementById('nav-dist');
  if (stateEl) stateEl.textContent = autoNavState;
  if (goalEl) goalEl.textContent = currentGoal ? `(${currentGoal.x.toFixed(1)}, ${currentGoal.z.toFixed(1)})` : 'None';
  if (distEl) distEl.textContent = currentGoal ? `${Math.hypot(currentGoal.x - rp.x, currentGoal.z - rp.z).toFixed(1)} m` : '--';
  const expOv = document.getElementById('explored-pct-ov');
  const frOv = document.getElementById('frontiers-count-ov');
  if (expOv) expOv.textContent = `${exploredPercent}% MAP`;
  if (frOv) frOv.textContent = `${frontiersCount} FRONTIERS`;
}


// ============================================================
// 15B. SLAM MAP RECORDER, TRAJECTORY LOGGER & EXPORTER
// ============================================================

let isMapRecording = false;
let recordedTrajectory = []; // Array of { x, z, yaw, timestamp }
let lastRecordSampleTime = 0;
let totalFreeCells = 0;
let totalOccupiedCells = 0;

function toggleMapRecording() {
  isMapRecording = !isMapRecording;
  const recBtn = document.getElementById('btn-record-map');
  const recStat = document.getElementById('rec-status');

  if (recBtn) {
    recBtn.classList.toggle('recording', isMapRecording);
    recBtn.textContent = isMapRecording ? '⏹️ STOP RECORDING' : '⏺️ START RECORDING MAP';
  }
  if (recStat) {
    recStat.textContent = isMapRecording ? 'REC ●' : 'PAUSED';
    recStat.className = 'ms-val ' + (isMapRecording ? 'sval-green' : 'sval-orange');
  }
}

function clearSlamMap() {
  slamGrid.fill(0);
  recordedTrajectory = [];
  discoveredObstacles.clear();
  nextObstacleId = 1;
  updateObstacleRegistryTable();
  currentGoal = null;
  const recNodes = document.getElementById('rec-nodes');
  const recCells = document.getElementById('rec-cells');
  if (recNodes) recNodes.textContent = '0 poses';
  if (recCells) recCells.textContent = '0 free / 0 occ';
}

function exportSlamMap() {
  if (!slamCanvas) return;

  // Create an offscreen canvas with high-resolution export for ROS / Research papers
  const expCanvas = document.createElement('canvas');
  expCanvas.width = MAP_DIM * 8;  // 640x640 high-res
  expCanvas.height = MAP_DIM * 8;
  const expCtx = expCanvas.getContext('2d');
  const cellPx = expCanvas.width / MAP_DIM;

  // Background (Unknown = 128 gray / ROS standard)
  expCtx.fillStyle = '#7f7f7f';
  expCtx.fillRect(0, 0, expCanvas.width, expCanvas.height);

  for (let gz = 0; gz < MAP_DIM; gz++) {
    for (let gx = 0; gx < MAP_DIM; gx++) {
      const val = slamGrid[gz * MAP_DIM + gx];
      if (val === 1) {
        // Free Space (White in standard ROS occupancy grids)
        expCtx.fillStyle = '#ffffff';
        expCtx.fillRect(gx * cellPx, gz * cellPx, cellPx, cellPx);
      } else if (val === 2) {
        // Obstacles (Black in standard ROS occupancy grids)
        expCtx.fillStyle = '#000000';
        expCtx.fillRect(gx * cellPx, gz * cellPx, cellPx, cellPx);
      }
    }
  }

  // Draw Trajectory Path on exported map
  if (recordedTrajectory.length > 1) {
    expCtx.strokeStyle = '#2979ff';
    expCtx.lineWidth = 3;
    expCtx.beginPath();
    recordedTrajectory.forEach((pose, idx) => {
      const g = worldToGrid(pose.x, pose.z);
      if (g) {
        const px = g.gx * cellPx + cellPx / 2;
        const pz = g.gz * cellPx + cellPx / 2;
        if (idx === 0) expCtx.moveTo(px, pz);
        else expCtx.lineTo(px, pz);
      }
    });
    expCtx.stroke();
  }

  // Trigger download
  const link = document.createElement('a');
  link.download = `slam_map_${currentWeather}_${Date.now()}.png`;
  link.href = expCanvas.toDataURL('image/png');
  link.click();
}

// Wire up Toolbar event listeners
const btnRec = document.getElementById('btn-record-map');
const btnExp = document.getElementById('btn-export-map');
const btnClr = document.getElementById('btn-clear-map');
if (btnRec) btnRec.addEventListener('click', toggleMapRecording);
if (btnExp) btnExp.addEventListener('click', exportSlamMap);
if (btnClr) btnClr.addEventListener('click', clearSlamMap);


// ============================================================
// 15C. DISCOVERED OBSTACLE REGISTRY & LANDMARK TRACKER
// ============================================================

let discoveredObstacles = new Map(); // Key: 'gx_gz' -> { id, type, x, z, size, rcs, timestamp, hitCount }
let nextObstacleId = 1;

function registerDiscoveredObstacle(worldX, worldZ, dist, azimuth) {
  const g = worldToGrid(worldX, worldZ);
  if (!g) return;
  const key = `${g.gx}_${g.gz}`;

  if (discoveredObstacles.has(key)) {
    const obs = discoveredObstacles.get(key);
    obs.hitCount++;
    return;
  }

  // Determine obstacle category based on proximity to ground objects
  let type = 'Hazard Block';
  let typeClass = 'crate';

  // Check against known ground bodies
  for (const obj of obstacles) {
    const bp = obj.body.position;
    if (Math.hypot(bp.x - worldX, bp.z - worldZ) < 1.6) {
      if (obj.body.shapes[0] instanceof CANNON.Sphere) {
        type = 'Boulder';
        typeClass = 'boulder';
      } else if (obj.body.mass === 0) {
        if (Math.abs(bp.x) > 16 || Math.abs(bp.z) > 16) {
          type = 'Perimeter Wall';
          typeClass = 'wall';
        } else {
          type = 'Freight Container';
          typeClass = 'container';
        }
      } else if (obj.body.mass <= 4) {
        type = 'Traffic Pylon';
        typeClass = 'pylon';
      } else {
        type = 'Industrial Crate';
        typeClass = 'crate';
      }
      break;
    }
  }

  const newObs = {
    id: `OBS-${String(nextObstacleId++).padStart(3, '0')}`,
    type,
    typeClass,
    x: Number(worldX.toFixed(2)),
    z: Number(worldZ.toFixed(2)),
    gx: g.gx,
    gz: g.gz,
    dist: Number(dist.toFixed(2)),
    time: new Date().toLocaleTimeString(),
    hitCount: 1,
  };

  discoveredObstacles.set(key, newObs);
  updateObstacleRegistryTable();
}

function updateObstacleRegistryTable() {
  const countEl = document.getElementById('rec-obs-count');
  const tbody = document.getElementById('obs-registry-tbody');
  if (countEl) countEl.textContent = `${discoveredObstacles.size} DETECTED`;

  if (!tbody) return;

  if (discoveredObstacles.size === 0) {
    tbody.innerHTML = '<tr><td colspan="5" style="color:#7a90b8;text-align:center;">No obstacles registered yet</td></tr>';
    return;
  }

  const items = Array.from(discoveredObstacles.values()).slice(-8).reverse(); // show latest 8
  tbody.innerHTML = items.map(o => `
    <tr>
      <td><b>${o.id}</b></td>
      <td><span class="obs-type-tag ${o.typeClass}">${o.type}</span></td>
      <td>(${o.x}, ${o.z})</td>
      <td>${o.dist}m (${o.hitCount} pts)</td>
      <td style="color:#7a90b8;">${o.time}</td>
    </tr>
  `).join('');
}

function exportObstaclesJson() {
  const data = Array.from(discoveredObstacles.values());
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
  const link = document.createElement('a');
  link.download = `recorded_obstacles_${Date.now()}.json`;
  link.href = URL.createObjectURL(blob);
  link.click();
}

function exportObstaclesCsv() {
  const items = Array.from(discoveredObstacles.values());
  let csv = 'ID,Type,World_X,World_Z,Detection_Dist,Detection_Time,Point_Count\n';
  items.forEach(o => {
    csv += `${o.id},"${o.type}",${o.x},${o.z},${o.dist},"${o.time}",${o.hitCount}\n`;
  });
  const blob = new Blob([csv], { type: 'text/csv' });
  const link = document.createElement('a');
  link.download = `recorded_obstacles_${Date.now()}.csv`;
  link.href = URL.createObjectURL(blob);
  link.click();
}

const btnExpJson = document.getElementById('btn-export-obs-json');
const btnExpCsv  = document.getElementById('btn-export-obs-csv');
if (btnExpJson) btnExpJson.addEventListener('click', exportObstaclesJson);
if (btnExpCsv)  btnExpCsv.addEventListener('click', exportObstaclesCsv);

// Render Frontier SLAM Map on 2D Canvas
function drawSlamMap() {
  if (!slamCtx) return;
  const W = slamCanvas.width;
  const H = slamCanvas.height;
  const cellPx = W / MAP_DIM;

  slamCtx.fillStyle = '#060a10';
  slamCtx.fillRect(0, 0, W, H);

  // Draw Grid Cells
  for (let gz = 0; gz < MAP_DIM; gz++) {
    for (let gx = 0; gx < MAP_DIM; gx++) {
      const val = slamGrid[gz * MAP_DIM + gx];
      if (val === 1) {
        // Free Space (Dark blue-green)
        slamCtx.fillStyle = '#0b2024';
        slamCtx.fillRect(gx * cellPx, gz * cellPx, cellPx + 0.5, cellPx + 0.5);
      } else if (val === 2) {
        // Occupied / Obstacle (Bright Orange-Red)
        slamCtx.fillStyle = '#ff4d4d';
        slamCtx.fillRect(gx * cellPx, gz * cellPx, cellPx + 0.5, cellPx + 0.5);
      }
    }
  }

  // Draw Frontier candidates
  const frontiers = extractFrontiers();
  slamCtx.fillStyle = '#00e5ff';
  frontiers.forEach(f => {
    slamCtx.fillRect(f.gx * cellPx, f.gz * cellPx, cellPx, cellPx);
  });

  // Draw Grid axes
  slamCtx.strokeStyle = 'rgba(74, 158, 255, 0.15)';
  slamCtx.lineWidth = 1;
  slamCtx.strokeRect(0, 0, W, H);
  slamCtx.beginPath();
  slamCtx.moveTo(W / 2, 0); slamCtx.lineTo(W / 2, H);
  slamCtx.moveTo(0, H / 2); slamCtx.lineTo(W, H / 2);
  slamCtx.stroke();

  const rp = chassisBody.position;
  const rq = chassisBody.quaternion;
  const yaw = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w), 'YXZ').y;
  const gRover = worldToGrid(rp.x, rp.z);

  // Record Pose History
  const now = performance.now();
  if (isMapRecording && (now - lastRecordSampleTime > 250)) {
    if (recordedTrajectory.length === 0 || rp.distanceTo(new THREE.Vector3(recordedTrajectory[recordedTrajectory.length - 1].x, 0, recordedTrajectory[recordedTrajectory.length - 1].z)) > 0.15) {
      recordedTrajectory.push({ x: rp.x, z: rp.z, yaw, time: now });
      lastRecordSampleTime = now;
      const recNodes = document.getElementById('rec-nodes');
      if (recNodes) recNodes.textContent = `${recordedTrajectory.length} poses`;
    }
  }

  // Draw Recorded Trajectory Breadcrumbs
  if (recordedTrajectory.length > 1) {
    slamCtx.strokeStyle = 'rgba(74, 158, 255, 0.7)';
    slamCtx.lineWidth = 1.5;
    slamCtx.beginPath();
    recordedTrajectory.forEach((p, idx) => {
      const g = worldToGrid(p.x, p.z);
      if (g) {
        const px = g.gx * cellPx + cellPx / 2;
        const pz = g.gz * cellPx + cellPx / 2;
        if (idx === 0) slamCtx.moveTo(px, pz);
        else slamCtx.lineTo(px, pz);
      }
    });
    slamCtx.stroke();
  }

  // Update Live Cell Count Stats
  let freeCnt = 0, occCnt = 0;
  for (let i = 0; i < slamGrid.length; i++) {
    if (slamGrid[i] === 1) freeCnt++;
    else if (slamGrid[i] === 2) occCnt++;
  }
  const recCells = document.getElementById('rec-cells');
  if (recCells) recCells.textContent = `${freeCnt} free / ${occCnt} occ`;

  // Draw Current Goal & Path Line

  if (currentGoal && gRover) {
    const gGoal = worldToGrid(currentGoal.x, currentGoal.z);
    if (gGoal) {
      slamCtx.strokeStyle = '#00e676';
      slamCtx.lineWidth = 2;
      slamCtx.setLineDash([4, 4]);
      slamCtx.beginPath();
      slamCtx.moveTo(gRover.gx * cellPx + cellPx / 2, gRover.gz * cellPx + cellPx / 2);
      slamCtx.lineTo(gGoal.gx * cellPx + cellPx / 2, gGoal.gz * cellPx + cellPx / 2);
      slamCtx.stroke();
      slamCtx.setLineDash([]);

      // Goal target marker
      slamCtx.fillStyle = '#00e676';
      slamCtx.beginPath();
      slamCtx.arc(gGoal.gx * cellPx + cellPx / 2, gGoal.gz * cellPx + cellPx / 2, 5, 0, Math.PI * 2);
      slamCtx.fill();
    }
  }

  // Draw Rover Position & Orientation
  if (gRover) {
    const rx = gRover.gx * cellPx + cellPx / 2;
    const rz = gRover.gz * cellPx + cellPx / 2;
    const rq = chassisBody.quaternion;
    const yaw = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w), 'YXZ').y;

    slamCtx.save();
    slamCtx.translate(rx, rz);
    slamCtx.rotate(-yaw);

    // Rover body on map
    slamCtx.fillStyle = '#4a9eff';
    slamCtx.strokeStyle = '#ffffff';
    slamCtx.lineWidth = 1;
    slamCtx.beginPath();
    slamCtx.moveTo(0, -6);
    slamCtx.lineTo(4, 5);
    slamCtx.lineTo(-4, 5);
    slamCtx.closePath();
    slamCtx.fill();
    slamCtx.stroke();
    slamCtx.restore();
  }
}

// ============================================================
// 16. WEATHER CONTROLS & SHORTCUTS
// ============================================================

document.querySelectorAll('.wbtn').forEach(btn => {
  btn.addEventListener('click', () => {
    setWeather(btn.dataset.weather);
  });
});

const WEATHER_KEYS = ['clear', 'fog', 'sandstorm', 'rain', 'night'];
document.addEventListener('keydown', e => {
  if (e.key === 'm' || e.key === 'M') toggleMapRecording();
  if (e.key === 'v' || e.key === 'V') {
    const idx = WEATHER_KEYS.indexOf(currentWeather);
    const nextKey = WEATHER_KEYS[(idx + 1) % WEATHER_KEYS.length];
    setWeather(nextKey);
  }
});

// Update particles animation in main loop
function updateWeatherParticles(dt, rp) {
  if (rainParticles.visible) {
    const pos = rainGeo.attributes.position.array;
    for (let i = 1; i < RAIN_COUNT * 3; i += 3) {
      pos[i] -= 22 * dt;
      if (pos[i] < 0) pos[i] = 20 + Math.random() * 5;
    }
    rainGeo.attributes.position.needsUpdate = true;
    rainParticles.position.set(rp.x, 0, rp.z);
  }

  if (dustParticles.visible) {
    const pos = dustGeo.attributes.position.array;
    for (let i = 0; i < DUST_COUNT * 3; i += 3) {
      pos[i] += 8 * dt; // Wind blown horizontally
      pos[i + 1] += Math.sin(Date.now() * 0.003 + i) * dt;
      if (pos[i] > 25) pos[i] = -25;
      if (pos[i + 1] < 0) pos[i + 1] = 10;
      if (pos[i + 1] > 12) pos[i + 1] = 0;
    }
    dustGeo.attributes.position.needsUpdate = true;
    dustParticles.position.set(rp.x, 0, rp.z);
  }
}
