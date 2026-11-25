/** Renderer, lights, ground plane and grid for the viewer. */

import * as THREE from "three";

export interface ViewerScene {
  readonly scene: THREE.Scene;
  readonly camera: THREE.PerspectiveCamera;
  readonly renderer: THREE.WebGLRenderer;
  readonly canvas: HTMLCanvasElement;
  resize(): void;
}

export function createViewerScene(container: HTMLElement): ViewerScene {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1020);
  scene.fog = new THREE.Fog(0x0b1020, 30, 110);

  const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 500);
  camera.position.set(3.4, 6.4, 4.2);
  camera.lookAt(0, 5, 0);

  const hemisphere = new THREE.HemisphereLight(0xbfd4ff, 0x172033, 1.5);
  scene.add(hemisphere);

  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(6, 12, 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  sun.shadow.camera.left = -12;
  sun.shadow.camera.right = 12;
  sun.shadow.camera.top = 12;
  sun.shadow.camera.bottom = -12;
  sun.shadow.camera.near = 0.5;
  sun.shadow.camera.far = 40;
  sun.shadow.bias = -0.0005;
  scene.add(sun);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(200, 200),
    new THREE.MeshStandardMaterial({
      color: 0x151b2e,
      roughness: 1,
      metalness: 0,
    }),
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  const grid = new THREE.GridHelper(100, 100, 0x3a4a6b, 0x202a40);
  grid.position.y = 0.002;
  const gridMaterial = Array.isArray(grid.material) ? grid.material[0] : grid.material;
  if (gridMaterial !== undefined) {
    gridMaterial.transparent = true;
    gridMaterial.opacity = 0.55;
  }
  scene.add(grid);

  const resize = (): void => {
    const width = container.clientWidth || window.innerWidth;
    const height = container.clientHeight || window.innerHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };
  resize();

  return {
    scene,
    camera,
    renderer,
    canvas: renderer.domElement,
    resize,
  };
}
