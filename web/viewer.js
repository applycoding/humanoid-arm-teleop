import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import URDFLoader from 'urdf-loader';

export function createViewer(container) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a2e);

  const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 20);
  camera.position.set(1.2, 1.0, 1.2);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);

  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 0.8);
  dir.position.set(2, 4, 2);
  scene.add(dir);

  const zUpRoot = new THREE.Group();
  zUpRoot.rotation.x = -Math.PI / 2;
  scene.add(zUpRoot);

  const axes = new THREE.AxesHelper(0.25);
  zUpRoot.add(axes);

  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.target.set(0.4, 0.3, 0);

  const targetMesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xff4444, wireframe: true })
  );
  zUpRoot.add(targetMesh);

  const transform = new TransformControls(camera, renderer.domElement);
  transform.attach(targetMesh);
  transform.setMode('translate');
  scene.add(transform.getHelper());
  transform.addEventListener('dragging-changed', (event) => {
    orbit.enabled = !event.value;
  });

  let robot = null;
  const loader = new URDFLoader();
  loader.load('/arm/arm.urdf', (r) => {
    robot = r;
    zUpRoot.add(robot);
  });

  function resize() {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function render() {
    renderer.render(scene, camera);
  }

  function setJointPositions(names, positions) {
    if (!robot) return;
    for (let i = 0; i < names.length; i++) {
      const joint = robot.joints[names[i]];
      if (joint) {
        joint.setJointValue(positions[i]);
      }
    }
  }

  function getTool0WorldPosition() {
    if (!robot || !robot.links.tool0) {
      return null;
    }
    const p = new THREE.Vector3();
    robot.links.tool0.getWorldPosition(p);
    return p;
  }

  function setTargetPosition(x, y, z) {
    targetMesh.position.set(x, y, z);
  }

  function getTargetPosition() {
    return targetMesh.position.clone();
  }

  function getTargetQuaternion() {
    return targetMesh.quaternion.clone();
  }

  function setTargetQuaternion(q) {
    targetMesh.quaternion.copy(q);
  }

  function countArmLinks() {
    if (!robot) return 0;
    return Object.keys(robot.links).filter((k) => k !== 'base_link').length;
  }

  transform.addEventListener('change', () => {
    if (transform.dragging && onTargetMoved) {
      onTargetMoved();
    }
  });

  let onTargetMoved = null;

  return {
    resize,
    render,
    setJointPositions,
    getTool0WorldPosition,
    setTargetPosition,
    getTargetPosition,
    getTargetQuaternion,
    setTargetQuaternion,
    countArmLinks,
    transform,
    targetMesh,
    set onTargetMoved(cb) {
      onTargetMoved = cb;
    },
    get onTargetMoved() {
      return onTargetMoved;
    },
  };
}
