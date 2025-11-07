import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.150.1/build/three.module.js';
import { OrbitControls } from './OrbitControls.js';

// Variables globales
let scene, camera, renderer, controls;
let joints = []; // Array para almacenar las articulaciones
let links = []; // Array para almacenar los eslabones
let collisionSpheres = []; // Array para almacenar las esferas de colisión
let axisHelpers = []; // Array para almacenar los ejes de referencia
// Configuración inicial del brazo robótico usando matrices de transformación
const armDefinition = [
    {
        name: "Base",
        transform: mat4.fromValues(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1  // Matriz identidad para la base
        ),
        angle: 0,
        link: {
            size: [0.8, 0.8, 0.4],
            transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 90, 0, 0), [0, 0.2, 0])
        },
        color: 0x444444,
        collisionSpheres: [
            { position: [0, 0.2, 0], radius: 0.4 }
        ]
    },
    {
        name: "Hombro",
        transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 90, 0, 0), [0, 0.8, 0.2]),
        angle: 0,
        link: {
            size: [.5, 2, .5],
            transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 0, 0, 90), [0.8, -0.3, 0]),
        },
        color: 0x0066cc,
        collisionSpheres: [
            { position: [0, -.25, 0], radius: 0.25 },
            { position: [.5, -.25, 0], radius: 0.25 },
            { position: [1, -.25, 0], radius: 0.25 },
            { position: [1.5, -.25, 0], radius: 0.25 },
        ]
    },
    {
        name: "Codo",
        transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 0, -110, 0), [2, 0, 0]),
        angle: 0,
        link: {
            size: [.4, 2, .4],
            transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 0, 0, 90), [.8, -.3, 0]),
        },
        color: 0xcc3300,
        collisionSpheres: [
            { position: [0, -.25, 0], radius: 0.2 },
            { position: [0.4, -.25, 0], radius: 0.2 },
            { position: [0.8, -.25, 0], radius: 0.2 },
            { position: [1.2, -.25, 0], radius: 0.2 },
            { position: [1.6, -.25, 0], radius: 0.2 },
        ]
    },
    {
        name: "Muñeca",

        transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 0, 0, -90), [2.2, -.3, 0]),
        angle: 0,
        link: {
            size: [.7, 0.4, 0.7],
            transform: mat4.fromValues(
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, -0.2, 0, 1  // Traslación en Y
            )
        },
        color: 0x00cc66,
        collisionSpheres: [
            { position: [0, -.1, 0], radius: 0.3 }
        ]
    },
    {
        name: "Muñeca-2",
        transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 90, 0, 0), [0, .25, .3]),
        angle: 0,
        link: {
            size: [.5, 1, .5],
            transform: mat4.fromRotationTranslation(mat4.create(), quat.fromEuler(quat.create(), 90, 90, 0), [.25, -.25, 0]),
        },
        color: 0xcc3300,
        collisionSpheres: [
            { position: [0, -.25, 0], radius: 0.25 },
            { position: [0.5, -.25, 0], radius: 0.25 },
        ]
    },
];
// Inicialización
function init() {
    // Crear escena
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

    // Crear cámara
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(5, 5, 5);

    // Crear renderizador
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.getElementById('container').appendChild(renderer.domElement);

    // Controles de órbita
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Añadir luces
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 20, 15);
    scene.add(directionalLight);

    // Añadir ejes de referencia
    const axesHelper = new THREE.AxesHelper(3);
    scene.add(axesHelper);

    // Crear el brazo robótico
    createRobotArm();

    // Crear controles de interfaz
    createJointControls();

    // Manejar redimensionamiento de ventana
    window.addEventListener('resize', onWindowResize);

    // Iniciar animación
    animate();
}

// Convertir matriz gl-matrix a THREE.Matrix4
function glMatrixToThreeMatrix4(glMatrix) {
    return new THREE.Matrix4().set(
        glMatrix[0], glMatrix[4], glMatrix[8], glMatrix[12],
        glMatrix[1], glMatrix[5], glMatrix[9], glMatrix[13],
        glMatrix[2], glMatrix[6], glMatrix[10], glMatrix[14],
        glMatrix[3], glMatrix[7], glMatrix[11], glMatrix[15]
    );
}

// Aplicar transformación matricial a un objeto Three.js
function applyTransformToObject(object, transformMatrix) {
    const threeMatrix = glMatrixToThreeMatrix4(transformMatrix);
    object.matrix.identity();
    object.applyMatrix4(threeMatrix);
    object.matrixAutoUpdate = false;
}

function getJointWorldTransform(jointIndex) {
    let worldMatrix = mat4.create();
    mat4.identity(worldMatrix);

    for (let i = 0; i <= jointIndex; i++) {
        // Aplicar transformación base de la articulación
        mat4.multiply(worldMatrix, worldMatrix, armDefinition[i].transform);

        // Aplicar rotación de la articulación si existe
        if (joints[i] && joints[i].angle !== 0) {
            const rotationMatrix = mat4.create();
            mat4.identity(rotationMatrix);
            mat4.rotateY(rotationMatrix, rotationMatrix, THREE.MathUtils.degToRad(joints[i].angle));
            mat4.multiply(worldMatrix, worldMatrix, rotationMatrix);
        }
    }

    return worldMatrix;
}

// Crear el brazo robótico
function createRobotArm() {
    // Inicializar articulaciones
    armDefinition.forEach((jointDef, index) => {
        joints.push({
            name: jointDef.name,
            angle: 0,
            worldMatrix: mat4.create()
        });
    });

    // Crear eslabones y esferas de colisión
    armDefinition.forEach((jointDef, jointIndex) => {
        // Crear el eslabón (caja)
        const linkGeometry = new THREE.BoxGeometry(
            jointDef.link.size[0],
            jointDef.link.size[1],
            jointDef.link.size[2]
        );
        const linkMaterial = new THREE.MeshPhongMaterial({
            color: jointDef.color,
            transparent: true,
            opacity: 0.3
        });
        const link = new THREE.Mesh(linkGeometry, linkMaterial);

        // Guardar referencia al eslabón
        links.push({
            mesh: link,
            jointIndex: jointIndex,
            localTransform: mat4.clone(jointDef.link.transform)
        });

        scene.add(link);

        // Añadir un pequeño marcador en el eje de rotación
        const axisMarkerGeometry = new THREE.CylinderGeometry(0.05, 0.05, 0.1, 8);
        const axisMarkerMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });
        const axisMarker = new THREE.Mesh(axisMarkerGeometry, axisMarkerMaterial);
        axisHelpers.push(axisMarker);
        scene.add(axisMarker);


        // Crear esferas de colisión
        jointDef.collisionSpheres.forEach((sphereDef, sphereIndex) => {
            const sphereGeometry = new THREE.SphereGeometry(sphereDef.radius, 16, 16);
            const sphereMaterial = new THREE.MeshPhongMaterial({
                color: 0x666666,
                transparent: false,
                opacity: 0.3
            });
            const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);

            // Posición local de la esfera
            const sphereLocalMatrix = mat4.create();
            mat4.identity(sphereLocalMatrix);
            mat4.translate(sphereLocalMatrix, sphereLocalMatrix, sphereDef.position);

            collisionSpheres.push({
                mesh: sphere,
                jointIndex: jointIndex,
                localTransform: sphereLocalMatrix
            });

            scene.add(sphere);
        });
    });

    // Actualizar todas las transformaciones
    updateAllTransforms();
}

// Actualizar todas las transformaciones del brazo
function updateAllTransforms() {
    // Actualizar transformaciones de eslabones
    links.forEach(link => {
        const jointWorldMatrix = getJointWorldTransform(link.jointIndex);
        const finalMatrix = mat4.create();
        mat4.multiply(finalMatrix, jointWorldMatrix, link.localTransform);
        applyTransformToObject(link.mesh, finalMatrix);
    });

    // Actualizar transformaciones de esferas de colisión
    collisionSpheres.forEach((sphere, index) => {
        const jointWorldMatrix = getJointWorldTransform(sphere.jointIndex);
        const finalMatrix = mat4.create();
        mat4.multiply(finalMatrix, jointWorldMatrix, sphere.localTransform);
        applyTransformToObject(sphere.mesh, finalMatrix);
    });

    axisHelpers.forEach((axisHelper, index) => {
        const jointWorldMatrix = getJointWorldTransform(index);
        applyTransformToObject(axisHelper, jointWorldMatrix);
    });
}

// Crear controles de interfaz para las articulaciones
function createJointControls() {
    const controlsContainer = document.getElementById('joint-controls');
    controlsContainer.innerHTML = '';

    joints.forEach((joint, index) => {
        const controlDiv = document.createElement('div');
        controlDiv.className = 'joint-control';

        const label = document.createElement('label');
        label.textContent = `${joint.name}: `;

        const angleValue = document.createElement('span');
        angleValue.className = 'angle-value';
        angleValue.textContent = '0°';

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.min = '0';
        slider.max = '180';
        slider.value = '0';
        slider.step = '1';

        slider.addEventListener('input', function () {
            const angle = parseInt(this.value);
            angleValue.textContent = `${angle}°`;
            setJointAngle(index, angle);
        });

        label.appendChild(angleValue);
        controlDiv.appendChild(label);
        controlDiv.appendChild(slider);
        controlsContainer.appendChild(controlDiv);
    });
}

// Establecer ángulo de una articulación
function setJointAngle(jointIndex, angle) {
    if (jointIndex >= 0 && jointIndex < joints.length) {
        joints[jointIndex].angle = angle;
        updateAllTransforms();
    }
}

// Resetear todos los ángulos a 0
function resetJointAngles() {
    joints.forEach((joint, index) => {
        setJointAngle(index, 0);
        // Actualizar sliders
        const sliders = document.querySelectorAll('#joint-controls input[type="range"]');
        if (sliders[index]) {
            sliders[index].value = '0';
            sliders[index].previousElementSibling.querySelector('.angle-value').textContent = '0°';
        }
    });
}

// Manejar redimensionamiento de ventana
function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

// Bucle de animación
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Inicializar cuando se carga la página
window.addEventListener('DOMContentLoaded', () => {
    init();
    resetJointAngles();

    // Configurar botón de reset
    document.getElementById('reset-btn').addEventListener('click', resetJointAngles);
});
window.armDefinition = armDefinition; // Exponer para depuración