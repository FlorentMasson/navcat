import GUI from 'lil-gui';
import { type Vec3, vec3 } from 'mathcat';
import {
    addOffMeshConnection,
    ANY_QUERY_FILTER,
    findNodePath,
    FindNodePathResultFlags,
    findRandomPoint,
    FindRandomPointResult,
    getNodeByRef,
    getNodeRefType,
    NodeType,
    OffMeshConnectionDirection,
    type OffMeshConnectionParams,
    type QueryFilter,
} from 'navcat';
import { generateTiledNavMesh, type TiledNavMeshInput, type TiledNavMeshOptions } from 'navcat/blocks';
import {
    createNavMeshHelper,
    createNavMeshOffMeshConnectionsHelper,
    getPositionsAndIndices,
} from 'navcat/three';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { createExample } from './common/example-base';
import { loadGLTF } from './common/load-gltf';

/* setup example scene */
const container = document.getElementById('root')!;
const { scene, camera, renderer } = await createExample(container);

camera.position.set(-2, 10, 10);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

const navTestModel = await loadGLTF('./models/nav-test.glb');
scene.add(navTestModel.scene);

const parameters = {
    iterations: 1e5,
}

/* setup gui */
const gui = new GUI();
gui.title('Benchmark Parameters');

gui.add(parameters, 'iterations', 1000, 1e6, 1000);
gui.add({ runBenchmark }, 'runBenchmark').name('Run benchmark');


/* generate navmesh */
const walkableMeshes: THREE.Mesh[] = [];
scene.traverse((object) => {
    if (object instanceof THREE.Mesh) {
        walkableMeshes.push(object);
    }
});

const [positions, indices] = getPositionsAndIndices(walkableMeshes);

const navMeshInput: TiledNavMeshInput = {
    positions,
    indices,
};

const cellSize = 0.15;
const cellHeight = 0.15;

const tileSizeVoxels = 64;
const tileSizeWorld = tileSizeVoxels * cellSize;

const walkableRadiusWorld = 0.1;
const walkableRadiusVoxels = Math.ceil(walkableRadiusWorld / cellSize);
const walkableClimbWorld = 0.5;
const walkableClimbVoxels = Math.ceil(walkableClimbWorld / cellHeight);
const walkableHeightWorld = 0.25;
const walkableHeightVoxels = Math.ceil(walkableHeightWorld / cellHeight);
const walkableSlopeAngleDegrees = 45;

const borderSize = 4;
const minRegionArea = 8;
const mergeRegionArea = 20;

const maxSimplificationError = 1.3;
const maxEdgeLength = 12;

const maxVerticesPerPoly = 5;

const detailSampleDistanceVoxels = 6;
const detailSampleDistance = detailSampleDistanceVoxels < 0.9 ? 0 : cellSize * detailSampleDistanceVoxels;

const detailSampleMaxErrorVoxels = 1;
const detailSampleMaxError = cellHeight * detailSampleMaxErrorVoxels;

const navMeshConfig: TiledNavMeshOptions = {
    cellSize,
    cellHeight,
    tileSizeVoxels,
    tileSizeWorld,
    walkableRadiusWorld,
    walkableRadiusVoxels,
    walkableClimbWorld,
    walkableClimbVoxels,
    walkableHeightWorld,
    walkableHeightVoxels,
    walkableSlopeAngleDegrees,
    borderSize,
    minRegionArea,
    mergeRegionArea,
    maxSimplificationError,
    maxEdgeLength,
    maxVerticesPerPoly,
    detailSampleDistance,
    detailSampleMaxError,
};

const navMeshResult = generateTiledNavMesh(navMeshInput, navMeshConfig);
const navMesh = navMeshResult.navMesh;

/* off mesh connection types */
enum OffMeshConnectionAreaType {
    TELEPORTER = 1,
    JUMP = 2,
    CLIMB = 3,
}

/* query filter */
const queryFilter: QueryFilter = {
    passFilter: (_nodeRef, _navMesh) => {
        return true;
    },
    getCost: (pa, pb, navMesh, _prevRef, curRef, _nextRef) => {
        // define the costs for traversing an off mesh connection
        if (curRef !== undefined && getNodeRefType(curRef) === NodeType.OFFMESH) {
            const { area } = getNodeByRef(navMesh, curRef);

            if (area === OffMeshConnectionAreaType.JUMP) {
                // regular distance
                return vec3.distance(pa, pb);
            } else if (area === OffMeshConnectionAreaType.CLIMB) {
                // distance * 4, big penalty
                return vec3.distance(pa, pb) * 4;
            } else if (area === OffMeshConnectionAreaType.TELEPORTER) {
                // low flat cost
                return 1;
            }
        }

        return vec3.distance(pa, pb);
    },
};

/* add off mesh connections */
const offMeshConnections: OffMeshConnectionParams[] = [
    {
        start: [-2.4799404316645157, 0.26716880587122915, 4.039628947351325],
        end: [-2.735661224133032, 2.3264200687408447, 0.9084349415865054],
        direction: OffMeshConnectionDirection.START_TO_END,
        radius: 0.5,
        area: OffMeshConnectionAreaType.TELEPORTER,
        flags: 0xffffff,
    },
    {
        start: [0.43153271761444056, 3.788429404449852, 2.549912418335899],
        end: [1.6203363597139502, 2.7055995008052136, 3.3892644209191634],
        direction: OffMeshConnectionDirection.START_TO_END,
        radius: 0.5,
        area: OffMeshConnectionAreaType.JUMP,
        flags: 0xffffff,
    },
    {
        start: [0.5997826320925559, 0.2668087168256541, 4.967287730406272],
        end: [1.580858144475107, 3.112976869830365, 4.670723413649996],
        direction: OffMeshConnectionDirection.START_TO_END,
        radius: 0.5,
        area: OffMeshConnectionAreaType.CLIMB,
        flags: 0xffffff,
    },
    {
        start: [3.54, 0.27, -3.89],
        end: [6.09, 0.69, -3.59],
        direction: OffMeshConnectionDirection.START_TO_END,
        radius: 0.5,
        area: 0,
        flags: 0xffffff,
    },
    {
        start: [6.09, 0.69, -3.59],
        end: [6.55, 0.39, -0.68],
        direction: OffMeshConnectionDirection.START_TO_END,
        radius: 0.5,
        area: 0,
        flags: 0xffffff,
    },
];

for (const connection of offMeshConnections) {
    addOffMeshConnection(navMesh, connection);
}

/* create debug helpers */
const navMeshHelper = createNavMeshHelper(navMesh);
navMeshHelper.object.position.y += 0.1;
scene.add(navMeshHelper.object);

const offMeshConnectionsHelper = createNavMeshOffMeshConnectionsHelper(navMesh);
scene.add(offMeshConnectionsHelper.object);

renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());

/* path stats */
const statsDiv = document.createElement('div');
statsDiv.style.position = 'absolute';
statsDiv.style.top = '10px';
statsDiv.style.left = '10px';
statsDiv.style.color = 'white';
statsDiv.style.fontFamily = 'monospace';
statsDiv.style.fontSize = '11px';
statsDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
statsDiv.style.padding = '10px';
statsDiv.style.borderRadius = '4px';
statsDiv.style.minWidth = '200px';
container.appendChild(statsDiv);

function runBenchmark() {
    let timeStart = performance.now();
    let pointPairs: Array<[number, Vec3, number, Vec3]> = [];
    let findRandomPointFailures = 0;
    for (let i = 0; i < parameters.iterations; i++) {
        let result: FindRandomPointResult, result2: FindRandomPointResult;
        do {
            result = findRandomPoint(navMesh, ANY_QUERY_FILTER, Math.random);
            result2 = findRandomPoint(navMesh, ANY_QUERY_FILTER, Math.random);
            if (!result.success) findRandomPointFailures++;
            if (!result2.success) findRandomPointFailures++;
        } while (!result.success || !result2.success);
        pointPairs.push([result.nodeRef, result.position, result2.nodeRef, result2.position]);
    }
    let timeAfterRandomPoint = performance.now();
    let pathFailures = 0;
    let partialPaths = 0;
    for (let i = 0; i < pointPairs.length; i++) {
        const result = findNodePath(navMesh, pointPairs[i][0], pointPairs[i][2], pointPairs[i][1], pointPairs[i][3], ANY_QUERY_FILTER);
        if ((result.flags & FindNodePathResultFlags.PARTIAL_PATH) !== 0)
            partialPaths++;
        if (!result.success) {
            pathFailures++;
        }
    }
    let timeAfterSearch = performance.now();

    let html = `<div style="margin-bottom: 8px; font-weight: bold; color: #00aaff;">Benchmark Report</div>`;

    // findRandomPoint
    html += `<div style="margin-bottom: 4px;">`;
    html += `<div style="color: #2196f3; font-weight: bold;">findRandomPoint</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">iterations: ${parameters.iterations}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">failures: ${findRandomPointFailures}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">total time: ${Math.round(timeAfterRandomPoint - timeStart)} ms</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">time per iteration: ${((timeAfterRandomPoint - timeStart) / parameters.iterations / 2).toFixed(4)} ms</div>`;
    html += `</div>`;

    // findNodePath
    html += `<div style="margin-bottom: 4px;">`;
    html += `<div style="color: #2196f3; font-weight: bold;">findNodePath</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">iterations: ${pointPairs.length}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">failures: ${pathFailures}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">partial paths: ${partialPaths} (${Math.round(partialPaths / pointPairs.length * 100)}%)</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">total time: ${Math.round(timeAfterSearch - timeAfterRandomPoint)} ms</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">time per iteration: ${((timeAfterSearch - timeAfterRandomPoint) / pointPairs.length).toFixed(4)} ms</div>`;
    html += `</div>`;

    statsDiv.innerHTML = html;

}

/* start loop */
function update() {
    requestAnimationFrame(update);

    orbitControls.update();
    renderer.render(scene, camera);
}

update();
