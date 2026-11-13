/*
  3D LiDAR mapping simulation for the Autonomous Pookie page.

  A model room (8 m x 5 m x 2.5 m, 1 unit = 1 cm) with furniture at real heights.
  The car drives a fixed path while a 16-beam spinning LiDAR, mounted 30 cm up,
  casts rays in 3D. Every ray that hits the floor, a wall, the ceiling, or furniture
  leaves a point, colored by distance from the car (blue = near, red = far), like the
  real point cloud scans. The camera orbits slowly; drag to rotate it.

  This is a model room, not recorded data. The page labels it "Simulation".
*/
(function runLidarSimulation() {
  const simulationFigure = document.getElementById("lidar-simulation");
  if (!simulationFigure) return;

  const displayCanvas = simulationFigure.querySelector(".lidar-sim__canvas");
  const toggleButton = document.getElementById("lidar-sim-toggle");
  const replayButton = document.getElementById("lidar-sim-replay");
  const pointsMappedLabel = document.getElementById("lidar-sim-points");
  const distanceDrivenLabel = document.getElementById("lidar-sim-distance");
  const displayContext = displayCanvas.getContext("2d");
  const prefersReducedMotion = window.matchMedia(
    "(prefers-reduced-motion: reduce)",
  ).matches;

  // ---------- Model room (centimeters; x = width, y = depth, z = height) ----------
  const roomWidth = 800;
  const roomDepth = 500;
  const roomHeight = 250;
  const doorway = { startX: 560, endX: 660, height: 210 }; // opening in the y = roomDepth wall

  const furnitureBoxes = [
    { name: "workbench", x: 60, y: 20, width: 220, depth: 60, height: 90 },
    { name: "shelf", x: 720, y: 80, width: 60, depth: 180, height: 180 },
    { name: "table", x: 330, y: 200, width: 160, depth: 90, height: 75 },
    { name: "storage box", x: 250, y: 250, width: 50, depth: 50, height: 50 },
    { name: "cabinet", x: 20, y: 380, width: 90, depth: 100, height: 100 },
    { name: "couch", x: 660, y: 300, width: 100, depth: 60, height: 80 },
    {
      name: "pillar",
      x: 600,
      y: 160,
      width: 30,
      depth: 30,
      height: roomHeight,
    },
  ].map((box) => ({
    minX: box.x,
    minY: box.y,
    minZ: 0,
    maxX: box.x + box.width,
    maxY: box.y + box.depth,
    maxZ: box.height,
  }));

  // ---------- Driving path: enters through the doorway and loops the room ----------
  const pathWaypoints = [
    [610, 470],
    [600, 400],
    [520, 320],
    [540, 140],
    [400, 130],
    [250, 140],
    [120, 200],
    [110, 300],
    [260, 430],
    [420, 400],
    [470, 360],
  ];

  // Smooth the waypoints with a Catmull-Rom curve, then measure its length for constant speed
  function buildSmoothPath(waypoints, samplesPerSegment) {
    const smoothPoints = [];
    for (
      let segmentIndex = 0;
      segmentIndex < waypoints.length - 1;
      segmentIndex++
    ) {
      const previousPoint = waypoints[Math.max(segmentIndex - 1, 0)];
      const startPoint = waypoints[segmentIndex];
      const endPoint = waypoints[segmentIndex + 1];
      const nextPoint =
        waypoints[Math.min(segmentIndex + 2, waypoints.length - 1)];
      for (
        let sampleIndex = 0;
        sampleIndex < samplesPerSegment;
        sampleIndex++
      ) {
        const t = sampleIndex / samplesPerSegment;
        const blend = (a, b, c, d) =>
          0.5 *
          (2 * b +
            (-a + c) * t +
            (2 * a - 5 * b + 4 * c - d) * t * t +
            (-a + 3 * b - 3 * c + d) * t * t * t);
        smoothPoints.push([
          blend(previousPoint[0], startPoint[0], endPoint[0], nextPoint[0]),
          blend(previousPoint[1], startPoint[1], endPoint[1], nextPoint[1]),
        ]);
      }
    }
    smoothPoints.push(waypoints[waypoints.length - 1]);

    const cumulativeLengths = [0];
    for (let pointIndex = 1; pointIndex < smoothPoints.length; pointIndex++) {
      const deltaX =
        smoothPoints[pointIndex][0] - smoothPoints[pointIndex - 1][0];
      const deltaY =
        smoothPoints[pointIndex][1] - smoothPoints[pointIndex - 1][1];
      cumulativeLengths.push(
        cumulativeLengths[pointIndex - 1] + Math.hypot(deltaX, deltaY),
      );
    }
    return {
      smoothPoints,
      cumulativeLengths,
      totalLength: cumulativeLengths[cumulativeLengths.length - 1],
    };
  }

  const drivingPath = buildSmoothPath(pathWaypoints, 40);

  function carPoseAtDistance(distanceAlongPath) {
    const { smoothPoints, cumulativeLengths } = drivingPath;
    let pointIndex = 1;
    while (
      pointIndex < cumulativeLengths.length - 1 &&
      cumulativeLengths[pointIndex] < distanceAlongPath
    )
      pointIndex++;
    const segmentStartLength = cumulativeLengths[pointIndex - 1];
    const segmentLength =
      cumulativeLengths[pointIndex] - segmentStartLength || 1;
    const fraction = Math.min(
      Math.max((distanceAlongPath - segmentStartLength) / segmentLength, 0),
      1,
    );
    const [startX, startY] = smoothPoints[pointIndex - 1];
    const [endX, endY] = smoothPoints[pointIndex];
    return {
      x: startX + (endX - startX) * fraction,
      y: startY + (endY - startY) * fraction,
      heading: Math.atan2(endY - startY, endX - startX),
    };
  }

  // ---------- LiDAR: 16 beams from -15 to +15 degrees, spinning ----------
  const lidarMountHeight = 30; // cm above the floor
  const lidarMaxRange = 600; // cm
  const beamElevationsInDegrees = Array.from(
    { length: 16 },
    (_, beamIndex) => -15 + beamIndex * 2,
  );
  const beamElevations = beamElevationsInDegrees.map(
    (degrees) => (degrees * Math.PI) / 180,
  );
  const horizontalStep = (2 * Math.PI) / 180; // 2 degree columns
  const columnsPerSecond = 540; // 3 revolutions per second
  const carSpeed = 110; // cm per second
  const measurementNoise = 1.5; // cm
  const voxelSize = 5; // keep at most one point per 5 cm cube, like a downsampled map

  // Distance color ramp, matching the real scans: blue (near) to red (far)
  const distanceColorStops = [
    [0.0, [47, 107, 255]],
    [0.25, [34, 211, 238]],
    [0.5, [74, 222, 128]],
    [0.75, [250, 204, 21]],
    [1.0, [248, 113, 113]],
  ];
  const nearDistance = 40;
  const farDistance = 450;
  const colorBucketCount = 24;
  const colorBuckets = Array.from(
    { length: colorBucketCount },
    (_, bucketIndex) => {
      const position = bucketIndex / (colorBucketCount - 1);
      for (
        let stopIndex = 1;
        stopIndex < distanceColorStops.length;
        stopIndex++
      ) {
        const [upperPosition, upperColor] = distanceColorStops[stopIndex];
        if (position <= upperPosition + 1e-9) {
          const [lowerPosition, lowerColor] = distanceColorStops[stopIndex - 1];
          const mix =
            (position - lowerPosition) / (upperPosition - lowerPosition);
          const channel = (index) =>
            Math.round(
              lowerColor[index] + (upperColor[index] - lowerColor[index]) * mix,
            );
          return `rgb(${channel(0)}, ${channel(1)}, ${channel(2)})`;
        }
      }
      return "rgb(248, 113, 113)";
    },
  );

  function colorBucketForDistance(distance) {
    const position = Math.min(
      Math.max((distance - nearDistance) / (farDistance - nearDistance), 0),
      1,
    );
    return Math.round(position * (colorBucketCount - 1));
  }

  // Distance along a 3D ray to the nearest surface, or null if nothing is in range
  function castRay(
    originX,
    originY,
    originZ,
    directionX,
    directionY,
    directionZ,
  ) {
    // Inside the room: the ray leaves through whichever wall, floor, or ceiling it reaches first
    const exitDistanceX =
      directionX > 0
        ? (roomWidth - originX) / directionX
        : directionX < 0
          ? -originX / directionX
          : Infinity;
    const exitDistanceY =
      directionY > 0
        ? (roomDepth - originY) / directionY
        : directionY < 0
          ? -originY / directionY
          : Infinity;
    const exitDistanceZ =
      directionZ > 0
        ? (roomHeight - originZ) / directionZ
        : directionZ < 0
          ? -originZ / directionZ
          : Infinity;
    let nearestHitDistance = Math.min(
      exitDistanceX,
      exitDistanceY,
      exitDistanceZ,
    );

    // Rays leaving through the open doorway hit nothing
    if (nearestHitDistance === exitDistanceY && directionY > 0) {
      const exitX = originX + directionX * nearestHitDistance;
      const exitZ = originZ + directionZ * nearestHitDistance;
      if (
        exitX > doorway.startX &&
        exitX < doorway.endX &&
        exitZ < doorway.height
      )
        nearestHitDistance = Infinity;
    }

    // Furniture: standard slab test against each box
    for (const box of furnitureBoxes) {
      let entryDistance = -Infinity;
      let leaveDistance = Infinity;
      for (const [origin, direction, minimum, maximum] of [
        [originX, directionX, box.minX, box.maxX],
        [originY, directionY, box.minY, box.maxY],
        [originZ, directionZ, box.minZ, box.maxZ],
      ]) {
        if (Math.abs(direction) < 1e-9) {
          if (origin < minimum || origin > maximum) {
            entryDistance = Infinity;
            break;
          }
          continue;
        }
        let nearPlane = (minimum - origin) / direction;
        let farPlane = (maximum - origin) / direction;
        if (nearPlane > farPlane) [nearPlane, farPlane] = [farPlane, nearPlane];
        entryDistance = Math.max(entryDistance, nearPlane);
        leaveDistance = Math.min(leaveDistance, farPlane);
      }
      if (
        entryDistance > 0 &&
        entryDistance <= leaveDistance &&
        entryDistance < nearestHitDistance
      ) {
        nearestHitDistance = entryDistance;
      }
    }
    return nearestHitDistance <= lidarMaxRange ? nearestHitDistance : null;
  }

  // ---------- Simulation state ----------
  let pointsByColorBucket = [];
  let mappedPointCount = 0;
  let occupiedVoxels = new Set();
  let latestSweepRays = []; // horizontal beam of the latest columns, drawn as a faint fan
  let drivenTrail = [];
  let distanceDriven = 0;
  let lidarHeading = 0;
  let pendingColumns = 0;
  let isRunning = false;
  let isFinished = false;
  let lastFrameTime = null;
  let animationFrameId = null;

  function resetSimulation() {
    pointsByColorBucket = Array.from({ length: colorBucketCount }, () => []);
    mappedPointCount = 0;
    occupiedVoxels = new Set();
    latestSweepRays = [];
    drivenTrail = [];
    distanceDriven = 0;
    lidarHeading = 0;
    pendingColumns = 0;
    isFinished = false;
    lastFrameTime = null;
  }

  function scanColumn(carPose) {
    lidarHeading += horizontalStep;
    const lidarX = carPose.x;
    const lidarY = carPose.y;
    const cosHeading = Math.cos(lidarHeading);
    const sinHeading = Math.sin(lidarHeading);
    beamElevations.forEach((elevation, beamIndex) => {
      const horizontalPart = Math.cos(elevation);
      const directionX = cosHeading * horizontalPart;
      const directionY = sinHeading * horizontalPart;
      const directionZ = Math.sin(elevation);
      const hitDistance = castRay(
        lidarX,
        lidarY,
        lidarMountHeight,
        directionX,
        directionY,
        directionZ,
      );
      if (beamIndex === 7)
        latestSweepRays.push({
          directionX,
          directionY,
          directionZ,
          length: hitDistance ?? lidarMaxRange,
        });
      if (hitDistance === null) return;

      const noisyDistance =
        hitDistance + (Math.random() - 0.5) * 2 * measurementNoise;
      const pointX = lidarX + directionX * noisyDistance;
      const pointY = lidarY + directionY * noisyDistance;
      const pointZ = lidarMountHeight + directionZ * noisyDistance;
      const voxelKey = `${Math.floor(pointX / voxelSize)},${Math.floor(pointY / voxelSize)},${Math.floor(pointZ / voxelSize)}`;
      if (occupiedVoxels.has(voxelKey)) return;
      occupiedVoxels.add(voxelKey);
      pointsByColorBucket[colorBucketForDistance(hitDistance)].push(
        pointX,
        pointY,
        pointZ,
      );
      mappedPointCount++;
    });
  }

  function advanceSimulation(elapsedSeconds) {
    distanceDriven = Math.min(
      distanceDriven + carSpeed * elapsedSeconds,
      drivingPath.totalLength,
    );
    const carPose = carPoseAtDistance(distanceDriven);
    drivenTrail.push([carPose.x, carPose.y]);

    pendingColumns += columnsPerSecond * elapsedSeconds;
    const columnsThisStep = Math.floor(pendingColumns);
    pendingColumns -= columnsThisStep;
    latestSweepRays = [];
    for (let columnIndex = 0; columnIndex < columnsThisStep; columnIndex++)
      scanColumn(carPose);
    if (distanceDriven >= drivingPath.totalLength) isFinished = true;
  }

  // ---------- Camera: orbits the room center; drag to rotate ----------
  const cameraTarget = { x: roomWidth / 2, y: roomDepth / 2, z: 40 };
  const cameraDistance = 980;
  let cameraAzimuth = (-115 * Math.PI) / 180;
  let cameraElevation = (38 * Math.PI) / 180;
  const orbitSpeed = (5 * Math.PI) / 180; // radians per second while the car drives
  let devicePixelScale = 1;
  let focalLength = 1;
  let cameraBasis = null;

  function sizeCanvas() {
    devicePixelScale = Math.min(window.devicePixelRatio || 1, 2);
    const displayWidth = displayCanvas.clientWidth || 800;
    displayCanvas.width = Math.round(displayWidth * devicePixelScale);
    displayCanvas.height = Math.round(
      displayWidth * (5 / 8) * devicePixelScale,
    );
    focalLength = displayCanvas.height * 1.35;
  }

  function updateCameraBasis() {
    const cosElevation = Math.cos(cameraElevation);
    const position = {
      x:
        cameraTarget.x +
        cameraDistance * cosElevation * Math.cos(cameraAzimuth),
      y:
        cameraTarget.y +
        cameraDistance * cosElevation * Math.sin(cameraAzimuth),
      z: cameraTarget.z + cameraDistance * Math.sin(cameraElevation),
    };
    const forward = normalize({
      x: cameraTarget.x - position.x,
      y: cameraTarget.y - position.y,
      z: cameraTarget.z - position.z,
    });
    const right = normalize(cross(forward, { x: 0, y: 0, z: 1 }));
    const up = cross(right, forward);
    cameraBasis = { position, forward, right, up };
  }

  function normalize(vector) {
    const length = Math.hypot(vector.x, vector.y, vector.z) || 1;
    return { x: vector.x / length, y: vector.y / length, z: vector.z / length };
  }

  function cross(a, b) {
    return {
      x: a.y * b.z - a.z * b.y,
      y: a.z * b.x - a.x * b.z,
      z: a.x * b.y - a.y * b.x,
    };
  }

  // World point to screen pixels; returns null for points behind the camera
  function projectPoint(worldX, worldY, worldZ) {
    const { position, forward, right, up } = cameraBasis;
    const offsetX = worldX - position.x;
    const offsetY = worldY - position.y;
    const offsetZ = worldZ - position.z;
    const depth =
      offsetX * forward.x + offsetY * forward.y + offsetZ * forward.z;
    if (depth < 20) return null;
    const sideways = offsetX * right.x + offsetY * right.y + offsetZ * right.z;
    const upward = offsetX * up.x + offsetY * up.y + offsetZ * up.z;
    return {
      screenX: displayCanvas.width / 2 + (sideways * focalLength) / depth,
      screenY: displayCanvas.height / 2 - (upward * focalLength) / depth,
      depth,
    };
  }

  function drawLine3D(fromX, fromY, fromZ, toX, toY, toZ) {
    const start = projectPoint(fromX, fromY, fromZ);
    const end = projectPoint(toX, toY, toZ);
    if (!start || !end) return;
    displayContext.moveTo(start.screenX, start.screenY);
    displayContext.lineTo(end.screenX, end.screenY);
  }

  function drawFrame() {
    updateCameraBasis();
    displayContext.fillStyle = "#0d141c";
    displayContext.fillRect(0, 0, displayCanvas.width, displayCanvas.height);

    // Faint 1 m floor grid
    displayContext.strokeStyle = "rgba(255, 255, 255, 0.06)";
    displayContext.lineWidth = 1;
    displayContext.beginPath();
    for (let gridX = 0; gridX <= roomWidth; gridX += 100)
      drawLine3D(gridX, 0, 0, gridX, roomDepth, 0);
    for (let gridY = 0; gridY <= roomDepth; gridY += 100)
      drawLine3D(0, gridY, 0, roomWidth, gridY, 0);
    displayContext.stroke();

    // Point cloud, one fill color per distance bucket
    const pointSizeAtTarget = 2.1 * devicePixelScale;
    for (let bucketIndex = 0; bucketIndex < colorBucketCount; bucketIndex++) {
      const bucketPoints = pointsByColorBucket[bucketIndex];
      if (!bucketPoints.length) continue;
      displayContext.fillStyle = colorBuckets[bucketIndex];
      for (
        let coordinateIndex = 0;
        coordinateIndex < bucketPoints.length;
        coordinateIndex += 3
      ) {
        const projected = projectPoint(
          bucketPoints[coordinateIndex],
          bucketPoints[coordinateIndex + 1],
          bucketPoints[coordinateIndex + 2],
        );
        if (!projected) continue;
        const pointSize = Math.min(
          Math.max(
            (pointSizeAtTarget * cameraDistance) / projected.depth,
            devicePixelScale,
          ),
          3.2 * devicePixelScale,
        );
        displayContext.fillRect(
          projected.screenX - pointSize / 2,
          projected.screenY - pointSize / 2,
          pointSize,
          pointSize,
        );
      }
    }

    // Path driven so far, on the floor
    if (drivenTrail.length > 1) {
      displayContext.strokeStyle = "rgba(255, 255, 255, 0.4)";
      displayContext.lineWidth = 1.5 * devicePixelScale;
      displayContext.setLineDash([4 * devicePixelScale, 4 * devicePixelScale]);
      displayContext.beginPath();
      for (
        let trailIndex = 1;
        trailIndex < drivenTrail.length;
        trailIndex += 2
      ) {
        const [fromX, fromY] = drivenTrail[trailIndex - 1];
        const [toX, toY] = drivenTrail[trailIndex];
        drawLine3D(fromX, fromY, 1, toX, toY, 1);
      }
      displayContext.stroke();
      displayContext.setLineDash([]);
    }

    const carPose = carPoseAtDistance(distanceDriven);

    // Current sweep of the near-horizontal beam
    displayContext.strokeStyle = "rgba(125, 211, 252, 0.2)";
    displayContext.lineWidth = 1 * devicePixelScale;
    displayContext.beginPath();
    latestSweepRays.forEach(
      ({ directionX, directionY, directionZ, length }) => {
        drawLine3D(
          carPose.x,
          carPose.y,
          lidarMountHeight,
          carPose.x + directionX * length,
          carPose.y + directionY * length,
          lidarMountHeight + directionZ * length,
        );
      },
    );
    displayContext.stroke();

    drawCar(carPose);

    pointsMappedLabel.textContent = mappedPointCount.toLocaleString("en-US");
    distanceDrivenLabel.textContent = (distanceDriven / 100).toFixed(1) + " m";
  }

  // Car: a 40 x 20 x 15 cm body with a darker nose, and the LiDAR on a short mast
  function drawCar(carPose) {
    const cosHeading = Math.cos(carPose.heading);
    const sinHeading = Math.sin(carPose.heading);
    const toWorld = (forwardOffset, sideOffset, height) => [
      carPose.x + forwardOffset * cosHeading - sideOffset * sinHeading,
      carPose.y + forwardOffset * sinHeading + sideOffset * cosHeading,
      height,
    ];
    const fillFace = (corners, color) => {
      const projectedCorners = corners.map((corner) => projectPoint(...corner));
      if (projectedCorners.some((corner) => !corner)) return;
      displayContext.fillStyle = color;
      displayContext.beginPath();
      projectedCorners.forEach(({ screenX, screenY }, cornerIndex) =>
        cornerIndex
          ? displayContext.lineTo(screenX, screenY)
          : displayContext.moveTo(screenX, screenY),
      );
      displayContext.closePath();
      displayContext.fill();
    };
    const bodyHeight = 15;
    fillFace(
      [
        toWorld(-20, -10, 0),
        toWorld(20, -10, 0),
        toWorld(20, -10, bodyHeight),
        toWorld(-20, -10, bodyHeight),
      ],
      "#94a3b8",
    );
    fillFace(
      [
        toWorld(-20, 10, 0),
        toWorld(20, 10, 0),
        toWorld(20, 10, bodyHeight),
        toWorld(-20, 10, bodyHeight),
      ],
      "#94a3b8",
    );
    fillFace(
      [
        toWorld(-20, -10, bodyHeight),
        toWorld(8, -10, bodyHeight),
        toWorld(8, 10, bodyHeight),
        toWorld(-20, 10, bodyHeight),
      ],
      "#e2e8f0",
    );
    fillFace(
      [
        toWorld(8, -10, bodyHeight),
        toWorld(20, -10, bodyHeight),
        toWorld(20, 10, bodyHeight),
        toWorld(8, 10, bodyHeight),
      ],
      "#3b6ea5",
    );

    const mastBase = projectPoint(carPose.x, carPose.y, bodyHeight);
    const lidarCenter = projectPoint(carPose.x, carPose.y, lidarMountHeight);
    if (mastBase && lidarCenter) {
      displayContext.strokeStyle = "#cbd5e1";
      displayContext.lineWidth = 2 * devicePixelScale;
      displayContext.beginPath();
      displayContext.moveTo(mastBase.screenX, mastBase.screenY);
      displayContext.lineTo(lidarCenter.screenX, lidarCenter.screenY);
      displayContext.stroke();
      displayContext.fillStyle = "#7dd3fc";
      displayContext.beginPath();
      displayContext.arc(
        lidarCenter.screenX,
        lidarCenter.screenY,
        3.5 * devicePixelScale,
        0,
        Math.PI * 2,
      );
      displayContext.fill();
    }
  }

  // ---------- Playback ----------
  function animationStep(frameTime) {
    if (!isRunning) return;
    const elapsedSeconds =
      lastFrameTime === null
        ? 0
        : Math.min((frameTime - lastFrameTime) / 1000, 0.05);
    lastFrameTime = frameTime;
    advanceSimulation(elapsedSeconds);
    if (!isDragging && Date.now() > autoOrbitResumesAt)
      cameraAzimuth += orbitSpeed * elapsedSeconds;
    if (isFinished) {
      latestSweepRays = [];
      stopPlayback();
      drawFrame();
      return;
    }
    drawFrame();
    animationFrameId = requestAnimationFrame(animationStep);
  }

  function startPlayback() {
    if (isRunning) return;
    if (isFinished) resetSimulation();
    isRunning = true;
    lastFrameTime = null;
    toggleButton.textContent = "Pause";
    toggleButton.setAttribute("aria-pressed", "false");
    animationFrameId = requestAnimationFrame(animationStep);
  }

  function stopPlayback() {
    isRunning = false;
    cancelAnimationFrame(animationFrameId);
    toggleButton.textContent = isFinished ? "Play" : "Resume";
    toggleButton.setAttribute("aria-pressed", "true");
  }

  // Run the whole drive at once (used for reduced motion)
  function jumpToFinishedMap() {
    resetSimulation();
    while (!isFinished) advanceSimulation(1 / 30);
    latestSweepRays = [];
    drawFrame();
    toggleButton.textContent = "Play";
  }

  toggleButton.addEventListener("click", () =>
    isRunning ? stopPlayback() : startPlayback(),
  );
  replayButton.addEventListener("click", () => {
    stopPlayback();
    resetSimulation();
    startPlayback();
  });

  // Drag to rotate (mouse: both directions; touch: sideways only, so the page still scrolls)
  let isDragging = false;
  let lastPointerX = 0;
  let lastPointerY = 0;
  let autoOrbitResumesAt = 0;
  displayCanvas.addEventListener("pointerdown", (pointerEvent) => {
    isDragging = true;
    lastPointerX = pointerEvent.clientX;
    lastPointerY = pointerEvent.clientY;
    displayCanvas.setPointerCapture(pointerEvent.pointerId);
  });
  displayCanvas.addEventListener("pointermove", (pointerEvent) => {
    if (!isDragging) return;
    cameraAzimuth -= (pointerEvent.clientX - lastPointerX) * 0.008;
    if (pointerEvent.pointerType === "mouse") {
      const minimumElevation = (8 * Math.PI) / 180;
      const maximumElevation = (82 * Math.PI) / 180;
      cameraElevation = Math.min(
        Math.max(
          cameraElevation + (pointerEvent.clientY - lastPointerY) * 0.006,
          minimumElevation,
        ),
        maximumElevation,
      );
    }
    lastPointerX = pointerEvent.clientX;
    lastPointerY = pointerEvent.clientY;
    autoOrbitResumesAt = Date.now() + 4000;
    if (!isRunning) drawFrame();
  });
  const endDrag = () => {
    isDragging = false;
  };
  displayCanvas.addEventListener("pointerup", endDrag);
  displayCanvas.addEventListener("pointercancel", endDrag);

  let resizeTimer = null;
  window.addEventListener("resize", () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(() => {
      sizeCanvas();
      drawFrame();
    }, 150);
  });

  sizeCanvas();
  resetSimulation();

  if (prefersReducedMotion) {
    jumpToFinishedMap();
    return;
  }

  // At rest: one full LiDAR revolution from the doorway, so the first frame already shows a cloud
  for (let columnIndex = 0; columnIndex < 180; columnIndex++)
    scanColumn(carPoseAtDistance(0));
  latestSweepRays = [];
  drawFrame();
  toggleButton.textContent = "Play";

  // Start when the figure is mostly on screen; pause when it scrolls away
  let startedByScroll = false;
  let pausedByScroll = false;
  const visibilityWatcher = new IntersectionObserver(
    ([visibilityEntry]) => {
      if (
        visibilityEntry.isIntersecting &&
        (!startedByScroll || pausedByScroll)
      ) {
        startedByScroll = true;
        pausedByScroll = false;
        startPlayback();
      } else if (!visibilityEntry.isIntersecting && isRunning) {
        pausedByScroll = true;
        stopPlayback();
      }
    },
    { threshold: 0.5 },
  );
  visibilityWatcher.observe(displayCanvas);
})();
