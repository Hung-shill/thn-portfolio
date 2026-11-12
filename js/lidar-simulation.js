/*
  LiDAR mapping simulation for the Autonomous Pookie page.

  A top-down model room (8 m x 5 m, 1 unit = 1 cm). The car drives a fixed path while a
  spinning LiDAR casts rays; every ray that hits a wall or piece of furniture leaves a dot,
  colored by distance from the car (blue = near, red = far), like the real point cloud scans.

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

  // ---------- Model room (units are centimeters) ----------
  const roomWidth = 800;
  const roomHeight = 500;
  const doorwayStartX = 560;
  const doorwayEndX = 660;

  const furnitureRectangles = [
    { x: 60, y: 20, width: 220, height: 60 }, // workbench along the top wall
    { x: 720, y: 80, width: 60, height: 180 }, // shelf on the right wall
    { x: 330, y: 200, width: 160, height: 90 }, // center table
    { x: 250, y: 250, width: 50, height: 50 }, // storage box
    { x: 20, y: 380, width: 90, height: 100 }, // cabinet in the bottom left corner
    { x: 660, y: 300, width: 100, height: 60 }, // couch
    { x: 600, y: 160, width: 30, height: 30 }, // pillar
  ];

  const wallSegments = [
    [0, 0, roomWidth, 0],
    [roomWidth, 0, roomWidth, roomHeight],
    [roomWidth, roomHeight, doorwayEndX, roomHeight],
    [doorwayStartX, roomHeight, 0, roomHeight],
    [0, roomHeight, 0, 0],
  ];
  furnitureRectangles.forEach(({ x, y, width, height }) => {
    wallSegments.push(
      [x, y, x + width, y],
      [x + width, y, x + width, y + height],
      [x + width, y + height, x, y + height],
      [x, y + height, x, y],
    );
  });

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
        const tSquared = t * t;
        const tCubed = tSquared * t;
        const blend = (a, b, c, d) =>
          0.5 *
          (2 * b +
            (-a + c) * t +
            (2 * a - 5 * b + 4 * c - d) * tSquared +
            (-a + 3 * b - 3 * c + d) * tCubed);
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

  // ---------- LiDAR ----------
  const lidarMaxRange = 600; // cm
  const raysPerRevolution = 360; // 1 degree steps
  const raysPerSecond = 1440; // 4 revolutions per second
  const carSpeed = 110; // cm per second
  const measurementNoise = 1.5; // cm

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

  function colorForDistance(distance) {
    const position = Math.min(
      Math.max((distance - nearDistance) / (farDistance - nearDistance), 0),
      1,
    );
    for (
      let stopIndex = 1;
      stopIndex < distanceColorStops.length;
      stopIndex++
    ) {
      const [upperPosition, upperColor] = distanceColorStops[stopIndex];
      if (position <= upperPosition) {
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
  }

  // Distance along a ray to the nearest wall, or null if nothing is in range
  function castRay(originX, originY, angle) {
    const directionX = Math.cos(angle);
    const directionY = Math.sin(angle);
    let nearestHitDistance = lidarMaxRange;
    for (const [x1, y1, x2, y2] of wallSegments) {
      const segmentX = x2 - x1;
      const segmentY = y2 - y1;
      const denominator = directionX * segmentY - directionY * segmentX;
      if (Math.abs(denominator) < 1e-9) continue;
      const offsetX = x1 - originX;
      const offsetY = y1 - originY;
      const hitDistance =
        (offsetX * segmentY - offsetY * segmentX) / denominator;
      const positionOnSegment =
        (offsetX * directionY - offsetY * directionX) / denominator;
      if (
        hitDistance > 0 &&
        positionOnSegment >= 0 &&
        positionOnSegment <= 1 &&
        hitDistance < nearestHitDistance
      ) {
        nearestHitDistance = hitDistance;
      }
    }
    return nearestHitDistance < lidarMaxRange ? nearestHitDistance : null;
  }

  // ---------- Simulation state ----------
  let mappedPoints = []; // { x, y, color }
  let recentRays = []; // last sweep, drawn as a faint fan
  let drivenTrail = [];
  let distanceDriven = 0;
  let lidarAngle = -Math.PI / 2;
  let pendingRays = 0;
  let isRunning = false;
  let isFinished = false;
  let lastFrameTime = null;
  let animationFrameId = null;

  function resetSimulation() {
    mappedPoints = [];
    recentRays = [];
    drivenTrail = [];
    distanceDriven = 0;
    lidarAngle = -Math.PI / 2;
    pendingRays = 0;
    isFinished = false;
    lastFrameTime = null;
    clearPointLayer();
  }

  function advanceSimulation(elapsedSeconds) {
    distanceDriven = Math.min(
      distanceDriven + carSpeed * elapsedSeconds,
      drivingPath.totalLength,
    );
    const carPose = carPoseAtDistance(distanceDriven);
    drivenTrail.push([carPose.x, carPose.y]);

    pendingRays += raysPerSecond * elapsedSeconds;
    const raysThisStep = Math.floor(pendingRays);
    pendingRays -= raysThisStep;
    recentRays = [];
    for (let rayIndex = 0; rayIndex < raysThisStep; rayIndex++) {
      lidarAngle += (Math.PI * 2) / raysPerRevolution;
      const hitDistance = castRay(carPose.x, carPose.y, lidarAngle);
      const rayLength = hitDistance ?? lidarMaxRange;
      recentRays.push({ angle: lidarAngle, length: rayLength });
      if (hitDistance !== null) {
        const noisyDistance =
          hitDistance + (Math.random() - 0.5) * 2 * measurementNoise;
        const newPoint = {
          x: carPose.x + Math.cos(lidarAngle) * noisyDistance,
          y: carPose.y + Math.sin(lidarAngle) * noisyDistance,
          color: colorForDistance(hitDistance),
        };
        mappedPoints.push(newPoint);
        drawPointOnLayer(newPoint);
      }
    }
    if (distanceDriven >= drivingPath.totalLength) isFinished = true;
  }

  // ---------- Drawing ----------
  const pointLayerCanvas = document.createElement("canvas"); // keeps every mapped point so frames stay cheap
  const pointLayerContext = pointLayerCanvas.getContext("2d");
  let pixelsPerUnit = 1;
  let devicePixelScale = 1;

  function sizeCanvases() {
    devicePixelScale = Math.min(window.devicePixelRatio || 1, 2);
    const displayWidth = displayCanvas.clientWidth || roomWidth;
    pixelsPerUnit = (displayWidth * devicePixelScale) / roomWidth;
    displayCanvas.width = Math.round(roomWidth * pixelsPerUnit);
    displayCanvas.height = Math.round(roomHeight * pixelsPerUnit);
    pointLayerCanvas.width = displayCanvas.width;
    pointLayerCanvas.height = displayCanvas.height;
    clearPointLayer();
    mappedPoints.forEach(drawPointOnLayer);
  }

  function clearPointLayer() {
    pointLayerContext.clearRect(
      0,
      0,
      pointLayerCanvas.width,
      pointLayerCanvas.height,
    );
  }

  function drawPointOnLayer({ x, y, color }) {
    const pointSize = 2.2 * devicePixelScale;
    pointLayerContext.fillStyle = color;
    pointLayerContext.fillRect(
      x * pixelsPerUnit - pointSize / 2,
      y * pixelsPerUnit - pointSize / 2,
      pointSize,
      pointSize,
    );
  }

  function drawFrame() {
    const scale = pixelsPerUnit;
    displayContext.fillStyle = "#0d141c";
    displayContext.fillRect(0, 0, displayCanvas.width, displayCanvas.height);

    // Faint 1 m grid, like the floor of a mapping tool
    displayContext.strokeStyle = "rgba(255, 255, 255, 0.05)";
    displayContext.lineWidth = 1;
    for (let gridX = 100; gridX < roomWidth; gridX += 100) {
      displayContext.beginPath();
      displayContext.moveTo(gridX * scale, 0);
      displayContext.lineTo(gridX * scale, displayCanvas.height);
      displayContext.stroke();
    }
    for (let gridY = 100; gridY < roomHeight; gridY += 100) {
      displayContext.beginPath();
      displayContext.moveTo(0, gridY * scale);
      displayContext.lineTo(displayCanvas.width, gridY * scale);
      displayContext.stroke();
    }

    displayContext.drawImage(pointLayerCanvas, 0, 0);

    // Path driven so far
    if (drivenTrail.length > 1) {
      displayContext.strokeStyle = "rgba(255, 255, 255, 0.35)";
      displayContext.lineWidth = 1.5 * devicePixelScale;
      displayContext.setLineDash([4 * devicePixelScale, 4 * devicePixelScale]);
      displayContext.beginPath();
      displayContext.moveTo(
        drivenTrail[0][0] * scale,
        drivenTrail[0][1] * scale,
      );
      drivenTrail.forEach(([trailX, trailY]) =>
        displayContext.lineTo(trailX * scale, trailY * scale),
      );
      displayContext.stroke();
      displayContext.setLineDash([]);
    }

    const carPose = carPoseAtDistance(distanceDriven);

    // Current LiDAR sweep
    displayContext.strokeStyle = "rgba(125, 211, 252, 0.18)";
    displayContext.lineWidth = 1 * devicePixelScale;
    recentRays.forEach(({ angle, length }) => {
      displayContext.beginPath();
      displayContext.moveTo(carPose.x * scale, carPose.y * scale);
      displayContext.lineTo(
        (carPose.x + Math.cos(angle) * length) * scale,
        (carPose.y + Math.sin(angle) * length) * scale,
      );
      displayContext.stroke();
    });

    // Car: a 40 x 20 cm body with a lighter nose showing which way it faces
    displayContext.save();
    displayContext.translate(carPose.x * scale, carPose.y * scale);
    displayContext.rotate(carPose.heading);
    displayContext.fillStyle = "#e2e8f0";
    displayContext.fillRect(-20 * scale, -10 * scale, 40 * scale, 20 * scale);
    displayContext.fillStyle = "#1e3a5f";
    displayContext.fillRect(8 * scale, -10 * scale, 12 * scale, 20 * scale);
    displayContext.fillStyle = "#7dd3fc";
    displayContext.beginPath();
    displayContext.arc(0, 0, 4.5 * scale, 0, Math.PI * 2); // LiDAR unit
    displayContext.fill();
    displayContext.restore();

    pointsMappedLabel.textContent = mappedPoints.length.toLocaleString("en-US");
    distanceDrivenLabel.textContent = (distanceDriven / 100).toFixed(1) + " m";
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
    drawFrame();
    if (isFinished) {
      stopPlayback();
      recentRays = [];
      drawFrame();
      return;
    }
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

  // Run the whole drive instantly (used for reduced motion)
  function jumpToFinishedMap() {
    resetSimulation();
    while (!isFinished) advanceSimulation(1 / 30);
    recentRays = [];
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

  let resizeTimer = null;
  window.addEventListener("resize", () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(() => {
      sizeCanvases();
      drawFrame();
    }, 150);
  });

  sizeCanvases();
  const prefersReducedMotion = window.matchMedia(
    "(prefers-reduced-motion: reduce)",
  ).matches;

  if (prefersReducedMotion) {
    jumpToFinishedMap();
    return;
  }

  // At rest: show the car after one full LiDAR revolution, so the first frame already has a map
  advanceSimulation(raysPerRevolution / raysPerSecond);
  distanceDriven = 0;
  drivenTrail = [];
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
