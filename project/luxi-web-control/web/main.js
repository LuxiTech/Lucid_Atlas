import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

window.addEventListener("error", (event) => {
  document.body.dataset.jsError = event.message || "unknown error";
});
window.addEventListener("unhandledrejection", (event) => {
  document.body.dataset.jsError = event.reason && event.reason.message
    ? event.reason.message
    : "unhandled promise rejection";
});

(function () {
  const ROSLIB = window.ROSLIB;
  const wsInput = document.getElementById("ws-url");
  const connectBtn = document.getElementById("connect-btn");
  const disconnectBtn = document.getElementById("disconnect-btn");
  const connectionPill = document.getElementById("connection-pill");
  const systemStatus = document.getElementById("system-status");
  const controlStatus = document.getElementById("control-status");
  const networkStatus = document.getElementById("network-status");
  const navStatus = document.getElementById("nav-status");
  const mappingStatus = document.getElementById("mapping-status");
  const mappingMapName = document.getElementById("mapping-map-name");
  const startMappingBtn = document.getElementById("start-mapping-btn");
  const stopMappingBtn = document.getElementById("stop-mapping-btn");
  const saveMappingBtn = document.getElementById("save-mapping-btn");
  const mappingHeightFilterEnabled = document.getElementById("mapping-height-filter-enabled");
  const mappingHeightMaxZ = document.getElementById("mapping-height-max-z");
  const mappingPreviewPanel = document.getElementById("mapping-preview-panel");
  const mappingPreviewCanvas = document.getElementById("mapping-preview-canvas");
  const mappingPreviewStatus = document.getElementById("mapping-preview-status");
  const savedMapSelect = document.getElementById("saved-map-select");
  const refreshMapsBtn = document.getElementById("refresh-maps-btn");
  const loadSavedMapBtn = document.getElementById("load-saved-map-btn");
  const convertSavedMapBtn = document.getElementById("convert-saved-map-btn");
  const savedMapStatus = document.getElementById("saved-map-status");
  const estopBtn = document.getElementById("estop-btn");
  const releaseEstopBtn = document.getElementById("release-estop-btn");
  const manualJoystickPad = document.getElementById("manual-joystick-pad");
  const manualJoystickKnob = document.getElementById("manual-joystick-knob");
  const manualWzSlider = document.getElementById("manual-wz-slider");
  const manualVxValue = document.getElementById("manual-vx-value");
  const manualVyValue = document.getElementById("manual-vy-value");
  const manualWzValue = document.getElementById("manual-wz-value");
  const zeroBtn = document.getElementById("zero-btn");
  const sendGoalBtn = document.getElementById("send-goal-btn");
  const pickGoalBtn = document.getElementById("pick-goal-btn");
  const goalX = document.getElementById("goal-x");
  const goalY = document.getElementById("goal-y");
  const goalYaw = document.getElementById("goal-yaw");
  const initialX = document.getElementById("initial-x");
  const initialY = document.getElementById("initial-y");
  const initialYaw = document.getElementById("initial-yaw");
  const pickInitialPoseBtn = document.getElementById("pick-initial-pose-btn");
  const sendInitialPoseBtn = document.getElementById("send-initial-pose-btn");
  const showCameraBtn = document.getElementById("show-camera-btn");
  const hideCameraBtn = document.getElementById("hide-camera-btn");
  const exposureTimeInput = document.getElementById("exposure-time-us");
  const applyExposureBtn = document.getElementById("apply-exposure-btn");
  const cameraStatus = document.getElementById("camera-status");
  const cameraPanel = document.getElementById("camera-panel");
  const cameraImage = document.getElementById("camera-image");
  const canvas = document.getElementById("map-canvas");
  const ctx = canvas.getContext("2d");
  const cloud3dCanvas = document.getElementById("cloud3d-canvas");
  const mapStatus = document.getElementById("map-status");
  const layerGrid = document.getElementById("layer-grid");
  const layerCostmap = document.getElementById("layer-costmap");
  const layerStaticCloud = document.getElementById("layer-static-cloud");
  const layerLiveCloud = document.getElementById("layer-live-cloud");
  const layerSavedMap = document.getElementById("layer-saved-map");
  const layerNavPath = document.getElementById("layer-nav-path");
  const layerLocalPath = document.getElementById("layer-local-path");
  const layerPose = document.getElementById("layer-pose");
  const zoomOutBtn = document.getElementById("zoom-out-btn");
  const zoomInBtn = document.getElementById("zoom-in-btn");
  const view2dBtn = document.getElementById("view-2d-btn");
  const view3dBtn = document.getElementById("view-3d-btn");
  const mapZoomLabel = document.getElementById("map-zoom-label");
  const resetMapViewBtn = document.getElementById("reset-map-view-btn");
  const navStartBtn = document.getElementById("nav-start-btn");
  const navStopBtn = document.getElementById("nav-stop-btn");

  let ros = null;
  let cmdTopic = null;
  let estopTopic = null;
  let goalTopic = null;
  let latestCmd = { linear_x: 0, linear_y: 0, angular_z: 0 };
  let connected = false;
  let rosConnected = false;
  let apiAvailable = false;
  let mapSnapshot = null;
  let mappingState = null;
  let savedMaps = [];
  let selectedSavedMap = null;
  let selectedSavedMapImage = null;
  let mapViewBounds = null;
  let mappingPreviewBounds = null;
  let mappingCloudNeedsFrame = true;
  let mapViewUserAdjusted = false;
  let lastMappingSourcePoints = 0;
  let mapSnapshotPollInFlight = false;
  let mappingStatusPollInFlight = false;
  let lastMapSnapshotOk = false;
  let pickingInitialPose = false;
  let pendingInitialPoseDirection = false;
  let initialDirectionPreview = null;
  let pickingGoal = false;
  let selectedInitialPose = null;
  let selectedGoal = null;
  let isPanningMap = false;
  let panStart = null;
  let mapDragged = false;
  let manualRepeatTimer = null;
  let manualStopBurstTimer = null;
  let manualStopBurstRemaining = 0;
  let manualJoystickPointerId = null;
  let manualJoystickX = 0;
  let manualJoystickY = 0;
  let manualCommandSeq = Math.floor(Date.now() * 1000);
  const manualCommandSession =
    `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
  const manualRepeatMs = 100;
  const manualStopBurstMs = 60;
  const manualStopBurstCount = 8;
  const maxManualLinearSpeed = 0.5;
  const maxManualAngularSpeed = 0.5;
  let mappingRequestInFlight = false;
  const pageParams = new URLSearchParams(window.location.search);
  const apiBaseOverride = pageParams.get("api_base") || pageParams.get("api") || "";
  const configuredApiPort = String(window.LUXI_WEB_CONFIG && window.LUXI_WEB_CONFIG.api_port
    ? window.LUXI_WEB_CONFIG.api_port
    : "8082");
  let viewMode = pageParams.get("view") === "3d" ? "3d" : "2d";
  const initialSavedMapName = pageParams.get("map") || "";
  let initialSavedMapLoaded = false;
  let cloud3dNeedsFrame = true;

  let webglAvailable = true;
  let mappingWebglAvailable = true;
  let renderer3d = null;
  let mappingRenderer3d = null;
  try {
    mappingRenderer3d = new THREE.WebGLRenderer({ canvas: mappingPreviewCanvas, antialias: true });
    mappingRenderer3d.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    mappingRenderer3d.setClearColor(0x101314, 1);
  } catch (error) {
    mappingWebglAvailable = false;
    document.body.dataset.mappingWebglError = error.message || "Mapping WebGL unavailable";
  }
  try {
    renderer3d = new THREE.WebGLRenderer({ canvas: cloud3dCanvas, antialias: true });
    renderer3d.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer3d.setClearColor(0x101314, 1);
  } catch (error) {
    webglAvailable = false;
    document.body.dataset.webglError = error.message || "WebGL unavailable";
  }

  const scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color(0x101314);

  const camera3d = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
  camera3d.up.set(0, 0, 1);
  camera3d.position.set(6, -8, 5);

  const controls3d = new OrbitControls(camera3d, cloud3dCanvas);
  controls3d.enableDamping = true;
  controls3d.dampingFactor = 0.08;
  controls3d.target.set(0, 0, 0);

  const grid3d = new THREE.GridHelper(30, 30, 0x334040, 0x243031);
  grid3d.rotation.x = Math.PI * 0.5;
  scene3d.add(grid3d);
  scene3d.add(new THREE.AxesHelper(1.5));

  const pointGeometry3d = new THREE.BufferGeometry();
  const pointMaterial3d = new THREE.PointsMaterial({
    size: 0.045,
    vertexColors: true,
    sizeAttenuation: true,
  });
  const pointCloud3d = new THREE.Points(pointGeometry3d, pointMaterial3d);
  scene3d.add(pointCloud3d);

  const robotMarker3d = new THREE.Mesh(
    new THREE.SphereGeometry(0.16, 20, 12),
    new THREE.MeshBasicMaterial({ color: 0xf2c35b }),
  );
  scene3d.add(robotMarker3d);
  const robotHeading3d = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0.12),
    0.7,
    0xf2c35b,
    0.18,
    0.12,
  );
  scene3d.add(robotHeading3d);

  const mappingScene3d = new THREE.Scene();
  mappingScene3d.background = new THREE.Color(0x101314);
  const mappingCamera3d = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
  mappingCamera3d.up.set(0, 0, 1);
  mappingCamera3d.position.set(6, -8, 5);
  const mappingControls3d = new OrbitControls(mappingCamera3d, mappingPreviewCanvas);
  mappingControls3d.enableDamping = true;
  mappingControls3d.dampingFactor = 0.08;
  mappingControls3d.target.set(0, 0, 0);
  mappingScene3d.add(new THREE.AxesHelper(1.2));
  const mappingPointGeometry3d = new THREE.BufferGeometry();
  const mappingPointMaterial3d = new THREE.PointsMaterial({
    size: 0.055,
    vertexColors: true,
    sizeAttenuation: true,
  });
  const mappingPointCloud3d = new THREE.Points(mappingPointGeometry3d, mappingPointMaterial3d);
  mappingScene3d.add(mappingPointCloud3d);
  const mappingRobotMarker3d = new THREE.Mesh(
    new THREE.SphereGeometry(0.16, 20, 12),
    new THREE.MeshBasicMaterial({ color: 0xf2c35b }),
  );
  mappingScene3d.add(mappingRobotMarker3d);
  const mappingRobotHeading3d = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0.12),
    0.7,
    0xf2c35b,
    0.18,
    0.12,
  );
  mappingScene3d.add(mappingRobotHeading3d);

  function defaultApiBaseUrl() {
    if (apiBaseOverride) {
      return apiBaseOverride.replace(/\/+$/, "");
    }
    if (window.LUXI_WEB_CONFIG && window.LUXI_WEB_CONFIG.api_base_url) {
      return String(window.LUXI_WEB_CONFIG.api_base_url).replace(/\/+$/, "");
    }
    if (window.location.port === configuredApiPort) {
      return window.location.origin;
    }
    const host = window.location.hostname || "localhost";
    const protocol = window.location.protocol === "https:" ? "https:" : "http:";
    return `${protocol}//${host}:${configuredApiPort}`;
  }

  function defaultRosbridgeUrl() {
    const protocol = window.location.protocol === "https:" ? "wss" : "ws";
    const host = window.location.hostname || "localhost";
    return `${protocol}://${host}:9090`;
  }

  function setConnectionState(isConnected, text) {
    connected = isConnected;
    connectionPill.textContent = text;
    connectionPill.classList.toggle("online", isConnected);
    connectionPill.classList.toggle("offline", !isConnected);
    drawScene();
  }

  function formatJson(raw) {
    try {
      return JSON.stringify(JSON.parse(raw), null, 2);
    } catch (error) {
      return raw || "无数据";
    }
  }

  function makeTwist(vx, vy, wz) {
    return new ROSLIB.Message({
      linear: { x: vx, y: vy, z: 0 },
      angular: { x: 0, y: 0, z: wz },
    });
  }

  function publishCmd(vx, vy, wz) {
    manualCommandSeq += 1;
    const payload = {
      linear_x: vx,
      linear_y: vy,
      angular_z: wz,
      command_seq: manualCommandSeq,
      command_session: manualCommandSession,
    };
    latestCmd = payload;
    if (apiAvailable || !cmdTopic) {
      fetch(`${defaultApiBaseUrl()}/api/cmd_vel`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      })
        .then((res) => res.json())
        .then((data) => {
          if (data && data.ok && !data.stale &&
            String(data.command_session || "") === payload.command_session &&
            Number(data.command_seq || 0) === payload.command_seq)
          {
            latestCmd = {
              linear_x: Number(data.linear_x || 0),
              linear_y: Number(data.linear_y || 0),
              angular_z: Number(data.angular_z || 0),
            };
            drawScene();
          }
        })
        .catch(() => {
          if (cmdTopic) {
            cmdTopic.publish(makeTwist(vx, vy, wz));
          } else if (!rosConnected) {
            setConnectionState(false, "未连接");
          }
        });
    } else {
      cmdTopic.publish(makeTwist(vx, vy, wz));
    }
    drawScene();
  }

  function sliderRatio(slider) {
    if (!slider) {
      return 0;
    }
    const value = Number(slider.value || 0);
    if (!Number.isFinite(value)) {
      return 0;
    }
    return Math.max(-1, Math.min(1, value / 100));
  }

  function currentManualVelocity() {
    return {
      vx: -manualJoystickY * maxManualLinearSpeed,
      vy: -manualJoystickX * maxManualLinearSpeed,
      wz: -sliderRatio(manualWzSlider) * maxManualAngularSpeed,
    };
  }

  function manualVelocityIsActive() {
    const velocity = currentManualVelocity();
    return (
      Math.abs(velocity.vx) > 1e-4 ||
      Math.abs(velocity.vy) > 1e-4 ||
      Math.abs(velocity.wz) > 1e-4
    );
  }

  function updateManualVelocityLabels() {
    const velocity = currentManualVelocity();
    if (manualVxValue) {
      manualVxValue.textContent = velocity.vx.toFixed(2);
    }
    if (manualVyValue) {
      manualVyValue.textContent = velocity.vy.toFixed(2);
    }
    if (manualWzValue) {
      manualWzValue.textContent = `${velocity.wz.toFixed(2)} rad/s`;
    }
  }

  function publishManualVelocity() {
    const velocity = currentManualVelocity();
    updateManualVelocityLabels();
    publishCmd(velocity.vx, velocity.vy, velocity.wz);
    if (manualVelocityIsActive()) {
      stopManualStopBurst();
      startManualRepeat();
    } else {
      stopManualSliderPublish();
    }
  }

  function startManualRepeat() {
    if (manualRepeatTimer) {
      return;
    }
    manualRepeatTimer = setInterval(() => {
      if (manualVelocityIsActive()) {
        publishManualVelocity();
      } else {
        stopManualSliderPublish();
      }
    }, manualRepeatMs);
  }

  function startManualSliderPublish() {
    publishManualVelocity();
  }

  function stopManualSliderPublish() {
    if (manualRepeatTimer) {
      clearInterval(manualRepeatTimer);
      manualRepeatTimer = null;
    }
  }

  function stopManualStopBurst() {
    if (manualStopBurstTimer) {
      clearInterval(manualStopBurstTimer);
      manualStopBurstTimer = null;
    }
    manualStopBurstRemaining = 0;
  }

  function publishStopBurst() {
    stopManualSliderPublish();
    stopManualStopBurst();
    latestCmd = { linear_x: 0, linear_y: 0, angular_z: 0 };
    updateManualVelocityLabels();
    publishCmd(0, 0, 0);
    manualStopBurstRemaining = manualStopBurstCount - 1;
    manualStopBurstTimer = setInterval(() => {
      if (manualStopBurstRemaining <= 0) {
        stopManualStopBurst();
        return;
      }
      manualStopBurstRemaining -= 1;
      latestCmd = { linear_x: 0, linear_y: 0, angular_z: 0 };
      publishCmd(0, 0, 0);
    }, manualStopBurstMs);
  }

  function updateManualJoystickKnob() {
    if (!manualJoystickKnob || !manualJoystickPad) {
      return;
    }
    const padRadius = manualJoystickPad.clientWidth * 0.5;
    const knobRadius = manualJoystickKnob.clientWidth * 0.5;
    const travel = Math.max(1, padRadius - knobRadius - 4);
    manualJoystickKnob.style.transform =
      `translate(calc(-50% + ${manualJoystickX * travel}px), calc(-50% + ${manualJoystickY * travel}px))`;
    manualJoystickKnob.classList.toggle(
      "active",
      Math.hypot(manualJoystickX, manualJoystickY) > 0.001,
    );
  }

  function setManualJoystickFromPointer(event) {
    if (!manualJoystickPad) {
      return;
    }
    const rect = manualJoystickPad.getBoundingClientRect();
    const radius = Math.max(1, Math.min(rect.width, rect.height) * 0.5);
    const cx = rect.left + rect.width * 0.5;
    const cy = rect.top + rect.height * 0.5;
    let x = (event.clientX - cx) / radius;
    let y = (event.clientY - cy) / radius;
    const length = Math.hypot(x, y);
    if (length > 1) {
      x /= length;
      y /= length;
    }
    manualJoystickX = x;
    manualJoystickY = y;
    updateManualJoystickKnob();
    publishManualVelocity();
  }

  function resetManualJoystick() {
    manualJoystickPointerId = null;
    manualJoystickX = 0;
    manualJoystickY = 0;
    if (manualWzSlider) {
      manualWzSlider.value = "0";
    }
    updateManualJoystickKnob();
    publishStopBurst();
  }

  function resetManualControls() {
    manualJoystickPointerId = null;
    manualJoystickX = 0;
    manualJoystickY = 0;
    updateManualJoystickKnob();
    if (manualWzSlider) {
      manualWzSlider.value = "0";
    }
    stopManualSliderPublish();
    stopManualStopBurst();
    updateManualVelocityLabels();
    publishStopBurst();
  }

  function releaseManualRotationControl() {
    if (!manualWzSlider) {
      return;
    }
    manualWzSlider.value = "0";
    updateManualVelocityLabels();
    if (Math.hypot(manualJoystickX, manualJoystickY) > 1e-4) {
      publishManualVelocity();
    } else {
      publishStopBurst();
    }
  }

  function publishEstop(value) {
    if (estopTopic) {
      estopTopic.publish(new ROSLIB.Message({ data: value }));
    } else {
      postApi("/api/estop", { active: value });
    }
  }

  function publishGoal() {
    if (!snapshotHasMapData(mapSnapshot)) {
      setMapStatus("请先选择并加载地图后再发送导航目标。");
      return;
    }
    const x = Number(goalX.value || 0);
    const y = Number(goalY.value || 0);
    const yaw = Number(goalYaw.value || 0);
    if (!goalTopic) {
      postApi("/api/goal_pose", { x, y, yaw });
      return;
    }
    const halfYaw = yaw * 0.5;
    goalTopic.publish(
      new ROSLIB.Message({
        header: {
          frame_id: "map",
          stamp: { sec: 0, nanosec: 0 },
        },
        pose: {
          position: { x, y, z: 0 },
          orientation: {
            x: 0,
            y: 0,
            z: Math.sin(halfYaw),
            w: Math.cos(halfYaw),
          },
        },
      }),
    );
  }

  function publishInitialPose() {
    if (!snapshotHasMapData(mapSnapshot)) {
      setMapStatus("请先选择并加载地图后再设置初始位姿。");
      return;
    }
    const x = Number(initialX.value || 0);
    const y = Number(initialY.value || 0);
    const yaw = Number(initialYaw.value || 0);
    fetch(`${defaultApiBaseUrl()}/api/map/initial_pose`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x, y, yaw }),
    })
      .then((res) => res.json().then((data) => ({ ok: res.ok && data.ok, data })))
      .then(({ ok, data }) => {
        if (!ok) {
          setMapStatus(data.error || "初始位姿设置失败");
          return;
        }
        selectedInitialPose = null;
        initialDirectionPreview = null;
        pendingInitialPoseDirection = false;
        pickingInitialPose = false;
        pickInitialPoseBtn.classList.remove("active");
        setMapStatus(`已发送初始位姿 x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${yaw.toFixed(2)}`);
        drawScene();
      })
      .catch(() => setMapStatus("初始位姿设置请求失败"));
  }

  function clearNavigationPathSnapshot() {
    if (!mapSnapshot) {
      return;
    }
    mapSnapshot.nav_path = {
      available: false,
      frame_id: "",
      source: "",
      stamp_sec: 0,
      points: [],
    };
    mapSnapshot.nav_local_path = {
      available: false,
      frame_id: "",
      source: "",
      stamp_sec: 0,
      points: [],
    };
  }

  function requestNavigationDrive(path, activeText) {
    const isStop = path.endsWith("/stop");
    if (!isStop && !snapshotHasMapData(mapSnapshot)) {
      setMapStatus("请先选择并加载地图后再出发导航。");
      return;
    }
    fetch(`${defaultApiBaseUrl()}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    })
      .then((res) => res.json().then((data) => ({ ok: res.ok && data.ok, data })))
      .then(({ ok, data }) => {
        if (!ok) {
          setMapStatus(data.error || "导航控制请求失败");
          return;
        }
        if (isStop) {
          clearNavigationPathSnapshot();
          drawScene();
        }
        setMapStatus(activeText);
        pollMapSnapshot();
      })
      .catch(() => setMapStatus("导航控制API未连接"));
  }

  function currentHeadingYaw() {
    const pose = mapSnapshot && mapSnapshot.pose && mapSnapshot.pose.available
      ? mapSnapshot.pose
      : (mapSnapshot && mapSnapshot.odom_pose && mapSnapshot.odom_pose.available ? mapSnapshot.odom_pose : null);
    const yaw = pose ? Number(pose.yaw) : NaN;
    return Number.isFinite(yaw) ? yaw : Number(initialYaw.value || 0);
  }

  function postApi(path, payload) {
    fetch(`${defaultApiBaseUrl()}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    }).catch(() => {
      if (!rosConnected) {
        setConnectionState(false, "未连接");
      }
    });
  }

  function formatMappingStatus(data) {
    if (!data) {
      return "建图状态：等待数据";
    }
    if (!data.ok) {
      return `建图状态：请求失败\n原因：${data.error || data.message || "未知错误"}`;
    }

    const isActive = Boolean(data.active);
    const lines = [
      `建图传输：${isActive ? "开启" : "停止"}`,
      `地图名称：${data.map_name || (mappingMapName ? mappingMapName.value : "") || "-"}`,
      `状态说明：${data.message || "-"}`,
      `FAST-LIO累计点云：${data.mapping_cloud_available ? "在线" : "等待"} / ${data.mapping_cloud_points || 0} 点`,
      `地图话题：${data.mapping_cloud_topic || "-"}`,
      `高度过滤：${mappingHeightFilterIsEnabled() ? `Z <= ${mappingMaxZ().toFixed(2)}m` : "不限高"}`,
      `保存服务：${data.save_service_available ? "可用" : "不可用"} ${data.save_service || ""}`,
    ];
    if (data.last_save_message || data.last_save_path) {
      lines.push(`最近保存：${data.last_save_success ? "成功" : "失败"} ${data.last_save_message || ""}`);
      if (data.last_save_path) {
        lines.push(`保存路径：${data.last_save_path}`);
      }
    }
    return lines.join("\n");
  }

  function setMappingButtonsBusy(isBusy) {
    mappingRequestInFlight = isBusy;
    const isActive = Boolean(mappingState && mappingState.ok && mappingState.active);
    if (startMappingBtn) {
      startMappingBtn.disabled = isBusy || isActive;
    }
    if (stopMappingBtn) {
      stopMappingBtn.disabled = isBusy || !isActive;
    }
    if (saveMappingBtn) {
      saveMappingBtn.disabled = isBusy;
    }
  }

  function syncMappingButtons(data) {
    if (!data || !data.ok || mappingRequestInFlight) {
      return;
    }
    setMappingButtonsBusy(false);
    syncMapLayerControls();
  }

  function setMappingStatus(data) {
    const wasMappingMode = isMappingMode();
    const wasPreviewVisible = shouldShowMappingPreview();
    mappingState = data;
    const nowMappingMode = isMappingMode();
    if (wasMappingMode !== nowMappingMode) {
      lastMappingSourcePoints = 0;
      mappingPreviewBounds = null;
      mappingCloudNeedsFrame = true;
    }
    mappingStatus.textContent = formatMappingStatus(data);
    syncMappingButtons(data);
    syncMappingPreviewVisibility(wasPreviewVisible);
    drawMappingPreview();
    drawScene();
  }

  function shouldShowMappingPreview() {
    return Boolean(mappingState && mappingState.ok && mappingState.active);
  }

  function clearMappingPreviewGeometry() {
    if (!mappingRenderer3d) {
      return;
    }
    mappingPointGeometry3d.setAttribute("position", new THREE.BufferAttribute(new Float32Array(0), 3));
    mappingPointGeometry3d.setAttribute("color", new THREE.BufferAttribute(new Float32Array(0), 3));
    mappingRobotMarker3d.visible = false;
    mappingRobotHeading3d.visible = false;
  }

  function syncMappingPreviewVisibility(wasVisible = shouldShowMappingPreview()) {
    const shouldShow = shouldShowMappingPreview();
    if (mappingPreviewPanel) {
      mappingPreviewPanel.hidden = !shouldShow;
    }
    if (!shouldShow) {
      clearMappingPreviewGeometry();
      return;
    }
    if (!wasVisible) {
      requestAnimationFrame(() => {
        resizeMappingPreview();
        drawMappingPreview();
      });
    }
  }

  function isMappingMode() {
    return Boolean(mappingState && mappingState.ok && mappingState.active && mappingState.mapping_cloud_available);
  }

  function is3dViewActive() {
    return webglAvailable && viewMode === "3d" && has3dCloudData();
  }

  function has3dCloudData() {
    const hasOfflineCloud = selectedSavedMap && selectedSavedMapImage && selectedSavedMap.pcd_cloud_3d &&
      selectedSavedMap.pcd_cloud_3d.points && selectedSavedMap.pcd_cloud_3d.points.length > 0;
    const hasProjectedOfflineCloud = selectedSavedMap && selectedSavedMapImage && selectedSavedMap.pcd_cloud &&
      selectedSavedMap.pcd_cloud.points && selectedSavedMap.pcd_cloud.points.length > 0;
    return Boolean(hasOfflineCloud || hasProjectedOfflineCloud);
  }

  function mappingHeightFilterIsEnabled() {
    return Boolean(mappingHeightFilterEnabled && mappingHeightFilterEnabled.checked);
  }

  function mappingMaxZ() {
    const value = Number(mappingHeightMaxZ ? mappingHeightMaxZ.value : 1.5);
    return Number.isFinite(value) ? value : 1.5;
  }

  function mappingPointPassesHeight(point) {
    if (!mappingHeightFilterIsEnabled()) {
      return true;
    }
    const z = Number(point && point[2]);
    return Number.isFinite(z) && z <= mappingMaxZ();
  }

  function countVisibleMappingPoints(cloud) {
    if (!cloud || !cloud.points) {
      return 0;
    }
    let count = 0;
    cloud.points.forEach((point) => {
      if (mappingPointPassesHeight(point)) {
        count += 1;
      }
    });
    return count;
  }

  function refreshMappingHeightFilter() {
    mapViewBounds = null;
    mappingPreviewBounds = null;
    mappingCloudNeedsFrame = true;
    if (mappingState) {
      mappingStatus.textContent = formatMappingStatus(mappingState);
    }
    updateMapStatus(mapSnapshot);
    drawMappingPreview();
    drawScene();
  }

  function syncMapLayerControls() {
    view3dBtn.disabled = !has3dCloudData() || !webglAvailable;
    view2dBtn.classList.toggle("active", !is3dViewActive());
    view3dBtn.classList.toggle("active", is3dViewActive());
    canvas.hidden = is3dViewActive();
    cloud3dCanvas.hidden = !is3dViewActive();
  }

  function mappingPayload() {
    return { map_name: (mappingMapName.value || "").trim() };
  }

  function clearSavedMapView() {
    mappingPreviewBounds = null;
    mappingCloudNeedsFrame = true;
  }

  function requestMapping(path, actionText, options = {}) {
    setMappingButtonsBusy(true);
    mappingStatus.textContent = actionText;
    setMapStatus(actionText);
    fetch(`${defaultApiBaseUrl()}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json;charset=UTF-8" },
      body: JSON.stringify(mappingPayload()),
    })
      .then((res) => res.json().then((data) => ({ ok: res.ok && data.ok, data })))
      .then(({ ok, data }) => {
        setMappingButtonsBusy(false);
        setMappingStatus(data);
        if (!ok) {
          setMapStatus(data.error || data.message || "建图请求失败");
          return;
        }
        if (options.clearCloud || options.isStartMapping) {
          clearSavedMapView();
          lastMappingSourcePoints = 0;
          mappingCloudNeedsFrame = true;
        }
        if (data.path) {
          setMapStatus(`PCD已保存: ${data.path}`);
          refreshSavedMaps();
        } else {
          setMapStatus(data.message || "建图状态已更新");
        }
      })
      .catch(() => {
        setMappingButtonsBusy(false);
        setMappingStatus({ ok: false, error: "建图API请求失败" });
        setMapStatus("建图API请求失败");
      });
  }

  function pollMappingStatus() {
    if (mappingStatusPollInFlight) {
      return;
    }
    mappingStatusPollInFlight = true;
    fetch(`${defaultApiBaseUrl()}/api/mapping/status?t=${Date.now()}`, {
      cache: "no-store",
    })
      .then((res) => res.json())
      .then((data) => setMappingStatus(data))
      .catch(() => setMappingStatus({ ok: false, error: "建图API未连接" }))
      .finally(() => {
        mappingStatusPollInFlight = false;
      });
  }

  function formatBytes(bytes) {
    const value = Number(bytes || 0);
    if (value >= 1024 * 1024) {
      return `${(value / 1024 / 1024).toFixed(1)} MB`;
    }
    if (value >= 1024) {
      return `${(value / 1024).toFixed(1)} KB`;
    }
    return `${value} B`;
  }

  function setSavedMapStatus(text) {
    savedMapStatus.textContent = text;
  }

  function selectedSavedMapName() {
    return savedMapSelect && savedMapSelect.value ? savedMapSelect.value : "";
  }

  function refreshSavedMaps() {
    fetch(`${defaultApiBaseUrl()}/api/maps?t=${Date.now()}`, { cache: "no-store" })
      .then((res) => res.json())
      .then((data) => {
        if (!data.ok) {
          setSavedMapStatus(data.error || "地图列表读取失败");
          return;
        }
        savedMaps = data.maps || [];
        const previous = selectedSavedMapName();
        savedMapSelect.innerHTML = "";
        savedMaps.forEach((map) => {
          const option = document.createElement("option");
          option.value = map.name;
          option.textContent = `${map.name} ${map.converted ? "[已转换]" : "[未转换]"} ${formatBytes(map.size_bytes)}`;
          savedMapSelect.appendChild(option);
        });
        if (previous && savedMaps.some((map) => map.name === previous)) {
          savedMapSelect.value = previous;
        } else if (initialSavedMapName && savedMaps.some((map) => map.name === initialSavedMapName)) {
          savedMapSelect.value = initialSavedMapName;
        }
        const lines = [
          `保存目录：${data.maps_dir || "-"}`,
          `地图数量：${savedMaps.length}`,
        ];
        if (savedMaps.length > 0) {
          const current = savedMaps.find((map) => map.name === selectedSavedMapName()) || savedMaps[0];
          lines.push(`当前：${current.name} / ${current.converted ? "已转换" : "未转换"} / ${formatBytes(current.size_bytes)}`);
        }
        setSavedMapStatus(lines.join("\n"));
        if (!initialSavedMapLoaded && initialSavedMapName &&
          savedMaps.some((map) => map.name === initialSavedMapName))
        {
          initialSavedMapLoaded = true;
          loadSelectedSavedMap();
        }
      })
      .catch(() => setSavedMapStatus("地图列表API未连接"));
  }

  function convertSelectedSavedMap() {
    const mapName = selectedSavedMapName();
    if (!mapName) {
      setSavedMapStatus("没有可转换的地图");
      return Promise.resolve(false);
    }
    setSavedMapStatus(`正在转换地图：${mapName}`);
    return fetch(`${defaultApiBaseUrl()}/api/maps/convert`, {
      method: "POST",
      headers: { "Content-Type": "text/plain;charset=UTF-8" },
      body: JSON.stringify({ map_name: mapName }),
    })
      .then((res) => res.json().then((data) => ({ ok: res.ok && data.ok, data })))
      .then(({ ok, data }) => {
        if (!ok) {
          setSavedMapStatus(data.error || "地图转换失败");
          return false;
        }
        setSavedMapStatus(`地图已转换：${data.converted_dir || ""}`);
        refreshSavedMaps();
        return true;
      })
      .catch(() => {
        setSavedMapStatus("地图转换API未连接");
        return false;
      });
  }

  function loadSelectedSavedMap() {
    const mapName = selectedSavedMapName();
    if (!mapName) {
      setSavedMapStatus("没有可加载的地图");
      return;
    }
    setSavedMapStatus(`正在加载地图：${mapName}`);
    fetch(`${defaultApiBaseUrl()}/api/maps/view?map_name=${encodeURIComponent(mapName)}&t=${Date.now()}`, {
      cache: "no-store",
    })
      .then((res) => res.json().then((data) => ({ ok: res.ok && data.ok, data })))
      .then(({ ok, data }) => {
        if (!ok) {
          setSavedMapStatus(data.error || "地图加载失败");
          return;
        }
        const image = new Image();
        image.onload = () => {
          selectedSavedMap = data;
          selectedSavedMapImage = image;
          mapViewBounds = savedMapBounds(data);
          mapViewUserAdjusted = false;
          cloud3dNeedsFrame = true;
          setSavedMapStatus([
            `已加载：${data.map_name}`,
            `PGM/YAML：${data.yaml_path}`,
            `尺寸：${data.width} x ${data.height} / ${Number(data.resolution || 0).toFixed(3)} m`,
            `PCD点云：${data.pcd_cloud ? data.pcd_cloud.points.length : 0}/${data.pcd_cloud ? data.pcd_cloud.source_points : 0}`,
          ].join("\n"));
          setMapStatus(`离线地图：${data.map_name} / 栅格 ${data.width}x${data.height} / PCD ${data.pcd_cloud ? data.pcd_cloud.points.length : 0}/${data.pcd_cloud ? data.pcd_cloud.source_points : 0}`);
          updateCloud3d();
          drawScene();
        };
        image.onerror = () => setSavedMapStatus("地图预览图加载失败");
        image.src = `${defaultApiBaseUrl()}${data.preview_url.replace("t=0", `t=${Date.now()}`)}`;
      })
      .catch(() => setSavedMapStatus("地图加载API未连接"));
  }

  function parseNestedStatus(raw) {
    if (!raw) {
      return "等待数据";
    }
    try {
      return JSON.stringify(JSON.parse(raw), null, 2);
    } catch (error) {
      return raw;
    }
  }

  function setCameraStatus(text) {
    cameraStatus.textContent = text;
  }

  function setMapStatus(text) {
    mapStatus.textContent = text;
  }

  function startCameraView() {
    cameraPanel.hidden = false;
    setCameraStatus("正在打开视频流");
    cameraImage.src = `${defaultApiBaseUrl()}/api/camera/stream.mjpg?t=${Date.now()}`;
  }

  function stopCameraView() {
    cameraPanel.hidden = true;
    cameraImage.removeAttribute("src");
    setCameraStatus("已停止");
  }

  function applyExposure() {
    const exposureTimeUs = Number(exposureTimeInput.value || 0);
    if (!Number.isFinite(exposureTimeUs) || exposureTimeUs <= 0) {
      setCameraStatus("曝光时间无效");
      return;
    }

    setCameraStatus("正在设置曝光");
    fetch(`${defaultApiBaseUrl()}/api/camera/exposure`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ exposure_time_us: exposureTimeUs }),
    })
      .then((res) => res.json().then((data) => ({ ok: res.ok && data.ok, data })))
      .then(({ ok, data }) => {
        if (!ok) {
          setCameraStatus(data.error || "曝光设置失败");
          return;
        }
        setCameraStatus(`曝光已设置 ${Math.round(data.exposure_time_us)} us`);
      })
      .catch(() => setCameraStatus("曝光设置请求失败"));
  }

  function pollApiStatus() {
    fetch(`${defaultApiBaseUrl()}/api/status`, { cache: "no-store" })
      .then((res) => res.json())
      .then((data) => {
        apiAvailable = true;
        if (!rosConnected) {
          setConnectionState(true, "API模式");
        }
        systemStatus.textContent = parseNestedStatus(data.system_status);
        controlStatus.textContent = parseNestedStatus(data.control_status);
        networkStatus.textContent = parseNestedStatus(data.network_status);
        navStatus.textContent = parseNestedStatus(data.nav_status);
      })
      .catch(() => {
        apiAvailable = false;
        if (!rosConnected) {
          setConnectionState(false, "未连接");
        }
      });
  }

  function pollMapSnapshot() {
    if (mapSnapshotPollInFlight) {
      return;
    }
    mapSnapshotPollInFlight = true;
    fetch(`${defaultApiBaseUrl()}/api/map/snapshot?t=${Date.now()}`, {
      cache: "no-store",
    })
      .then((res) => res.json())
      .then((data) => {
        mapSnapshot = data;
        lastMapSnapshotOk = true;
        refreshMapBoundsForSnapshot(data);
        updateMapStatus(data);
        drawMappingPreview();
        drawScene();
      })
      .catch(() => {
        lastMapSnapshotOk = false;
        setMapStatus("雷达地图：API未连接");
        drawMappingPreview();
        drawScene();
      })
      .finally(() => {
        mapSnapshotPollInFlight = false;
      });
  }

  function updateMapStatus(snapshot) {
    if (!snapshot || !snapshot.ok) {
      setMapStatus("雷达地图：等待API");
      return;
    }
    const mappingPoints = snapshot.mapping_cloud && snapshot.mapping_cloud.points
      ? snapshot.mapping_cloud.points.length
      : 0;
    const pose = snapshot.pose && snapshot.pose.available;
    const odomPose = snapshot.odom_pose && snapshot.odom_pose.available;
    if (isMappingMode()) {
      const sourcePoints = snapshot.mapping_cloud ? Number(snapshot.mapping_cloud.source_points || 0) : 0;
      const visibleMappingPoints = countVisibleMappingPoints(snapshot.mapping_cloud);
      const filterText = mappingHeightFilterIsEnabled()
        ? `高度过滤 Z<=${mappingMaxZ().toFixed(2)}m`
        : "不限高";
      setMapStatus(
        `雷达地图：建图点云在右侧独立窗口显示 / FAST-LIO ${visibleMappingPoints}/${mappingPoints}/${sourcePoints} / ${filterText} / 机器人 ${odomPose ? "odom在线" : pose ? "定位在线" : "等待"}`,
      );
      return;
    }
    if (selectedSavedMap && selectedSavedMapImage) {
      const cloud = selectedSavedMap.pcd_cloud || {};
      const livePoints = snapshot.live_cloud && snapshot.live_cloud.points
        ? snapshot.live_cloud.points.length
        : 0;
      const navPathPoints = snapshot.nav_path && snapshot.nav_path.points ? snapshot.nav_path.points.length : 0;
      const localPathPoints = snapshot.nav_local_path && snapshot.nav_local_path.points
        ? snapshot.nav_local_path.points.length
        : 0;
      const localPathLength = pathLengthMeters(snapshot.nav_local_path && snapshot.nav_local_path.points);
      const navState = parseStatusState(snapshot.nav_status);
      setMapStatus(
        `离线地图：${selectedSavedMap.map_name} / 栅格 ${selectedSavedMap.width}x${selectedSavedMap.height} / PCD ${(cloud.points || []).length}/${cloud.source_points || 0} / 实时点云 ${livePoints} / 定位 ${pose ? "map在线" : odomPose ? "odom临时" : "等待"} / 路径 ${navPathPoints} / 局部 ${localPathPoints}点 ${localPathLength.toFixed(2)}m${navState ? ` / ${navState}` : ""}`,
      );
      return;
    }
    setMapStatus("地图：请先在保存地图列表中选择并加载地图；未加载地图前不会显示后台默认地图或允许点选导航。");
    return;
    const grid = snapshot.grid && snapshot.grid.available;
    const staticPoints = snapshot.static_cloud && snapshot.static_cloud.points
      ? snapshot.static_cloud.points.length
      : 0;
    const livePoints = snapshot.live_cloud && snapshot.live_cloud.points
      ? snapshot.live_cloud.points.length
      : 0;
    const localizationStatus = snapshot.localization_status || "";
    const navPathPoints = snapshot.nav_path && snapshot.nav_path.points ? snapshot.nav_path.points.length : 0;
    const localPathPoints = snapshot.nav_local_path && snapshot.nav_local_path.points
      ? snapshot.nav_local_path.points.length
      : 0;
    const localPathLength = pathLengthMeters(snapshot.nav_local_path && snapshot.nav_local_path.points);
    const navState = parseStatusState(snapshot.nav_status);
    if (!grid && staticPoints === 0 && livePoints === 0 && !pose) {
      setMapStatus(`雷达地图：等待 /map 和点云数据${localizationStatus ? ` / ${localizationStatus}` : ""}`);
      return;
    }
    setMapStatus(
      `雷达地图：栅格 ${grid ? "在线" : "等待"} / 地图点云 ${staticPoints} / 实时点云 ${livePoints} / 建图点云 ${mappingPoints} / 定位 ${pose ? "map在线" : odomPose ? "odom临时" : "等待"} / 路径 ${navPathPoints} / 局部 ${localPathPoints}点 ${localPathLength.toFixed(2)}m${navState ? ` / ${navState}` : ""}${localizationStatus ? ` / ${localizationStatus}` : ""}`,
    );
  }

  function refreshMapBoundsForSnapshot(snapshot) {
    if (!snapshotHasMapData(snapshot)) {
      return;
    }

    if (!mapViewBounds) {
      mapViewBounds = fullMapBounds();
    }
  }

  function parseStatusState(raw) {
    if (!raw) {
      return "";
    }
    try {
      const data = JSON.parse(raw);
      return data.state || "";
    } catch (error) {
      return "";
    }
  }

  function subscribeStringTopic(name, outputEl) {
    const topic = new ROSLIB.Topic({
      ros,
      name,
      messageType: "std_msgs/String",
    });
    topic.subscribe((msg) => {
      outputEl.textContent = formatJson(msg.data);
      drawScene();
    });
    return topic;
  }

  function connect() {
    if (!ROSLIB) {
      setConnectionState(false, "ROSLIB 缺失");
      return;
    }
    if (ros) {
      ros.close();
    }

    ros = new ROSLIB.Ros({ url: wsInput.value.trim() || defaultRosbridgeUrl() });

    ros.on("connection", () => {
      rosConnected = true;
      setConnectionState(true, "已连接");

      cmdTopic = new ROSLIB.Topic({
        ros,
        name: "/web/cmd_vel",
        messageType: "geometry_msgs/Twist",
      });
      estopTopic = new ROSLIB.Topic({
        ros,
        name: "/web/estop",
        messageType: "std_msgs/Bool",
      });
      goalTopic = new ROSLIB.Topic({
        ros,
        name: "/web/goal_pose",
        messageType: "geometry_msgs/PoseStamped",
      });

      subscribeStringTopic("/web/system_status", systemStatus);
      subscribeStringTopic("/web/control_status", controlStatus);
      subscribeStringTopic("/web/network_status", networkStatus);
    });

    ros.on("error", () => {
      rosConnected = false;
      if (!apiAvailable) {
        setConnectionState(false, "连接错误");
      }
    });

    ros.on("close", () => {
      rosConnected = false;
      cmdTopic = null;
      estopTopic = null;
      goalTopic = null;
      setConnectionState(apiAvailable, apiAvailable ? "API模式" : "未连接");
    });
  }

  function disconnect() {
    if (ros) {
      ros.close();
      ros = null;
    }
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const scale = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.max(1, Math.floor(rect.width * scale));
    canvas.height = Math.max(1, Math.floor(rect.height * scale));
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    resizeCloud3d();
    resizeMappingPreview();
    drawScene();
  }

  function resizeMappingPreview() {
    if (!mappingRenderer3d || !mappingPreviewCanvas) {
      return;
    }
    if (mappingPreviewPanel && mappingPreviewPanel.hidden) {
      return;
    }
    const rect = mappingPreviewCanvas.getBoundingClientRect();
    const width = Math.max(1, Math.floor(rect.width || 1));
    const height = Math.max(1, Math.floor(rect.height || 1));
    mappingRenderer3d.setSize(width, height, false);
    mappingCamera3d.aspect = width / height;
    mappingCamera3d.updateProjectionMatrix();
    drawMappingPreview();
  }

  function resizeCloud3d() {
    if (!renderer3d) {
      return;
    }
    const rect = cloud3dCanvas.getBoundingClientRect();
    const width = Math.max(1, Math.floor(rect.width || canvas.clientWidth || 1));
    const height = Math.max(1, Math.floor(rect.height || canvas.clientHeight || 1));
    renderer3d.setSize(width, height, false);
    camera3d.aspect = width / height;
    camera3d.updateProjectionMatrix();
  }

  function snapshotHasMapData(snapshot) {
    return Boolean(selectedSavedMap && selectedSavedMapImage);
  }

  function frameMatches(a, b) {
    return Boolean(a && b && String(a) === String(b));
  }

  function mappingPose(snapshot) {
    if (!snapshot) {
      return null;
    }
    const mapFrame = snapshot.mapping_cloud && snapshot.mapping_cloud.frame_id;
    const odomPose = snapshot.odom_pose;
    const localizedPose = snapshot.pose;
    if (odomPose && odomPose.available && frameMatches(odomPose.frame_id, mapFrame)) {
      return odomPose;
    }
    if (localizedPose && localizedPose.available && frameMatches(localizedPose.frame_id, mapFrame)) {
      return localizedPose;
    }
    return null;
  }

  function includePoint(bounds, x, y) {
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return bounds;
    }
    if (!bounds) {
      return { minX: x, maxX: x, minY: y, maxY: y };
    }
    bounds.minX = Math.min(bounds.minX, x);
    bounds.maxX = Math.max(bounds.maxX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.maxY = Math.max(bounds.maxY, y);
    return bounds;
  }

  function includeCloudBounds(bounds, cloud, predicate) {
    if (!cloud || !cloud.points) {
      return bounds;
    }
    cloud.points.forEach((point) => {
      if (predicate && !predicate(point)) {
        return;
      }
      bounds = includePoint(bounds, Number(point[0]), Number(point[1]));
    });
    return bounds;
  }

  function pathLengthMeters(points) {
    if (!points || points.length < 2) {
      return 0;
    }
    let length = 0;
    for (let i = 1; i < points.length; i += 1) {
      const ax = Number(points[i - 1][0]);
      const ay = Number(points[i - 1][1]);
      const bx = Number(points[i][0]);
      const by = Number(points[i][1]);
      if (Number.isFinite(ax) && Number.isFinite(ay) && Number.isFinite(bx) && Number.isFinite(by)) {
        length += Math.hypot(bx - ax, by - ay);
      }
    }
    return length;
  }

  function computeMapBounds(snapshot) {
    let bounds = null;
    if (selectedSavedMap && selectedSavedMapImage) {
      bounds = includeSavedMapBounds(bounds);
      bounds = includeCloudBounds(bounds, selectedSavedMap.pcd_cloud);
      bounds = includeCloudBounds(bounds, snapshot && snapshot.nav_path);
      bounds = includeCloudBounds(bounds, snapshot && snapshot.nav_local_path);
      if (snapshot && snapshot.pose && snapshot.pose.available) {
        bounds = includePoint(bounds, Number(snapshot.pose.x), Number(snapshot.pose.y));
      } else if (snapshot && snapshot.odom_pose && snapshot.odom_pose.available) {
        bounds = includePoint(bounds, Number(snapshot.odom_pose.x), Number(snapshot.odom_pose.y));
      }
      if (bounds) {
        const width = Math.max(1, bounds.maxX - bounds.minX);
        const height = Math.max(1, bounds.maxY - bounds.minY);
        const padding = Math.max(width, height) * 0.08 + 0.5;
        return {
          minX: bounds.minX - padding,
          maxX: bounds.maxX + padding,
          minY: bounds.minY - padding,
          maxY: bounds.maxY + padding,
        };
      }
    }
    if (!selectedSavedMap || !selectedSavedMapImage) {
      return { minX: -5, maxX: 5, minY: -5, maxY: 5 };
    }
    const grid = snapshot && snapshot.grid;
    if (grid && grid.available && grid.width > 0 && grid.height > 0) {
      const minX = Number(grid.origin.x || 0);
      const minY = Number(grid.origin.y || 0);
      const maxX = minX + Number(grid.width) * Number(grid.resolution || 1);
      const maxY = minY + Number(grid.height) * Number(grid.resolution || 1);
      bounds = { minX, maxX, minY, maxY };
    }
    bounds = includeCloudBounds(bounds, snapshot && snapshot.static_cloud);
    bounds = includeCloudBounds(bounds, snapshot && snapshot.live_cloud);
    bounds = includeCloudBounds(bounds, snapshot && snapshot.nav_path);
    bounds = includeCloudBounds(bounds, snapshot && snapshot.nav_local_path);
    if (snapshot && snapshot.pose && snapshot.pose.available) {
      bounds = includePoint(bounds, Number(snapshot.pose.x), Number(snapshot.pose.y));
    } else if (snapshot && snapshot.odom_pose && snapshot.odom_pose.available) {
      bounds = includePoint(bounds, Number(snapshot.odom_pose.x), Number(snapshot.odom_pose.y));
    }
    if (!bounds) {
      return { minX: -5, maxX: 5, minY: -5, maxY: 5 };
    }
    const width = Math.max(1, bounds.maxX - bounds.minX);
    const height = Math.max(1, bounds.maxY - bounds.minY);
    const padding = Math.max(width, height) * 0.08 + 0.5;
    return {
      minX: bounds.minX - padding,
      maxX: bounds.maxX + padding,
      minY: bounds.minY - padding,
      maxY: bounds.maxY + padding,
    };
  }

  function computeMappingBounds(snapshot) {
    let bounds = null;
    bounds = includeCloudBounds(bounds, snapshot && snapshot.mapping_cloud, mappingPointPassesHeight);
    const pose = mappingPose(snapshot);
    if (pose && pose.available) {
      bounds = includePoint(bounds, Number(pose.x), Number(pose.y));
    }
    if (!bounds) {
      return { minX: -5, maxX: 5, minY: -5, maxY: 5 };
    }
    const width = Math.max(1, bounds.maxX - bounds.minX);
    const height = Math.max(1, bounds.maxY - bounds.minY);
    const padding = Math.max(width, height) * 0.18 + 0.8;
    return {
      minX: bounds.minX - padding,
      maxX: bounds.maxX + padding,
      minY: bounds.minY - padding,
      maxY: bounds.maxY + padding,
    };
  }

  function savedMapBounds(map) {
    if (!map || !map.origin) {
      return null;
    }
    const resolution = Number(map.resolution || 0.05);
    const width = Number(map.width || 0);
    const height = Number(map.height || 0);
    const minX = Number(map.origin.x || 0);
    const minY = Number(map.origin.y || 0);
    if (!Number.isFinite(resolution) || width <= 0 || height <= 0) {
      return null;
    }
    return {
      minX,
      maxX: minX + width * resolution,
      minY,
      maxY: minY + height * resolution,
    };
  }

  function includeSavedMapBounds(bounds) {
    if (!selectedSavedMap) {
      return bounds;
    }
    return mergeBounds(bounds, savedMapBounds(selectedSavedMap));
  }

  function visibleMappingPoints() {
    const cloud = mapSnapshot && mapSnapshot.mapping_cloud;
    if (!cloud || !cloud.available || !cloud.points) {
      return [];
    }
    return cloud.points.filter(mappingPointPassesHeight);
  }

  function visibleSavedMapPoints() {
    const cloud = selectedSavedMap && selectedSavedMap.pcd_cloud;
    if (!cloud || !cloud.available || !cloud.points) {
      return [];
    }
    return cloud.points;
  }

  function visibleSavedMap3dPoints() {
    const cloud = selectedSavedMap && selectedSavedMap.pcd_cloud_3d &&
      selectedSavedMap.pcd_cloud_3d.available
      ? selectedSavedMap.pcd_cloud_3d
      : (selectedSavedMap && selectedSavedMap.pcd_cloud);
    if (!cloud || !cloud.available || !cloud.points) {
      return [];
    }
    return cloud.points;
  }

  function appendCloud3dPoints(items, points, source) {
    if (!points || !points.length) {
      return;
    }
    points.forEach((point) => items.push({ point, source }));
  }

  function activeCloud3dPoints() {
    const items = [];
    if (selectedSavedMap && selectedSavedMapImage && layerSavedMap.checked) {
      appendCloud3dPoints(items, visibleSavedMap3dPoints(), "saved");
    }
    if (selectedSavedMap && selectedSavedMapImage && layerLiveCloud.checked) {
      const liveCloud3d = mapSnapshot && mapSnapshot.live_cloud_3d &&
        mapSnapshot.live_cloud_3d.available
        ? mapSnapshot.live_cloud_3d
        : (mapSnapshot && mapSnapshot.live_cloud);
      appendCloud3dPoints(items, liveCloud3d && liveCloud3d.points, "live");
    }
    return items;
  }

  function colorForZ(z, minZ, maxZ) {
    const span = Math.max(0.001, maxZ - minZ);
    const t = Math.min(1, Math.max(0, (z - minZ) / span));
    return {
      r: 1.0,
      g: 0.38 + 0.42 * (1 - t),
      b: 0.18 + 0.60 * t,
    };
  }

  function colorForCloud3dPoint(source, z, minZ, maxZ) {
    if (source === "live") {
      return { r: 1.0, g: 0.80, b: 0.18 };
    }
    if (source === "mapping") {
      return { r: 1.0, g: 0.38, b: 0.22 };
    }
    if (source === "static") {
      return { r: 0.35, g: 0.68, b: 0.95 };
    }
    return colorForZ(z, minZ, maxZ);
  }

  function updateCloud3d() {
    if (!is3dViewActive()) {
      return;
    }

    const pointItems = activeCloud3dPoints();
    const positions = new Float32Array(pointItems.length * 3);
    const colors = new Float32Array(pointItems.length * 3);
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    let minZ = Infinity;
    let maxZ = -Infinity;

    pointItems.forEach((item) => {
      const point = item.point;
      const x = Number(point[0]);
      const y = Number(point[1]);
      const z = Number(point[2]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        return;
      }
      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
      minZ = Math.min(minZ, z);
      maxZ = Math.max(maxZ, z);
    });

    pointItems.forEach((item, index) => {
      const point = item.point;
      const x = Number(point[0]);
      const y = Number(point[1]);
      const z = Number(point[2]);
      const color = colorForCloud3dPoint(item.source, z, minZ, maxZ);
      const offset = index * 3;
      positions[offset] = x;
      positions[offset + 1] = y;
      positions[offset + 2] = z;
      colors[offset] = color.r;
      colors[offset + 1] = color.g;
      colors[offset + 2] = color.b;
    });

    pointGeometry3d.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    pointGeometry3d.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    pointGeometry3d.computeBoundingSphere();

    const pose = mapSnapshot && mapSnapshot.pose && mapSnapshot.pose.available
      ? mapSnapshot.pose
      : (mapSnapshot && mapSnapshot.odom_pose);
    robotMarker3d.visible = Boolean(pose && pose.available);
    robotHeading3d.visible = robotMarker3d.visible;
    if (robotMarker3d.visible) {
      const x = Number(pose.x || 0);
      const y = Number(pose.y || 0);
      const z = Number(pose.z || 0);
      const yaw = Number(pose.yaw || 0);
      robotMarker3d.position.set(x, y, z);
      robotHeading3d.position.set(x, y, z + 0.08);
      robotHeading3d.setDirection(new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0).normalize());
    }

    if (pointItems.length > 0 && Number.isFinite(minX) && cloud3dNeedsFrame) {
      const center = new THREE.Vector3(
        (minX + maxX) * 0.5,
        (minY + maxY) * 0.5,
        (minZ + maxZ) * 0.5,
      );
      const spanX = Math.max(0.5, maxX - minX);
      const spanY = Math.max(0.5, maxY - minY);
      const spanZ = Math.max(0.5, maxZ - minZ);
      const radius = Math.max(spanX, spanY, spanZ);
      controls3d.target.copy(center);
      camera3d.position.set(center.x + radius * 0.75, center.y - radius * 1.15, center.z + radius * 0.75);
      camera3d.near = Math.max(0.01, radius / 1000);
      camera3d.far = Math.max(100, radius * 8);
      camera3d.updateProjectionMatrix();
      controls3d.update();
      cloud3dNeedsFrame = false;
    }
  }

  function renderCloud3d() {
    requestAnimationFrame(renderCloud3d);
    controls3d.update();
    if (renderer3d && is3dViewActive()) {
      renderer3d.render(scene3d, camera3d);
    }
  }

  function renderMappingPreview3d() {
    requestAnimationFrame(renderMappingPreview3d);
    mappingControls3d.update();
    if (mappingRenderer3d) {
      mappingRenderer3d.render(mappingScene3d, mappingCamera3d);
    }
  }

  function boundsWidth(bounds) {
    return Math.max(0.1, bounds.maxX - bounds.minX);
  }

  function boundsHeight(bounds) {
    return Math.max(0.1, bounds.maxY - bounds.minY);
  }

  function boundsContainBounds(outer, inner) {
    if (!outer || !inner) {
      return false;
    }
    return inner.minX >= outer.minX &&
      inner.maxX <= outer.maxX &&
      inner.minY >= outer.minY &&
      inner.maxY <= outer.maxY;
  }

  function mergeBounds(a, b) {
    if (!a) {
      return b;
    }
    if (!b) {
      return a;
    }
    return {
      minX: Math.min(a.minX, b.minX),
      maxX: Math.max(a.maxX, b.maxX),
      minY: Math.min(a.minY, b.minY),
      maxY: Math.max(a.maxY, b.maxY),
    };
  }

  function fullMapBounds() {
    return computeMapBounds(mapSnapshot);
  }

  function currentMapBounds() {
    if (!mapViewBounds) {
      mapViewBounds = fullMapBounds();
    }
    return mapViewBounds;
  }

  function updateZoomLabel() {
    if (!mapZoomLabel || !snapshotHasMapData(mapSnapshot)) {
      return;
    }
    const full = fullMapBounds();
    const current = currentMapBounds();
    const fullSpan = Math.max(boundsWidth(full), boundsHeight(full));
    const currentSpan = Math.max(boundsWidth(current), boundsHeight(current));
    const zoom = Math.max(0.1, fullSpan / Math.max(0.1, currentSpan));
    mapZoomLabel.textContent = `${zoom.toFixed(1)}x`;
  }

  function zoomMap(factor, anchorCanvasPoint) {
    if (!snapshotHasMapData(mapSnapshot)) {
      return;
    }
    const bounds = currentMapBounds();
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const anchor = anchorCanvasPoint
      ? canvasToWorld(anchorCanvasPoint.x, anchorCanvasPoint.y, bounds, width, height)
      : {
        x: (bounds.minX + bounds.maxX) * 0.5,
        y: (bounds.minY + bounds.maxY) * 0.5,
      };
    const minSpan = 0.35;
    const maxSpan = Math.max(boundsWidth(fullMapBounds()), boundsHeight(fullMapBounds())) * 2.5;
    let nextWidth = boundsWidth(bounds) * factor;
    let nextHeight = boundsHeight(bounds) * factor;
    const nextSpan = Math.max(nextWidth, nextHeight);
    if (nextSpan < minSpan || nextSpan > maxSpan) {
      return;
    }
    mapViewBounds = {
      minX: anchor.x - (anchor.x - bounds.minX) * factor,
      maxX: anchor.x + (bounds.maxX - anchor.x) * factor,
      minY: anchor.y - (anchor.y - bounds.minY) * factor,
      maxY: anchor.y + (bounds.maxY - anchor.y) * factor,
    };
    mapViewUserAdjusted = true;
    drawScene();
  }

  function panMap(deltaCanvasX, deltaCanvasY, startBounds) {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const start = startBounds || currentMapBounds();
    const p0 = canvasToWorld(0, 0, start, width, height);
    const p1 = canvasToWorld(deltaCanvasX, deltaCanvasY, start, width, height);
    const dx = p1.x - p0.x;
    const dy = p1.y - p0.y;
    mapViewBounds = {
      minX: start.minX - dx,
      maxX: start.maxX - dx,
      minY: start.minY - dy,
      maxY: start.maxY - dy,
    };
    mapViewUserAdjusted = true;
    drawScene();
  }

  function worldToCanvas(x, y, bounds, width, height) {
    const pad = 24;
    const worldWidth = Math.max(0.1, bounds.maxX - bounds.minX);
    const worldHeight = Math.max(0.1, bounds.maxY - bounds.minY);
    const scale = Math.min((width - pad * 2) / worldWidth, (height - pad * 2) / worldHeight);
    const offsetX = (width - worldWidth * scale) * 0.5;
    const offsetY = (height - worldHeight * scale) * 0.5;
    return {
      x: offsetX + (x - bounds.minX) * scale,
      y: height - offsetY - (y - bounds.minY) * scale,
      scale,
    };
  }

  function canvasToWorld(canvasX, canvasY, bounds, width, height) {
    const pad = 24;
    const worldWidth = Math.max(0.1, bounds.maxX - bounds.minX);
    const worldHeight = Math.max(0.1, bounds.maxY - bounds.minY);
    const scale = Math.min((width - pad * 2) / worldWidth, (height - pad * 2) / worldHeight);
    const offsetX = (width - worldWidth * scale) * 0.5;
    const offsetY = (height - worldHeight * scale) * 0.5;
    return {
      x: bounds.minX + (canvasX - offsetX) / scale,
      y: bounds.minY + (height - offsetY - canvasY) / scale,
    };
  }

  function metricGridStep(span) {
    const targetLines = 8;
    const raw = Math.max(0.1, span / targetLines);
    const base = 10 ** Math.floor(Math.log10(raw));
    const scaled = raw / base;
    if (scaled >= 5) {
      return base * 5;
    }
    if (scaled >= 2) {
      return base * 2;
    }
    return base;
  }

  function drawWorldGrid(width, height, bounds) {
    if (!bounds) {
      return;
    }
    const span = Math.max(boundsWidth(bounds), boundsHeight(bounds));
    const step = metricGridStep(span);
    const xStart = Math.floor(bounds.minX / step) * step;
    const xEnd = Math.ceil(bounds.maxX / step) * step;
    const yStart = Math.floor(bounds.minY / step) * step;
    const yEnd = Math.ceil(bounds.maxY / step) * step;

    ctx.save();
    ctx.lineWidth = 1;
    ctx.font = "11px sans-serif";
    ctx.textBaseline = "top";

    for (let x = xStart; x <= xEnd + step * 0.5; x += step) {
      const p = worldToCanvas(x, bounds.minY, bounds, width, height);
      const isAxis = Math.abs(x) < step * 0.001;
      ctx.strokeStyle = isAxis ? "rgba(83, 194, 164, 0.48)" : "rgba(38, 53, 54, 0.72)";
      ctx.beginPath();
      ctx.moveTo(p.x, 0);
      ctx.lineTo(p.x, height);
      ctx.stroke();
      if (p.x >= 4 && p.x <= width - 36) {
        ctx.fillStyle = isAxis ? "rgba(128, 232, 200, 0.92)" : "rgba(142, 165, 159, 0.75)";
        ctx.fillText(x.toFixed(step < 1 ? 1 : 0), p.x + 3, 6);
      }
    }

    for (let y = yStart; y <= yEnd + step * 0.5; y += step) {
      const p = worldToCanvas(bounds.minX, y, bounds, width, height);
      const isAxis = Math.abs(y) < step * 0.001;
      ctx.strokeStyle = isAxis ? "rgba(242, 195, 91, 0.50)" : "rgba(38, 53, 54, 0.72)";
      ctx.beginPath();
      ctx.moveTo(0, p.y);
      ctx.lineTo(width, p.y);
      ctx.stroke();
      if (p.y >= 18 && p.y <= height - 14) {
        ctx.fillStyle = isAxis ? "rgba(246, 211, 130, 0.92)" : "rgba(142, 165, 159, 0.75)";
        ctx.fillText(y.toFixed(step < 1 ? 1 : 0), 6, p.y + 3);
      }
    }

    ctx.restore();
  }

  function drawOccupancyGrid(snapshot, bounds, width, height) {
    const grid = snapshot.grid;
    if (!layerGrid.checked || !grid || !grid.available || !grid.data || grid.data.length === 0) {
      return;
    }
    const resolution = Number(grid.resolution || 1);
    const originX = Number(grid.origin.x || 0);
    const originY = Number(grid.origin.y || 0);
    const cell = Math.max(1, worldToCanvas(originX + resolution, originY, bounds, width, height).x -
      worldToCanvas(originX, originY, bounds, width, height).x);
    for (let y = 0; y < grid.height; y += 1) {
      for (let x = 0; x < grid.width; x += 1) {
        const value = Number(grid.data[y * grid.width + x]);
        if (value < 0) {
          continue;
        }
        const p = worldToCanvas(originX + x * resolution, originY + y * resolution, bounds, width, height);
        if (value >= 65) {
          ctx.fillStyle = "rgba(225, 232, 226, 0.78)";
        } else if (value <= 20) {
          ctx.fillStyle = "rgba(64, 90, 86, 0.22)";
        } else {
          ctx.fillStyle = "rgba(130, 148, 142, 0.34)";
        }
        ctx.fillRect(p.x, p.y - cell, cell, cell);
      }
    }
  }

  function drawCostmapGrid(snapshot, bounds, width, height) {
    const grid = snapshot.nav_costmap;
    if (!layerCostmap.checked || !grid || !grid.available || !grid.data || grid.data.length === 0) {
      return;
    }
    const resolution = Number(grid.resolution || 1);
    const originX = Number(grid.origin.x || 0);
    const originY = Number(grid.origin.y || 0);
    const cell = Math.max(1, worldToCanvas(originX + resolution, originY, bounds, width, height).x -
      worldToCanvas(originX, originY, bounds, width, height).x);
    ctx.save();
    for (let y = 0; y < grid.height; y += 1) {
      for (let x = 0; x < grid.width; x += 1) {
        const value = Number(grid.data[y * grid.width + x]);
        if (value <= 0) {
          continue;
        }
        const p = worldToCanvas(originX + x * resolution, originY + y * resolution, bounds, width, height);
        ctx.fillStyle = value >= 100 ? "rgba(255, 55, 82, 0.38)" : "rgba(255, 181, 65, 0.24)";
        ctx.fillRect(p.x, p.y - cell, cell, cell);
      }
    }
    ctx.restore();
  }

  function drawSavedMap(bounds, width, height) {
    if (!layerSavedMap.checked || !selectedSavedMap || !selectedSavedMapImage) {
      return;
    }
    const mapBounds = savedMapBounds(selectedSavedMap);
    if (!mapBounds) {
      return;
    }
    const topLeft = worldToCanvas(mapBounds.minX, mapBounds.maxY, bounds, width, height);
    const bottomRight = worldToCanvas(mapBounds.maxX, mapBounds.minY, bounds, width, height);
    const drawWidth = bottomRight.x - topLeft.x;
    const drawHeight = bottomRight.y - topLeft.y;
    ctx.save();
    ctx.globalAlpha = 0.82;
    ctx.drawImage(selectedSavedMapImage, topLeft.x, topLeft.y, drawWidth, drawHeight);
    ctx.restore();
  }

  function drawCloud(cloud, bounds, width, height, color, size, predicate) {
    if (!cloud || !cloud.available || !cloud.points) {
      return;
    }
    ctx.fillStyle = color;
    cloud.points.forEach((point) => {
      if (predicate && !predicate(point)) {
        return;
      }
      const p = worldToCanvas(Number(point[0]), Number(point[1]), bounds, width, height);
      ctx.fillRect(p.x - size * 0.5, p.y - size * 0.5, size, size);
    });
  }

  function drawRobotPose(snapshot, bounds, width, height) {
    const pose = snapshot.pose && snapshot.pose.available ? snapshot.pose : snapshot.odom_pose;
    if (!layerPose.checked || !pose || !pose.available) {
      return;
    }
    const p = worldToCanvas(Number(pose.x), Number(pose.y), bounds, width, height);
    const yaw = Number(pose.yaw || 0);
    const bodyRadius = 10;
    const headingLength = 24;
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-yaw);
    const isMapPose = pose.source === "/localization_2d";
    ctx.fillStyle = isMapPose ? "#53c2a4" : "#f2c35b";
    ctx.strokeStyle = "#eafff7";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, bodyRadius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(headingLength, 0);
    ctx.lineTo(4, -7);
    ctx.lineTo(4, 7);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function drawMappingPreview() {
    if (!shouldShowMappingPreview()) {
      clearMappingPreviewGeometry();
      return;
    }
    if (!mappingRenderer3d || !mappingWebglAvailable) {
      if (mappingPreviewStatus) {
        mappingPreviewStatus.textContent = "建图点云：WebGL不可用";
      }
      return;
    }
    const cloud = mapSnapshot && mapSnapshot.mapping_cloud;
    const points = cloud && cloud.available && cloud.points ? cloud.points.filter(mappingPointPassesHeight) : [];
    if (!points.length) {
      mappingPointGeometry3d.setAttribute("position", new THREE.BufferAttribute(new Float32Array(0), 3));
      mappingPointGeometry3d.setAttribute("color", new THREE.BufferAttribute(new Float32Array(0), 3));
      mappingRobotMarker3d.visible = false;
      mappingRobotHeading3d.visible = false;
      if (mappingPreviewStatus) {
        mappingPreviewStatus.textContent = isMappingMode()
          ? "3D建图点云：等待 /Laser_map"
          : "3D建图点云：未开始";
      }
      return;
    }

    const positions = new Float32Array(points.length * 3);
    const colors = new Float32Array(points.length * 3);
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    let minZ = Infinity;
    let maxZ = -Infinity;
    points.forEach((point) => {
      const x = Number(point[0]);
      const y = Number(point[1]);
      const z = Number(point[2]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        return;
      }
      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
      minZ = Math.min(minZ, z);
      maxZ = Math.max(maxZ, z);
    });

    points.forEach((point, index) => {
      const x = Number(point[0]);
      const y = Number(point[1]);
      const z = Number(point[2]);
      const color = colorForZ(z, minZ, maxZ);
      const offset = index * 3;
      positions[offset] = x;
      positions[offset + 1] = y;
      positions[offset + 2] = z;
      colors[offset] = color.r;
      colors[offset + 1] = color.g;
      colors[offset + 2] = color.b;
    });

    mappingPointGeometry3d.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    mappingPointGeometry3d.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    mappingPointGeometry3d.computeBoundingSphere();

    const pose = mappingPose(mapSnapshot);
    mappingRobotMarker3d.visible = Boolean(pose && pose.available);
    mappingRobotHeading3d.visible = mappingRobotMarker3d.visible;
    if (pose && pose.available) {
      const x = Number(pose.x || 0);
      const y = Number(pose.y || 0);
      const z = Number(pose.z || 0);
      const yaw = Number(pose.yaw || 0);
      mappingRobotMarker3d.position.set(x, y, z);
      mappingRobotHeading3d.position.set(x, y, z + 0.08);
      mappingRobotHeading3d.setDirection(new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0).normalize());
    }

    if (points.length > 0 && Number.isFinite(minX) && mappingCloudNeedsFrame) {
      const center = new THREE.Vector3(
        (minX + maxX) * 0.5,
        (minY + maxY) * 0.5,
        (minZ + maxZ) * 0.5,
      );
      const spanX = Math.max(0.5, maxX - minX);
      const spanY = Math.max(0.5, maxY - minY);
      const spanZ = Math.max(0.5, maxZ - minZ);
      const radius = Math.max(spanX, spanY, spanZ);
      mappingControls3d.target.copy(center);
      mappingCamera3d.position.set(center.x + radius * 0.75, center.y - radius * 1.15, center.z + radius * 0.75);
      mappingCamera3d.near = Math.max(0.01, radius / 1000);
      mappingCamera3d.far = Math.max(100, radius * 8);
      mappingCamera3d.updateProjectionMatrix();
      mappingControls3d.update();
      mappingCloudNeedsFrame = false;
    }

    const sourcePoints = Number(cloud.source_points || 0);
    lastMappingSourcePoints = sourcePoints;
    if (mappingPreviewStatus) {
      mappingPreviewStatus.textContent =
        `3D建图点云：${points.length}/${cloud.points.length}/${sourcePoints} / ${cloud.floor_plane_source || "floor"}`;
    }
  }

  function drawNavPath(snapshot, bounds, width, height) {
    if (!layerNavPath.checked || !snapshot.nav_path || !snapshot.nav_path.available ||
      !snapshot.nav_path.points || snapshot.nav_path.points.length === 0) {
      return;
    }
    const points = snapshot.nav_path.points;
    ctx.save();
    ctx.strokeStyle = "#ff7f50";
    ctx.lineWidth = 4;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    points.forEach((point, index) => {
      const p = worldToCanvas(Number(point[0]), Number(point[1]), bounds, width, height);
      if (index === 0) {
        ctx.moveTo(p.x, p.y);
      } else {
        ctx.lineTo(p.x, p.y);
      }
    });
    ctx.stroke();
    points.forEach((point, index) => {
      if (index !== 0 && index !== points.length - 1) {
        return;
      }
      const p = worldToCanvas(Number(point[0]), Number(point[1]), bounds, width, height);
      ctx.fillStyle = index === 0 ? "#53c2a4" : "#ff7f50";
      ctx.beginPath();
      ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.restore();
  }

  function drawLocalPath(snapshot, bounds, width, height) {
    if (!layerLocalPath.checked || !snapshot.nav_local_path || !snapshot.nav_local_path.available ||
      !snapshot.nav_local_path.points || snapshot.nav_local_path.points.length === 0) {
      return;
    }
    const points = snapshot.nav_local_path.points;
    const canvasPoints = points
      .map((point) => worldToCanvas(Number(point[0]), Number(point[1]), bounds, width, height))
      .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
    if (canvasPoints.length === 0) {
      return;
    }
    ctx.save();
    ctx.lineJoin = "round";
    ctx.lineCap = "round";

    ctx.beginPath();
    canvasPoints.forEach((p, index) => {
      if (index === 0) {
        ctx.moveTo(p.x, p.y);
      } else {
        ctx.lineTo(p.x, p.y);
      }
    });
    ctx.strokeStyle = "rgba(1, 11, 18, 0.88)";
    ctx.lineWidth = 8;
    ctx.setLineDash([10, 6]);
    ctx.stroke();

    ctx.beginPath();
    canvasPoints.forEach((p, index) => {
      if (index === 0) {
        ctx.moveTo(p.x, p.y);
      } else {
        ctx.lineTo(p.x, p.y);
      }
    });
    ctx.strokeStyle = "#19d8ff";
    ctx.lineWidth = 4;
    ctx.setLineDash([10, 6]);
    ctx.stroke();

    const start = canvasPoints[0];
    const end = canvasPoints[canvasPoints.length - 1];
    let screenLength = 0;
    for (let i = 1; i < canvasPoints.length; i += 1) {
      screenLength += Math.hypot(
        canvasPoints[i].x - canvasPoints[i - 1].x,
        canvasPoints[i].y - canvasPoints[i - 1].y,
      );
    }
    let arrowStart = start;
    let arrowEnd = end;
    if (screenLength < 34 && canvasPoints.length >= 2) {
      const previous = canvasPoints[Math.max(0, canvasPoints.length - 2)];
      const dx = end.x - previous.x;
      const dy = end.y - previous.y;
      const length = Math.hypot(dx, dy) || 1;
      arrowStart = start;
      arrowEnd = {
        x: start.x + (dx / length) * 38,
        y: start.y + (dy / length) * 38,
      };
      ctx.setLineDash([]);
      ctx.strokeStyle = "rgba(25, 216, 255, 0.92)";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(arrowStart.x, arrowStart.y);
      ctx.lineTo(arrowEnd.x, arrowEnd.y);
      ctx.stroke();
    }

    const angle = Math.atan2(arrowEnd.y - arrowStart.y, arrowEnd.x - arrowStart.x);
    ctx.setLineDash([]);
    ctx.fillStyle = "#19d8ff";
    ctx.strokeStyle = "rgba(1, 11, 18, 0.9)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(arrowEnd.x, arrowEnd.y);
    ctx.lineTo(arrowEnd.x - Math.cos(angle - 0.55) * 13, arrowEnd.y - Math.sin(angle - 0.55) * 13);
    ctx.lineTo(arrowEnd.x - Math.cos(angle + 0.55) * 13, arrowEnd.y - Math.sin(angle + 0.55) * 13);
    ctx.closePath();
    ctx.stroke();
    ctx.fill();

    ctx.fillStyle = "#06121a";
    ctx.strokeStyle = "#19d8ff";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(start.x, start.y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  function drawSelectedInitialPose(bounds, width, height) {
    if (!selectedInitialPose || !pickingInitialPose) {
      return;
    }
    const p = worldToCanvas(selectedInitialPose.x, selectedInitialPose.y, bounds, width, height);
    const yaw = pendingInitialPoseDirection && initialDirectionPreview
      ? Math.atan2(
        initialDirectionPreview.y - selectedInitialPose.y,
        initialDirectionPreview.x - selectedInitialPose.x,
      )
      : Number(initialYaw.value || 0);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-yaw);
    ctx.strokeStyle = "#ff6f91";
    ctx.fillStyle = "rgba(255, 111, 145, 0.25)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 13, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(24, 0);
    ctx.lineTo(5, -7);
    ctx.lineTo(5, 7);
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  function drawSelectedGoal(bounds, width, height) {
    if (!selectedGoal) {
      return;
    }
    const p = worldToCanvas(selectedGoal.x, selectedGoal.y, bounds, width, height);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.strokeStyle = "#ff7f50";
    ctx.fillStyle = "rgba(255, 127, 80, 0.22)";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(0, 0, 14, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(-18, 0);
    ctx.lineTo(18, 0);
    ctx.moveTo(0, -18);
    ctx.lineTo(0, 18);
    ctx.stroke();
    ctx.restore();
  }

  function drawScene() {
    syncMapLayerControls();
    updateCloud3d();
    if (is3dViewActive()) {
      return;
    }

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#121819";
    ctx.fillRect(0, 0, width, height);

    const hasOfflineMap = Boolean(selectedSavedMap && selectedSavedMapImage);
    const hasMapData = snapshotHasMapData(mapSnapshot) || hasOfflineMap;
    const bounds = hasMapData
      ? (mapViewBounds || fullMapBounds())
      : { minX: -5, maxX: 5, minY: -5, maxY: 5 };
    drawWorldGrid(width, height, bounds);

    if (hasMapData) {
      drawSavedMap(bounds, width, height);
      if (hasOfflineMap) {
        if (layerStaticCloud.checked) {
          drawCloud(selectedSavedMap.pcd_cloud, bounds, width, height, "rgba(85, 171, 232, 0.72)", 1.8);
        }
      } else {
        drawOccupancyGrid(mapSnapshot, bounds, width, height);
        drawCostmapGrid(mapSnapshot, bounds, width, height);
        if (layerStaticCloud.checked) {
          drawCloud(mapSnapshot.static_cloud, bounds, width, height, "rgba(85, 171, 232, 0.58)", 2);
        }
      }
      if (layerLiveCloud.checked) {
        drawCloud(mapSnapshot.live_cloud, bounds, width, height, "rgba(242, 195, 91, 0.86)", 3);
      }
      drawNavPath(mapSnapshot, bounds, width, height);
      drawLocalPath(mapSnapshot, bounds, width, height);
      drawRobotPose(mapSnapshot, bounds, width, height);
      drawSelectedInitialPose(bounds, width, height);
      drawSelectedGoal(bounds, width, height);
    } else {
      const cx = width * 0.5;
      const cy = height * 0.5;
      ctx.strokeStyle = connected ? "#53c2a4" : "#6e7a77";
      ctx.fillStyle = connected ? "#53c2a4" : "#6e7a77";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(cx, cy, 24, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(cx + 34, cy);
      ctx.lineTo(cx + 3, cy - 9);
      ctx.lineTo(cx + 3, cy + 9);
      ctx.closePath();
      ctx.fill();
    }

    const vx = latestCmd.linear_x;
    const vy = latestCmd.linear_y;
    const wz = latestCmd.angular_z;
    const cx = width * 0.5;
    const cy = height * 0.5;
    ctx.strokeStyle = "#f2c35b";
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx - vy * 220, cy - vx * 220);
    ctx.stroke();

    ctx.fillStyle = "#cbd6d2";
    ctx.font = "14px system-ui";
    ctx.fillText(`vx ${vx.toFixed(2)}  vy ${vy.toFixed(2)}  wz ${wz.toFixed(2)}`, 18, 28);
    const navState = mapSnapshot ? parseStatusState(mapSnapshot.nav_status) : "";
    ctx.fillText(lastMapSnapshotOk ? `Map API: /api/map/snapshot${navState ? ` / nav ${navState}` : ""}` : "Map API: waiting", 18, 52);
    updateZoomLabel();
  }

  if (manualJoystickPad) {
    manualJoystickPad.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      stopManualStopBurst();
      manualJoystickPointerId = event.pointerId;
      try {
        manualJoystickPad.setPointerCapture(event.pointerId);
      } catch (error) {
      }
      setManualJoystickFromPointer(event);
    });
    manualJoystickPad.addEventListener("pointermove", (event) => {
      if (manualJoystickPointerId !== event.pointerId) {
        return;
      }
      event.preventDefault();
      setManualJoystickFromPointer(event);
    });
    const endJoystickPointer = (event) => {
      if (manualJoystickPointerId !== event.pointerId) {
        return;
      }
      if (manualJoystickPad.hasPointerCapture(event.pointerId)) {
        manualJoystickPad.releasePointerCapture(event.pointerId);
      }
      resetManualJoystick();
    };
    manualJoystickPad.addEventListener("pointerup", endJoystickPointer);
    manualJoystickPad.addEventListener("pointercancel", endJoystickPointer);
    manualJoystickPad.addEventListener("lostpointercapture", (event) => {
      if (manualJoystickPointerId === event.pointerId) {
        resetManualJoystick();
      }
    });
    window.addEventListener("pointerup", endJoystickPointer);
    window.addEventListener("pointercancel", endJoystickPointer);
    window.addEventListener("blur", () => {
      if (manualJoystickPointerId !== null || manualVelocityIsActive()) {
        resetManualControls();
      }
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden && (manualJoystickPointerId !== null || manualVelocityIsActive())) {
        resetManualControls();
      }
    });
  }
  if (manualWzSlider) {
    manualWzSlider.addEventListener("input", startManualSliderPublish);
    manualWzSlider.addEventListener("change", releaseManualRotationControl);
    manualWzSlider.addEventListener("pointerdown", startManualSliderPublish);
    manualWzSlider.addEventListener("pointerup", releaseManualRotationControl);
    manualWzSlider.addEventListener("pointercancel", releaseManualRotationControl);
  }
  if (zeroBtn) {
    zeroBtn.addEventListener("click", resetManualControls);
  }
  window.addEventListener("blur", resetManualControls);

  connectBtn.addEventListener("click", pollApiStatus);
  disconnectBtn.addEventListener("click", disconnect);
  startMappingBtn.addEventListener(
    "click",
    () => requestMapping(
      "/api/mapping/start",
      "正在重启FAST-LIO并开启建图传输...",
      { isStartMapping: true, clearCloud: true },
    ),
  );
  stopMappingBtn.addEventListener(
    "click",
    () => requestMapping(
      "/api/mapping/stop",
      "正在停止网页建图点云传输...",
      { clearCloud: true },
    ),
  );
  saveMappingBtn.addEventListener(
    "click",
    () => requestMapping("/api/mapping/save", "正在保存当前FAST-LIO累计地图..."),
  );
  estopBtn.addEventListener("click", () => publishEstop(true));
  releaseEstopBtn.addEventListener("click", () => publishEstop(false));
  sendGoalBtn.addEventListener("click", publishGoal);
  navStartBtn.addEventListener("click", () => requestNavigationDrive("/api/nav/start", "导航已出发：DWB速度将转发到底盘控制链路"));
  navStopBtn.addEventListener("click", () => requestNavigationDrive("/api/nav/stop", "导航已停止，已发送零速度"));
  pickGoalBtn.addEventListener("click", () => {
    if (!snapshotHasMapData(mapSnapshot)) {
      pickingGoal = false;
      pickGoalBtn.classList.remove("active");
      setMapStatus("请先选择并加载地图后再点选导航目标。");
      return;
    }
    pickingGoal = !pickingGoal;
    if (pickingGoal) {
      pickingInitialPose = false;
      pendingInitialPoseDirection = false;
      initialDirectionPreview = null;
      selectedInitialPose = null;
      pickInitialPoseBtn.classList.remove("active");
    }
    pickGoalBtn.classList.toggle("active", pickingGoal);
    setMapStatus(pickingGoal ? "请在地图上点击导航目标点" : "已取消目标点选");
  });
  pickInitialPoseBtn.addEventListener("click", () => {
    if (!snapshotHasMapData(mapSnapshot)) {
      pickingInitialPose = false;
      pendingInitialPoseDirection = false;
      selectedInitialPose = null;
      initialDirectionPreview = null;
      pickInitialPoseBtn.classList.remove("active");
      setMapStatus("请先选择并加载地图后再设置初始位姿。");
      return;
    }
    pickingInitialPose = !pickingInitialPose;
    pendingInitialPoseDirection = false;
    initialDirectionPreview = null;
    selectedInitialPose = null;
    if (pickingInitialPose) {
      pickingGoal = false;
      pickGoalBtn.classList.remove("active");
    }
    pickInitialPoseBtn.classList.toggle("active", pickingInitialPose);
    setMapStatus(pickingInitialPose ? "请先点击机器人当前位置，再点击车头朝向" : "已取消地图点选");
  });
  sendInitialPoseBtn.addEventListener("click", publishInitialPose);
  showCameraBtn.addEventListener("click", startCameraView);
  hideCameraBtn.addEventListener("click", stopCameraView);
  applyExposureBtn.addEventListener("click", applyExposure);
  cameraImage.addEventListener("load", () => setCameraStatus("RGB 画面在线"));
  cameraImage.addEventListener("error", () => setCameraStatus("等待相机帧"));
  [layerGrid, layerCostmap, layerStaticCloud, layerLiveCloud, layerSavedMap, layerNavPath, layerLocalPath, layerPose].forEach((input) => {
    input.addEventListener("change", drawScene);
  });
  refreshMapsBtn.addEventListener("click", refreshSavedMaps);
  savedMapSelect.addEventListener("change", loadSelectedSavedMap);
  loadSavedMapBtn.addEventListener("click", loadSelectedSavedMap);
  convertSavedMapBtn.addEventListener("click", convertSelectedSavedMap);
  mappingHeightFilterEnabled.addEventListener("change", refreshMappingHeightFilter);
  mappingHeightMaxZ.addEventListener("input", refreshMappingHeightFilter);
  resetMapViewBtn.addEventListener("click", () => {
    mapViewBounds = fullMapBounds();
    mapViewUserAdjusted = false;
    cloud3dNeedsFrame = true;
    updateCloud3d();
    drawScene();
  });
  view2dBtn.addEventListener("click", () => {
    viewMode = "2d";
    syncMapLayerControls();
    drawScene();
  });
  view3dBtn.addEventListener("click", () => {
    viewMode = "3d";
    cloud3dNeedsFrame = true;
    syncMapLayerControls();
    resizeCloud3d();
    updateCloud3d();
  });
  zoomOutBtn.addEventListener("click", () => zoomMap(1.25));
  zoomInBtn.addEventListener("click", () => zoomMap(0.8));
  canvas.addEventListener("wheel", (event) => {
    if (!snapshotHasMapData(mapSnapshot)) {
      return;
    }
    event.preventDefault();
    const rect = canvas.getBoundingClientRect();
    zoomMap(event.deltaY > 0 ? 1.18 : 0.85, {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top,
    });
  }, { passive: false });
  canvas.addEventListener("pointerdown", (event) => {
    if (pickingInitialPose || pickingGoal || !snapshotHasMapData(mapSnapshot)) {
      return;
    }
    isPanningMap = true;
    mapDragged = false;
    panStart = {
      x: event.clientX,
      y: event.clientY,
      bounds: { ...currentMapBounds() },
    };
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (pendingInitialPoseDirection && selectedInitialPose && snapshotHasMapData(mapSnapshot)) {
      const rect = canvas.getBoundingClientRect();
      const bounds = mapViewBounds || fullMapBounds();
      initialDirectionPreview = canvasToWorld(
        event.clientX - rect.left,
        event.clientY - rect.top,
        bounds,
        canvas.clientWidth,
        canvas.clientHeight,
      );
      drawScene();
      return;
    }
    if (!isPanningMap || !panStart) {
      return;
    }
    const dx = event.clientX - panStart.x;
    const dy = event.clientY - panStart.y;
    if (Math.abs(dx) + Math.abs(dy) > 3) {
      mapDragged = true;
    }
    panMap(dx, dy, panStart.bounds);
  });
  canvas.addEventListener("pointerup", (event) => {
    if (isPanningMap) {
      isPanningMap = false;
      panStart = null;
      try {
        canvas.releasePointerCapture(event.pointerId);
      } catch (error) {
      }
    }
  });
  canvas.addEventListener("pointerleave", () => {
    isPanningMap = false;
    panStart = null;
  });
  canvas.addEventListener("click", (event) => {
    if (mapDragged) {
      mapDragged = false;
      return;
    }
    if ((!pickingInitialPose && !pickingGoal) || !snapshotHasMapData(mapSnapshot)) {
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const bounds = mapViewBounds || fullMapBounds();
    const point = canvasToWorld(
      event.clientX - rect.left,
      event.clientY - rect.top,
      bounds,
      canvas.clientWidth,
      canvas.clientHeight,
    );
    if (pickingGoal) {
      selectedGoal = point;
      goalX.value = point.x.toFixed(2);
      goalY.value = point.y.toFixed(2);
      pickingGoal = false;
      pickGoalBtn.classList.remove("active");
      setMapStatus(`已选择目标点 x=${point.x.toFixed(2)} y=${point.y.toFixed(2)}，正在请求规划`);
      publishGoal();
    } else {
      if (!pendingInitialPoseDirection) {
        selectedInitialPose = point;
        initialDirectionPreview = null;
        initialX.value = point.x.toFixed(2);
        initialY.value = point.y.toFixed(2);
        initialYaw.value = currentHeadingYaw().toFixed(2);
        pendingInitialPoseDirection = true;
        setMapStatus(`已选择位置 x=${point.x.toFixed(2)} y=${point.y.toFixed(2)}，请点击车头朝向`);
      } else {
        const dx = point.x - selectedInitialPose.x;
        const dy = point.y - selectedInitialPose.y;
        if (Math.hypot(dx, dy) < 0.05) {
          setMapStatus("方向点太近，请在机器人前方再点一次");
          return;
        }
        initialDirectionPreview = point;
        initialYaw.value = Math.atan2(dy, dx).toFixed(2);
        pickingInitialPose = false;
        pendingInitialPoseDirection = false;
        pickInitialPoseBtn.classList.remove("active");
        setMapStatus(`已选择初始位姿 x=${initialX.value} y=${initialY.value} yaw=${initialYaw.value}，正在发送`);
        publishInitialPose();
      }
    }
    drawScene();
  });
  window.addEventListener("resize", resizeCanvas);

  wsInput.value = defaultApiBaseUrl();
  resizeCanvas();
  setInterval(pollApiStatus, 1000);
  setInterval(pollMapSnapshot, 1200);
  setInterval(pollMappingStatus, 1500);
  pollApiStatus();
  pollMapSnapshot();
  pollMappingStatus();
  refreshSavedMaps();
  renderCloud3d();
  renderMappingPreview3d();
})();
