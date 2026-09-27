import Foundation
import AVFoundation
import ARKit

/// Runs on the camera/AR queue:
/// detect markers -> calibrate -> change detection -> perceive -> send.
final class VisionPipeline {
    private let detector = ArucoDetectorBridge()
    private let changeDetector = ChangeDetectorBridge()
    let perception: Perception
    let sender = ObservationSender()

    private var frameTimes: [Double] = []
    private var lastDebugPush = 0.0
    private var lastCar: (pose: Pose, t: Double)?
    private var latestTime = 0.0

    /// The car's latest pose *from its marker* and its age, if within 0.5 s
    /// (processing queue only). Depth-tracked poses are deliberately excluded:
    /// the tracker must only be re-anchored by real marker readings.
    var lastMarkerCarPose: (pose: Pose, age: Double)? {
        guard let c = lastCar, latestTime - c.t < 0.5 else { return nil }
        return (c.pose, latestTime - c.t)
    }
    private var captureCountdown = -1
    private var backgroundNote: String?

    // Requests from the main thread
    private let requestLock = NSLock()
    private var captureRequested = false
    private var clearRequested = false

    /// Throttled to ~10 Hz for the UI. Called on the processing queue.
    var onDebug: ((PhoneDebugState) -> Void)?

    init(settings: Perception.Settings) {
        perception = Perception(settings: settings)
    }

    func requestBackgroundCapture() {
        requestLock.lock(); captureRequested = true; requestLock.unlock()
    }

    func requestClearBackground() {
        requestLock.lock(); clearRequested = true; requestLock.unlock()
    }

    /// - Parameters:
    ///   - arMode: calibration comes only from `projection` (never from visible corner markers).
    ///   - projection: AR-pinned corners projected into this frame, if available.
    ///   - depthOccupancy: LiDAR obstacle map; used instead of background subtraction when present.
    ///   - allowCameraBackground: false in LiDAR mode. Background subtraction assumes a
    ///     still phone, so it must never kick in for a handheld LiDAR phone.
    /// - Returns: the markers detected in this frame.
    @discardableResult
    func handle(pixelBuffer: CVPixelBuffer, time t: Double,
                arMode: Bool, projection: ARProjection?,
                depthOccupancy: Data? = nil,
                clearance: ClearanceInfo? = nil,
                depthCar: Pose? = nil,
                depthMarkerAge: Double? = nil,
                depthTrackQuality: Double? = nil,
                allowCameraBackground: Bool = true) -> [ArucoMarker] {
        let processingStarted = Date().timeIntervalSince1970
        latestTime = t
        frameTimes.append(t)
        frameTimes.removeAll { t - $0 > 1.0 }

        let markers = detector.detect(pixelBuffer: pixelBuffer)
        if arMode {
            if let p = projection {
                perception.setProjectedCorners(floor: p.floor, car: p.car, goal: p.goal)
            } else {
                perception.pollReset()   // keep the last mapping during brief tracking hiccups
            }
        } else {
            perception.updateCalibration(markers: markers)
        }

        handleBackgroundRequests(pixelBuffer)

        var raw: Data?
        var source: String?
        if let depthOccupancy {
            raw = depthOccupancy
            source = "LiDAR"
        } else if allowCameraBackground && changeDetector.hasBackground {
            source = "camera"
            let s = perception.settings
            // In AR mode, re-warp with the current mapping so small phone movements don't break it.
            let m = arMode ? perception.topDownMatrix()?.map { NSNumber(value: $0) } : nil
            raw = changeDetector.occupancy(pixelBuffer: pixelBuffer,
                                           threshold: Int32(s.changeThreshold),
                                           minChangedFraction: s.minChangedFraction,
                                           matrix: m)
        }

        var (msg, frameDebug) = perception.process(markers: markers, rawOccupancy: raw,
                                                   clearance: clearance, depthCar: depthCar,
                                                   maskCameraChanges: depthOccupancy == nil,
                                                   time: t, fps: Double(frameTimes.count))
        msg.arena = perception.settings.arena
        msg.carSource = frameDebug.carSource
        if frameDebug.carSource == "LiDAR" {
            // Python trusts a depth-tracked pose only with a recent marker and a good fit.
            msg.markerAge = depthMarkerAge
            msg.trackQuality = depthTrackQuality
        }
        msg.sentAt = processingStarted
        if arMode && projection == nil {
            msg.calibrated = false
            msg.car = nil
            msg.grid = nil
            msg.veto = true
            msg.vetoReason = "AR tracking unavailable"
            frameDebug.calibrated = false
        }
        sender.send(msg)
        if let car = msg.car, frameDebug.carSource == "marker" { lastCar = (car, t) }

        if t - lastDebugPush > 0.1 {
            lastDebugPush = t
            var dbg = frameDebug
            dbg.hasBackground = changeDetector.hasBackground
            dbg.obstacleSource = source
            dbg.backgroundNote = backgroundNote
            onDebug?(dbg)
        }
        return markers
    }

    private func handleBackgroundRequests(_ pixelBuffer: CVPixelBuffer) {
        requestLock.lock()
        let wantClear = clearRequested
        let wantCapture = captureRequested
        clearRequested = false
        captureRequested = false
        requestLock.unlock()

        if wantClear {
            changeDetector.clearBackground()
            captureCountdown = -1
            backgroundNote = nil
        }
        if wantCapture {
            // Wait ~10 frames so the just-locked exposure settles first.
            captureCountdown = 10
            backgroundNote = "Capturing…"
        }

        if captureCountdown > 0 {
            captureCountdown -= 1
        } else if captureCountdown == 0 {
            captureCountdown = -1
            guard let m = perception.topDownMatrix() else {
                backgroundNote = "Calibrate first"
                return
            }
            let (cols, rows) = GridSpec.dims(perception.settings.arena)
            let ok = changeDetector.captureBackground(pixelBuffer: pixelBuffer,
                                                      matrix: m.map { NSNumber(value: $0) },
                                                      cols: Int32(cols), rows: Int32(rows),
                                                      cellPixels: Int32(GridSpec.pixelsPerCell))
            backgroundNote = ok ? nil : "Background capture failed"
        }
    }
}

/// UI-facing state. Published properties are only mutated on the main thread.
///
/// Two camera modes (chosen at launch, "useAR" setting):
/// - AR: ARKit world tracking. Corners are pinned by walking up to them and stay
///   put when off-screen. No zoom (ARKit controls the camera).
/// - Classic: AVFoundation with zoom; all four corners must be visible to calibrate.
final class PhoneModel: ObservableObject {
    @Published var navigationStatus = "Navigation offline"
    @Published var navigationOnline = false
    @Published var navigationState = "waiting"
    @Published var navigationPath: [Vec2] = []
    @Published var navigationIssue: String?
    private var navigationSeen = Date.distantPast
    private var navigationTimer: Timer?

    /// Explain the same conditions used by the button, rather than silently disabling it.
    var navigationStartBlocker: String? {
        if !navigationOnline { return "Waiting for a reply from Python" }
        if navigationState == "blocked" { return navigationStatus }
        if !debug.calibrated { return "Waiting for valid arena calibration / AR tracking" }
        // Brief gaps (a missed frame, LiDAR tracking) don't disable Start: Python checks
        // the pose it actually plans from and replies with the reason if it can't use it.
        if (debug.carMissingFor ?? 0) > 1 { return "Waiting for car marker ID 0" }
        if debug.goal == nil { return "Waiting for the pinned goal coordinates" }
        return nil
    }

    func startNavigation() {
        navigationIssue = nil
        if let blocker = navigationStartBlocker {
            navigationIssue = blocker
            return
        }
        let arena = pipeline.perception.settings.arena
        if arMode, let w = arStatus.measuredWidth, let h = arStatus.measuredHeight,
           abs(w - arena.width) > max(5, w * 0.1) || abs(h - arena.height) > max(5, h * 0.1) {
            stopNavigation()
            navigationIssue = "Use the measured arena dimensions first"
            return
        }
        pipeline.sender.requestNavigation("start")
    }
    func stopNavigation() {
        navigationIssue = nil
        pipeline.sender.requestNavigation("stop")
    }
    @Published var debug = PhoneDebugState()
    @Published var connectionStatus = "Not started"
    /// Laptop clock minus phone clock, and the round trip it was measured over (s).
    @Published var clockSync: (offset: Double, roundTrip: Double)?
    @Published var cameraError: String?
    @Published var arStatus = ARCalibrationStatus()
    /// Classic mode zoom, in device units (see ZoomInfo).
    @Published var zoomFactor: CGFloat = 1
    @Published var zoomInfo = ZoomInfo()

    let arMode: Bool
    /// LiDAR obstacle detection is possible (AR mode on a Pro iPhone).
    let lidarAvailable: Bool
    let camera: CameraManager?
    let ar: ARCameraManager?
    let pipeline: VisionPipeline

    private var started = false
    private var settledZoom: CGFloat = 1
    private var initialZoom: CGFloat?

    init() {
        let wantsAR = UserDefaults.standard.object(forKey: "useAR") as? Bool ?? true
        arMode = wantsAR && ARWorldTrackingConfiguration.isSupported
        lidarAvailable = arMode && ARCameraManager.lidarAvailable

        pipeline = VisionPipeline(settings: .init(cameraId: "cam", arena: ArenaConfig()))
        let p = pipeline

        if arMode {
            let a = ARCameraManager()
            a.onFrame = { [weak a] pb, t, projection, depth in
                let lidarMode = ARCameraManager.lidarAvailable && (a?.useDepth ?? false)
                return p.handle(pixelBuffer: pb, time: t, arMode: true, projection: projection,
                                depthOccupancy: depth.occupancy, clearance: depth.clearance,
                                depthCar: depth.depthTrackedCar,
                                depthMarkerAge: depth.depthMarkerAge,
                                depthTrackQuality: depth.depthTrackQuality,
                                allowCameraBackground: !lidarMode)
            }
            a.carPose = { p.lastMarkerCarPose }
            ar = a
            camera = nil
        } else {
            let c = CameraManager()
            c.onFrame = { pb, t in
                _ = p.handle(pixelBuffer: pb, time: CMTimeGetSeconds(t), arMode: false, projection: nil)
            }
            camera = c
            ar = nil
        }

        pipeline.onDebug = { [weak self] d in
            DispatchQueue.main.async { self?.debug = d }
        }
        pipeline.sender.onClock = { [weak self] offset, roundTrip in
            DispatchQueue.main.async {
                guard let self else { return }
                // Republish only on a visible change (replies arrive many times a second).
                if let c = self.clockSync, abs(c.offset - offset) < 0.001, abs(c.roundTrip - roundTrip) < 0.001 { return }
                self.clockSync = (offset, roundTrip)
            }
        }
        pipeline.sender.onStatus = { [weak self] s in
            DispatchQueue.main.async { self?.connectionStatus = s }
        }
        pipeline.sender.onNavigation = { [weak self] status in
            DispatchQueue.main.async {
                guard let self else { return }
                self.navigationSeen = Date()
                self.navigationOnline = true
                self.navigationStatus = status.message
                self.navigationState = status.state
                self.navigationPath = status.path
            }
        }
        navigationTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            guard let self, self.navigationOnline,
                  Date().timeIntervalSince(self.navigationSeen) > 0.7 else { return }
            self.navigationOnline = false
            self.navigationStatus = "Navigation offline"
            self.navigationPath = []
            self.stopNavigation()
        }
        ar?.onStatus = { [weak self] s in
            DispatchQueue.main.async { if self?.arStatus != s { self?.arStatus = s } }
        }
    }

    func start(manualHost: String, zoom: CGFloat?) {
        guard !started else { return }
        started = true
        initialZoom = zoom

        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            startCamera()
        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { granted in
                DispatchQueue.main.async {
                    if granted {
                        self.startCamera()
                    } else {
                        self.cameraError = "Camera access denied."
                    }
                }
            }
        default:
            cameraError = "Camera access denied. Enable it in Settings."
        }
        pipeline.sender.restart(manualHost: manualHost)
    }

    func apply(settings: Perception.Settings, manualHost: String, carMarkerHeightCm: Double,
               useLidar: Bool = true) {
        stopNavigation()
        pipeline.perception.settings = settings
        ar?.carMarkerHeight = Float(carMarkerHeightCm / 100)
        ar?.setMarkerToCenter(settings.markerToCenter)
        ar?.arena = settings.arena
        ar?.useDepth = useLidar
        if started { pipeline.sender.restart(manualHost: manualHost) }
    }

    /// Arena settings changed: rebuild the mapping and drop the background,
    /// but keep AR-pinned corners (they're physical points, not settings).
    func settingsChanged() {
        stopNavigation()
        setCameraLocked(false)
        pipeline.requestClearBackground()
        pipeline.perception.requestRecalibration()
    }

    /// Full reset: clears calibration (and AR-pinned corners) and the background.
    func recalibrate() {
        ar?.resetCorners()
        settingsChanged()
    }

    /// Call with the arena EMPTY (car parked outside). Locks the camera first.
    func captureBackground() {
        stopNavigation()
        setCameraLocked(true)
        pipeline.requestBackgroundCapture()
    }

    /// LiDAR mode: measure the car's outline (keep it clear; slowly circle it).
    func learnCarShape() {
        stopNavigation()
        ar?.startLearningCarShape()
    }

    /// LiDAR mode: forget all mapped obstacles (e.g. after rearranging the arena).
    func clearObstacles() {
        stopNavigation()
        ar?.clearObstacleMap()
    }

    /// AR mode: when true, the next tap on the preview places the goal.
    /// Opt-in so an accidental touch on a mounted phone can't move the goal.
    @Published var placingGoal = false

    /// AR mode: pin (or move) the goal to the tapped floor spot.
    func placeGoal(atViewPoint point: CGPoint) {
        guard placingGoal, let ar else { return }
        stopNavigation()
        if ar.placeGoal(atViewPoint: point) { placingGoal = false }
    }

    /// AR mode: forget the pinned goal. Rescan the goal marker or tap to place a new one.
    func resetGoal() {
        stopNavigation()
        placingGoal = false
        ar?.resetGoal()
    }

    // MARK: - Classic-mode zoom

    /// Live zoom while pinching. Call zoomDidSettle() when the gesture ends.
    func setZoom(_ factor: CGFloat) {
        guard let camera else { return }
        let z = clamp(factor, zoomInfo.min, zoomInfo.max)
        zoomFactor = z
        camera.setZoom(z)
    }

    /// Zooming changes how pixels map to the floor, so the calibration and the
    /// obstacle background are no longer valid. Corners must stay in view.
    func zoomDidSettle() {
        guard camera != nil, abs(zoomFactor - settledZoom) > 0.01 else { return }
        settledZoom = zoomFactor
        recalibrate()
    }

    // MARK: - Private

    private func setCameraLocked(_ locked: Bool) {
        camera?.setLocked(locked)
        ar?.setLocked(locked)
    }

    private func startCamera() {
        if let ar {
            ar.start()
            return
        }
        guard let camera else { return }
        do {
            try camera.configure(initialZoom: initialZoom)
            zoomInfo = camera.zoomInfo
            zoomFactor = clamp(initialZoom ?? zoomInfo.oneX, zoomInfo.min, zoomInfo.max)
            settledZoom = zoomFactor
            camera.start()
        } catch {
            cameraError = error.localizedDescription
        }
    }
}
