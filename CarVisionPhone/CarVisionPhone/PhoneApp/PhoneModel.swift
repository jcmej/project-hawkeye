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
    /// - Returns: the markers detected in this frame.
    @discardableResult
    func handle(pixelBuffer: CVPixelBuffer, time t: Double,
                arMode: Bool, projection: ARProjection?) -> [ArucoMarker] {
        frameTimes.append(t)
        frameTimes.removeAll { t - $0 > 1.0 }

        let markers = detector.detect(pixelBuffer: pixelBuffer)
        if arMode {
            if let p = projection {
                perception.setProjectedCorners(floor: p.floor, car: p.car)
            } else {
                perception.pollReset()   // keep the last mapping during brief tracking hiccups
            }
        } else {
            perception.updateCalibration(markers: markers)
        }

        handleBackgroundRequests(pixelBuffer)

        var raw: Data?
        if changeDetector.hasBackground {
            let s = perception.settings
            // In AR mode, re-warp with the current mapping so small phone movements don't break it.
            let m = arMode ? perception.topDownMatrix()?.map { NSNumber(value: $0) } : nil
            raw = changeDetector.occupancy(pixelBuffer: pixelBuffer,
                                           threshold: Int32(s.changeThreshold),
                                           minChangedFraction: s.minChangedFraction,
                                           matrix: m)
        }

        let (msg, frameDebug) = perception.process(markers: markers, rawOccupancy: raw,
                                                   time: t, fps: Double(frameTimes.count))
        sender.send(msg)

        if t - lastDebugPush > 0.1 {
            lastDebugPush = t
            var dbg = frameDebug
            dbg.hasBackground = changeDetector.hasBackground
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
    @Published var debug = PhoneDebugState()
    @Published var connectionStatus = "Not started"
    @Published var cameraError: String?
    @Published var arStatus = ARCalibrationStatus()
    /// Classic mode zoom, in device units (see ZoomInfo).
    @Published var zoomFactor: CGFloat = 1
    @Published var zoomInfo = ZoomInfo()

    let arMode: Bool
    let camera: CameraManager?
    let ar: ARCameraManager?
    let pipeline: VisionPipeline

    private var started = false
    private var settledZoom: CGFloat = 1
    private var initialZoom: CGFloat?

    init() {
        let wantsAR = UserDefaults.standard.object(forKey: "useAR") as? Bool ?? true
        arMode = wantsAR && ARWorldTrackingConfiguration.isSupported

        pipeline = VisionPipeline(settings: .init(cameraId: "cam", arena: ArenaConfig()))
        let p = pipeline

        if arMode {
            let a = ARCameraManager()
            a.onFrame = { pb, t, projection in
                p.handle(pixelBuffer: pb, time: t, arMode: true, projection: projection)
            }
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
        pipeline.sender.onStatus = { [weak self] s in
            DispatchQueue.main.async { self?.connectionStatus = s }
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

    func apply(settings: Perception.Settings, manualHost: String, carMarkerHeightCm: Double) {
        pipeline.perception.settings = settings
        ar?.carMarkerHeight = Float(carMarkerHeightCm / 100)
        if started { pipeline.sender.restart(manualHost: manualHost) }
    }

    /// Arena settings changed: rebuild the mapping and drop the background,
    /// but keep AR-pinned corners (they're physical points, not settings).
    func settingsChanged() {
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
        setCameraLocked(true)
        pipeline.requestBackgroundCapture()
    }

    /// AR mode: tap the floor to pin the next missing corner.
    func placeCorner(atViewPoint point: CGPoint) {
        ar?.placeNextCorner(atViewPoint: point)
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
