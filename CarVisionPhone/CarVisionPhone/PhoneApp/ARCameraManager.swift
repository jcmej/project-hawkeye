import ARKit
import SceneKit
import UIKit

/// Per-frame result of AR calibration, in captured-image pixel coordinates.
struct ARProjection {
    /// The four pinned corners (IDs 2,3,4,5 in order) projected into this frame, on the floor.
    var floor: [CGPoint]
    /// The same corners lifted to the car marker's height.
    var car: [CGPoint]
    /// The pinned goal projected into this frame, if a goal is pinned.
    var goal: CGPoint?
}

/// The car's footprint relative to its marker, in cm. Car frame: +forward = the
/// marker's "top" direction, +left = 90° counterclockwise from that.
struct CarProfile: Codable, Equatable {
    var minForward: Double, maxForward: Double
    var minLeft: Double, maxLeft: Double
    var height: Double

    var length: Double { maxForward - minForward }
    var width: Double { maxLeft - minLeft }

    /// Distance (cm) from a car-frame point to the outline; 0 if inside.
    func distance(to q: Vec2) -> Double {
        let dx = max(minForward - q.x, 0, q.x - maxForward)
        let dy = max(minLeft - q.y, 0, q.y - maxLeft)
        return (dx * dx + dy * dy).squareRoot()
    }

    /// Outline corners in arena coordinates for a given car pose.
    func corners(at pose: Pose) -> [Vec2] {
        [Vec2(maxForward, maxLeft), Vec2(maxForward, minLeft), Vec2(minForward, minLeft), Vec2(minForward, maxLeft)]
            .map { pose.position + $0.rotated(by: pose.heading) }
    }
}

/// Everything the LiDAR produces for one frame.
struct DepthResult {
    /// Obstacle map bytes (see ChangeDetectorBridge format), nil if not mapping.
    var occupancy: Data?
    var clearance: ClearanceInfo?
    /// Car pose found by matching the learned outline to depth points, when the
    /// marker wasn't readable this frame (nil if the marker was used or tracking failed).
    var depthTrackedCar: Pose?
    /// With `depthTrackedCar`: seconds since a marker reading anchored the tracker.
    var depthMarkerAge: Double?
    /// With `depthTrackedCar`: the fit relative to the car's usual one, 0...1
    /// (the weaker of the point count and outline coverage ratios).
    var depthTrackQuality: Double?
}

/// Nearest obstacle to the car's outline, from one frame of LiDAR.
struct ClearanceInfo {
    /// cm from the outline to the nearest obstacle; nil = nothing within 1 m.
    var distance: Double?
    var point: Vec2?
    /// Angle in the car's frame (0 = ahead, positive = left), degrees.
    var bearingDegrees: Double?
    /// Obstacle points within 60 cm and their distance from the outline (for time-to-collision).
    var nearby: [(point: Vec2, distance: Double)]
}

struct ARCalibrationStatus: Equatable {
    var tracking = "Starting…"
    var trackingNormal = false
    var floorFound = false
    /// Corner IDs pinned so far.
    var captured: [Int] = []
    /// Samples collected for corners currently being pinned (0...needed).
    var progress: [Int: Int] = [:]
    /// True once the goal is pinned (by scanning its marker or tapping).
    var goalPinned = false
    /// Measured by ARKit, in cm: corner 2→3 and 2→5.
    var measuredWidth: Double?
    var measuredHeight: Double?
    /// LiDAR floor check: tilt of the fitted floor plane, and how much higher the far
    /// floor reads than the near floor under a flat-floor assumption (the "wedge" test).
    var floorTiltDegrees: Double?
    var floorFarMinusNearCm: Double?
    var depthWarning: String?
    /// Learned car outline (nil until learned).
    var carProfile: CarProfile?
    /// Depth tracking lost the car; it resumes only after the car's marker is seen again.
    var carLost = false
    /// Frames collected while learning the car shape (nil when not learning).
    var carLearnProgress: Int?
    var carLearnMessage: String?

    var complete: Bool { captured.count == 4 }
}

private struct FrameGeometry {
    let fx: Float, fy: Float, cx: Float, cy: Float
    let cameraToWorld: simd_float4x4
    let worldToCamera: simd_float4x4

    init(_ camera: ARCamera) {
        let k = camera.intrinsics            // column-major: [2][0] = cx, [2][1] = cy
        fx = k[0][0]; fy = k[1][1]; cx = k[2][0]; cy = k[2][1]
        cameraToWorld = camera.transform
        worldToCamera = simd_inverse(camera.transform)
    }

    /// World point -> captured-image pixel. ARKit camera space: x right, y up, looking down -z.
    /// Points behind the camera still give a valid projective point for the homography.
    func project(_ p: simd_float3) -> CGPoint? {
        let c = worldToCamera * simd_float4(p, 1)
        guard abs(c.z) > 1e-4 else { return nil }
        return CGPoint(x: CGFloat(cx + fx * c.x / -c.z),
                       y: CGFloat(cy + fy * c.y / c.z))
    }

    /// True if a world point is in front of the camera and inside the image.
    func isInView(_ p: simd_float3, width: CGFloat, height: CGFloat) -> Bool {
        let c = worldToCamera * simd_float4(p, 1)
        guard c.z < -0.05 else { return false }            // must be in front of the camera
        guard let px = project(p) else { return false }
        return px.x >= 0 && px.x <= width && px.y >= 0 && px.y <= height
    }

    /// Captured-image pixel -> point on the horizontal plane at height `planeY`.
    func floorPoint(pixel: CGPoint, planeY: Float) -> simd_float3? {
        let dirCam = simd_float4((Float(pixel.x) - cx) / fx, -(Float(pixel.y) - cy) / fy, -1, 0)
        let dir = simd_normalize(simd_make_float3(cameraToWorld * dirCam))
        let origin = simd_make_float3(cameraToWorld.columns.3)
        guard abs(dir.y) > 1e-4 else { return nil }
        let t = (planeY - origin.y) / dir.y
        guard t > 0 else { return nil }
        return origin + t * dir
    }
}

/// Runs an ARKit world-tracking session. Corners (and the goal) are pinned in 3D
/// by walking the phone up to each marker, after which they are projected into
/// every frame, even when off-screen. The goal can also be placed by tapping.
final class ARCameraManager: NSObject, ARSessionDelegate {
    let sceneView = ARSCNView(frame: .zero)
    private let queue = DispatchQueue(label: "ar.frames", qos: .userInteractive)

    /// Called on the AR queue (~30 Hz) with the camera image, the projection of
    /// pinned points, and (LiDAR phones) a depth-based occupancy grid. Returns the
    /// markers detected in the frame so corner/goal markers can be pinned.
    var onFrame: ((CVPixelBuffer, Double, ARProjection?, DepthResult) -> [ArucoMarker])?

    /// True on phones with a LiDAR sensor (iPhone 12 Pro and later Pro models).
    static var lidarAvailable: Bool {
        ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
    }
    /// Called on the AR queue a few times per second.
    var onStatus: ((ARCalibrationStatus) -> Void)?

    private let heightLock = NSLock()
    private var _carMarkerHeight: Float = 0.10
    /// Height of the car's marker above the floor, in meters.
    var carMarkerHeight: Float {
        get { heightLock.lock(); defer { heightLock.unlock() }; return _carMarkerHeight }
        set { heightLock.lock(); _carMarkerHeight = newValue; heightLock.unlock() }
    }

    private var _arena = ArenaConfig()
    private var _useDepth = true
    /// Arena settings, needed to build the depth occupancy grid.
    var arena: ArenaConfig {
        get { heightLock.lock(); defer { heightLock.unlock() }; return _arena }
        set { heightLock.lock(); _arena = newValue; heightLock.unlock() }
    }
    /// Use LiDAR depth for obstacle detection (when available).
    var useDepth: Bool {
        get { heightLock.lock(); defer { heightLock.unlock() }; return _useDepth }
        set { heightLock.lock(); _useDepth = newValue; heightLock.unlock() }
    }

    /// Returns the car's latest pose *from its marker* and how old it is (s). Called on
    /// the AR queue. Anchors the depth tracker and is required for shape learning.
    var carPose: (() -> (pose: Pose, age: Double)?)?

    // Depth tracking of the car (AR-queue state)
    private var trackedCar: Pose?
    private var trackedTime: TimeInterval = 0       // last time the pose was confirmed
    private var trackVelocity = Vec2.zero          // cm/s
    private var trackTurnRate = 0.0                // rad/s
    private var typicalInliers = 0.0               // points inside the outline when the marker is seen
    private var pendingDepthCar: Pose?
    private var pendingDepthMarkerAge: Double?
    private var pendingDepthQuality: Double?
    private let trackHold = 0.7                    // s to keep the last pose if fits fail (for masking)
    private var typicalCoverage = 0.0              // fraction of outline cells with points, marker-seen
    private var lastMarkerAnchor: TimeInterval = 0 // last time a marker re-anchored the tracker
    private var trackingLocked = false             // lost: wait for a marker before depth tracking again
    private let maxDepthOnly = 5.0                 // s of depth-only tracking between marker readings
    private let minInlierFraction = 0.6            // of the car's typical point count
    private let minCoverageFraction = 0.8          // of the car's typical outline coverage (2 cm cells)

    // Car shape + clearance (AR-queue state)
    private var carProfile: CarProfile?
    private var learning = false
    private var learnFrames = 0
    private var learnCounts: [Int] = []
    private var learnHeights: [Float] = []
    private var pendingClearance: ClearanceInfo?
    private let learnWindow = 40.0                 // cm half-size of the car-frame learning grid
    private let learnCell = 2.0                    // cm
    private let learnTargetFrames = 30
    private let carMargin = 3.0                    // cm around the outline treated as car
    private let clearanceMinHeight: Float = 0.04   // m; raised points that count as obstacles
    private let clearanceRange = 100.0             // cm; look this far from the car
    private static let profileKey = "carProfile"

    // Depth obstacle map tuning
    /// Try true if the phone will be mounted still (smoother, but smears when moving).
    static let useSmoothedDepth = false
    private let minObstacleHeight: Float = 0.05    // m above floor to count as an obstacle
    private let minRayDrop: Float = 0.2            // skip rays < ~12° below horizontal (grazing)
    private let floorBand: Float = 0.03            // m; |height| below this = floor
    private let maxObstacleHeight: Float = 1.2     // ignore things far above the arena
    private let maxHitRange: Float = 3.5           // m; obstacle evidence only from nearer points
    private let minHitPoints = 3                   // high-confidence raised points for a "hit"
    private let minFreePoints = 6                  // floor points (and no raised ones) for a "miss"
    private let maxTurnRate: Float = 1.2           // rad/s; skip mapping while turning faster
    private let maxMoveSpeed: Float = 1.0          // m/s; skip mapping while moving faster
    private let carClearPadding = 8.0              // cm around the car kept free

    // Persistent obstacle map (log-odds per 5 cm cell). Survives the phone looking away.
    private var mapLogOdds: [Float] = []
    private var mapObserved: [Bool] = []
    private var mapDims = (cols: 0, rows: 0)
    private let hitWeight: Float = 1.0, missWeight: Float = -0.5
    private let minLogOdds: Float = -2, maxLogOdds: Float = 3
    private let occupiedAt: Float = 1.5            // ~2 consistent hits to mark a cell
    private var floorOffset: Float = 0             // floor-height correction learned from depth
    private var lastPose: (t: TimeInterval, transform: simd_float4x4)?

    // AR-queue state
    private var floorY: Float?
    private var corners: [Int: simd_float3] = [:]
    private var goal: simd_float3?
    private var samples: [Int: [simd_float3]] = [:]
    private var lastProcessed: TimeInterval = 0
    private var lastStatusPush: TimeInterval = 0
    private var status = ARCalibrationStatus()

    // Main-thread state
    private var cornerNodes: [Int: [SCNNode]] = [:]
    private var goalNodes: [SCNNode] = []
    private var outlineNodes: [SCNNode] = []

    private let neededSamples = 12
    private let maxSpread: Float = 0.02          // m; samples must agree within 2 cm
    private let minMarkerSidePx: CGFloat = 35    // come close enough for a precise read

    override init() {
        super.init()
        if let data = UserDefaults.standard.data(forKey: Self.profileKey),
           let profile = try? JSONDecoder().decode(CarProfile.self, from: data) {
            carProfile = profile
            status.carProfile = profile
        }
    }

    /// Start learning the car's outline: keep the car clear and slowly circle it (main thread).
    func startLearningCarShape() {
        queue.async {
            self.learning = true
            self.learnFrames = 0
            let m = Int(2 * self.learnWindow / self.learnCell)
            self.learnCounts = [Int](repeating: 0, count: m * m)
            self.learnHeights = []
            self.trackedCar = nil
            self.trackingLocked = false
            self.typicalInliers = 0
            self.typicalCoverage = 0
            self.status.carLost = false
            self.status.carLearnProgress = 0
            self.status.carLearnMessage = nil
        }
    }

    func start() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal]
        // Use a 4:3 format so the camera image and the 4:3 depth map cover the same view.
        if let format = ARWorldTrackingConfiguration.supportedVideoFormats.first(where: {
            abs($0.imageResolution.width / $0.imageResolution.height - 4.0 / 3.0) < 0.01
                && $0.imageResolution.width <= 1920
        }) {
            config.videoFormat = format
        }
        if Self.lidarAvailable {
            // Raw depth by default: ARKit's smoothing blends frames, which smears depth
            // when the phone moves. The persistent map does our own temporal filtering.
            config.frameSemantics.insert(Self.useSmoothedDepth ? .smoothedSceneDepth : .sceneDepth)
        }
        sceneView.session.delegate = self
        sceneView.session.delegateQueue = queue
        sceneView.autoenablesDefaultLighting = true
        sceneView.session.run(config)
    }

    /// Forget all pinned corners (main thread).
    func resetCorners() {
        queue.async {
            self.corners = [:]
            self.samples = [:]
            self.trackedCar = nil
            self.clearMapState()
            self.floorOffset = 0
            self.updateMeasured()
        }
        for nodes in cornerNodes.values { nodes.forEach { $0.removeFromParentNode() } }
        cornerNodes = [:]
        outlineNodes.forEach { $0.removeFromParentNode() }
        outlineNodes = []
    }

    /// Forget the pinned goal (main thread). Scan the goal marker or tap to pin a new one.
    func resetGoal() {
        queue.async {
            self.goal = nil
            self.samples[MarkerIDs.goal] = nil
            self.status.progress[MarkerIDs.goal] = nil
            self.status.goalPinned = false
        }
        goalNodes.forEach { $0.removeFromParentNode() }
        goalNodes = []
    }

    /// Measure-app style: pin (or move) the goal to the tapped spot on the floor (main thread).
    /// Returns false if nothing on the floor was hit.
    @discardableResult
    func placeGoal(atViewPoint point: CGPoint) -> Bool {
        guard let query = sceneView.raycastQuery(from: point, allowing: .estimatedPlane, alignment: .horizontal),
              let hit = sceneView.session.raycast(query).first else { return false }
        var p = simd_make_float3(hit.worldTransform.columns.3)
        queue.async {
            if let fy = self.floorY { p.y = fy }
            self.pinGoal(at: p)
        }
        return true
    }

    /// Lock exposure, white balance, and focus (for background subtraction).
    func setLocked(_ locked: Bool) {
        guard let d = ARWorldTrackingConfiguration.configurableCaptureDeviceForPrimaryCamera else { return }
        do {
            try d.lockForConfiguration()
            if locked {
                if d.isExposureModeSupported(.locked) { d.exposureMode = .locked }
                if d.isWhiteBalanceModeSupported(.locked) { d.whiteBalanceMode = .locked }
                if d.isFocusModeSupported(.locked) { d.focusMode = .locked }
            } else {
                if d.isExposureModeSupported(.continuousAutoExposure) { d.exposureMode = .continuousAutoExposure }
                if d.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) { d.whiteBalanceMode = .continuousAutoWhiteBalance }
                if d.isFocusModeSupported(.continuousAutoFocus) { d.focusMode = .continuousAutoFocus }
            }
            d.unlockForConfiguration()
        } catch {
            print("Could not change AR camera lock: \(error)")
        }
    }

    // MARK: - ARSessionDelegate (called on `queue`)

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let t = frame.timestamp
        guard t - lastProcessed >= 1.0 / 30.0 else { return }   // ARKit runs at 60 fps
        lastProcessed = t

        updateTracking(frame.camera.trackingState)
        if corners.count < 4 { updateFloor(frame.anchors) }     // floor freezes once pinned

        let geo = FrameGeometry(frame.camera)
        var projection: ARProjection?
        if corners.count == 4 && status.trackingNormal {
            projection = makeProjection(geo)
        }

        pendingClearance = nil
        pendingDepthCar = nil
        pendingDepthMarkerAge = nil
        pendingDepthQuality = nil
        let occupancy = projection != nil ? depthOccupancy(frame, geo) : nil
        let result = DepthResult(occupancy: occupancy, clearance: pendingClearance,
                                 depthTrackedCar: pendingDepthCar,
                                 depthMarkerAge: pendingDepthMarkerAge,
                                 depthTrackQuality: pendingDepthQuality)

        // Do not keep a reference to `frame` beyond this call (ARKit stalls if frames are retained).
        let markers = onFrame?(frame.capturedImage, t, projection, result) ?? []

        if status.trackingNormal && (corners.count < 4 || goal == nil) {
            absorbMarkers(markers, geo)
        }

        if t - lastStatusPush > 0.2 {
            lastStatusPush = t
            onStatus?(status)
        }
    }

    // MARK: - Private (AR queue)

    private func updateTracking(_ state: ARCamera.TrackingState) {
        switch state {
        case .normal:
            status.tracking = "Tracking OK"
        case .notAvailable:
            status.tracking = "Tracking unavailable"
        case .limited(let reason):
            switch reason {
            case .initializing: status.tracking = "Starting up — move the phone slowly"
            case .excessiveMotion: status.tracking = "Moving too fast"
            case .insufficientFeatures: status.tracking = "Not enough texture in view"
            case .relocalizing: status.tracking = "Relocalizing…"
            @unknown default: status.tracking = "Tracking limited"
            }
        }
        if case .normal = state { status.trackingNormal = true } else { status.trackingNormal = false }
    }

    /// Floor = the lowest reasonably large horizontal plane ARKit has found.
    private func updateFloor(_ anchors: [ARAnchor]) {
        let planes = anchors.compactMap { $0 as? ARPlaneAnchor }
            .filter { $0.alignment == .horizontal && $0.planeExtent.width * $0.planeExtent.height > 0.1 }
        if let lowest = planes.map({ $0.transform.columns.3.y }).min() {
            floorY = lowest
            status.floorFound = true
        }
    }

    /// Pins corner and goal markers seen up close. Each needs 12 readings that agree within 2 cm.
    private func absorbMarkers(_ markers: [ArucoMarker], _ geo: FrameGeometry) {
        guard let fy = floorY else { return }
        for m in markers {
            let id = Int(m.markerId)
            let wanted = (MarkerIDs.corners.contains(id) && corners[id] == nil)
                || (id == MarkerIDs.goal && goal == nil)
            guard wanted else { continue }
            guard minSide(m) >= minMarkerSidePx else { continue }
            let center = CGPoint(x: (m.c0.x + m.c1.x + m.c2.x + m.c3.x) / 4,
                                 y: (m.c0.y + m.c1.y + m.c2.y + m.c3.y) / 4)
            guard let p = geo.floorPoint(pixel: center, planeY: fy) else { continue }

            var s = samples[id, default: []]
            s.append(p)
            if s.count > neededSamples { s.removeFirst() }
            samples[id] = s
            status.progress[id] = s.count

            if s.count == neededSamples {
                let mean = s.reduce(simd_float3(repeating: 0), +) / Float(s.count)
                let spread = s.map { simd_distance($0, mean) }.max() ?? 1
                if spread < maxSpread {
                    if id == MarkerIDs.goal { pinGoal(at: mean) } else { place(id, at: mean) }
                }
            }
        }
    }

    private func place(_ id: Int, at p: simd_float3) {
        corners[id] = p
        samples[id] = nil
        status.progress[id] = nil
        updateMeasured()
        let all = corners
        DispatchQueue.main.async { self.drawCorner(id, p, all: all) }
    }

    /// Forget all mapped obstacles (main thread).
    func clearObstacleMap() {
        queue.async { self.clearMapState() }
    }

    private func clearMapState() {
        mapLogOdds = []
        mapObserved = []
        mapDims = (0, 0)
    }

    /// LiDAR obstacle mapping for a phone that may be moving.
    ///
    /// Each frame, depth points inside the arena are converted to 3D. For every 5 cm
    /// cell in view: enough raised, high-confidence points = a "hit"; plenty of floor
    /// points and nothing raised = a "miss". Hits and misses update a persistent
    /// log-odds map, so cells out of view keep their last known state, one noisy
    /// frame can't create or erase an obstacle, and removed objects clear in ~0.1 s
    /// once seen again. Integration pauses while the phone moves fast (blurry depth).
    ///
    /// Output matches ChangeDetectorBridge: cols*rows bytes,
    /// bit 0 = occupied, bit 1 = known (observed at least once).
    private func depthOccupancy(_ frame: ARFrame, _ geo: FrameGeometry) -> Data? {
        guard useDepth, corners.count == 4, let fy = floorY else { return nil }

        let arena = self.arena
        let (cols, rows) = GridSpec.dims(arena)
        let n = cols * rows
        if mapDims.cols != cols || mapDims.rows != rows {
            mapLogOdds = [Float](repeating: 0, count: n)
            mapObserved = [Bool](repeating: false, count: n)
            mapDims = (cols, rows)
        }

        if let depth = (Self.useSmoothedDepth ? frame.smoothedSceneDepth : frame.sceneDepth),
           !isMovingTooFast(frame) {
            integrate(depth, frame, geo, floorY: fy, arena: arena, cols: cols, rows: rows)
        }
        lastPose = (frame.timestamp, frame.camera.transform)

        var out = Data(count: n)
        out.withUnsafeMutableBytes { (buf: UnsafeMutableRawBufferPointer) in
            for k in 0..<n {
                var b: UInt8 = mapObserved[k] ? 2 : 0
                if mapLogOdds[k] >= occupiedAt { b |= 1 }
                buf[k] = b
            }
        }
        return out
    }

    private func isMovingTooFast(_ frame: ARFrame) -> Bool {
        guard let last = lastPose else { return false }
        let dt = Float(frame.timestamp - last.t)
        guard dt > 0.001 else { return false }
        let turn = (simd_quatf(last.transform).inverse * simd_quatf(frame.camera.transform)).angle
        let move = simd_distance(simd_make_float3(last.transform.columns.3),
                                 simd_make_float3(frame.camera.transform.columns.3))
        return turn / dt > maxTurnRate || move / dt > maxMoveSpeed
    }

    private func integrate(_ depth: ARDepthData, _ frame: ARFrame, _ geo: FrameGeometry,
                           floorY fy: Float, arena: ArenaConfig, cols: Int, rows: Int) {
        let n = cols * rows
        // Pinned corners (world x, z) -> arena coordinates (cm).
        let src = MarkerIDs.corners.compactMap { corners[$0] }.map { CGPoint(x: CGFloat($0.x), y: CGFloat($0.z)) }
        guard src.count == 4,
              let toArena = Homography.fromFourPoints(src: src, dst: MarkerIDs.cornerWorldPositions(arena))
        else { return }

        let dm = depth.depthMap
        CVPixelBufferLockBaseAddress(dm, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(dm, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(dm) else { return }
        let w = CVPixelBufferGetWidth(dm), h = CVPixelBufferGetHeight(dm)
        let bpr = CVPixelBufferGetBytesPerRow(dm)

        // The depth map must cover the same view as the camera image.
        let imgW = Float(frame.camera.imageResolution.width)
        let imgH = Float(frame.camera.imageResolution.height)
        if abs(imgW / imgH - Float(w) / Float(h)) > 0.01 {
            status.depthWarning = "Camera \(Int(imgW))×\(Int(imgH)) and depth \(w)×\(h) don't match"
            return
        }
        status.depthWarning = nil

        // Intrinsics rescaled from the camera image to the depth map's resolution.
        let kx = Float(w) / imgW, ky = Float(h) / imgH
        let fxD = geo.fx * kx, fyD = geo.fy * ky, cxD = geo.cx * kx, cyD = geo.cy * ky

        var confBase: UnsafeMutableRawPointer?
        var confBpr = 0
        if let cm = depth.confidenceMap {
            CVPixelBufferLockBaseAddress(cm, .readOnly)
            confBase = CVPixelBufferGetBaseAddress(cm)
            confBpr = CVPixelBufferGetBytesPerRow(cm)
        }
        defer { if let cm = depth.confidenceMap { CVPixelBufferUnlockBaseAddress(cm, .readOnly) } }

        func depthRow(_ j: Int) -> UnsafeMutablePointer<Float32> {
            base.advanced(by: j * bpr).assumingMemoryBound(to: Float32.self)
        }

        let camToWorld = geo.cameraToWorld
        let camPos = simd_make_float3(camToWorld.columns.3)
        let floorRef = fy + floorOffset

        // ---- Pass 1: filter depth pixels and convert them to world points ----
        var px: [Float] = [], py: [Float] = [], pz: [Float] = [], pd: [Float] = []
        var pk: [Int32] = []
        var pax: [Float] = [], pay: [Float] = []      // arena coordinates (cm)
        pax.reserveCapacity(w * h); pay.reserveCapacity(w * h)
        px.reserveCapacity(w * h); py.reserveCapacity(w * h); pz.reserveCapacity(w * h)
        pd.reserveCapacity(w * h); pk.reserveCapacity(w * h)
        var hist = [Int](repeating: 0, count: 61)   // 1 cm bins, -30...+30 cm vs floorRef

        for j in 1..<(h - 1) {
            let row = depthRow(j), up = depthRow(j - 1), down = depthRow(j + 1)
            let conf = confBase?.advanced(by: j * confBpr).assumingMemoryBound(to: UInt8.self)
            for i in 1..<(w - 1) {
                if let conf, conf[i] < 2 { continue }                  // high confidence only
                let d = row[i]
                guard d > 0.1, d < 5 else { continue }

                // "Flying pixels": at object edges LiDAR returns depths between the object
                // and the background, i.e. points floating in midair. Skip depth jumps.
                let jump = max(abs(d - row[i - 1]), abs(d - row[i + 1]),
                               abs(d - up[i]), abs(d - down[i]))
                if jump > 0.03 + 0.02 * d { continue }

                let pc = simd_float4((Float(i) + 0.5 - cxD) / fxD * d,
                                     -(Float(j) + 0.5 - cyD) / fyD * d, -d, 1)
                let pw = simd_make_float3(camToWorld * pc)

                // Grazing rays (nearly parallel to the floor) give unreliable floor depth.
                let ray = pw - camPos
                if -ray.y / simd_length(ray) < minRayDrop { continue }

                let rel = pw.y - floorRef
                guard rel > -0.3, rel < maxObstacleHeight else { continue }
                guard let a = toArena.apply(CGPoint(x: CGFloat(pw.x), y: CGFloat(pw.z))) else { continue }
                let col = Int(floor(a.x / GridSpec.cellSize)), r = Int(floor(a.y / GridSpec.cellSize))
                guard col >= 0, col < cols, r >= 0, r < rows else { continue }

                px.append(pw.x); py.append(pw.y); pz.append(pw.z); pd.append(d)
                pk.append(Int32(r * cols + col))
                pax.append(Float(a.x)); pay.append(Float(a.y))
                if rel < 0.3 { hist[Int((rel * 100).rounded()) + 30] += 1 }
            }
        }

        // ---- Coarse floor correction (large errors): fix in one step, skip this frame ----
        if let peak = hist.indices.max(by: { hist[$0] < hist[$1] }), hist[peak] > 200 {
            let error = Float(peak - 30) / 100
            if abs(error) > 0.02 {
                floorOffset += error
                return
            }
        }

        // ---- Fit a (possibly tilted) floor plane y = a*x + b*z + c ----
        let plane = fitFloorPlane(px, py, pz, floorRef: floorRef, camPos: camPos)
        let pa = plane.a, pb = plane.b, pc0 = plane.c
        let planeAtCam = pa * camPos.x + pb * camPos.z + pc0
        floorOffset += 0.3 * (planeAtCam - floorRef)          // track slow drift

        // Diagnostics: tilt, and far-vs-near floor height if the floor were assumed flat.
        var nearH: [Float] = [], farH: [Float] = []
        for idx in stride(from: 0, to: px.count, by: 4) {
            let res = py[idx] - (pa * px[idx] + pb * pz[idx] + pc0)
            guard abs(res) < floorBand else { continue }        // floor points only
            if pd[idx] < 1.5 { nearH.append(py[idx] - floorRef) }
            if pd[idx] > 2.5 { farH.append(py[idx] - floorRef) }
        }
        status.floorTiltDegrees = Double(atan(sqrt(pa * pa + pb * pb))) * 180 / .pi
        if nearH.count > 20 && farH.count > 20 {
            let nm = nearH.sorted()[nearH.count / 2], fm = farH.sorted()[farH.count / 2]
            status.floorFarMinusNearCm = Double(fm - nm) * 100
        } else {
            status.floorFarMinusNearCm = nil
        }

        // ---- Pass 2: classify each point against the fitted floor ----
        var floorPts = [UInt16](repeating: 0, count: n)
        var raisedPts = [UInt16](repeating: 0, count: n)
        var hitPts = [UInt16](repeating: 0, count: n)
        var heights = [Float](repeating: 0, count: px.count)
        for idx in px.indices {
            let height = py[idx] - (pa * px[idx] + pb * pz[idx] + pc0)
            heights[idx] = height
            let k = Int(pk[idx])
            if height > minObstacleHeight {
                raisedPts[k] &+= 1
                if pd[idx] < maxHitRange { hitPts[k] &+= 1 }
            } else if abs(height) < floorBand {
                floorPts[k] &+= 1
            }
        }

        // ---- Car pose: marker when fresh, otherwise depth tracking of the learned outline ----
        let marker = carPose?()
        // Arena (cm) -> world, to check whether a car position is inside the camera's view.
        let arenaCorners = MarkerIDs.cornerWorldPositions(arena).map { CGPoint(x: $0.x, y: $0.y) }
        let worldXZ = MarkerIDs.corners.compactMap { corners[$0] }.map { Vec2(Double($0.x), Double($0.z)) }
        let toWorld = Homography.fromFourPoints(src: arenaCorners, dst: worldXZ)
        let imgSize = frame.camera.imageResolution
        let carMidHeight = Float((carProfile?.height ?? 8) / 200)
        func inView(_ c: Vec2) -> Bool {
            guard let toWorld, let w = toWorld.apply(CGPoint(x: c.x, y: c.y)) else { return false }
            let p = simd_float3(Float(w.x), pc0 + pa * Float(w.x) + pb * Float(w.y) + carMidHeight, Float(w.y))
            return geo.isInView(p, width: imgSize.width, height: imgSize.height)
        }
        let pose = updateCarTracking(marker: marker, t: frame.timestamp, ax: pax, ay: pay,
                                     heights: heights, inView: inView)
        let profile = carProfile

        // ---- Update the persistent map ----
        let carClear = arena.carRadius + carClearPadding
        func isCar(_ c: Vec2) -> Bool {
            guard let pose else { return false }
            // A single fixed phone can't see the car's far side, so a learned outline may
            // cover only part of it. The configured radius (set to enclose the whole car)
            // is the minimum; otherwise uncovered chassis becomes "obstacles" around the car.
            if c.distance(to: pose.position) < arena.carRadius { return true }
            if let profile {
                let q = (c - pose.position).rotated(by: -pose.heading)
                return profile.distance(to: q) < carMargin + GridSpec.cellSize / 2
            }
            return c.distance(to: pose.position) < carClear
        }
        for k in 0..<n {
            if isCar(GridSpec.center(k, cols: cols)) {
                mapLogOdds[k] = minLogOdds          // the car itself is not an obstacle
                mapObserved[k] = true
                continue
            }
            if Int(hitPts[k]) >= minHitPoints {
                mapLogOdds[k] = min(maxLogOdds, mapLogOdds[k] + hitWeight)
                mapObserved[k] = true
            } else if Int(floorPts[k]) >= minFreePoints && raisedPts[k] == 0 {
                mapLogOdds[k] = max(minLogOdds, mapLogOdds[k] + missWeight)
                mapObserved[k] = true
            }
            // Otherwise: out of view or ambiguous this frame — keep what we knew.
        }

        // ---- Car shape learning (marker only) and clearance ----
        if learning, let m = marker, m.age < 0.1 {
            learnStep(pose: m.pose, ax: pax, ay: pay, heights: heights)
        }
        if let pose {
            if let profile {
                pendingClearance = computeClearance(pose: pose, profile: profile,
                                                    ax: pax, ay: pay, heights: heights, cols: cols)
            }
        }
    }

    /// Car pose for this frame.
    ///
    /// - A fresh marker reading is authoritative: it re-anchors the tracker and unlocks it.
    /// - Otherwise, with a learned outline, search small moves/turns around the predicted
    ///   pose for the best fit of raised depth points to the outline. A fit counts only if
    ///   it explains most of the points the car normally shows AND covers most of its
    ///   outline (so smaller clutter can't pass as the car).
    /// - The car is declared LOST — and depth tracking stays off until the marker is seen
    ///   again — if its predicted position leaves the camera's view, fits fail for
    ///   `trackHold`, or it's been tracked by depth alone for `maxDepthOnly`.
    private func updateCarTracking(marker: (pose: Pose, age: Double)?, t: TimeInterval,
                                   ax: [Float], ay: [Float], heights: [Float],
                                   inView: (Vec2) -> Bool) -> Pose? {
        guard let profile = carProfile else {
            trackedCar = nil
            status.carLost = false
            return marker?.pose                     // no outline yet: marker only
        }

        // Raised points that could belong to the car.
        let maxH = Float(profile.height / 100) + 0.05
        func candidates(near c: Vec2) -> [Vec2] {
            var pts: [Vec2] = []
            for i in ax.indices where heights[i] > 0.015 && heights[i] < maxH {
                let p = Vec2(Double(ax[i]), Double(ay[i]))
                if p.distance(to: c) < 45 { pts.append(p) }
            }
            return pts
        }
        func subsample(_ pts: [Vec2]) -> [Vec2] {   // for the pose search (speed)
            guard pts.count > 150 else { return pts }
            let step = Double(pts.count) / 150
            return (0..<150).map { pts[Int(Double($0) * step)] }
        }
        // Points inside the outline at a pose.
        func inliers(_ pose: Pose, _ pts: [Vec2]) -> Int {
            var n = 0
            let c = cos(-pose.heading), s = sin(-pose.heading)
            for p in pts {
                let dx = p.x - pose.position.x, dy = p.y - pose.position.y
                if profile.distance(to: Vec2(dx * c - dy * s, dx * s + dy * c)) < 1 { n += 1 }
            }
            return n
        }
        // Fraction of the outline's 2 cm cells containing at least one point (all points,
        // not the subsample). A smaller object can't fill the car's outline.
        let cellsF = max(1, Int(ceil(profile.length / 2))), cellsL = max(1, Int(ceil(profile.width / 2)))
        func coverage(_ pose: Pose, _ pts: [Vec2]) -> Double {
            var covered = Set<Int>()
            let c = cos(-pose.heading), s = sin(-pose.heading)
            for p in pts {
                let dx = p.x - pose.position.x, dy = p.y - pose.position.y
                let q = Vec2(dx * c - dy * s, dx * s + dy * c)
                guard profile.distance(to: q) < 1 else { continue }
                let fi = min(cellsF - 1, max(0, Int((q.x - profile.minForward) / 2)))
                let li = min(cellsL - 1, max(0, Int((q.y - profile.minLeft) / 2)))
                covered.insert(fi * cellsL + li)
            }
            return Double(covered.count) / Double(cellsF * cellsL)
        }
        func lose() -> Pose? {
            trackedCar = nil
            trackVelocity = .zero
            trackTurnRate = 0
            trackingLocked = true
            status.carLost = true
            return nil
        }

        // 1) Marker: authoritative, re-anchors and unlocks.
        if let m = marker, m.age < 0.1 {
            if let prev = trackedCar, t - trackedTime > 0.01, t - trackedTime < 0.5 {
                let dt = t - trackedTime
                trackVelocity = trackVelocity * 0.5 + (m.pose.position - prev.position) * (0.5 / dt)
                trackTurnRate = trackTurnRate * 0.5 + wrapAngle(m.pose.heading - prev.heading) * (0.5 / dt)
            }
            let all = candidates(near: m.pose.position)
            let n = Double(inliers(m.pose, all))
            if n >= 10 {                            // learn what the car normally looks like
                let cov = coverage(m.pose, all)
                typicalInliers = typicalInliers == 0 ? n : typicalInliers * 0.9 + n * 0.1
                typicalCoverage = typicalCoverage == 0 ? cov : typicalCoverage * 0.9 + cov * 0.1
            }
            trackedCar = m.pose
            trackedTime = t
            lastMarkerAnchor = t
            trackingLocked = false
            status.carLost = false
            return m.pose
        }

        // 2) Depth tracking, only if not locked out and we know what the car looks like.
        guard !trackingLocked, let last = trackedCar, typicalInliers >= 20 else {
            if trackedCar != nil || trackingLocked { return lose() }
            return nil
        }
        let dt = min(t - trackedTime, 0.2)
        let predicted = Pose(position: last.position + trackVelocity * dt,
                             heading: wrapAngle(last.heading + trackTurnRate * dt))

        // Out of the camera's view => the phone can't be seeing it: lost, not "somewhere nearby".
        if !inView(predicted.position) { return lose() }
        // Depth alone for too long => require a marker reading before trusting it again.
        if t - lastMarkerAnchor > maxDepthOnly { return lose() }

        let allPts = candidates(near: predicted.position)
        let pts = subsample(allPts)
        func search(around c: Pose, range: Double, step: Double, turn: Double, turnStep: Double)
            -> (pose: Pose, inliers: Int) {
            var best = (pose: c, inliers: -1)
            var bestScore = -Double.infinity
            var dx = -range
            while dx <= range + 1e-9 {
                var dy = -range
                while dy <= range + 1e-9 {
                    var dth = -turn
                    while dth <= turn + 1e-9 {
                        let cand = Pose(position: c.position + Vec2(dx, dy), heading: wrapAngle(c.heading + dth))
                        let n = inliers(cand, pts)
                        let score = Double(n) - 0.02 * (dx * dx + dy * dy).squareRoot() - 2 * abs(dth)
                        if score > bestScore { bestScore = score; best = (cand, n) }
                        dth += turnStep
                    }
                    dy += step
                }
                dx += step
            }
            return best
        }
        let deg = Double.pi / 180
        let coarse = search(around: predicted, range: 8, step: 2, turn: 15 * deg, turnStep: 5 * deg)
        let fine = search(around: coarse.pose, range: 1.5, step: 0.5, turn: 3 * deg, turnStep: 1 * deg)

        // Judge the fit on ALL nearby points (the subsample is only for searching quickly;
        // with clutter nearby, fewer of its points land on the car).
        let fitPoints = Double(inliers(fine.pose, allPts))
        let enoughPoints = fitPoints >= max(20, minInlierFraction * typicalInliers)
        let fitCoverage = enoughPoints ? coverage(fine.pose, allPts) : 0
        let enoughCoverage = enoughPoints && fitCoverage >= minCoverageFraction * typicalCoverage
        if enoughPoints && enoughCoverage {
            pendingDepthMarkerAge = t - lastMarkerAnchor
            pendingDepthQuality = min(1, fitPoints / typicalInliers, fitCoverage / max(typicalCoverage, 1e-6))
            let ddt = max(t - trackedTime, 0.01)
            trackVelocity = trackVelocity * 0.5 + (fine.pose.position - last.position) * (0.5 / ddt)
            trackTurnRate = trackTurnRate * 0.5 + wrapAngle(fine.pose.heading - last.heading) * (0.5 / ddt)
            trackedCar = fine.pose
            trackedTime = t
            pendingDepthCar = fine.pose
            status.carLost = false
            return fine.pose
        }

        // Fit failed: hold the last pose briefly (keeps its footprint masked), then give up.
        if t - trackedTime < trackHold { return last }
        return lose()
    }

    /// One frame of car-shape learning. The marker says where the car is, so the car
    /// is the connected cluster of raised points under the marker. Accumulated in the
    /// car's own frame, so circling the car fills in all its sides.
    private func learnStep(pose: Pose, ax: [Float], ay: [Float], heights: [Float]) {
        let m = Int(2 * learnWindow / learnCell)
        var counts = [Int](repeating: 0, count: m * m)
        var cellMaxH = [Float](repeating: 0, count: m * m)
        for i in ax.indices {
            let hgt = heights[i]
            guard hgt > 0.015, hgt < 0.4 else { continue }
            let q = (Vec2(Double(ax[i]), Double(ay[i])) - pose.position).rotated(by: -pose.heading)
            let gx = Int(floor((q.x + learnWindow) / learnCell)), gy = Int(floor((q.y + learnWindow) / learnCell))
            guard gx >= 0, gx < m, gy >= 0, gy < m else { continue }
            counts[gy * m + gx] += 1
            cellMaxH[gy * m + gx] = max(cellMaxH[gy * m + gx], hgt)
        }

        // Seed at the marker (grid center), or the nearest raised cell within 6 cm.
        let mid = m / 2
        var seed: Int?
        search: for radius in 0...3 {
            for dy in -radius...radius {
                for dx in -radius...radius where counts[(mid + dy) * m + mid + dx] >= 2 {
                    seed = (mid + dy) * m + mid + dx
                    break search
                }
            }
        }
        guard let seed else {
            status.carLearnMessage = "Can't see the car's marker area — point at the car"
            return
        }

        // Flood fill over raised cells (4-connected) = the car.
        var inCluster = [Bool](repeating: false, count: m * m)
        var stack = [seed]
        inCluster[seed] = true
        var clusterMaxH: Float = 0
        while let c = stack.popLast() {
            clusterMaxH = max(clusterMaxH, cellMaxH[c])
            let cx = c % m, cy = c / m
            for (nx, ny) in [(cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)] {
                guard nx >= 0, nx < m, ny >= 0, ny < m else { continue }
                let nk = ny * m + nx
                if !inCluster[nk] && counts[nk] >= 2 {
                    inCluster[nk] = true
                    stack.append(nk)
                }
            }
        }
        for k in inCluster.indices where inCluster[k] { learnCounts[k] += 1 }
        learnHeights.append(clusterMaxH)
        learnFrames += 1
        status.carLearnProgress = learnFrames
        status.carLearnMessage = nil
        if learnFrames >= learnTargetFrames { finishLearning(m: m) }
    }

    private func finishLearning(m: Int) {
        learning = false
        status.carLearnProgress = nil
        // Cells that were part of the car in at least half the frames.
        let keep = learnCounts.indices.filter { learnCounts[$0] * 2 >= learnFrames }
        guard keep.count >= 8 else {
            status.carLearnMessage = "Couldn't isolate the car — make sure nothing touches it"
            return
        }
        let fs = keep.map { Double($0 % m) * learnCell - learnWindow }
        let ls = keep.map { Double($0 / m) * learnCell - learnWindow }
        let heightsSorted = learnHeights.sorted()
        let profile = CarProfile(minForward: fs.min()!, maxForward: fs.max()! + learnCell,
                                 minLeft: ls.min()!, maxLeft: ls.max()! + learnCell,
                                 height: Double(heightsSorted[heightsSorted.count / 2]) * 100)
        guard (6...70).contains(profile.length), (6...70).contains(profile.width) else {
            status.carLearnMessage = String(format: "Measured %.0f×%.0f cm, which doesn't look like the car — try again with it clear",
                                            profile.length, profile.width)
            return
        }
        carProfile = profile
        status.carProfile = profile
        status.carLearnMessage = nil
        if let data = try? JSONEncoder().encode(profile) {
            UserDefaults.standard.set(data, forKey: Self.profileKey)
        }
    }

    /// Distance from the car's outline to the nearest obstacle: live LiDAR points on a
    /// 2 cm grid (3+ points per cell) plus remembered map cells for things out of view.
    private func computeClearance(pose: Pose, profile: CarProfile, ax: [Float], ay: [Float],
                                  heights: [Float], cols: Int) -> ClearanceInfo {
        var cells: [Int: Int] = [:]
        for i in ax.indices {
            let hgt = heights[i]
            guard hgt > clearanceMinHeight, hgt < maxObstacleHeight else { continue }
            let p = Vec2(Double(ax[i]), Double(ay[i]))
            let q = (p - pose.position).rotated(by: -pose.heading)
            guard q.length < clearanceRange, profile.distance(to: q) >= carMargin else { continue }
            cells[Int(floor(p.x / 2)) * 100_000 + Int(floor(p.y / 2)), default: 0] += 1
        }
        var candidates: [(point: Vec2, distance: Double)] = []
        for (key, count) in cells where count >= 3 {
            let p = Vec2(Double(key / 100_000) * 2 + 1, Double(key % 100_000) * 2 + 1)
            let q = (p - pose.position).rotated(by: -pose.heading)
            candidates.append((p, profile.distance(to: q)))
        }
        // Remembered obstacles (5 cm cells) that may be out of view right now.
        for k in mapLogOdds.indices where mapLogOdds[k] >= occupiedAt {
            let p = GridSpec.center(k, cols: cols)
            let q = (p - pose.position).rotated(by: -pose.heading)
            guard q.length < clearanceRange else { continue }
            let d = profile.distance(to: q)
            guard d >= carMargin + GridSpec.cellSize / 2 else { continue }
            candidates.append((p, max(0, d - GridSpec.cellSize / 2)))
        }

        guard let nearest = candidates.min(by: { $0.distance < $1.distance }) else {
            return ClearanceInfo(distance: nil, point: nil, bearingDegrees: nil, nearby: [])
        }
        let q = (nearest.point - pose.position).rotated(by: -pose.heading)
        return ClearanceInfo(distance: nearest.distance, point: nearest.point,
                             bearingDegrees: atan2(q.y, q.x) * 180 / .pi,
                             nearby: candidates.filter { $0.distance < 60 })
    }

    /// Robust floor fit: RANSAC over points near the expected floor, then least squares
    /// on the inliers. Tilt is capped at ~6°, and the plane must pass within 8 cm of the
    /// expected floor under the camera, so a large box top can't be mistaken for the floor.
    /// Falls back to a flat floor if there aren't enough floor points.
    private func fitFloorPlane(_ px: [Float], _ py: [Float], _ pz: [Float],
                               floorRef: Float, camPos: simd_float3) -> (a: Float, b: Float, c: Float) {
        let flat = (a: Float(0), b: Float(0), c: floorRef)
        var cand: [Int] = []
        for i in px.indices where abs(py[i] - floorRef) < 0.15 { cand.append(i) }
        guard cand.count >= 200 else { return flat }
        let step = max(1, cand.count / 3000)
        let sample = stride(from: 0, to: cand.count, by: step).map { cand[$0] }

        var seed: UInt32 = 2463534242
        func rand(_ m: Int) -> Int {
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5
            return Int(seed % UInt32(m))
        }
        func valid(_ a: Float, _ b: Float, _ c: Float) -> Bool {
            abs(a) < 0.1 && abs(b) < 0.1 && abs(a * camPos.x + b * camPos.z + c - floorRef) < 0.08
        }

        var best = flat
        var bestCount = -1
        for _ in 0..<60 {
            let i1 = sample[rand(sample.count)], i2 = sample[rand(sample.count)], i3 = sample[rand(sample.count)]
            let m = simd_float3x3(rows: [simd_float3(px[i1], pz[i1], 1),
                                         simd_float3(px[i2], pz[i2], 1),
                                         simd_float3(px[i3], pz[i3], 1)])
            guard abs(simd_determinant(m)) > 1e-4 else { continue }
            let sol = simd_inverse(m) * simd_float3(py[i1], py[i2], py[i3])
            guard valid(sol.x, sol.y, sol.z) else { continue }
            var count = 0
            for i in sample where abs(py[i] - (sol.x * px[i] + sol.y * pz[i] + sol.z)) < 0.015 { count += 1 }
            if count > bestCount { bestCount = count; best = (sol.x, sol.y, sol.z) }
        }
        guard bestCount >= 100 else { return flat }

        // Least-squares refinement on the inliers (double precision).
        var m = simd_double3x3()
        var v = simd_double3(0, 0, 0)
        for i in sample where abs(py[i] - (best.a * px[i] + best.b * pz[i] + best.c)) < 0.015 {
            let q = simd_double3(Double(px[i]), Double(pz[i]), 1)
            m += simd_double3x3(columns: (q * q.x, q * q.y, q * q.z))
            v += q * Double(py[i])
        }
        guard abs(simd_determinant(m)) > 1e-9 else { return best }
        let r = simd_inverse(m) * v
        let refined = (a: Float(r.x), b: Float(r.y), c: Float(r.z))
        return valid(refined.a, refined.b, refined.c) ? refined : best
    }

    private func pinGoal(at p: simd_float3) {
        goal = p
        samples[MarkerIDs.goal] = nil
        status.progress[MarkerIDs.goal] = nil
        status.goalPinned = true
        DispatchQueue.main.async { self.drawGoal(p) }
    }

    private func updateMeasured() {
        status.captured = MarkerIDs.corners.filter { corners[$0] != nil }
        if let c2 = corners[2], let c3 = corners[3] {
            status.measuredWidth = Double(simd_distance(c2, c3)) * 100
        } else {
            status.measuredWidth = nil
        }
        if let c2 = corners[2], let c5 = corners[5] {
            status.measuredHeight = Double(simd_distance(c2, c5)) * 100
        } else {
            status.measuredHeight = nil
        }
    }

    private func makeProjection(_ geo: FrameGeometry) -> ARProjection? {
        let lift = simd_float3(0, carMarkerHeight, 0)   // ARKit world +y is up
        var floor: [CGPoint] = []
        var car: [CGPoint] = []
        for id in MarkerIDs.corners {
            guard let c = corners[id], let a = geo.project(c), let b = geo.project(c + lift) else { return nil }
            floor.append(a)
            car.append(b)
        }
        return ARProjection(floor: floor, car: car, goal: goal.flatMap { geo.project($0) })
    }

    private func minSide(_ m: ArucoMarker) -> CGFloat {
        let pts = [m.c0, m.c1, m.c2, m.c3]
        return (0..<4).map { i in
            let a = pts[i], b = pts[(i + 1) % 4]
            return hypot(a.x - b.x, a.y - b.y)
        }.min() ?? 0
    }

    // MARK: - Drawing (main thread)

    private func drawCorner(_ id: Int, _ p: simd_float3, all: [Int: simd_float3]) {
        cornerNodes[id]?.forEach { $0.removeFromParentNode() }

        let sphere = SCNSphere(radius: 0.015)
        sphere.firstMaterial?.diffuse.contents = UIColor.systemYellow
        let dot = SCNNode(geometry: sphere)
        dot.simdPosition = p

        let text = SCNText(string: "\(id)", extrusionDepth: 0.5)
        text.font = .boldSystemFont(ofSize: 5)
        text.firstMaterial?.diffuse.contents = UIColor.white
        let label = SCNNode(geometry: text)
        label.scale = SCNVector3(0.01, 0.01, 0.01)
        label.simdPosition = p + simd_float3(0, 0.04, 0)
        label.constraints = [SCNBillboardConstraint()]

        sceneView.scene.rootNode.addChildNode(dot)
        sceneView.scene.rootNode.addChildNode(label)
        cornerNodes[id] = [dot, label]

        // Arena outline once all four are pinned.
        outlineNodes.forEach { $0.removeFromParentNode() }
        outlineNodes = []
        let order = MarkerIDs.corners
        let pts = order.compactMap { all[$0] }
        if pts.count == 4 {
            for i in 0..<4 {
                let n = line(from: pts[i], to: pts[(i + 1) % 4])
                sceneView.scene.rootNode.addChildNode(n)
                outlineNodes.append(n)
            }
        }
    }

    private func drawGoal(_ p: simd_float3) {
        goalNodes.forEach { $0.removeFromParentNode() }

        // Flat green ring on the floor plus a floating label.
        let ring = SCNTorus(ringRadius: 0.08, pipeRadius: 0.006)
        ring.firstMaterial?.diffuse.contents = UIColor.systemGreen
        let ringNode = SCNNode(geometry: ring)
        ringNode.simdPosition = p + simd_float3(0, 0.005, 0)

        let dot = SCNSphere(radius: 0.02)
        dot.firstMaterial?.diffuse.contents = UIColor.systemGreen
        let dotNode = SCNNode(geometry: dot)
        dotNode.simdPosition = p + simd_float3(0, 0.02, 0)

        let text = SCNText(string: "GOAL", extrusionDepth: 0.5)
        text.font = .boldSystemFont(ofSize: 5)
        text.firstMaterial?.diffuse.contents = UIColor.white
        let label = SCNNode(geometry: text)
        label.scale = SCNVector3(0.01, 0.01, 0.01)
        label.simdPosition = p + simd_float3(0, 0.07, 0)
        label.constraints = [SCNBillboardConstraint()]

        [ringNode, dotNode, label].forEach { sceneView.scene.rootNode.addChildNode($0) }
        goalNodes = [ringNode, dotNode, label]
    }

    private func line(from a: simd_float3, to b: simd_float3) -> SCNNode {
        let d = simd_distance(a, b)
        let cylinder = SCNCylinder(radius: 0.004, height: CGFloat(d))
        cylinder.firstMaterial?.diffuse.contents = UIColor.systemGreen
        let node = SCNNode(geometry: cylinder)
        node.simdPosition = (a + b) / 2
        node.simdOrientation = simd_quatf(from: simd_float3(0, 1, 0), to: simd_normalize(b - a))
        return node
    }
}
