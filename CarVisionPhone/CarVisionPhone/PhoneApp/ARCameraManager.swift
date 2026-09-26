import ARKit
import SceneKit
import UIKit

/// Per-frame result of AR calibration, in captured-image pixel coordinates.
struct ARProjection {
    /// The four pinned corners (IDs 2,3,4,5 in order) projected into this frame, on the floor.
    var floor: [CGPoint]
    /// The same corners lifted to the car marker's height.
    var car: [CGPoint]
}

struct ARCalibrationStatus: Equatable {
    var tracking = "Starting…"
    var trackingNormal = false
    var floorFound = false
    /// Corner IDs pinned so far.
    var captured: [Int] = []
    /// Samples collected for corners currently being pinned (0...needed).
    var progress: [Int: Int] = [:]
    /// Measured by ARKit, in cm: corner 2→3 and 2→5.
    var measuredWidth: Double?
    var measuredHeight: Double?

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

/// Runs an ARKit world-tracking session. Corners are pinned in 3D by walking the
/// phone up to each corner marker (or tapping the floor), after which they are
/// projected into every frame, even when off-screen.
final class ARCameraManager: NSObject, ARSessionDelegate {
    let sceneView = ARSCNView(frame: .zero)
    private let queue = DispatchQueue(label: "ar.frames", qos: .userInteractive)

    /// Called on the AR queue (~30 Hz). Returns the markers detected in the frame
    /// so corner markers can be pinned.
    var onFrame: ((CVPixelBuffer, Double, ARProjection?) -> [ArucoMarker])?
    /// Called on the AR queue a few times per second.
    var onStatus: ((ARCalibrationStatus) -> Void)?

    private let heightLock = NSLock()
    private var _carMarkerHeight: Float = 0.10
    /// Height of the car's marker above the floor, in meters.
    var carMarkerHeight: Float {
        get { heightLock.lock(); defer { heightLock.unlock() }; return _carMarkerHeight }
        set { heightLock.lock(); _carMarkerHeight = newValue; heightLock.unlock() }
    }

    // AR-queue state
    private var floorY: Float?
    private var corners: [Int: simd_float3] = [:]
    private var samples: [Int: [simd_float3]] = [:]
    private var lastProcessed: TimeInterval = 0
    private var lastStatusPush: TimeInterval = 0
    private var status = ARCalibrationStatus()

    // Main-thread state
    private var cornerNodes: [Int: [SCNNode]] = [:]
    private var outlineNodes: [SCNNode] = []

    private let neededSamples = 12
    private let maxSpread: Float = 0.02          // m; samples must agree within 2 cm
    private let minMarkerSidePx: CGFloat = 35    // come close enough for a precise read

    func start() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal]
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
            self.updateMeasured()
        }
        for nodes in cornerNodes.values { nodes.forEach { $0.removeFromParentNode() } }
        cornerNodes = [:]
        outlineNodes.forEach { $0.removeFromParentNode() }
        outlineNodes = []
    }

    /// Measure-app style: tap the floor to pin the next missing corner (main thread).
    func placeNextCorner(atViewPoint point: CGPoint) {
        guard let query = sceneView.raycastQuery(from: point, allowing: .estimatedPlane, alignment: .horizontal),
              let hit = sceneView.session.raycast(query).first else { return }
        var p = simd_make_float3(hit.worldTransform.columns.3)
        queue.async {
            guard let id = MarkerIDs.corners.first(where: { self.corners[$0] == nil }) else { return }
            if let fy = self.floorY { p.y = fy }
            self.place(id, at: p)
        }
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

        // Do not keep a reference to `frame` beyond this call (ARKit stalls if frames are retained).
        let markers = onFrame?(frame.capturedImage, t, projection) ?? []

        if corners.count < 4 && status.trackingNormal {
            absorbCornerMarkers(markers, geo)
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

    private func absorbCornerMarkers(_ markers: [ArucoMarker], _ geo: FrameGeometry) {
        guard let fy = floorY else { return }
        for m in markers {
            let id = Int(m.markerId)
            guard MarkerIDs.corners.contains(id), corners[id] == nil else { continue }
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
                if spread < maxSpread { place(id, at: mean) }
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
        return ARProjection(floor: floor, car: car)
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
