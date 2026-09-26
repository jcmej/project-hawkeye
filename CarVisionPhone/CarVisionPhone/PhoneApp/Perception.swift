import Foundation
import CoreGraphics

/// Maps image pixels to floor coordinates (cm). Built from the four corner
/// markers, so it works regardless of phone orientation or tilt.
struct Homography {
    let m: [Double]   // 3x3 row-major, m[8] == 1

    func apply(_ p: CGPoint) -> Vec2? {
        let x = Double(p.x), y = Double(p.y)
        let w = m[6] * x + m[7] * y + m[8]
        guard abs(w) > 1e-12 else { return nil }
        return Vec2((m[0] * x + m[1] * y + m[2]) / w,
                    (m[3] * x + m[4] * y + m[5]) / w)
    }

    /// Solves the standard 8-unknown linear system from 4 correspondences.
    static func fromFourPoints(src: [CGPoint], dst: [Vec2]) -> Homography? {
        guard src.count == 4, dst.count == 4 else { return nil }
        var a = [[Double]](repeating: [Double](repeating: 0, count: 9), count: 8)
        for i in 0..<4 {
            let x = Double(src[i].x), y = Double(src[i].y)
            let u = dst[i].x, v = dst[i].y
            a[2 * i]     = [x, y, 1, 0, 0, 0, -u * x, -u * y, u]
            a[2 * i + 1] = [0, 0, 0, x, y, 1, -v * x, -v * y, v]
        }
        // Gauss-Jordan elimination with partial pivoting.
        for col in 0..<8 {
            var pivot = col
            for r in (col + 1)..<8 where abs(a[r][col]) > abs(a[pivot][col]) { pivot = r }
            guard abs(a[pivot][col]) > 1e-12 else { return nil }
            a.swapAt(col, pivot)
            for r in 0..<8 where r != col {
                let f = a[r][col] / a[col][col]
                if f != 0 {
                    for c in col..<9 { a[r][c] -= f * a[col][c] }
                }
            }
        }
        var h = (0..<8).map { a[$0][8] / a[$0][$0] }
        h.append(1)
        return Homography(m: h)
    }
}

/// Snapshot for the phone's on-screen status and mini-map.
struct PhoneDebugState {
    var calibrated = false
    var markerIds: [Int] = []
    var car: Pose?
    var goal: Vec2?
    var obstacles: [Vec2] = []
    var occupied: [Vec2] = []
    var hasBackground = false
    var backgroundNote: String?
    var calibrationNote: String?
    var carSpeed: Double = 0
    var veto = false
    var vetoReason: String?
    var fps: Double = 0
}

/// Turns raw detections into a world-frame ObservationMessage.
/// All methods except `settings` and `requestRecalibration()` must be called
/// from the same queue (the camera queue).
final class Perception {
    struct Settings {
        var cameraId: String
        var arena: ArenaConfig
        /// How far ahead (seconds) the collision check projects the car's motion.
        var vetoHorizon: Double = 0.6
        /// Brightness difference (0-255) that counts as "changed".
        var changeThreshold: Int = 28
        /// Fraction of a cell's pixels that must change for it to be occupied.
        var minChangedFraction: Double = 0.3
        /// Extra radius around the car ignored by change detection (the car itself changes the scene).
        var carMaskPadding: Double = 8
        /// Radius around the goal marker ignored by change detection.
        var goalMaskRadius: Double = 15
    }

    private let lock = NSLock()
    private var _settings: Settings
    private var _recalibrateRequested = false

    var settings: Settings {
        get { lock.lock(); defer { lock.unlock() }; return _settings }
        set { lock.lock(); _settings = newValue; lock.unlock() }
    }

    // Camera-queue-only state
    private var homography: Homography?
    /// AR mode only: exact mapping for the plane at the car marker's height.
    private var carHomography: Homography?
    private var carHistory: [(t: Double, p: Vec2)] = []
    private var seq = 0

    // Calibration guard: corner pixel positions must be stable before the first
    // calibration, and must stay near the calibrated ones afterwards.
    private var calibratedCorners: [CGPoint]?
    private var candidateCorners: [CGPoint]?
    private var candidateCount = 0
    private var cornerMismatchFrames = 0
    private var calibrationNote: String?

    // Persistence filter: a marker only counts once seen in several recent
    // frames. This rejects one-frame false matches from textured floors and
    // smooths over brief detection dropouts.
    private var hits: [Int: [Double]] = [:]
    private var held: [Int: (position: Vec2, lastConfirmed: Double)] = [:]
    private let hitWindow = 0.35      // s
    private let minHits = 3           // for goal and obstacles
    private let minCarHits = 2        // car needs to stay responsive
    private let holdTime = 0.3        // s to keep reporting after a dropout

    init(settings: Settings) {
        _settings = settings
    }

    /// Forget the cached calibration (use after moving/bumping the phone).
    func requestRecalibration() {
        lock.lock(); _recalibrateRequested = true; lock.unlock()
    }

    /// Step 1 each frame: refresh calibration if all four corners are visible.
    /// The first calibration needs the corners steady for 5 frames; after that,
    /// updates are only accepted if every corner is close to where it was
    /// (the phone is assumed fixed), so a false corner match can't corrupt it.
    func updateCalibration(markers: [ArucoMarker]) {
        consumeResetRequest()

        var byId: [Int: ArucoMarker] = [:]
        for m in markers { byId[Int(m.markerId)] = m }
        let cornerMarkers = MarkerIDs.corners.compactMap { byId[$0] }
        guard cornerMarkers.count == 4 else { return }
        let pts = cornerMarkers.map(center(of:))

        func allWithin(_ a: [CGPoint], _ b: [CGPoint], _ tol: CGFloat) -> Bool {
            zip(a, b).allSatisfy { hypot($0.x - $1.x, $0.y - $1.y) < tol }
        }

        if let cal = calibratedCorners {
            if allWithin(pts, cal, 25) {
                cornerMismatchFrames = 0
                calibrationNote = nil
                if let h = Homography.fromFourPoints(src: pts, dst: MarkerIDs.cornerWorldPositions(settings.arena)) {
                    homography = h
                    calibratedCorners = pts
                }
            } else {
                cornerMismatchFrames += 1
                if cornerMismatchFrames > 30 {
                    calibrationNote = "Corners moved — tap Recalibrate"
                }
            }
        } else {
            if let cand = candidateCorners, allWithin(pts, cand, 5) {
                candidateCount += 1
            } else {
                candidateCount = 1
            }
            candidateCorners = pts
            if candidateCount >= 5,
               let h = Homography.fromFourPoints(src: pts, dst: MarkerIDs.cornerWorldPositions(settings.arena)) {
                homography = h
                calibratedCorners = pts
            }
        }
    }

    /// AR mode: calibration comes from corners pinned in 3D and projected into
    /// this frame, so they don't need to be visible. Also builds a second mapping
    /// at the car marker's height, which removes the elevated-marker error.
    func setProjectedCorners(floor: [CGPoint], car: [CGPoint]) {
        consumeResetRequest()
        let dst = MarkerIDs.cornerWorldPositions(settings.arena)
        if let h = Homography.fromFourPoints(src: floor, dst: dst) { homography = h }
        carHomography = Homography.fromFourPoints(src: car, dst: dst)
        calibrationNote = nil
    }

    /// AR mode before corners are pinned: just honor reset requests.
    func pollReset() { consumeResetRequest() }

    private func consumeResetRequest() {
        lock.lock()
        let reset = _recalibrateRequested
        _recalibrateRequested = false
        lock.unlock()
        guard reset else { return }
        homography = nil
        carHomography = nil
        calibratedCorners = nil
        candidateCorners = nil
        candidateCount = 0
        cornerMismatchFrames = 0
        calibrationNote = nil
    }

    /// Records a detection and returns true if the marker is confirmed.
    private func confirm(_ id: Int, at t: Double, minHits: Int) -> Bool {
        var h = hits[id, default: []]
        h.append(t)
        h.removeAll { t - $0 > hitWindow }
        hits[id] = h
        return h.count >= minHits
    }

    /// Matrix mapping image pixels -> the top-down grid image used for change
    /// detection (row 0 = the arena's +y edge). Nil until calibrated.
    func topDownMatrix() -> [Double]? {
        guard let h = homography else { return nil }
        let s = Double(GridSpec.pixelsPerCell) / GridSpec.cellSize
        let rowsPx = Double(GridSpec.dims(settings.arena).rows * GridSpec.pixelsPerCell)
        let m = h.m
        // [[s,0,0],[0,-s,rowsPx],[0,0,1]] * H   (world cm -> top-down pixels, y flipped)
        return [s * m[0], s * m[1], s * m[2],
                -s * m[3] + rowsPx * m[6], -s * m[4] + rowsPx * m[7], -s * m[5] + rowsPx * m[8],
                m[6], m[7], m[8]]
    }

    /// Step 2 each frame: build the observation from markers and (optional) raw occupancy.
    func process(markers: [ArucoMarker], rawOccupancy: Data?, time t: Double, fps: Double)
        -> (ObservationMessage, PhoneDebugState) {
        let s = settings

        var byId: [Int: ArucoMarker] = [:]
        for m in markers { byId[Int(m.markerId)] = m }

        var msg = ObservationMessage(cameraId: s.cameraId, seq: seq,
                                     calibrated: homography != nil,
                                     car: nil, carConfidence: 0, goal: nil,
                                     obstacles: [], veto: false, vetoReason: nil, fps: fps)
        seq += 1

        var dbg = PhoneDebugState()
        dbg.markerIds = markers.map { Int($0.markerId) }.sorted()
        dbg.fps = fps
        dbg.calibrated = homography != nil
        dbg.calibrationNote = calibrationNote

        guard let hom = homography else { return (msg, dbg) }

        // Car: confirmed if seen in 2+ recent frames; no hold (control needs fresh poses).
        if let m = byId[MarkerIDs.car], confirm(MarkerIDs.car, at: t, minHits: minCarHits),
           let pose = pose(of: m, carHomography ?? hom) {
            msg.car = pose
            msg.carConfidence = confidence(of: m)
            carHistory.append((t: t, p: pose.position))
        }
        carHistory.removeAll { t - $0.t > 0.5 }

        // Goal and obstacles: confirmed after 3 hits, then held briefly through dropouts.
        let stationaryIds = [MarkerIDs.goal] + Array(MarkerIDs.obstacles)
        for id in stationaryIds {
            if let m = byId[id], confirm(id, at: t, minHits: minHits),
               let p = hom.apply(center(of: m)) {
                held[id] = (p, t)
            }
        }
        held = held.filter { t - $0.value.lastConfirmed <= holdTime }
        msg.goal = held[MarkerIDs.goal]?.position
        msg.obstacles = held
            .filter { MarkerIDs.obstacles.contains($0.key) }
            .map { ObstacleObservation(id: $0.key, position: $0.value.position) }
            .sorted { $0.id < $1.id }

        // Markerless occupancy
        var occupiedCenters: [Vec2] = []
        if let raw = rawOccupancy {
            let (cols, rows) = GridSpec.dims(s.arena)
            let bytes = [UInt8](raw)
            if bytes.count == cols * rows {
                var occ = bytes.map { $0 & 1 != 0 }
                let vis = bytes.map { $0 & 2 != 0 }

                // Ignore changes caused by things that are supposed to be there.
                var masks: [(center: Vec2, radius: Double)] = []
                if let carP = msg.car?.position ?? carHistory.last?.p {
                    masks.append((carP, s.arena.carRadius + s.carMaskPadding))
                }
                if let g = msg.goal {
                    masks.append((g, s.goalMaskRadius))
                }
                for i in occ.indices where occ[i] {
                    let c = GridSpec.center(i, cols: cols)
                    if masks.contains(where: { c.distance(to: $0.center) < $0.radius }) {
                        occ[i] = false
                    }
                }

                msg.grid = GridMessage(cols: cols, rows: rows,
                                       occupied: GridMessage.pack(occ),
                                       visible: GridMessage.pack(vis))
                occupiedCenters = occ.indices.filter { occ[$0] }.map { GridSpec.center($0, cols: cols) }
            }
        }

        // Safety veto
        if let car = msg.car {
            let v = velocity()
            dbg.carSpeed = v.length
            if let reason = collisionCheck(position: car.position, velocity: v,
                                           obstacles: msg.obstacles,
                                           occupied: occupiedCenters, settings: s) {
                msg.veto = true
                msg.vetoReason = reason
            }
        }

        dbg.car = msg.car
        dbg.goal = msg.goal
        dbg.obstacles = msg.obstacles.map(\.position)
        dbg.occupied = occupiedCenters
        dbg.veto = msg.veto
        dbg.vetoReason = msg.vetoReason
        return (msg, dbg)
    }

    // MARK: - Helpers

    private func center(of m: ArucoMarker) -> CGPoint {
        CGPoint(x: (m.c0.x + m.c1.x + m.c2.x + m.c3.x) / 4,
                y: (m.c0.y + m.c1.y + m.c2.y + m.c3.y) / 4)
    }

    /// Heading = direction from the marker's bottom edge to its top edge, in world frame.
    private func pose(of m: ArucoMarker, _ h: Homography) -> Pose? {
        guard let w0 = h.apply(m.c0), let w1 = h.apply(m.c1),
              let w2 = h.apply(m.c2), let w3 = h.apply(m.c3) else { return nil }
        let center = (w0 + w1 + w2 + w3) * 0.25
        let front = (w0 + w1) * 0.5 - (w3 + w2) * 0.5
        return Pose(position: center, heading: atan2(front.y, front.x))
    }

    /// Bigger marker in the image = more pixels = more trustworthy estimate.
    private func confidence(of m: ArucoMarker) -> Double {
        func d(_ a: CGPoint, _ b: CGPoint) -> Double {
            Double(hypot(a.x - b.x, a.y - b.y))
        }
        let side = (d(m.c0, m.c1) + d(m.c1, m.c2) + d(m.c2, m.c3) + d(m.c3, m.c0)) / 4
        return clamp(side / 80.0, 0.1, 1.0)
    }

    private func velocity() -> Vec2 {
        guard let first = carHistory.first, let last = carHistory.last,
              last.t - first.t > 0.15 else { return .zero }
        return (last.p - first.p) * (1 / (last.t - first.t))
    }

    /// Vetoes only when the car is MOVING toward trouble, so a stopped car
    /// never gets stuck in a permanent veto.
    private func collisionCheck(position p: Vec2, velocity v: Vec2,
                                obstacles: [ObstacleObservation],
                                occupied: [Vec2],
                                settings s: Settings) -> String? {
        guard v.length > 3 else { return nil }   // cm/s
        let predicted = p + v * s.vetoHorizon
        func approaching(_ q: Vec2) -> Bool { v.dot((q - p).normalized) > 0 }

        let markerClearance = s.arena.carRadius + s.arena.obstacleRadius
        for o in obstacles where approaching(o.position) {
            if predicted.distance(to: o.position) < markerClearance {
                return "Obstacle \(o.id) ahead"
            }
        }

        let cellClearance = s.arena.carRadius + GridSpec.cellSize / 2
        for c in occupied where approaching(c) {
            if predicted.distance(to: c) < cellClearance {
                return "Something in the way"
            }
        }

        func outside(_ q: Vec2) -> Double {
            max(0, -q.x, -q.y, q.x - s.arena.width, q.y - s.arena.height)
        }
        let margin = s.arena.carRadius * 0.5
        if outside(predicted) > margin && outside(predicted) > outside(p) {
            return "Leaving arena"
        }
        return nil
    }
}
