import Foundation

struct CameraStatus: Identifiable {
    let id: String
    var age: Double
    var calibrated: Bool
    var seesCar: Bool
    var fps: Double
    var veto: Bool
    var vetoReason: String?
    var hasGrid: Bool
    var gridMismatch: Bool
}

struct FusedObstacle: Identifiable {
    let id: Int
    var position: Vec2
}

struct WorldState {
    var car: Pose?
    var carSources = 0
    var goal: Vec2?
    var obstacles: [FusedObstacle] = []
    var veto = false
    var vetoReasons: [String] = []
    /// Fused markerless occupancy (GridSpec layout), nil if no camera sends a grid.
    var occupancy: [Bool]?
    var gridCameras = 0
}

/// Combines the latest observation from each camera into one world estimate.
/// - Car pose: confidence-weighted average; with 3+ cameras, outliers far
///   from the median are dropped first (the "voting" step).
/// - Obstacles: union across cameras, averaged per marker ID.
/// - Veto: ANY fresh camera veto stops the car.
/// - Markerless occupancy: with `requireAgreement`, a cell is occupied only if
///   EVERY camera that can see it says so. This trims the "smear" a tall object
///   leaves behind itself when projected onto the floor from an angle.
final class Fusion {
    var staleAfter: TimeInterval = 0.4
    var outlierDistance = 15.0   // cm

    private var latest: [String: (msg: ObservationMessage, received: Date)] = [:]

    func ingest(_ msg: ObservationMessage, at time: Date) {
        latest[msg.cameraId] = (msg, time)
    }

    func fuse(now: Date, arena: ArenaConfig, requireAgreement: Bool) -> WorldState {
        let fresh = latest.values
            .filter { now.timeIntervalSince($0.received) < staleAfter && $0.msg.calibrated }
            .map(\.msg)

        var ws = WorldState()

        // Car pose
        var cars: [(pose: Pose, w: Double)] = fresh.compactMap { m in
            guard let c = m.car, m.carConfidence > 0 else { return nil }
            return (pose: c, w: m.carConfidence)
        }
        if cars.count >= 3 {
            let med = Vec2(median(cars.map { $0.pose.position.x }),
                           median(cars.map { $0.pose.position.y }))
            let kept = cars.filter { $0.pose.position.distance(to: med) <= outlierDistance }
            if !kept.isEmpty { cars = kept }
        }
        let wsum = cars.reduce(0.0) { $0 + $1.w }
        if wsum > 0 {
            var p = Vec2.zero
            var s = 0.0, c = 0.0
            for e in cars {
                p += e.pose.position * e.w
                s += sin(e.pose.heading) * e.w     // circular mean for angles
                c += cos(e.pose.heading) * e.w
            }
            ws.car = Pose(position: p * (1 / wsum), heading: atan2(s, c))
            ws.carSources = cars.count
        }

        // Goal
        let goals = fresh.compactMap(\.goal)
        if !goals.isEmpty { ws.goal = average(goals) }

        // Obstacles
        var byId: [Int: [Vec2]] = [:]
        for m in fresh {
            for o in m.obstacles { byId[o.id, default: []].append(o.position) }
        }
        ws.obstacles = byId
            .map { FusedObstacle(id: $0.key, position: average($0.value)) }
            .sorted { $0.id < $1.id }

        // Markerless occupancy
        let (cols, rows) = GridSpec.dims(arena)
        let n = cols * rows
        var seen = [Int](repeating: 0, count: n)
        var occ = [Int](repeating: 0, count: n)
        for m in fresh {
            guard let g = m.grid, g.cols == cols, g.rows == rows,
                  let o = GridMessage.unpack(g.occupied, count: n),
                  let v = GridMessage.unpack(g.visible, count: n) else { continue }
            ws.gridCameras += 1
            for i in 0..<n where v[i] {
                seen[i] += 1
                if o[i] { occ[i] += 1 }
            }
        }
        if ws.gridCameras > 0 {
            ws.occupancy = (0..<n).map { i in
                requireAgreement ? (occ[i] > 0 && occ[i] == seen[i]) : occ[i] > 0
            }
        }

        // Safety veto
        for m in fresh where m.veto {
            ws.veto = true
            ws.vetoReasons.append("\(m.cameraId): \(m.vetoReason ?? "veto")")
        }
        return ws
    }

    func statuses(now: Date, arena: ArenaConfig) -> [CameraStatus] {
        let (cols, rows) = GridSpec.dims(arena)
        return latest.map { key, value in
            CameraStatus(id: key,
                         age: now.timeIntervalSince(value.received),
                         calibrated: value.msg.calibrated,
                         seesCar: value.msg.car != nil,
                         fps: value.msg.fps,
                         veto: value.msg.veto,
                         vetoReason: value.msg.vetoReason,
                         hasGrid: value.msg.grid != nil,
                         gridMismatch: value.msg.grid.map { $0.cols != cols || $0.rows != rows } ?? false)
        }
        .sorted { $0.id < $1.id }
    }

    private func median(_ v: [Double]) -> Double {
        let s = v.sorted()
        let n = s.count
        return n % 2 == 1 ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2
    }

    private func average(_ pts: [Vec2]) -> Vec2 {
        var sum = Vec2.zero
        for p in pts { sum += p }
        return sum * (1 / Double(pts.count))
    }
}
