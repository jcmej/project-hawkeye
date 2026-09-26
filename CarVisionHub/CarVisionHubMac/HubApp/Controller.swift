import Foundation

/// Path follower for a mecanum (holonomic) car: it strafes toward a
/// lookahead point on the path while holding a fixed heading.
final class PursuitController {
    var speedScale = 0.6          // overall speed, 0...1 (start slow!)
    var minSpeedFraction = 0.35   // don't crawl too slowly near the goal
    var lookahead = 15.0          // cm
    var goalTolerance = 5.0       // cm
    var slowRadius = 30.0         // start slowing within this distance of goal
    var headingGain = 1.2
    var maxOmega = 0.5

    /// Returns the command and whether the goal has been reached.
    func command(pose: Pose, path: [Vec2], goal: Vec2, holdHeading: Double) -> (DriveCommand, Bool) {
        let dGoal = pose.position.distance(to: goal)
        if dGoal < goalTolerance { return (.stop, true) }

        let target = lookaheadPoint(on: path, from: pose.position)
        let dir = (target - pose.position).normalized
        let frac = clamp(dGoal / slowRadius, minSpeedFraction, 1.0)
        let vWorld = dir * (frac * speedScale)

        // World-frame velocity -> car frame (vx forward, vy left).
        let vCar = vWorld.rotated(by: -pose.heading)
        let omega = clamp(headingGain * wrapAngle(holdHeading - pose.heading), -maxOmega, maxOmega)
        return (DriveCommand(vx: vCar.x, vy: vCar.y, omega: omega), false)
    }

    func lookaheadPoint(on path: [Vec2], from p: Vec2) -> Vec2 {
        guard path.count >= 2 else { return path.last ?? p }

        // Closest point on the polyline.
        var bestSeg = 0, bestT = 0.0, bestD = Double.infinity
        for i in 0..<(path.count - 1) {
            let a = path[i], ab = path[i + 1] - path[i]
            let len2 = ab.dot(ab)
            let t = len2 > 1e-9 ? clamp((p - a).dot(ab) / len2, 0, 1) : 0
            let d = (a + ab * t).distance(to: p)
            if d < bestD { bestD = d; bestSeg = i; bestT = t }
        }

        // Walk forward `lookahead` cm along the path from there.
        var remaining = lookahead
        var a = path[bestSeg] + (path[bestSeg + 1] - path[bestSeg]) * bestT
        var i = bestSeg + 1
        while i < path.count {
            let seg = path[i].distance(to: a)
            if seg >= remaining { return a + (path[i] - a).normalized * remaining }
            remaining -= seg
            a = path[i]
            i += 1
        }
        return path[path.count - 1]
    }
}
