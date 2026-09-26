import Foundation

// Add this file to BOTH the iOS (PhoneApp) and macOS (HubApp) targets.
//
// World frame: centimeters, origin at the center of corner marker ID 2.
// +x points from marker 2 toward marker 3, +y from marker 2 toward marker 5.
// Headings are radians, 0 = +x, counterclockwise positive (viewed from above).

struct Vec2: Codable, Equatable {
    var x: Double
    var y: Double

    init(_ x: Double, _ y: Double) {
        self.x = x
        self.y = y
    }

    static let zero = Vec2(0, 0)

    static func + (a: Vec2, b: Vec2) -> Vec2 { Vec2(a.x + b.x, a.y + b.y) }
    static func - (a: Vec2, b: Vec2) -> Vec2 { Vec2(a.x - b.x, a.y - b.y) }
    static func * (a: Vec2, s: Double) -> Vec2 { Vec2(a.x * s, a.y * s) }
    static func += (a: inout Vec2, b: Vec2) { a = a + b }

    var length: Double { (x * x + y * y).squareRoot() }
    var normalized: Vec2 {
        let l = length
        return l > 1e-9 ? self * (1 / l) : .zero
    }
    func dot(_ o: Vec2) -> Double { x * o.x + y * o.y }
    func distance(to o: Vec2) -> Double { (self - o).length }
    func rotated(by a: Double) -> Vec2 {
        Vec2(x * cos(a) - y * sin(a), x * sin(a) + y * cos(a))
    }
}

struct Pose: Codable, Equatable {
    var position: Vec2
    var heading: Double
}

/// Physical layout of the arena. Must match on phones and hub.
struct ArenaConfig: Codable, Equatable {
    /// Center-to-center distance between corner markers 2 and 3 (cm).
    var width: Double = 200
    /// Center-to-center distance between corner markers 2 and 5 (cm).
    var height: Double = 150
    /// Rough radius of the car's footprint (cm).
    var carRadius: Double = 13
    /// Radius assumed for each obstacle marker's box (cm).
    var obstacleRadius: Double = 12
    /// Extra clearance the planner keeps around obstacles (cm).
    var safetyMargin: Double = 5
}

func wrapAngle(_ a: Double) -> Double {
    var r = fmod(a + .pi, 2 * .pi)
    if r < 0 { r += 2 * .pi }
    return r - .pi
}

func clamp<T: Comparable>(_ v: T, _ lo: T, _ hi: T) -> T {
    min(max(v, lo), hi)
}
