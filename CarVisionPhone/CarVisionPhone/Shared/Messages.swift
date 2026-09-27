import Foundation

// Add this file to BOTH targets.

/// ArUco dictionary: DICT_4X4_50. Print markers from this dictionary only.
enum MarkerIDs {
    static let car = 0
    static let goal = 1
    /// Arena corners, placed counterclockwise when viewed from above:
    /// 2 = (0,0), 3 = (width,0), 4 = (width,height), 5 = (0,height)
    static let corners = [2, 3, 4, 5]
    /// Each marked obstacle gets its own marker taped on top. Kept to the IDs
    /// we actually print: textured floors can produce false matches for any ID.
    static let obstacles = 10...15

    static func cornerWorldPositions(_ a: ArenaConfig) -> [Vec2] {
        [Vec2(0, 0), Vec2(a.width, 0), Vec2(a.width, a.height), Vec2(0, a.height)]
    }
}

enum NetConfig {
    static let bonjourType = "_carvision._udp"
    /// Port the hub listens on for phone observations.
    static let hubPort: UInt16 = 47800
    /// Port the real car link sends to (placeholder until the car protocol is decided).
    static let carPort: UInt16 = 47801
}

/// Occupancy grid layout shared by phones and hub. Cell index = row * cols + col,
/// with row 0 at y = 0 (next to corner markers 2 and 3).
enum GridSpec {
    static let cellSize = 5.0        // cm per cell
    static let pixelsPerCell = 5     // phone's top-down image: 1 px per cm

    static func dims(_ a: ArenaConfig) -> (cols: Int, rows: Int) {
        (max(1, Int(ceil(a.width / cellSize))), max(1, Int(ceil(a.height / cellSize))))
    }

    static func center(_ i: Int, cols: Int) -> Vec2 {
        Vec2((Double(i % cols) + 0.5) * cellSize, (Double(i / cols) + 0.5) * cellSize)
    }
}

/// Markerless obstacle grid from one camera, as base64 bitmasks.
struct GridMessage: Codable {
    var cols: Int
    var rows: Int
    /// Cells where something changed compared to the empty-arena background.
    var occupied: String
    /// Cells this camera can actually see (used for multi-camera agreement).
    var visible: String

    static func pack(_ bits: [Bool]) -> String {
        var bytes = [UInt8](repeating: 0, count: (bits.count + 7) / 8)
        for (i, b) in bits.enumerated() where b {
            bytes[i / 8] |= UInt8(1 << (i % 8))
        }
        return Data(bytes).base64EncodedString()
    }

    static func unpack(_ s: String, count: Int) -> [Bool]? {
        guard let data = Data(base64Encoded: s), data.count * 8 >= count else { return nil }
        let bytes = [UInt8](data)
        return (0..<count).map { bytes[$0 / 8] & UInt8(1 << ($0 % 8)) != 0 }
    }
}

struct ObstacleObservation: Codable, Equatable {
    var id: Int
    var position: Vec2
}

/// Nearest obstacle to the car, measured from the car's learned outline (LiDAR phones).
struct ClearanceMessage: Codable {
    /// cm from the car's outline to the nearest obstacle.
    var distance: Double
    /// Direction relative to the car: "front", "front-left", "left", ... "front-right".
    var bearing: String
    /// The obstacle point, in arena cm.
    var point: Vec2
    /// Seconds until contact at the current velocity, if the car is closing in.
    var timeToCollision: Double?

    /// `degrees` = angle in the car's frame, 0 = straight ahead, positive = to the left.
    static func bearingLabel(_ degrees: Double) -> String {
        let labels = ["front", "front-left", "left", "back-left", "back", "back-right", "right", "front-right"]
        var d = degrees.truncatingRemainder(dividingBy: 360)
        if d < 0 { d += 360 }
        return labels[Int((d + 22.5) / 45) % 8]
    }
}

/// Sent by each phone ~30 times per second as a single UDP datagram of JSON.
struct ObservationMessage: Codable {
    var type = "obs"
    var cameraId: String
    var seq: Int
    /// True once this phone has seen all four corner markers.
    var calibrated: Bool
    var car: Pose?
    /// 0...1, based on how large the car marker appears in this camera.
    var carConfidence: Double
    var goal: Vec2?
    var obstacles: [ObstacleObservation]
    /// Phone-side safety check: true if this camera predicts a collision.
    var veto: Bool
    var vetoReason: String?
    var fps: Double
    /// Present once this phone has captured a background.
    var grid: GridMessage? = nil
    /// Nearest obstacle to the car (LiDAR phones with a learned car shape).
    var clearance: ClearanceMessage? = nil
}

/// Car-frame velocity command, each component in -1...1.
/// vx = forward, vy = left (strafe), omega = counterclockwise rotation.
struct DriveCommand: Codable, Equatable {
    var vx: Double
    var vy: Double
    var omega: Double

    static let stop = DriveCommand(vx: 0, vy: 0, omega: 0)
}
