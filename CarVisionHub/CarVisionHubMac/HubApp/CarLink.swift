import Foundation
import Network

/// Everything the hub needs from a car. Swap implementations without
/// touching fusion, planning, or control.
protocol CarLink: AnyObject {
    var name: String { get }
    func send(_ cmd: DriveCommand)
}

/// Simulated mecanum car with response lag, weaker strafing, and noise,
/// so the controller gets tested against something imperfect.
final class MockCarLink: CarLink {
    let name = "Mock car"
    private(set) var pose: Pose
    private var commanded = DriveCommand.stop
    private var actual = DriveCommand.stop

    var topSpeed = 30.0          // cm/s at |command| = 1
    var topTurnRate = 1.8        // rad/s at omega = 1
    var responseTime = 0.15      // s, first-order lag
    var strafeEfficiency = 0.8   // mecanum strafing is less efficient
    var positionNoise = 1.5      // cm/s of random jitter while moving

    init(start: Pose) { pose = start }

    func send(_ cmd: DriveCommand) { commanded = cmd }

    func step(dt: Double) {
        guard dt > 0 else { return }
        let a = min(1, dt / responseTime)
        actual.vx += (commanded.vx - actual.vx) * a
        actual.vy += (commanded.vy - actual.vy) * a
        actual.omega += (commanded.omega - actual.omega) * a

        let vCar = Vec2(actual.vx * topSpeed, actual.vy * topSpeed * strafeEfficiency)
        let moving = vCar.length > 0.5
        var vWorld = vCar.rotated(by: pose.heading)
        if moving {
            vWorld += Vec2(Double.random(in: -positionNoise...positionNoise),
                           Double.random(in: -positionNoise...positionNoise))
        }
        pose.position += vWorld * dt
        let yawNoise = moving ? Double.random(in: -0.02...0.02) : 0
        pose.heading = wrapAngle(pose.heading + actual.omega * topTurnRate * dt + yawNoise)
    }

    func reset(to p: Pose) {
        pose = p
        commanded = .stop
        actual = .stop
    }
}

/// PLACEHOLDER link to the real Zeus Car. Sends "vx,vy,omega\n" over UDP.
/// TODO(real car): replace send() with the car's actual protocol once the
/// ESP32/Uno firmware format is decided. The car firmware should also stop
/// the motors if no command arrives for ~300 ms (the hub sends at 20 Hz).
final class UDPCarLink: CarLink {
    let host: String
    var name: String { "UDP \(host):\(NetConfig.carPort)" }
    private let connection: NWConnection?

    init(host: String) {
        self.host = host
        if let port = NWEndpoint.Port(rawValue: NetConfig.carPort), !host.isEmpty {
            let c = NWConnection(host: NWEndpoint.Host(host), port: port, using: .udp)
            c.start(queue: .global(qos: .userInitiated))
            connection = c
        } else {
            connection = nil
        }
    }

    func send(_ cmd: DriveCommand) {
        let line = String(format: "%.3f,%.3f,%.3f\n", cmd.vx, cmd.vy, cmd.omega)
        connection?.send(content: line.data(using: .utf8), completion: .contentProcessed { _ in })
    }

    deinit { connection?.cancel() }
}
