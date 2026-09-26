import Foundation

/// Main control loop, 20 Hz on the main thread:
/// fuse observations -> check veto -> plan (5 Hz) -> control -> send command.
final class HubModel: ObservableObject {
    @Published var arena = ArenaConfig()
    @Published var world = WorldState()
    @Published var cameras: [CameraStatus] = []
    @Published var path: [Vec2] = []
    @Published private(set) var autonomy = false
    @Published var useMockCar = true
    @Published var goalOverride: Vec2?
    @Published var manualObstacles: [Vec2] = []
    @Published var status = "Idle"
    @Published var networkStatus = "Starting…"
    @Published var mockPose: Pose
    @Published var lastCommand = DriveCommand.stop
    @Published var realCarHost = ""
    @Published var useOccupancy = true
    @Published var requireAgreement = true
    /// Centers of fused, masked occupied cells (for drawing).
    @Published var occupiedCenters: [Vec2] = []

    let controller = PursuitController()
    private let fusion = Fusion()
    private let planner = Planner()
    private let receiver = ObservationReceiver()
    private let mockCar: MockCarLink
    private var realCar: UDPCarLink?
    private var timer: Timer?
    private var lastTick = Date()
    private var lastPlan = Date.distantPast
    private var heldHeading: Double?

    private static let mockStart = Pose(position: Vec2(30, 30), heading: 0)

    init() {
        mockCar = MockCarLink(start: Self.mockStart)
        mockPose = Self.mockStart

        receiver.onObservation = { [weak self] msg in
            let t = Date()
            DispatchQueue.main.async { self?.fusion.ingest(msg, at: t) }
        }
        receiver.onStatus = { [weak self] s in
            DispatchQueue.main.async { self?.networkStatus = s }
        }
        do {
            try receiver.start()
        } catch {
            networkStatus = "Listener failed: \(error.localizedDescription)"
        }

        let t = Timer(timeInterval: 0.05, repeats: true) { [weak self] _ in self?.tick() }
        RunLoop.main.add(t, forMode: .common)   // keeps ticking during window drags
        timer = t
    }

    /// Obstacles the planner sees: from cameras plus any added by ⌥-click.
    var allObstacles: [Vec2] { world.obstacles.map(\.position) + manualObstacles }
    var activeGoal: Vec2? { goalOverride ?? world.goal }
    var carPose: Pose? { useMockCar ? mockPose : world.car }

    // MARK: - Commands

    func startAutonomy() {
        heldHeading = nil
        lastPlan = .distantPast
        autonomy = true
        status = "Starting…"
    }

    func stopAll() {
        autonomy = false
        activeLink().send(.stop)
        lastCommand = .stop
        status = "Stopped"
    }

    func resetMockCar() {
        mockCar.reset(to: Self.mockStart)
        mockPose = Self.mockStart
    }

    // MARK: - Loop

    private func tick() {
        let now = Date()
        let dt = min(now.timeIntervalSince(lastTick), 0.1)
        lastTick = now

        if useMockCar {
            mockCar.step(dt: dt)
            mockPose = mockCar.pose
        }
        world = fusion.fuse(now: now, arena: arena, requireAgreement: requireAgreement)
        cameras = fusion.statuses(now: now, arena: arena)
        let occupancy = maskedOccupancy()
        let cols = GridSpec.dims(arena).cols
        occupiedCenters = occupancy.map { o in
            o.indices.filter { o[$0] }.map { GridSpec.center($0, cols: cols) }
        } ?? []

        guard autonomy else { return }
        let link = activeLink()

        guard let pose = carPose else { halt(link, "Car not visible"); return }
        guard let goal = activeGoal else { halt(link, "No goal"); return }
        if world.veto {
            halt(link, "VETO — " + world.vetoReasons.joined(separator: "; "))
            lastPlan = .distantPast   // replan as soon as the veto clears
            return
        }

        if now.timeIntervalSince(lastPlan) > 0.2 {
            path = planner.plan(from: pose.position, to: goal, obstacles: allObstacles,
                                occupied: occupancy, arena: arena) ?? []
            lastPlan = now
        }
        guard !path.isEmpty else { halt(link, "No path to goal (blocked?)"); return }

        if heldHeading == nil { heldHeading = pose.heading }
        let (cmd, reached) = controller.command(pose: pose, path: path, goal: goal,
                                                holdHeading: heldHeading ?? pose.heading)
        if reached {
            halt(link, "Goal reached ✓")
            autonomy = false
            return
        }
        link.send(cmd)
        lastCommand = cmd
        status = "Driving"
    }

    /// Fused occupancy with the car and goal areas cleared (the car itself and
    /// the goal marker always show up as "changes").
    private func maskedOccupancy() -> [Bool]? {
        guard useOccupancy, var occ = world.occupancy else { return nil }
        let cols = GridSpec.dims(arena).cols
        var masks: [(Vec2, Double)] = []
        if let p = carPose { masks.append((p.position, arena.carRadius + 8)) }
        if let g = activeGoal { masks.append((g, 15)) }
        if !masks.isEmpty {
            for i in occ.indices where occ[i] {
                let c = GridSpec.center(i, cols: cols)
                if masks.contains(where: { c.distance(to: $0.0) < $0.1 }) { occ[i] = false }
            }
        }
        return occ
    }

    private func halt(_ link: CarLink, _ message: String) {
        link.send(.stop)
        lastCommand = .stop
        status = message
    }

    private func activeLink() -> CarLink {
        if useMockCar { return mockCar }
        if realCar == nil || realCar?.host != realCarHost {
            realCar = UDPCarLink(host: realCarHost)
        }
        return realCar!
    }
}
