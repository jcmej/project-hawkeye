import SwiftUI
import AppKit

@main
struct CarVisionHubApp: App {
    @StateObject private var model = HubModel()

    var body: some Scene {
        WindowGroup {
            HubView()
                .environmentObject(model)
                .frame(minWidth: 950, minHeight: 620)
        }
    }
}

struct HubView: View {
    @EnvironmentObject var model: HubModel

    var body: some View {
        HStack(spacing: 0) {
            map
            Divider()
            ScrollView { sidebar.padding() }
                .frame(width: 320)
        }
    }

    // MARK: - Map

    private var map: some View {
        GeometryReader { geo in
            Canvas { ctx, size in
                let t = ArenaTransform(arena: model.arena, size: size)
                ArenaRenderer.draw(&ctx, t,
                                   car: model.carPose,
                                   ghostCar: model.useMockCar ? model.world.car : nil,
                                   goal: model.activeGoal,
                                   obstacles: model.allObstacles,
                                   path: model.path,
                                   occupied: model.occupiedCenters,
                                   nearestObstacle: model.world.clearance?.point,
                                   showInflation: true)
            }
            .contentShape(Rectangle())
            .onTapGesture(coordinateSpace: .local) { loc in
                let w = ArenaTransform(arena: model.arena, size: geo.size).toWorld(loc)
                if NSEvent.modifierFlags.contains(.option) {
                    model.manualObstacles.append(w)
                } else {
                    model.goalOverride = w
                }
            }
        }
        .background(Color(nsColor: .textBackgroundColor))
    }

    // MARK: - Sidebar

    private var sidebar: some View {
        VStack(alignment: .leading, spacing: 14) {
            Text(model.status)
                .font(.title3.bold())
                .foregroundColor(model.status.hasPrefix("VETO") ? .red : .primary)

            HStack {
                Button("Start") { model.startAutonomy() }
                    .buttonStyle(.borderedProminent)
                    .disabled(model.autonomy)
                Button("STOP (space)") { model.stopAll() }
                    .buttonStyle(.borderedProminent)
                    .tint(.red)
                    .keyboardShortcut(.space, modifiers: [])
            }

            clearanceBox

            GroupBox("Car") {
                VStack(alignment: .leading, spacing: 6) {
                    Toggle("Use mock car", isOn: $model.useMockCar)
                        .disabled(model.autonomy)
                    if !model.useMockCar {
                        TextField("Car IP (placeholder link)", text: $model.realCarHost)
                    }
                    if let p = model.carPose {
                        Text(String(format: "Pose (%.0f, %.0f) cm  %.0f°",
                                    p.position.x, p.position.y, p.heading * 180 / .pi))
                            .font(.caption.monospaced())
                    } else {
                        Text("Car not visible").font(.caption).foregroundColor(.secondary)
                    }
                    Text(String(format: "Cmd vx %.2f  vy %.2f  ω %.2f",
                                model.lastCommand.vx, model.lastCommand.vy, model.lastCommand.omega))
                        .font(.caption.monospaced())
                    HStack {
                        Text("Speed")
                        Slider(value: Binding(get: { model.controller.speedScale },
                                              set: { model.controller.speedScale = $0 }),
                               in: 0.2...1.0)
                    }
                    Button("Reset mock car") { model.resetMockCar() }
                        .disabled(!model.useMockCar)
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }

            GroupBox("Arena (must match phones)") {
                VStack(alignment: .leading, spacing: 6) {
                    numberRow("Width 2→3 (cm)", $model.arena.width)
                    numberRow("Height 2→5 (cm)", $model.arena.height)
                    numberRow("Car radius (cm)", $model.arena.carRadius)
                    numberRow("Obstacle radius (cm)", $model.arena.obstacleRadius)
                    numberRow("Safety margin (cm)", $model.arena.safetyMargin)
                }
            }

            GroupBox("Markerless obstacles") {
                VStack(alignment: .leading, spacing: 6) {
                    Toggle("Use camera change detection", isOn: $model.useOccupancy)
                    Toggle("Require all cameras to agree", isOn: $model.requireAgreement)
                        .disabled(!model.useOccupancy)
                    Text("\(model.world.gridCameras) camera(s) sending grids · \(model.occupiedCenters.count) cells occupied")
                        .font(.caption).foregroundColor(.secondary)
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }

            GroupBox("Map editing") {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Click: set goal override\n⌥-click: add test obstacle")
                        .font(.caption).foregroundColor(.secondary)
                    HStack {
                        Button("Clear goal") { model.goalOverride = nil }
                        Button("Clear obstacles") { model.manualObstacles.removeAll() }
                    }
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }

            GroupBox("Cameras") {
                VStack(alignment: .leading, spacing: 6) {
                    Text(model.networkStatus).font(.caption).foregroundColor(.secondary)
                    if model.cameras.isEmpty {
                        Text("No phones connected yet").font(.caption)
                    }
                    ForEach(model.cameras) { cam in
                        cameraRow(cam)
                    }
                    Text("Car fused from \(model.world.carSources) camera(s)")
                        .font(.caption).foregroundColor(.secondary)
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
    }

    private var clearanceBox: some View {
        let c = model.world.clearance
        let color: Color = c.map { $0.distance < 8 ? .red : ($0.distance < 20 ? .orange : .green) } ?? .secondary
        return GroupBox("Nearest obstacle to car") {
            VStack(alignment: .leading, spacing: 4) {
                if let c {
                    Text(String(format: "%.0f cm %@", c.distance, c.bearing))
                        .font(.title2.bold().monospacedDigit())
                        .foregroundColor(color)
                    if let ttc = c.timeToCollision {
                        Text(String(format: "Contact in %.1f s at current speed", ttc))
                            .font(.caption).foregroundColor(ttc < 1 ? .red : .secondary)
                    }
                } else {
                    Text("Nothing within 1 m, or no LiDAR phone with a learned car shape")
                        .font(.caption).foregroundColor(.secondary)
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    private func cameraRow(_ cam: CameraStatus) -> some View {
        let stale = cam.age > 0.4
        return VStack(alignment: .leading, spacing: 2) {
            HStack {
                Circle().fill(stale ? .gray : (cam.veto ? .red : .green)).frame(width: 8, height: 8)
                Text(cam.id).bold()
                Spacer()
                Text("\(Int(cam.fps)) fps").font(.caption)
            }
            Text("\(cam.calibrated ? "calibrated" : "NOT calibrated") · \(cam.seesCar ? "sees car" : "no car")"
                 + (cam.hasGrid ? " · grid" : " · no BG")
                 + (stale ? " · stale" : ""))
                .font(.caption).foregroundColor(.secondary)
            if cam.gridMismatch {
                Text("grid size mismatch: arena settings differ from hub")
                    .font(.caption).foregroundColor(.orange)
            }
            if cam.veto, let r = cam.vetoReason {
                Text("veto: \(r)").font(.caption).foregroundColor(.red)
            }
        }
    }

    private func numberRow(_ title: String, _ value: Binding<Double>) -> some View {
        HStack {
            Text(title)
            Spacer()
            TextField("", value: value, format: .number)
                .frame(width: 70)
                .multilineTextAlignment(.trailing)
        }
    }
}
