import SwiftUI
import AVFoundation
import UIKit
import ARKit

@main
struct CarVisionPhoneApp: App {
    var body: some Scene {
        WindowGroup { ContentView() }
    }
}

struct ContentView: View {
    @StateObject private var model = PhoneModel()
    @AppStorage("cameraId") private var cameraId = ""
    @AppStorage("arenaWidth") private var arenaWidth = 200.0
    @AppStorage("arenaHeight") private var arenaHeight = 150.0
    @AppStorage("carRadius") private var carRadius = 13.0
    @AppStorage("obstacleRadius") private var obstacleRadius = 12.0
    @AppStorage("manualHost") private var manualHost = ""
    @AppStorage("zoomFactor") private var savedZoom = 0.0      // device units; 0 = default
    @AppStorage("previewFill") private var fillScreen = true
    @AppStorage("useAR") private var useAR = true
    @AppStorage("carMarkerHeight") private var carMarkerHeight = 10.0   // cm above the floor
    @State private var showSettings = false
    @State private var showMap = true
    @State private var pinchBase: CGFloat?

    private var arena: ArenaConfig {
        var a = ArenaConfig()
        a.width = arenaWidth
        a.height = arenaHeight
        a.carRadius = carRadius
        a.obstacleRadius = obstacleRadius
        return a
    }

    private var settings: Perception.Settings {
        .init(cameraId: cameraId, arena: arena)
    }

    var body: some View {
        ZStack {
            if let ar = model.ar {
                ARPreview(view: ar.sceneView)
                    .ignoresSafeArea()
                    .contentShape(Rectangle())
                    .onTapGesture(coordinateSpace: .local) { point in
                        model.placeGoal(atViewPoint: point)   // only acts in "Tap to place goal" mode
                    }
            } else if let camera = model.camera {
                CameraPreview(session: camera.session, fill: fillScreen)
                    .ignoresSafeArea()
                    .contentShape(Rectangle())
                    .gesture(pinchToZoom)
                    .onTapGesture(count: 2) { fillScreen.toggle() }
            }

            overlays
        }
        .background(Color.black)
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
        .sheet(isPresented: $showSettings) { settingsSheet }
        .onAppear {
            UIApplication.shared.isIdleTimerDisabled = true   // keep screen awake
            if cameraId.isEmpty { cameraId = "cam-\(Int.random(in: 100...999))" }
            model.apply(settings: settings, manualHost: manualHost, carMarkerHeightCm: carMarkerHeight)
            model.start(manualHost: manualHost, zoom: savedZoom > 0 ? CGFloat(savedZoom) : nil)
        }
        .onChange(of: model.zoomFactor) { _, z in savedZoom = Double(z) }
    }

    // MARK: - Zoom

    private var pinchToZoom: some Gesture {
        MagnifyGesture()
            .onChanged { value in
                let base = pinchBase ?? model.zoomFactor
                if pinchBase == nil { pinchBase = base }
                model.setZoom(base * value.magnification)
            }
            .onEnded { _ in
                pinchBase = nil
                model.zoomDidSettle()
            }
    }

    private var zoomControls: some View {
        let oneX = model.zoomInfo.oneX
        let current = model.zoomFactor / oneX
        let range = model.zoomInfo.min...model.zoomInfo.max
        let presets: [CGFloat] = [0.5, 1, 2, 3].filter { range.contains($0 * oneX) }
        return VStack(spacing: 8) {
            ForEach(presets, id: \.self) { p in
                let selected = abs(current - p) < 0.05
                Button(p < 1 ? "0.5×" : "\(Int(p))×") {
                    model.setZoom(p * oneX)
                    model.zoomDidSettle()
                }
                .font(.caption.bold())
                .frame(width: 44, height: 44)
                .foregroundColor(selected ? .black : .white)
                .background(selected ? Color.yellow : Color.black.opacity(0.55), in: Circle())
            }
            Text(String(format: "%.1f×", current))
                .font(.caption.monospaced())
                .foregroundColor(.white)
                .padding(.horizontal, 8).padding(.vertical, 4)
                .background(Color.black.opacity(0.55), in: Capsule())
        }
    }

    // MARK: - AR calibration

    private var arCalibrationPanel: some View {
        let st = model.arStatus
        return VStack(alignment: .leading, spacing: 6) {
            HStack {
                Image(systemName: st.trackingNormal ? "location.fill" : "location.slash")
                Text(st.tracking)
            }
            .font(.caption.bold())
            .foregroundColor(st.trackingNormal ? .green : .orange)

            if !st.floorFound {
                Text("Step 1: Point at the floor and move the phone slowly until it's found.")
                    .font(.caption)
            } else if !st.complete {
                Text("Step 2: Walk to each corner marker and point the phone at it from close up.")
                    .font(.caption)
                HStack(spacing: 6) {
                    ForEach(MarkerIDs.corners, id: \.self) { id in
                        cornerChip(id, st)
                    }
                }
            } else {
                Text("Arena pinned ✓ Step 3: mount the phone. Corners can be off-screen now.")
                    .font(.caption.bold())
                    .foregroundColor(.green)
            }

            if st.floorFound {
                goalRow(st)
            }

            if let w = st.measuredWidth, let h = st.measuredHeight {
                let off = abs(w - arenaWidth) / arenaWidth > 0.05 || abs(h - arenaHeight) / arenaHeight > 0.05
                HStack {
                    Text(String(format: "Measured %.0f × %.0f cm", w, h))
                    if off {
                        Button("Use") {
                            arenaWidth = (w).rounded()
                            arenaHeight = (h).rounded()
                            model.apply(settings: settings, manualHost: manualHost, carMarkerHeightCm: carMarkerHeight)
                            model.settingsChanged()
                        }
                        .font(.caption.bold())
                    }
                }
                .font(.caption)
                .foregroundColor(off ? .orange : .secondary)
                if off {
                    Text("Settings say \(Int(arenaWidth)) × \(Int(arenaHeight)). Update here AND on the hub.")
                        .font(.caption2).foregroundColor(.orange)
                }
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private func goalRow(_ st: ARCalibrationStatus) -> some View {
        HStack(spacing: 8) {
            if model.placingGoal {
                Text("Tap the floor where the goal should go")
                    .font(.caption.bold()).foregroundColor(.yellow)
                Spacer()
                Button("Cancel") { model.placingGoal = false }
                    .font(.caption.bold())
            } else {
                if st.goalPinned {
                    Text("Goal pinned ✓").font(.caption.bold()).foregroundColor(.green)
                } else if let n = st.progress[MarkerIDs.goal] {
                    Text("Goal: scanning \(n * 100 / 12)%").font(.caption.bold()).foregroundColor(.yellow)
                } else {
                    Text("Goal: scan the goal marker up close, or")
                        .font(.caption)
                }
                Spacer()
                Button(st.goalPinned ? "Move goal" : "Tap to place") { model.placingGoal = true }
                    .font(.caption.bold())
                    .buttonStyle(.bordered)
            }
        }
    }

    private func cornerChip(_ id: Int, _ st: ARCalibrationStatus) -> some View {
        let done = st.captured.contains(id)
        let progress = st.progress[id]
        let label = done ? "\(id) ✓" : (progress.map { "\(id) \($0 * 100 / 12)%" } ?? "\(id)")
        return Text(label)
            .font(.caption.bold())
            .padding(.horizontal, 8).padding(.vertical, 4)
            .background(done ? Color.green : (progress != nil ? Color.yellow : Color.gray.opacity(0.5)),
                        in: Capsule())
            .foregroundColor(done || progress != nil ? .black : .white)
    }

    // MARK: - Overlays

    private var overlays: some View {
        VStack(spacing: 10) {
            statusPanel
                .padding(10)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))

            if model.arMode {
                arCalibrationPanel
                    .padding(10)
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
            }

            if model.debug.veto {
                Text("VETO: \(model.debug.vetoReason ?? "")")
                    .font(.headline).padding(8)
                    .frame(maxWidth: .infinity)
                    .background(.red, in: RoundedRectangle(cornerRadius: 10))
                    .foregroundColor(.white)
            }

            Spacer()

            HStack(alignment: .bottom) {
                if showMap {
                    Canvas { ctx, size in
                        let t = ArenaTransform(arena: arena, size: size)
                        ArenaRenderer.draw(&ctx, t, car: model.debug.car, goal: model.debug.goal,
                                           obstacles: model.debug.obstacles,
                                           occupied: model.debug.occupied)
                    }
                    .frame(width: 170, height: 130)
                    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
                    .onTapGesture { showMap = false }
                }
                Spacer()
                if model.camera != nil { zoomControls }
            }

            HStack(spacing: 8) {
                Button(model.arMode ? "Reset corners" : "Recalibrate") { model.recalibrate() }
                if model.arMode {
                    Button("Reset goal") { model.resetGoal() }
                        .disabled(!model.arStatus.goalPinned && !model.placingGoal)
                }
                Button(model.debug.hasBackground ? "Recapture BG" : "Capture BG") {
                    model.captureBackground()
                }
                .disabled(!model.debug.calibrated)
                if !showMap {
                    Button("Map") { showMap = true }
                }
                Spacer()
                Button { showSettings = true } label: { Image(systemName: "gearshape") }
            }
            .font(.subheadline)
            .buttonStyle(.bordered)
            .tint(.white)
            .padding(8)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 14))
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
    }

    private var statusPanel: some View {
        let d = model.debug
        return VStack(alignment: .leading, spacing: 4) {
            if let err = model.cameraError {
                Text(err).foregroundColor(.red)
            }
            HStack {
                Text("\(cameraId) · \(model.connectionStatus)").lineLimit(1)
                Spacer()
                Text("\(Int(d.fps)) fps")
            }
            .font(.subheadline)
            Label(d.calibrated ? "Calibrated" : "Need corners 2–5 in view",
                  systemImage: d.calibrated ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                .font(.subheadline)
                .foregroundColor(d.calibrated ? .green : .orange)
            if let note = d.calibrationNote {
                Text(note).font(.caption.bold()).foregroundColor(.orange)
            }
            Group {
                if let note = d.backgroundNote {
                    Text("Obstacle detection: \(note)").foregroundColor(.orange)
                } else if d.hasBackground {
                    Text("Obstacle detection: on · \(d.occupied.count) cells occupied")
                        .foregroundColor(.green)
                } else {
                    Text("Obstacle detection: off (clear arena, then Capture BG)")
                        .foregroundColor(.secondary)
                }
            }
            .font(.caption)
            Text("Markers: \(d.markerIds.isEmpty ? "none" : d.markerIds.map(String.init).joined(separator: ", "))")
                .font(.caption).foregroundColor(.secondary)
            if let car = d.car {
                Text(String(format: "Car (%.0f, %.0f) cm  %.0f°  %.0f cm/s",
                            car.position.x, car.position.y, car.heading * 180 / .pi, d.carSpeed))
                    .font(.caption.monospaced())
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    // MARK: - Settings

    private var settingsSheet: some View {
        NavigationStack {
            Form {
                Section {
                    TextField("Camera ID", text: $cameraId)
                    Toggle("AR calibration", isOn: $useAR)
                    if model.arMode {
                        numberField("Car marker height (cm)", $carMarkerHeight)
                    } else {
                        Toggle("Fill screen (double-tap preview)", isOn: $fillScreen)
                    }
                } header: {
                    Text("This camera")
                } footer: {
                    Text("AR calibration pins corners in 3D so they can be off-screen, and corrects for the car marker's height. Classic mode supports zoom but needs all four corners in view. Restart the app after switching.")
                }
                Section("Arena (must match the hub)") {
                    numberField("Width, marker 2→3 (cm)", $arenaWidth)
                    numberField("Height, marker 2→5 (cm)", $arenaHeight)
                    numberField("Car radius (cm)", $carRadius)
                    numberField("Obstacle radius (cm)", $obstacleRadius)
                }
                Section {
                    TextField("Hub IP (blank = auto-discover)", text: $manualHost)
                        .keyboardType(.numbersAndPunctuation)
                        .autocorrectionDisabled()
                        .textInputAutocapitalization(.never)
                } header: {
                    Text("Network")
                } footer: {
                    Text("Use a manual IP if auto-discovery fails on venue Wi-Fi.")
                }
            }
            .navigationTitle("Settings")
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Apply") {
                        model.apply(settings: settings, manualHost: manualHost, carMarkerHeightCm: carMarkerHeight)
                        model.settingsChanged()
                        showSettings = false
                    }
                }
            }
        }
    }

    private func numberField(_ title: String, _ value: Binding<Double>) -> some View {
        HStack {
            Text(title)
            Spacer()
            TextField(title, value: value, format: .number)
                .keyboardType(.decimalPad)
                .multilineTextAlignment(.trailing)
                .frame(width: 80)
        }
    }
}

/// Live camera preview. Portrait orientation; fill = edge-to-edge (crops the
/// sides slightly), fit = shows the full frame the app processes.
struct CameraPreview: UIViewRepresentable {
    let session: AVCaptureSession
    var fill: Bool

    final class PreviewView: UIView {
        override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
        var previewLayer: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }

        override func layoutSubviews() {
            super.layoutSubviews()
            if let c = previewLayer.connection, c.isVideoRotationAngleSupported(90) {
                c.videoRotationAngle = 90
            }
        }
    }

    func makeUIView(context: Context) -> PreviewView {
        let v = PreviewView()
        v.previewLayer.session = session
        v.previewLayer.videoGravity = fill ? .resizeAspectFill : .resizeAspect
        v.backgroundColor = .black
        return v
    }

    func updateUIView(_ uiView: PreviewView, context: Context) {
        uiView.previewLayer.videoGravity = fill ? .resizeAspectFill : .resizeAspect
        uiView.setNeedsLayout()
    }
}

/// Hosts the ARSCNView owned by ARCameraManager (camera feed + pinned corners and arena outline).
struct ARPreview: UIViewRepresentable {
    let view: ARSCNView

    func makeUIView(context: Context) -> ARSCNView { view }
    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}
