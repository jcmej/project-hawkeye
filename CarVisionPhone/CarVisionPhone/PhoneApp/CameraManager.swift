import AVFoundation
import CoreGraphics

enum CameraError: LocalizedError {
    case noCamera
    var errorDescription: String? { "No back camera available." }
}

/// Zoom limits in device units. `oneX` is the factor the Camera app would call "1×"
/// (on phones with an ultra-wide lens, device factor 1.0 is the ultra-wide "0.5×").
struct ZoomInfo {
    var min: CGFloat = 1
    var max: CGFloat = 1
    var oneX: CGFloat = 1
}

/// Captures PORTRAIT 720x1280 NV12 frames at up to 30 fps from the best back
/// camera, preferring a multi-lens virtual camera so zoom can go below 1×.
final class CameraManager: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    let session = AVCaptureSession()
    private let sessionQueue = DispatchQueue(label: "camera.session")
    private let videoQueue = DispatchQueue(label: "camera.video", qos: .userInteractive)
    private var configured = false
    private var device: AVCaptureDevice?

    private(set) var zoomInfo = ZoomInfo()

    /// Called on the video queue for every frame.
    var onFrame: ((CVPixelBuffer, CMTime) -> Void)?

    private static func bestBackCamera() -> AVCaptureDevice? {
        AVCaptureDevice.default(.builtInTripleCamera, for: .video, position: .back)
            ?? AVCaptureDevice.default(.builtInDualWideCamera, for: .video, position: .back)
            ?? AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back)
    }

    /// `initialZoom` is in device units; nil = the standard "1×" lens.
    func configure(initialZoom: CGFloat?) throws {
        guard !configured else { return }
        session.beginConfiguration()
        session.sessionPreset = .hd1280x720

        guard let device = Self.bestBackCamera() else {
            session.commitConfiguration()
            throw CameraError.noCamera
        }
        let input = try AVCaptureDeviceInput(device: device)
        if session.canAddInput(input) { session.addInput(input) }

        let output = AVCaptureVideoDataOutput()
        output.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        // Drop frames rather than queue them: we always want the freshest image.
        output.alwaysDiscardsLateVideoFrames = true
        output.setSampleBufferDelegate(self, queue: videoQueue)
        if session.canAddOutput(output) { session.addOutput(output) }

        // Deliver portrait (vertical) frames: 720 wide x 1280 tall.
        if let conn = output.connection(with: .video), conn.isVideoRotationAngleSupported(90) {
            conn.videoRotationAngle = 90
        }
        session.commitConfiguration()

        // Zoom range. For multi-lens cameras, the first switch-over factor is the wide ("1×") lens.
        let oneX = device.virtualDeviceSwitchOverVideoZoomFactors.first.map { CGFloat(truncating: $0) } ?? 1
        let maxZoom = min(device.maxAvailableVideoZoomFactor, oneX * 8)
        zoomInfo = ZoomInfo(min: device.minAvailableVideoZoomFactor, max: maxZoom, oneX: oneX)

        do {
            try device.lockForConfiguration()
            device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)
            device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 30)
            // The ultra-wide lens has barrel distortion; correcting it keeps the
            // flat-floor math (homography) accurate.
            if device.isGeometricDistortionCorrectionSupported {
                device.isGeometricDistortionCorrectionEnabled = true
            }
            device.videoZoomFactor = clamp(initialZoom ?? oneX, zoomInfo.min, zoomInfo.max)
            device.unlockForConfiguration()
        } catch {
            print("Could not configure camera: \(error)")
        }
        self.device = device
        configured = true
    }

    func start() {
        sessionQueue.async { [session] in
            if !session.isRunning { session.startRunning() }
        }
    }

    func stop() {
        sessionQueue.async { [session] in
            if session.isRunning { session.stopRunning() }
        }
    }

    /// Zoom in device units (clamped to the supported range).
    func setZoom(_ factor: CGFloat) {
        sessionQueue.async { [weak self] in
            guard let self, let d = self.device else { return }
            do {
                try d.lockForConfiguration()
                d.videoZoomFactor = clamp(factor, self.zoomInfo.min, self.zoomInfo.max)
                d.unlockForConfiguration()
            } catch {
                print("Could not set zoom: \(error)")
            }
        }
    }

    /// Locks exposure, white balance, focus, and the active lens so the image only
    /// changes when the scene does. Required for background-subtraction obstacle detection.
    func setLocked(_ locked: Bool) {
        sessionQueue.async { [weak self] in
            guard let d = self?.device else { return }
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
                // Multi-lens cameras can silently switch lenses (e.g. in low light),
                // which would shift the image and break the background.
                if d.isVirtualDevice {
                    d.setPrimaryConstituentDeviceSwitchingBehavior(locked ? .locked : .auto,
                                                                   restrictedSwitchingBehaviorConditions: [])
                }
                d.unlockForConfiguration()
            } catch {
                print("Could not change camera lock: \(error)")
            }
        }
    }

    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        onFrame?(pixelBuffer, CMSampleBufferGetPresentationTimeStamp(sampleBuffer))
    }
}
