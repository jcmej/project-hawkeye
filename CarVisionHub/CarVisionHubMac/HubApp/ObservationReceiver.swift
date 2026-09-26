import Foundation
import Network

/// Listens for phone observations on UDP and advertises itself via Bonjour.
final class ObservationReceiver {
    private let queue = DispatchQueue(label: "net.receiver")
    private let decoder = JSONDecoder()
    private var listener: NWListener?
    private var connections: [ObjectIdentifier: NWConnection] = [:]

    /// Called on a background queue for every valid message.
    var onObservation: ((ObservationMessage) -> Void)?
    var onStatus: ((String) -> Void)?

    func start() throws {
        let params = NWParameters.udp
        params.allowLocalEndpointReuse = true
        guard let port = NWEndpoint.Port(rawValue: NetConfig.hubPort) else { return }
        let l = try NWListener(using: params, on: port)
        l.service = NWListener.Service(name: "CarVision Hub", type: NetConfig.bonjourType)
        l.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready: self?.onStatus?("Listening on UDP \(NetConfig.hubPort)")
            case .failed(let e): self?.onStatus?("Listener failed: \(e.localizedDescription)")
            default: break
            }
        }
        l.newConnectionHandler = { [weak self] c in self?.accept(c) }
        l.start(queue: queue)
        listener = l
    }

    private func accept(_ c: NWConnection) {
        let key = ObjectIdentifier(c)
        connections[key] = c
        c.stateUpdateHandler = { [weak self] state in
            switch state {
            case .failed, .cancelled: self?.connections[key] = nil
            default: break
            }
        }
        c.start(queue: queue)
        receive(on: c)
    }

    private func receive(on c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, error in
            guard let self else { return }
            if let data, let msg = try? self.decoder.decode(ObservationMessage.self, from: data) {
                self.onObservation?(msg)
            }
            if error == nil { self.receive(on: c) }
        }
    }
}
