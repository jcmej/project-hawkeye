import Foundation
import Network

/// Finds the hub via Bonjour (or uses a manual IP) and sends each
/// ObservationMessage as one UDP datagram of JSON.
final class ObservationSender {
    private let queue = DispatchQueue(label: "net.sender")
    private let encoder = JSONEncoder()
    private var browser: NWBrowser?
    private var connection: NWConnection?
    private var currentEndpoint: NWEndpoint?

    /// Human-readable connection status. Called on a background queue.
    var onStatus: ((String) -> Void)?

    /// Empty host = auto-discover with Bonjour. Otherwise send to that IP.
    func restart(manualHost: String) {
        queue.async {
            self.browser?.cancel()
            self.browser = nil
            self.connection?.cancel()
            self.connection = nil
            self.currentEndpoint = nil

            let host = manualHost.trimmingCharacters(in: .whitespacesAndNewlines)
            if host.isEmpty {
                self.startBrowsing()
            } else if let port = NWEndpoint.Port(rawValue: NetConfig.hubPort) {
                self.connect(to: .hostPort(host: NWEndpoint.Host(host), port: port))
                self.report("Sending to \(host):\(NetConfig.hubPort)")
            }
        }
    }

    func send(_ msg: ObservationMessage) {
        queue.async {
            guard let c = self.connection, let data = try? self.encoder.encode(msg) else { return }
            c.send(content: data, completion: .contentProcessed { _ in })
        }
    }

    // MARK: - Private (all on `queue`)

    private func startBrowsing() {
        report("Searching for hub…")
        let b = NWBrowser(for: .bonjour(type: NetConfig.bonjourType, domain: nil), using: NWParameters())
        b.browseResultsChangedHandler = { [weak self] results, _ in
            guard let self, let result = results.first else { return }
            if self.currentEndpoint == result.endpoint { return }
            self.connect(to: result.endpoint)
            if case let .service(name, _, _, _) = result.endpoint {
                self.report("Connected to hub “\(name)”")
            } else {
                self.report("Connected to hub")
            }
        }
        b.stateUpdateHandler = { [weak self] state in
            if case .failed(let error) = state {
                self?.report("Browse failed: \(error.localizedDescription)")
            }
        }
        b.start(queue: queue)
        browser = b
    }

    private func connect(to endpoint: NWEndpoint) {
        connection?.cancel()
        let c = NWConnection(to: endpoint, using: .udp)
        c.stateUpdateHandler = { [weak self] state in
            if case .failed(let error) = state {
                self?.report("Connection failed: \(error.localizedDescription)")
            }
        }
        c.start(queue: queue)
        connection = c
        currentEndpoint = endpoint
    }

    private func report(_ s: String) { onStatus?(s) }
}
