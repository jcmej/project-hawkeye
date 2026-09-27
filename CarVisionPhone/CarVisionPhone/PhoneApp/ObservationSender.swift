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
    private let sessionId = UUID().uuidString
    private var hubId = ""
    private var requestId = UUID().uuidString
    private var action = "stop"
    private var generation = 0
    private var sent: [Int: Date] = [:]
    private var lastReplySeq = -1
    var onNavigation: ((NavigationStatus) -> Void)?

    func requestNavigation(_ action: String) {
        queue.async {
            self.action = action
            self.generation += 1
            self.requestId = UUID().uuidString
            if action == "stop", let c = self.connection {
                // Stop need not wait for another camera frame.
                let stop: [String: Any] = ["type": "stop", "sessionId": self.sessionId,
                    "hubId": self.hubId, "generation": self.generation, "requestId": self.requestId]
                if let data = try? JSONSerialization.data(withJSONObject: stop) {
                    c.send(content: data, completion: .contentProcessed { _ in })
                }
            }
        }
    }

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
            self.hubId = ""
            self.action = "stop"
            self.generation += 1
            self.requestId = UUID().uuidString
            self.sent = [:]
            self.lastReplySeq = -1

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
            var msg = msg
            msg.sessionId = self.sessionId
            msg.navigation = NavigationRequest(id: self.requestId, generation: self.generation,
                                               action: self.action, hubId: self.hubId)
            guard let c = self.connection, let data = try? self.encoder.encode(msg) else { return }
            self.sent[msg.seq] = Date()
            self.sent = self.sent.filter { Date().timeIntervalSince($0.value) < 1 }
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
        receive(on: c)
    }

    private func receive(on c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, error in
            guard let self, self.connection === c else { return }
            if let data, let status = try? JSONDecoder().decode(NavigationStatus.self, from: data),
               status.type == "navStatus", status.sessionId == self.sessionId,
               status.seq >= self.lastReplySeq, let when = self.sent[status.seq],
               Date().timeIntervalSince(when) < 0.7 {
                self.lastReplySeq = status.seq
                if self.hubId != status.hubId {
                    self.hubId = status.hubId
                    self.action = "stop"
                    self.generation += 1
                    self.requestId = UUID().uuidString
                }
                self.onNavigation?(status)
            }
            if error == nil { self.receive(on: c) }
        }
    }

    private func report(_ s: String) { onStatus?(s) }
}
