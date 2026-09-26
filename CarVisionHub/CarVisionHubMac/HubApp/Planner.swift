import Foundation

/// A* on an occupancy grid (GridSpec layout). Marker obstacles and markerless
/// occupied cells are both inflated so the car's body keeps clear of them.
/// Blocked cells are expensive rather than impassable, so if the car ends up
/// inside an inflated zone it can still plan its way out.
final class Planner {
    var blockedCostFactor = 50.0

    func plan(from start: Vec2, to goal: Vec2, obstacles: [Vec2], occupied: [Bool]?,
              arena: ArenaConfig) -> [Vec2]? {
        let cellSize = GridSpec.cellSize
        let (cols, rows) = GridSpec.dims(arena)
        let n = cols * rows
        let inflate = arena.obstacleRadius + arena.carRadius + arena.safetyMargin

        func center(_ i: Int) -> Vec2 {
            Vec2((Double(i % cols) + 0.5) * cellSize, (Double(i / cols) + 0.5) * cellSize)
        }
        func index(of p: Vec2) -> Int {
            let c = clamp(Int(floor(p.x / cellSize)), 0, cols - 1)
            let r = clamp(Int(floor(p.y / cellSize)), 0, rows - 1)
            return r * cols + c
        }

        var blocked = [Bool](repeating: false, count: n)
        for i in 0..<n {
            let p = center(i)
            if obstacles.contains(where: { $0.distance(to: p) < inflate }) { blocked[i] = true }
        }

        // Inflate markerless occupied cells by the car's radius + margin.
        let occ = (occupied?.count == n) ? occupied : nil
        if let occ {
            let r = arena.carRadius + arena.safetyMargin
            let k = Int(ceil(r / cellSize))
            for i in 0..<n where occ[i] {
                let ci = i % cols, ri = i / cols
                let ce = center(i)
                for dr in -k...k {
                    for dc in -k...k {
                        let nc = ci + dc, nr = ri + dr
                        guard nc >= 0, nc < cols, nr >= 0, nr < rows else { continue }
                        let j = nr * cols + nc
                        if !blocked[j] && center(j).distance(to: ce) <= r { blocked[j] = true }
                    }
                }
            }
        }

        let startIdx = index(of: start)
        let goalIdx = index(of: goal)
        // Only refuse if the goal itself is covered, not merely near something.
        let goalCovered = obstacles.contains { $0.distance(to: goal) < arena.obstacleRadius }
            || (occ?[goalIdx] ?? false)
        if goalCovered { return nil }

        // A*
        let goalCenter = center(goalIdx)
        func h(_ i: Int) -> Double { center(i).distance(to: goalCenter) }

        var g = [Double](repeating: .infinity, count: n)
        var cameFrom = [Int](repeating: -1, count: n)
        var closed = [Bool](repeating: false, count: n)
        var heap = MinHeap()
        g[startIdx] = 0
        heap.push(h(startIdx), startIdx)

        let s2 = 2.0.squareRoot()
        let moves: [(Int, Int, Double)] = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                                           (1, 1, s2), (1, -1, s2), (-1, 1, s2), (-1, -1, s2)]

        while let top = heap.pop() {
            let cur = top.1
            if cur == goalIdx { break }
            if closed[cur] { continue }
            closed[cur] = true
            let cc = cur % cols, cr = cur / cols
            for (dc, dr, w) in moves {
                let nc = cc + dc, nr = cr + dr
                guard nc >= 0, nc < cols, nr >= 0, nr < rows else { continue }
                let ni = nr * cols + nc
                if closed[ni] { continue }
                var cost = w * cellSize
                if blocked[ni] { cost *= blockedCostFactor }
                let ng = g[cur] + cost
                if ng < g[ni] {
                    g[ni] = ng
                    cameFrom[ni] = cur
                    heap.push(ng + h(ni), ni)
                }
            }
        }
        guard g[goalIdx].isFinite else { return nil }

        var cells: [Int] = []
        var k = goalIdx
        while k != -1 {
            cells.append(k)
            k = cameFrom[k]
        }
        cells.reverse()

        var pts = cells.map(center)
        if pts.count == 1 {
            pts = [start, goal]
        } else {
            pts[0] = start
            pts[pts.count - 1] = goal
        }

        // Line-of-sight smoothing: skip waypoints when a straight line is clear.
        func lineFree(_ a: Vec2, _ b: Vec2) -> Bool {
            let steps = max(1, Int(a.distance(to: b) / (cellSize * 0.5)))
            for s in 0...steps {
                let p = a + (b - a) * (Double(s) / Double(steps))
                if blocked[index(of: p)] { return false }
            }
            return true
        }
        var out = [pts[0]]
        var i = 0
        while i < pts.count - 1 {
            var j = pts.count - 1
            while j > i + 1 && !lineFree(pts[i], pts[j]) { j -= 1 }
            out.append(pts[j])
            i = j
        }
        return out
    }
}

private struct MinHeap {
    private var a: [(Double, Int)] = []

    mutating func push(_ f: Double, _ i: Int) {
        a.append((f, i))
        var k = a.count - 1
        while k > 0 {
            let p = (k - 1) / 2
            if a[p].0 <= a[k].0 { break }
            a.swapAt(p, k)
            k = p
        }
    }

    mutating func pop() -> (Double, Int)? {
        guard !a.isEmpty else { return nil }
        a.swapAt(0, a.count - 1)
        let top = a.removeLast()
        var k = 0
        while true {
            let l = 2 * k + 1, r = l + 1
            var m = k
            if l < a.count && a[l].0 < a[m].0 { m = l }
            if r < a.count && a[r].0 < a[m].0 { m = r }
            if m == k { break }
            a.swapAt(k, m)
            k = m
        }
        return top
    }
}
