import SwiftUI

// Add this file to BOTH targets. Draws the arena top-down with +y pointing up.

struct ArenaTransform {
    let arena: ArenaConfig
    let size: CGSize

    private var w: CGFloat { CGFloat(max(arena.width, 1)) }
    private var h: CGFloat { CGFloat(max(arena.height, 1)) }

    var scale: CGFloat { min(size.width / w, size.height / h) * 0.9 }
    var origin: CGPoint {
        CGPoint(x: (size.width - w * scale) / 2, y: (size.height - h * scale) / 2)
    }

    func toView(_ p: Vec2) -> CGPoint {
        CGPoint(x: origin.x + CGFloat(p.x) * scale,
                y: origin.y + (h - CGFloat(p.y)) * scale)
    }

    func toWorld(_ q: CGPoint) -> Vec2 {
        Vec2(Double((q.x - origin.x) / scale),
             Double(h) - Double((q.y - origin.y) / scale))
    }

    func length(_ d: Double) -> CGFloat { CGFloat(d) * scale }
}

enum ArenaRenderer {
    static func draw(_ ctx: inout GraphicsContext,
                     _ t: ArenaTransform,
                     car: Pose?,
                     ghostCar: Pose? = nil,
                     goal: Vec2?,
                     obstacles: [Vec2],
                     path: [Vec2] = [],
                     occupied: [Vec2] = [],
                     showInflation: Bool = false) {
        let a = t.arena

        // Markerless occupancy cells (drawn first, underneath everything)
        let half = t.length(GridSpec.cellSize / 2)
        for c in occupied {
            let p = t.toView(c)
            ctx.fill(Path(CGRect(x: p.x - half, y: p.y - half, width: 2 * half, height: 2 * half)),
                     with: .color(.red.opacity(0.35)))
        }

        // Arena outline
        let topLeft = t.toView(Vec2(0, a.height))
        let rect = CGRect(origin: topLeft, size: CGSize(width: t.length(a.width), height: t.length(a.height)))
        ctx.stroke(Path(rect), with: .color(.gray), lineWidth: 2)

        // Corner labels
        for (id, p) in zip(MarkerIDs.corners, MarkerIDs.cornerWorldPositions(a)) {
            ctx.draw(Text("\(id)").font(.caption2).foregroundColor(.secondary), at: t.toView(p))
        }

        // Obstacles
        let inflate = a.obstacleRadius + a.carRadius + a.safetyMargin
        for o in obstacles {
            let c = t.toView(o)
            ctx.fill(circle(c, t.length(a.obstacleRadius)), with: .color(.red.opacity(0.55)))
            if showInflation {
                ctx.stroke(circle(c, t.length(inflate)), with: .color(.red.opacity(0.4)),
                           style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
            }
        }

        // Planned path
        if path.count >= 2 {
            var p = Path()
            p.move(to: t.toView(path[0]))
            for q in path.dropFirst() { p.addLine(to: t.toView(q)) }
            ctx.stroke(p, with: .color(.orange), lineWidth: 3)
        }

        // Goal
        if let g = goal {
            let c = t.toView(g)
            ctx.fill(circle(c, t.length(5)), with: .color(.green))
            ctx.stroke(circle(c, t.length(9)), with: .color(.green), lineWidth: 2)
        }

        // Vision-estimated car (shown as a dashed ghost when the mock car is driving)
        if let gc = ghostCar {
            drawCar(&ctx, t, gc, color: .purple, dashed: true)
        }
        if let c = car {
            drawCar(&ctx, t, c, color: .blue, dashed: false)
        }
    }

    private static func drawCar(_ ctx: inout GraphicsContext, _ t: ArenaTransform, _ pose: Pose,
                                color: Color, dashed: Bool) {
        let r = t.arena.carRadius
        let c = t.toView(pose.position)
        let tip = t.toView(pose.position + Vec2(cos(pose.heading), sin(pose.heading)) * r)
        let body = circle(c, t.length(r))
        if dashed {
            ctx.stroke(body, with: .color(color), style: StrokeStyle(lineWidth: 2, dash: [5, 4]))
        } else {
            ctx.fill(body, with: .color(color.opacity(0.25)))
            ctx.stroke(body, with: .color(color), lineWidth: 2)
        }
        var heading = Path()
        heading.move(to: c)
        heading.addLine(to: tip)
        ctx.stroke(heading, with: .color(color), lineWidth: 3)
    }

    private static func circle(_ c: CGPoint, _ r: CGFloat) -> Path {
        Path(ellipseIn: CGRect(x: c.x - r, y: c.y - r, width: 2 * r, height: 2 * r))
    }
}
