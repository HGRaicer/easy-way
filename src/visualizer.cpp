#include "visualizer.hpp"

#include <QtCore/QRectF>
#include <QtCore/QSizeF>
#include <QtCore/QString>
#include <QtGui/QColor>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;
    constexpr double kMinZoom = 1000.0;
    constexpr double kMaxZoom = 90000000.0;

    struct DrawEdge {
        std::uint32_t u = 0;
        std::uint32_t v = 0;
        float speed_mps = 0.0f;
    };

    struct RoadCountStats {
        std::uint64_t all = 0;
        std::uint64_t k30 = 0;
        std::uint64_t k50 = 0;
        std::uint64_t k70 = 0;
        std::uint64_t k90 = 0;
    };

    struct RouteSegmentDraw {
        std::size_t route_index = 0;
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        double width = 0.0;
        double casing_width = 0.0;
    };

    QColor to_qcolor(const Rgba& c) {
        return QColor(c.r, c.g, c.b, c.a);
    }

    Rgba route_palette_color(std::size_t index) {
        static const std::array<Rgba, 8> colors = { {
            { 74, 144, 226, 255}, // soft blue
            { 64, 196, 178, 255}, // soft teal
            {155, 119, 212, 255}, // soft violet
            {241, 159,  84, 255}, // warm orange
            {112, 173,  96, 255}, // muted green
            {226, 116, 116, 255}, // coral
            { 91, 170, 216, 255}, // sky blue
            {226, 198,  74, 255}  // muted yellow
        } };
        return colors[index % colors.size()];
    }

    std::uint64_t make_edge_key(std::uint32_t a, std::uint32_t b) {
        const std::uint32_t lo = std::min(a, b);
        const std::uint32_t hi = std::max(a, b);
        return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
    }

}

GraphVisualizer::WorldPoint GraphVisualizer::to_world(double lat, double lon) const {
    const double x = lon * kDegToRad;
    const double y = std::log(std::tan(kPi / 4.0 + lat * kDegToRad / 2.0));
    return { x, -y };
}

GraphVisualizer::WorldPoint GraphVisualizer::to_world(std::uint32_t node) const {
    return to_world(graph.coords[node].lat, graph.coords[node].lon);
}

void GraphVisualizer::load(const std::string& bin_path) {
    graph = load_graph_bin(bin_path);
    world_coords.clear();
    grid.clear();
    grid_cols = 0;
    grid_rows = 0;
    cell_size = 0.0;

    if (graph.N == 0) {
        std::cerr << "Failed to load graph: " << bin_path << "\n";
        return;
    }

    world_coords.resize(graph.N);
    min_x = min_y = std::numeric_limits<double>::max();
    max_x = max_y = std::numeric_limits<double>::lowest();

    for (std::uint32_t i = 0; i < graph.N; ++i) {
        world_coords[i] = to_world(i);
        min_x = std::min(min_x, world_coords[i].x);
        max_x = std::max(max_x, world_coords[i].x);
        min_y = std::min(min_y, world_coords[i].y);
        max_y = std::max(max_y, world_coords[i].y);
    }

    build_spatial_grid();

    std::cerr << "Graph loaded: " << graph.N << " nodes, grid "
        << grid_cols << "x" << grid_rows << "\n";
}

void GraphVisualizer::build_spatial_grid() {
    if (graph.N == 0 || world_coords.empty()) return;

    const double width = std::max(1e-9, max_x - min_x);
    const double height = std::max(1e-9, max_y - min_y);
    const double max_dim = std::max(width, height);

    cell_size = std::max(max_dim / 350.0, 0.00002);
    grid_cols = std::max(1, static_cast<int>(std::ceil(width / cell_size)) + 1);
    grid_rows = std::max(1, static_cast<int>(std::ceil(height / cell_size)) + 1);
    grid.assign(static_cast<std::size_t>(grid_cols) * static_cast<std::size_t>(grid_rows), {});

    for (std::uint32_t u = 0; u < graph.N; ++u) {
        const int gx = std::clamp(static_cast<int>((world_coords[u].x - min_x) / cell_size), 0, grid_cols - 1);
        const int gy = std::clamp(static_cast<int>((world_coords[u].y - min_y) / cell_size), 0, grid_rows - 1);
        grid[static_cast<std::size_t>(gy) * static_cast<std::size_t>(grid_cols) + static_cast<std::size_t>(gx)].push_back(u);
    }
}

void GraphVisualizer::add_waypoint(double lat, double lon) {
    waypoints.push_back(Waypoint{ lat, lon });
}

class QtGraphWidget final : public QWidget {
public:
    explicit QtGraphWidget(const GraphVisualizer& visualizer, QWidget* parent = nullptr)
        : QWidget(parent), viz_(visualizer) {
        setWindowTitle("OSM Route Visualizer - Qt");
        resize(1600, 1000);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
        build_route_index();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(15, 15, 18));

        if (viz_.graph.N == 0) {
            painter.setPen(Qt::white);
            painter.drawText(20, 30, "Graph is empty");
            return;
        }

        ensure_initial_view();
        draw_roads(painter);
        draw_routes(painter);
        draw_waypoints(painter);
        draw_overlay(painter);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton || event->button() == Qt::RightButton) {
            dragging_ = true;
            last_mouse_ = event->position();
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (!dragging_) return;
        const QPointF delta = event->position() - last_mouse_;
        center_.rx() -= delta.x() / zoom_;
        center_.ry() -= delta.y() / zoom_;
        last_mouse_ = event->position();
        update();
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton || event->button() == Qt::RightButton) {
            dragging_ = false;
        }
    }

    void wheelEvent(QWheelEvent* event) override {
        if (viz_.graph.N == 0) return;

        const QPointF cursor = event->position();
        const QPointF before = screen_to_world(cursor);
        const double factor = std::pow(1.0015, event->angleDelta().y());
        zoom_ = std::clamp(zoom_ * factor, kMinZoom, kMaxZoom);
        const QPointF after = screen_to_world(cursor);
        center_ += before - after;
        update();
    }

    void keyPressEvent(QKeyEvent* event) override {
        if (event->key() == Qt::Key_R) {
            initialized_ = false;
            update();
            return;
        }

        QWidget::keyPressEvent(event);
    }

private:
    static constexpr std::uint64_t kMaxVisibleRoadsBeforeFiltering = 500000;

    const GraphVisualizer& viz_;

    QPointF center_{ 0.0, 0.0 };
    double zoom_ = 18000.0;
    bool initialized_ = false;
    bool dragging_ = false;
    QPointF last_mouse_{ 0.0, 0.0 };

    std::unordered_map<std::uint64_t, std::vector<std::size_t>> route_edge_routes_;

    std::uint64_t last_drawn_roads_ = 0;
    std::uint64_t last_candidate_roads_ = 0;
    std::uint64_t last_total_visible_roads_ = 0;
    int last_min_speed_kph_ = 0;

    QPointF world_to_screen(const GraphVisualizer::WorldPoint& p) const {
        return QPointF(width() * 0.5 + (p.x - center_.x()) * zoom_,
            height() * 0.5 + (p.y - center_.y()) * zoom_);
    }

    QPointF screen_to_world(const QPointF& p) const {
        return QPointF(center_.x() + (p.x() - width() * 0.5) / zoom_,
            center_.y() + (p.y() - height() * 0.5) / zoom_);
    }

    QRectF visible_world_rect() const {
        const QPointF a = screen_to_world(QPointF(0.0, 0.0));
        const QPointF b = screen_to_world(QPointF(width(), height()));

        return QRectF(
            QPointF(std::min(a.x(), b.x()), std::min(a.y(), b.y())),
            QPointF(std::max(a.x(), b.x()), std::max(a.y(), b.y()))
        );
    }

    void ensure_initial_view() {
        if (initialized_) return;

        QRectF bounds = route_world_bounds();
        if (!bounds.isValid() || bounds.isEmpty()) {
            bounds = QRectF(QPointF(viz_.min_x, viz_.min_y), QPointF(viz_.max_x, viz_.max_y));
        }

        if (!bounds.isValid() || bounds.width() <= 0.0 || bounds.height() <= 0.0) {
            center_ = QPointF(0.0, 0.0);
            zoom_ = 18000.0;
            initialized_ = true;
            return;
        }

        center_ = bounds.center();
        const double sx = std::max(1.0, static_cast<double>(width())) / bounds.width();
        const double sy = std::max(1.0, static_cast<double>(height())) / bounds.height();
        zoom_ = std::clamp(std::min(sx, sy) * 0.82, kMinZoom, kMaxZoom);
        initialized_ = true;
    }

    QRectF route_world_bounds() const {
        bool has = false;
        QRectF bounds;

        for (const Route& route : viz_.routes) {
            if (!route.visible) continue;

            for (std::uint32_t node : route.nodes) {
                if (node >= viz_.graph.N) continue;

                const auto& p = viz_.world_coords[node];
                const QRectF point(QPointF(p.x, p.y), QSizeF(0.0, 0.0));

                if (!has) {
                    bounds = point;
                    has = true;
                }
                else {
                    bounds = bounds.united(point);
                }
            }
        }

        if (!has) return QRectF();

        const double pad_x = std::max(bounds.width() * 0.12, 0.0005);
        const double pad_y = std::max(bounds.height() * 0.12, 0.0005);
        return bounds.adjusted(-pad_x, -pad_y, pad_x, pad_y);
    }

    float edge_speed(std::uint32_t edge_index) const {
        if (viz_.graph.speed_present && edge_index < viz_.graph.speeds.size()) {
            return viz_.graph.speeds[edge_index];
        }
        return 0.0f;
    }

    bool edge_intersects_view(std::uint32_t u, std::uint32_t v, const QRectF& view) const {
        if (u >= viz_.graph.N || v >= viz_.graph.N) return false;

        const auto& a = viz_.world_coords[u];
        const auto& b = viz_.world_coords[v];

        if (a.x < view.left() && b.x < view.left()) return false;
        if (a.x > view.right() && b.x > view.right()) return false;
        if (a.y < view.top() && b.y < view.top()) return false;
        if (a.y > view.bottom() && b.y > view.bottom()) return false;

        return true;
    }

    bool speed_passes_filter(float speed_mps, int min_kph) const {
        if (min_kph <= 0) return true;
        if (!viz_.graph.speed_present) return false;
        if (speed_mps <= 0.0f) return false;
        return speed_mps * 3.6f >= static_cast<float>(min_kph);
    }

    std::uint64_t count_for_min_speed(const RoadCountStats& stats, int min_kph) const {
        if (min_kph >= 90) return stats.k90;
        if (min_kph >= 70) return stats.k70;
        if (min_kph >= 50) return stats.k50;
        if (min_kph >= 30) return stats.k30;
        return stats.all;
    }

    int choose_min_speed_by_count(const RoadCountStats& stats) const {
        if (!viz_.graph.speed_present) return 0;

        if (stats.all <= kMaxVisibleRoadsBeforeFiltering) return 0;
        if (stats.k30 <= kMaxVisibleRoadsBeforeFiltering) return 30;
        if (stats.k50 <= kMaxVisibleRoadsBeforeFiltering) return 50;
        if (stats.k70 <= kMaxVisibleRoadsBeforeFiltering) return 70;

        return 90;
    }

    bool grid_range_for_view(const QRectF& view, int& gx1, int& gx2, int& gy1, int& gy2) const {
        if (viz_.grid.empty() || viz_.cell_size <= 0.0) return false;

        gx1 = static_cast<int>(std::floor((view.left() - viz_.min_x) / viz_.cell_size));
        gx2 = static_cast<int>(std::floor((view.right() - viz_.min_x) / viz_.cell_size));
        gy1 = static_cast<int>(std::floor((view.top() - viz_.min_y) / viz_.cell_size));
        gy2 = static_cast<int>(std::floor((view.bottom() - viz_.min_y) / viz_.cell_size));

        gx1 = std::clamp(gx1, 0, viz_.grid_cols - 1);
        gx2 = std::clamp(gx2, 0, viz_.grid_cols - 1);
        gy1 = std::clamp(gy1, 0, viz_.grid_rows - 1);
        gy2 = std::clamp(gy2, 0, viz_.grid_rows - 1);

        return gx1 <= gx2 && gy1 <= gy2;
    }

    RoadCountStats count_visible_roads_by_speed(const QRectF& view) const {
        RoadCountStats stats;

        int gx1 = 0, gx2 = 0, gy1 = 0, gy2 = 0;
        if (!grid_range_for_view(view, gx1, gx2, gy1, gy2)) return stats;

        for (int gy = gy1; gy <= gy2; ++gy) {
            for (int gx = gx1; gx <= gx2; ++gx) {
                const auto& bucket = viz_.grid[
                    static_cast<std::size_t>(gy) * static_cast<std::size_t>(viz_.grid_cols) +
                        static_cast<std::size_t>(gx)
                ];

                for (std::uint32_t u : bucket) {
                    if (u >= viz_.graph.N) continue;

                    const std::uint32_t start = (u == 0 ? 0u : viz_.graph.offsets[u - 1]);
                    const std::uint32_t end = viz_.graph.offsets[u];

                    for (std::uint32_t i = start; i < end && i < viz_.graph.adjacency.size(); ++i) {
                        const std::uint32_t v = viz_.graph.adjacency[i];
                        if (v >= viz_.graph.N) continue;
                        if (!edge_intersects_view(u, v, view)) continue;

                        ++stats.all;

                        if (!viz_.graph.speed_present || i >= viz_.graph.speeds.size()) continue;

                        const float speed_mps = viz_.graph.speeds[i];
                        if (speed_mps <= 0.0f) continue;

                        const float speed_kph = speed_mps * 3.6f;
                        if (speed_kph >= 30.0f) ++stats.k30;
                        if (speed_kph >= 50.0f) ++stats.k50;
                        if (speed_kph >= 70.0f) ++stats.k70;
                        if (speed_kph >= 90.0f) ++stats.k90;
                    }
                }
            }
        }

        return stats;
    }

    void draw_visible_roads_with_filter(QPainter& painter, const QRectF& view, int min_kph) {
        int gx1 = 0, gx2 = 0, gy1 = 0, gy2 = 0;
        if (!grid_range_for_view(view, gx1, gx2, gy1, gy2)) return;

        for (int gy = gy1; gy <= gy2; ++gy) {
            for (int gx = gx1; gx <= gx2; ++gx) {
                const auto& bucket = viz_.grid[
                    static_cast<std::size_t>(gy) * static_cast<std::size_t>(viz_.grid_cols) +
                        static_cast<std::size_t>(gx)
                ];

                for (std::uint32_t u : bucket) {
                    if (u >= viz_.graph.N) continue;

                    const std::uint32_t start = (u == 0 ? 0u : viz_.graph.offsets[u - 1]);
                    const std::uint32_t end = viz_.graph.offsets[u];

                    for (std::uint32_t i = start; i < end && i < viz_.graph.adjacency.size(); ++i) {
                        const std::uint32_t v = viz_.graph.adjacency[i];
                        if (v >= viz_.graph.N) continue;
                        if (!edge_intersects_view(u, v, view)) continue;

                        const float speed = edge_speed(i);
                        if (!speed_passes_filter(speed, min_kph)) continue;

                        painter.drawLine(world_to_screen(viz_.world_coords[u]),
                            world_to_screen(viz_.world_coords[v]));
                        ++last_drawn_roads_;
                    }
                }
            }
        }
    }

    void draw_roads(QPainter& painter) {
        last_drawn_roads_ = 0;
        last_candidate_roads_ = 0;
        last_total_visible_roads_ = 0;
        last_min_speed_kph_ = 0;

        if (viz_.grid.empty() || viz_.cell_size <= 0.0) return;

        const QRectF view = visible_world_rect().adjusted(
            -viz_.cell_size, -viz_.cell_size,
            viz_.cell_size, viz_.cell_size
        );

        const RoadCountStats stats = count_visible_roads_by_speed(view);
        const int min_kph = choose_min_speed_by_count(stats);
        const std::uint64_t selected_count = count_for_min_speed(stats, min_kph);

        last_min_speed_kph_ = min_kph;
        last_candidate_roads_ = selected_count;
        last_total_visible_roads_ = stats.all;

        QPen pen(QColor(92, 94, 102));
        if (min_kph >= 90) pen.setWidthF(1.25);
        else if (min_kph >= 70) pen.setWidthF(1.10);
        else if (min_kph >= 50) pen.setWidthF(1.00);
        else if (min_kph >= 30) pen.setWidthF(0.90);
        else pen.setWidthF(0.75);
        pen.setCosmetic(true);

        painter.setPen(pen);
        painter.setRenderHint(QPainter::Antialiasing, false);
        draw_visible_roads_with_filter(painter, view, min_kph);
    }

    void build_route_index() {
        route_edge_routes_.clear();

        for (std::size_t route_index = 0; route_index < viz_.routes.size(); ++route_index) {
            const Route& route = viz_.routes[route_index];
            if (!route.visible || route.nodes.size() < 2) continue;

            for (std::size_t i = 1; i < route.nodes.size(); ++i) {
                const std::uint32_t a = route.nodes[i - 1];
                const std::uint32_t b = route.nodes[i];
                if (a >= viz_.graph.N || b >= viz_.graph.N || a == b) continue;

                auto& routes = route_edge_routes_[make_edge_key(a, b)];
                if (std::find(routes.begin(), routes.end(), route_index) == routes.end()) {
                    routes.push_back(route_index);
                }
            }
        }

        for (auto& kv : route_edge_routes_) {
            std::sort(kv.second.begin(), kv.second.end());
        }
    }

    QString marker_label(int index) const {
        if (index < 0) return QString();

        QString label;
        int value = index;
        do {
            const int rem = value % 26;
            label.prepend(QChar('A' + rem));
            value = value / 26 - 1;
        } while (value >= 0);

        return label;
    }

    RouteSegmentDraw make_route_segment_draw(std::size_t route_index, std::uint32_t a, std::uint32_t b) const {
        RouteSegmentDraw seg;
        seg.route_index = route_index;
        seg.a = a;
        seg.b = b;

        int count = 1;
        int order = 0;

        const auto it = route_edge_routes_.find(make_edge_key(a, b));
        if (it != route_edge_routes_.end() && !it->second.empty()) {
            count = static_cast<int>(it->second.size());
            const auto found = std::find(it->second.begin(), it->second.end(), route_index);
            if (found != it->second.end()) {
                order = static_cast<int>(std::distance(it->second.begin(), found));
            }
        }

        constexpr double base_width = 4.4;
        constexpr double band_step = 5.8;
        seg.width = base_width + static_cast<double>(count - 1 - order) * band_step;
        seg.casing_width = base_width + static_cast<double>(count - 1) * band_step + 2.8;

        return seg;
    }

    std::vector<RouteSegmentDraw> collect_visible_route_segments(const QRectF& view) const {
        std::vector<RouteSegmentDraw> segments;

        for (std::size_t route_index = 0; route_index < viz_.routes.size(); ++route_index) {
            const Route& route = viz_.routes[route_index];
            if (!route.visible || route.nodes.size() < 2) continue;

            for (std::size_t i = 1; i < route.nodes.size(); ++i) {
                const std::uint32_t a = route.nodes[i - 1];
                const std::uint32_t b = route.nodes[i];
                if (a >= viz_.graph.N || b >= viz_.graph.N || a == b) continue;
                if (!edge_intersects_view(a, b, view)) continue;

                segments.push_back(make_route_segment_draw(route_index, a, b));
            }
        }

        return segments;
    }

    void draw_routes(QPainter& painter) {
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setBrush(Qt::NoBrush);

        const QRectF view = visible_world_rect().adjusted(
            -viz_.cell_size, -viz_.cell_size,
            viz_.cell_size, viz_.cell_size
        );

        std::vector<RouteSegmentDraw> segments = collect_visible_route_segments(view);
        if (segments.empty()) return;

        {
            std::vector<RouteSegmentDraw> casing_segments = segments;
            std::sort(casing_segments.begin(), casing_segments.end(),
                [](const RouteSegmentDraw& lhs, const RouteSegmentDraw& rhs) {
                    return lhs.casing_width > rhs.casing_width;
                });

            QPen casing(QColor(6, 7, 9, 215));
            casing.setCapStyle(Qt::RoundCap);
            casing.setJoinStyle(Qt::RoundJoin);
            casing.setCosmetic(true);
            painter.setPen(casing);

            std::unordered_set<std::uint64_t> drawn_casings;
            drawn_casings.reserve(casing_segments.size());

            for (const RouteSegmentDraw& seg : casing_segments) {
                if (seg.a >= viz_.graph.N || seg.b >= viz_.graph.N) continue;

                const std::uint64_t key = make_edge_key(seg.a, seg.b);
                if (!drawn_casings.insert(key).second) continue;

                casing.setWidthF(seg.casing_width);
                painter.setPen(casing);
                painter.drawLine(world_to_screen(viz_.world_coords[seg.a]),
                    world_to_screen(viz_.world_coords[seg.b]));
            }
        }

        std::sort(segments.begin(), segments.end(),
            [](const RouteSegmentDraw& lhs, const RouteSegmentDraw& rhs) {
                if (lhs.width != rhs.width) return lhs.width > rhs.width;
                return lhs.route_index < rhs.route_index;
            });

        for (const RouteSegmentDraw& seg : segments) {
            if (seg.a >= viz_.graph.N || seg.b >= viz_.graph.N) continue;

            QPen route_pen(to_qcolor(route_palette_color(seg.route_index)));
            route_pen.setWidthF(seg.width);
            route_pen.setCapStyle(Qt::RoundCap);
            route_pen.setJoinStyle(Qt::RoundJoin);
            route_pen.setCosmetic(true);

            painter.setPen(route_pen);
            painter.drawLine(world_to_screen(viz_.world_coords[seg.a]),
                world_to_screen(viz_.world_coords[seg.b]));
        }

        draw_route_endpoint_markers(painter);
    }

    void draw_route_endpoint_markers(QPainter& painter) {
        struct EndpointMarker {
            std::uint32_t node = 0;
            int label_index = 0;
        };

        std::unordered_map<std::uint32_t, int> label_by_node;
        std::vector<EndpointMarker> markers;

        auto add_endpoint = [&](std::uint32_t node) {
            if (node >= viz_.graph.N) return;
            const auto it = label_by_node.find(node);
            if (it != label_by_node.end()) return;
            const int label_index = static_cast<int>(label_by_node.size());
            label_by_node.emplace(node, label_index);
            markers.push_back(EndpointMarker{ node, label_index });
            };

        for (const Route& route : viz_.routes) {
            if (!route.visible || route.nodes.size() < 2) continue;

            std::uint32_t first = std::numeric_limits<std::uint32_t>::max();
            std::uint32_t last = std::numeric_limits<std::uint32_t>::max();

            for (std::uint32_t node : route.nodes) {
                if (node >= viz_.graph.N) continue;
                if (first == std::numeric_limits<std::uint32_t>::max()) first = node;
                last = node;
            }

            if (first != std::numeric_limits<std::uint32_t>::max()) add_endpoint(first);
            if (last != std::numeric_limits<std::uint32_t>::max()) add_endpoint(last);
        }

        for (const EndpointMarker& marker : markers) {
            draw_marker(painter,
                world_to_screen(viz_.world_coords[marker.node]),
                marker_label(marker.label_index));
        }
    }

    void draw_waypoints(QPainter& painter) {
        if (viz_.waypoints.empty()) return;

        painter.setRenderHint(QPainter::Antialiasing, true);

        int index = 1;
        for (const Waypoint& waypoint : viz_.waypoints) {
            const auto p = viz_.to_world(waypoint.lat, waypoint.lon);
            draw_marker(painter, world_to_screen(p), QString::number(index));
            ++index;
        }
    }

    void draw_marker(QPainter& painter, const QPointF& p, const QString& text) {
        const QColor red(230, 60, 60);

        painter.setPen(QPen(Qt::white, 2.0));
        painter.setBrush(red);
        painter.drawEllipse(p, 8.5, 8.5);

        painter.setPen(Qt::white);
        const QRectF label_rect(p.x() - 8.5, p.y() - 9.0, 17.0, 17.0);
        painter.drawText(label_rect, Qt::AlignCenter, text);
    }

    void draw_overlay(QPainter& painter) {
        painter.setRenderHint(QPainter::Antialiasing, true);

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 0, 0, 165));
        painter.drawRoundedRect(QRectF(10, 10, 980, 132), 8, 8);

        painter.setPen(QColor(245, 245, 245));

        painter.drawText(24, 34,
            QString("Nodes: %1    Zoom: %2    Roads drawn: %3")
            .arg(viz_.graph.N)
            .arg(zoom_, 0, 'f', 0)
            .arg(static_cast<qulonglong>(last_drawn_roads_))
        );

        QString filter_text;
        if (last_min_speed_kph_ <= 0) {
            filter_text = "all visible roads";
        }
        else {
            filter_text = QString("roads >= %1 km/h").arg(last_min_speed_kph_);
        }

        painter.drawText(24, 58,
            QString("Navigator mode: visible grid    Road filter: %1    visible: %2    selected: %3")
            .arg(filter_text)
            .arg(static_cast<qulonglong>(last_total_visible_roads_))
            .arg(static_cast<qulonglong>(last_candidate_roads_))
        );

        painter.drawText(24, 82,
            "Mouse drag - pan | Wheel - zoom | R - fit route/map | shared routes use nested color bands"
        );

        if (!viz_.routes.empty()) {
            const Route& r = viz_.routes.front();
            painter.drawText(24, 106,
                QString("Routes: %1    First route: %2 nodes, distance %3 m, time %4 s")
                .arg(viz_.routes.size())
                .arg(r.nodes.size())
                .arg(r.total_dist_m, 0, 'f', 1)
                .arg(r.total_time_s, 0, 'f', 1)
            );

            int x = 24;
            int y = 128;
            for (std::size_t i = 0; i < viz_.routes.size() && i < 8; ++i) {
                painter.setPen(Qt::NoPen);
                painter.setBrush(to_qcolor(route_palette_color(i)));
                painter.drawRoundedRect(QRectF(x, y - 9, 26, 7), 3, 3);

                painter.setPen(QColor(245, 245, 245));
                painter.drawText(x + 34, y, QString("Route %1").arg(i + 1));
                x += 120;
            }
        }
        else {
            painter.drawText(24, 106, "Routes: 0    No route was loaded from result.txt");
        }
    }
};

void GraphVisualizer::run() {
    if (graph.N == 0) return;

    int argc = 1;
    char app_name[] = "graph_builder";
    char* argv[] = { app_name, nullptr };

    QApplication app(argc, argv);

    QtGraphWidget window(*this);
    window.show();

    app.exec();
}
