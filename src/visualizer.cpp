#include "visualizer.hpp"

#include <QtCore/QLineF>
#include <QtCore/QRectF>
#include <QtCore/QSizeF>
#include <QtCore/QVector>
#include <QtCore/QString>
#include <QtGui/QColor>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QPixmap>
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
#include <utility>
#include <vector>
#include <iomanip>

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

    static QString format_time_hms(double total_seconds) {
        if (total_seconds < 0) total_seconds = 0;
        unsigned long long total_secs = static_cast<unsigned long long>(total_seconds);
        unsigned int hours = total_secs / 3600;
        unsigned int minutes = (total_secs % 3600) / 60;
        unsigned int seconds = total_secs % 60;

        return QString("%1:%2:%3")
            .arg(hours, 2, 10, QChar('0'))
            .arg(minutes, 2, 10, QChar('0'))
            .arg(seconds, 2, 10, QChar('0'));
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
        setWindowTitle(viz_.routes.empty() ? "OSM Map Visualizer - Qt" : "OSM Route Visualizer - Qt");
        resize(1600, 1000);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
        build_route_index();
        build_road_tile_index();
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

    std::string format_seconds_to_hhmmss(double seconds) {
        int total_seconds = static_cast<int>(seconds);
        int hours = total_seconds / 3600;
        int minutes = (total_seconds % 3600) / 60;
        int secs = total_seconds % 60;

        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(2) << hours << ":"
            << std::setw(2) << minutes << ":"
            << std::setw(2) << secs;
        return oss.str();
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
        clear_road_tile_cache();
        const QPointF after = screen_to_world(cursor);
        center_ += before - after;
        update();
    }

    void keyPressEvent(QKeyEvent* event) override {
        if (is_reset_view_key(event)) {
            initialized_ = false;
            clear_road_tile_cache();
            update();
            return;
        }

        QWidget::keyPressEvent(event);
    }

private:
    bool is_reset_view_key(const QKeyEvent* event) const {
        if (!event) return false;

        // Qt::Key_R works only when the active keyboard layout produces Latin R.
        // With the Russian layout the same physical key produces Cyrillic "к".
        if (event->key() == Qt::Key_R) return true;

        const QString text = event->text();
        if (text.compare(QString::fromUtf8("r"), Qt::CaseInsensitive) == 0) return true;
        if (text.compare(QString::fromUtf8("к"), Qt::CaseInsensitive) == 0) return true;

        // Physical R key on common Windows/Linux keyboard scan-code sets.
        // This keeps reset independent from the selected input language.
        return event->nativeScanCode() == 0x13;
    }

    static constexpr std::uint64_t kMaxVisibleRoadsBeforeFiltering = 500000;
    static constexpr std::uint64_t kMaxRoadsPerFrame = 120000;
    static constexpr int kRoadLineBatchSize = 8192;
    static constexpr int kRasterTilePixels = 512;
    static constexpr std::size_t kMaxCachedRasterTiles = 256;

    struct RoadEdgeItem {
        std::uint32_t u = 0;
        std::uint32_t v = 0;
        float speed_mps = 0.0f;
        std::uint8_t tier = 0; // 0: all/unknown, 1: >=30, 2: >=50, 3: >=70, 4: >=90 km/h
    };

    struct RoadTileLevel {
        int cols = 0;
        int rows = 0;
        double tile_size = 0.0;
        int min_tier = 0;
        const char* name = "";
        std::vector<std::vector<std::uint32_t>> tiles;
    };

    struct RasterTileKey {
        int level = 0;
        int x = 0;
        int y = 0;

        bool operator==(const RasterTileKey& other) const noexcept {
            return level == other.level && x == other.x && y == other.y;
        }
    };

    struct RasterTileKeyHash {
        std::size_t operator()(const RasterTileKey& key) const noexcept {
            std::uint64_t h = 1469598103934665603ull;
            auto mix = [&h](int value) {
                h ^= static_cast<std::uint32_t>(value);
                h *= 1099511628211ull;
                };
            mix(key.level);
            mix(key.x);
            mix(key.y);
            return static_cast<std::size_t>(h);
        }
    };

    struct CachedRoadRasterTile {
        QPixmap pixmap;
        std::uint64_t roads_drawn = 0;
        int last_used_frame = 0;
    };

    const GraphVisualizer& viz_;

    QPointF center_{ 0.0, 0.0 };
    double zoom_ = 18000.0;
    bool initialized_ = false;
    bool dragging_ = false;
    QPointF last_mouse_{ 0.0, 0.0 };

    std::unordered_map<std::uint64_t, std::vector<std::size_t>> route_edge_routes_;
    std::vector<RoadEdgeItem> road_edges_;
    std::array<std::vector<std::vector<std::uint32_t>>, 5> road_edge_grid_; // legacy grid, kept unused
    std::array<RoadTileLevel, 5> road_tile_levels_;
    std::unordered_map<RasterTileKey, CachedRoadRasterTile, RasterTileKeyHash> road_raster_tile_cache_;
    int road_raster_frame_ = 0;

    std::uint64_t last_drawn_roads_ = 0;
    std::uint64_t last_candidate_roads_ = 0;
    std::uint64_t last_total_visible_roads_ = 0;
    int last_min_speed_kph_ = 0;
    int last_tile_level_ = 0;

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

    bool first_route_start_point(QPointF& out) const {
        for (const Route& route : viz_.routes) {
            if (!route.visible) continue;

            for (std::uint32_t node : route.nodes) {
                if (node >= viz_.graph.N) continue;

                const auto& p = viz_.world_coords[node];
                out = QPointF(p.x, p.y);
                return true;
            }
        }

        return false;
    }

    void ensure_initial_view() {
        if (initialized_) return;

        QPointF route_start;
        if (first_route_start_point(route_start)) {
            center_ = route_start;

            // Start at the first point of the first loaded route, not at the
            // whole-route bounding box.  This keeps the user focused on where
            // the route begins while still allowing R to refit later.
            zoom_ = std::clamp(180000.0, kMinZoom, kMaxZoom);
            initialized_ = true;
            return;
        }

        QRectF bounds = QRectF(QPointF(viz_.min_x, viz_.min_y), QPointF(viz_.max_x, viz_.max_y));

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


    std::uint8_t road_tier_for_speed(float speed_mps) const {
        if (!viz_.graph.speed_present || speed_mps <= 0.0f) return 0;

        const float speed_kph = speed_mps * 3.6f;
        if (speed_kph >= 90.0f) return 4;
        if (speed_kph >= 70.0f) return 3;
        if (speed_kph >= 50.0f) return 2;
        if (speed_kph >= 30.0f) return 1;
        return 0;
    }

    int min_kph_for_tier(int tier) const {
        if (tier >= 4) return 90;
        if (tier >= 3) return 70;
        if (tier >= 2) return 50;
        if (tier >= 1) return 30;
        return 0;
    }

    int base_road_tier_for_zoom() const {
        if (!viz_.graph.speed_present) return 0;

        // Far zoom levels must not even iterate over local streets.
        // The exact numbers are intentionally conservative and can be tuned
        // from the overlay by watching Zoom / Roads drawn.
        if (zoom_ < 6000.0) return 4;    // only highways / very fast roads
        if (zoom_ < 14000.0) return 3;   // >=70 km/h
        if (zoom_ < 35000.0) return 2;   // >=50 km/h
        if (zoom_ < 90000.0) return 1;   // >=30 km/h
        return 0;
    }

    int tile_level_for_zoom() const {
        if (!viz_.graph.speed_present) return 4;

        // One global LOD is selected for the whole frame. Do not change it per
        // row/tile/candidate count; otherwise the top and bottom of the screen
        // can show different map layers.
        if (zoom_ < 6000.0) return 0;    // only very fast roads
        if (zoom_ < 14000.0) return 1;   // >=70 km/h
        if (zoom_ < 35000.0) return 2;   // >=50 km/h
        if (zoom_ < 90000.0) return 3;   // >=30 km/h
        return 4;                        // all roads
    }

    QColor road_color_for_tier(int tier) const {
        if (tier >= 4) return QColor(126, 130, 140);
        if (tier >= 3) return QColor(108, 112, 122);
        if (tier >= 2) return QColor(92, 96, 106);
        if (tier >= 1) return QColor(72, 76, 86);
        return QColor(55, 58, 67);
    }

    double road_width_for_tier(int tier) const {
        if (tier >= 4) return 1.35;
        if (tier >= 3) return 1.15;
        if (tier >= 2) return 0.95;
        if (tier >= 1) return 0.78;
        return 0.62;
    }

    void init_road_tile_levels() {
        const double width = std::max(1e-9, viz_.max_x - viz_.min_x);
        const double height = std::max(1e-9, viz_.max_y - viz_.min_y);
        const double max_dim = std::max(width, height);

        struct LevelDef {
            int tiles_on_max_axis;
            int min_tier;
            const char* name;
        };

        const std::array<LevelDef, 5> defs = { {
            { 16, 4, "z0 highways" },
            { 32, 3, "z1 major" },
            { 64, 2, "z2 primary" },
            {128, 1, "z3 regional" },
            {256, 0, "z4 all" }
        } };

        for (std::size_t i = 0; i < road_tile_levels_.size(); ++i) {
            RoadTileLevel& level = road_tile_levels_[i];
            level.tile_size = std::max(max_dim / static_cast<double>(defs[i].tiles_on_max_axis), 1e-9);
            level.cols = std::max(1, static_cast<int>(std::ceil(width / level.tile_size)) + 1);
            level.rows = std::max(1, static_cast<int>(std::ceil(height / level.tile_size)) + 1);
            level.min_tier = defs[i].min_tier;
            level.name = defs[i].name;
            level.tiles.clear();
            level.tiles.resize(static_cast<std::size_t>(level.cols) * static_cast<std::size_t>(level.rows));
        }
    }

    void add_edge_to_tile_level(int level_index, std::uint32_t edge_index) {
        if (level_index < 0 || level_index >= static_cast<int>(road_tile_levels_.size())) return;
        if (edge_index >= road_edges_.size()) return;

        RoadTileLevel& level = road_tile_levels_[static_cast<std::size_t>(level_index)];
        if (level.tiles.empty() || level.tile_size <= 0.0 || level.cols <= 0 || level.rows <= 0) return;

        const RoadEdgeItem& edge = road_edges_[edge_index];
        const auto& a = viz_.world_coords[edge.u];
        const auto& b = viz_.world_coords[edge.v];

        const double x1 = std::min(a.x, b.x);
        const double x2 = std::max(a.x, b.x);
        const double y1 = std::min(a.y, b.y);
        const double y2 = std::max(a.y, b.y);

        const int gx1 = std::clamp(static_cast<int>(std::floor((x1 - viz_.min_x) / level.tile_size)), 0, level.cols - 1);
        const int gx2 = std::clamp(static_cast<int>(std::floor((x2 - viz_.min_x) / level.tile_size)), 0, level.cols - 1);
        const int gy1 = std::clamp(static_cast<int>(std::floor((y1 - viz_.min_y) / level.tile_size)), 0, level.rows - 1);
        const int gy2 = std::clamp(static_cast<int>(std::floor((y2 - viz_.min_y) / level.tile_size)), 0, level.rows - 1);

        for (int gy = gy1; gy <= gy2; ++gy) {
            for (int gx = gx1; gx <= gx2; ++gx) {
                const std::size_t tile = static_cast<std::size_t>(gy) * static_cast<std::size_t>(level.cols) +
                    static_cast<std::size_t>(gx);
                level.tiles[tile].push_back(edge_index);
            }
        }
    }

    void build_road_tile_index() {
        road_edges_.clear();
        init_road_tile_levels();

        if (viz_.graph.N == 0 || viz_.graph.offsets.empty()) return;

        std::unordered_set<std::uint64_t> seen_edges;
        seen_edges.reserve(viz_.graph.adjacency.size());
        road_edges_.reserve(viz_.graph.adjacency.size() / 2 + 1);

        for (std::uint32_t u = 0; u < viz_.graph.N; ++u) {
            const std::uint32_t start = (u == 0 ? 0u : viz_.graph.offsets[u - 1]);
            const std::uint32_t end = viz_.graph.offsets[u];

            for (std::uint32_t i = start; i < end && i < viz_.graph.adjacency.size(); ++i) {
                const std::uint32_t v = viz_.graph.adjacency[i];
                if (v >= viz_.graph.N || u == v) continue;

                const std::uint64_t key = make_edge_key(u, v);
                if (!seen_edges.insert(key).second) continue;

                const float speed = edge_speed(i);
                const std::uint8_t tier = road_tier_for_speed(speed);
                const std::uint32_t edge_index = static_cast<std::uint32_t>(road_edges_.size());
                road_edges_.push_back(RoadEdgeItem{ u, v, speed, tier });

                for (int level_index = 0; level_index < static_cast<int>(road_tile_levels_.size()); ++level_index) {
                    if (tier < road_tile_levels_[static_cast<std::size_t>(level_index)].min_tier) continue;
                    add_edge_to_tile_level(level_index, edge_index);
                }
            }
        }
    }

    bool tile_range_for_view(const QRectF& view, const RoadTileLevel& level,
        int& gx1, int& gx2, int& gy1, int& gy2) const {
        if (level.tiles.empty() || level.tile_size <= 0.0 || level.cols <= 0 || level.rows <= 0) return false;

        gx1 = static_cast<int>(std::floor((view.left() - viz_.min_x) / level.tile_size));
        gx2 = static_cast<int>(std::floor((view.right() - viz_.min_x) / level.tile_size));
        gy1 = static_cast<int>(std::floor((view.top() - viz_.min_y) / level.tile_size));
        gy2 = static_cast<int>(std::floor((view.bottom() - viz_.min_y) / level.tile_size));

        gx1 = std::clamp(gx1, 0, level.cols - 1);
        gx2 = std::clamp(gx2, 0, level.cols - 1);
        gy1 = std::clamp(gy1, 0, level.rows - 1);
        gy2 = std::clamp(gy2, 0, level.rows - 1);

        return gx1 <= gx2 && gy1 <= gy2;
    }

    void build_road_lod_index() {
        road_edges_.clear();

        const std::size_t cell_count = static_cast<std::size_t>(viz_.grid_cols) *
            static_cast<std::size_t>(viz_.grid_rows);

        for (auto& tier_grid : road_edge_grid_) {
            tier_grid.clear();
            tier_grid.resize(cell_count);
        }

        if (viz_.graph.N == 0 || viz_.graph.offsets.empty() || viz_.cell_size <= 0.0 || cell_count == 0) {
            return;
        }

        std::unordered_set<std::uint64_t> seen_edges;
        seen_edges.reserve(viz_.graph.adjacency.size());
        road_edges_.reserve(viz_.graph.adjacency.size() / 2 + 1);

        for (std::uint32_t u = 0; u < viz_.graph.N; ++u) {
            const std::uint32_t start = (u == 0 ? 0u : viz_.graph.offsets[u - 1]);
            const std::uint32_t end = viz_.graph.offsets[u];

            for (std::uint32_t i = start; i < end && i < viz_.graph.adjacency.size(); ++i) {
                const std::uint32_t v = viz_.graph.adjacency[i];
                if (v >= viz_.graph.N || u == v) continue;

                // For map drawing, direction does not matter. Road graphs often
                // contain both u->v and v->u; drawing both halves costs FPS with no visual gain.
                const std::uint64_t key = make_edge_key(u, v);
                if (!seen_edges.insert(key).second) continue;

                const float speed = edge_speed(i);
                const std::uint8_t tier = road_tier_for_speed(speed);
                const std::uint32_t edge_index = static_cast<std::uint32_t>(road_edges_.size());
                road_edges_.push_back(RoadEdgeItem{ u, v, speed, tier });

                const auto& a = viz_.world_coords[u];
                const auto& b = viz_.world_coords[v];

                // Important: index a road into every spatial cell touched by its bbox,
                // not just by the midpoint. Midpoint-only indexing causes roads to break
                // near viewport/cell boundaries.
                const double x1 = std::min(a.x, b.x);
                const double x2 = std::max(a.x, b.x);
                const double y1 = std::min(a.y, b.y);
                const double y2 = std::max(a.y, b.y);

                const int gx1 = std::clamp(static_cast<int>(std::floor((x1 - viz_.min_x) / viz_.cell_size)), 0, viz_.grid_cols - 1);
                const int gx2 = std::clamp(static_cast<int>(std::floor((x2 - viz_.min_x) / viz_.cell_size)), 0, viz_.grid_cols - 1);
                const int gy1 = std::clamp(static_cast<int>(std::floor((y1 - viz_.min_y) / viz_.cell_size)), 0, viz_.grid_rows - 1);
                const int gy2 = std::clamp(static_cast<int>(std::floor((y2 - viz_.min_y) / viz_.cell_size)), 0, viz_.grid_rows - 1);

                for (int gy = gy1; gy <= gy2; ++gy) {
                    for (int gx = gx1; gx <= gx2; ++gx) {
                        const std::size_t cell = static_cast<std::size_t>(gy) * static_cast<std::size_t>(viz_.grid_cols) +
                            static_cast<std::size_t>(gx);
                        road_edge_grid_[tier][cell].push_back(edge_index);
                    }
                }
            }
        }
    }

    std::uint64_t estimate_indexed_roads(const QRectF& view, int min_tier) const {
        int gx1 = 0, gx2 = 0, gy1 = 0, gy2 = 0;
        if (!grid_range_for_view(view, gx1, gx2, gy1, gy2)) return 0;

        std::uint64_t count = 0;
        const int tier_start = std::clamp(min_tier, 0, 4);

        for (int gy = gy1; gy <= gy2; ++gy) {
            for (int gx = gx1; gx <= gx2; ++gx) {
                const std::size_t cell = static_cast<std::size_t>(gy) * static_cast<std::size_t>(viz_.grid_cols) +
                    static_cast<std::size_t>(gx);

                for (int tier = tier_start; tier <= 4; ++tier) {
                    if (cell < road_edge_grid_[tier].size()) {
                        count += static_cast<std::uint64_t>(road_edge_grid_[tier][cell].size());
                    }
                }
            }
        }

        return count;
    }

    int choose_road_tier_for_view(const QRectF& view) const {
        int tier = base_road_tier_for_zoom();

        // If a dense city is still too heavy at this zoom, raise the LOD until
        // the number of candidate segments is bounded.
        while (tier < 4 && estimate_indexed_roads(view, tier) > kMaxVisibleRoadsBeforeFiltering) {
            ++tier;
        }

        return tier;
    }

    bool road_too_short_on_screen(const RoadEdgeItem& e, double min_px) const {
        const auto& a = viz_.world_coords[e.u];
        const auto& b = viz_.world_coords[e.v];
        const double dx = (a.x - b.x) * zoom_;
        const double dy = (a.y - b.y) * zoom_;
        return dx * dx + dy * dy < min_px * min_px;
    }

    void flush_road_lines(QPainter& painter, QVector<QLineF>& lines) {
        if (lines.isEmpty()) return;
        painter.drawLines(lines.constData(), static_cast<int>(lines.size()));
        last_drawn_roads_ += static_cast<std::uint64_t>(lines.size());
        lines.clear();
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

    void clear_road_tile_cache() {
        road_raster_tile_cache_.clear();
    }

    bool raster_tile_range_for_view(const QRectF& view, int& tx1, int& tx2, int& ty1, int& ty2) const {
        if (zoom_ <= 0.0) return false;

        const double map_width = std::max(0.0, viz_.max_x - viz_.min_x);
        const double map_height = std::max(0.0, viz_.max_y - viz_.min_y);
        const double max_px_x = map_width * zoom_;
        const double max_px_y = map_height * zoom_;
        if (max_px_x <= 0.0 || max_px_y <= 0.0) return false;

        const double left_px = (view.left() - viz_.min_x) * zoom_;
        const double right_px = (view.right() - viz_.min_x) * zoom_;
        const double top_px = (view.top() - viz_.min_y) * zoom_;
        const double bottom_px = (view.bottom() - viz_.min_y) * zoom_;

        if (right_px < 0.0 || left_px > max_px_x || bottom_px < 0.0 || top_px > max_px_y) {
            return false;
        }

        const int max_tx = std::max(0, static_cast<int>(std::ceil(max_px_x / static_cast<double>(kRasterTilePixels))) - 1);
        const int max_ty = std::max(0, static_cast<int>(std::ceil(max_px_y / static_cast<double>(kRasterTilePixels))) - 1);

        tx1 = std::clamp(static_cast<int>(std::floor(left_px / static_cast<double>(kRasterTilePixels))), 0, max_tx);
        tx2 = std::clamp(static_cast<int>(std::floor(right_px / static_cast<double>(kRasterTilePixels))), 0, max_tx);
        ty1 = std::clamp(static_cast<int>(std::floor(top_px / static_cast<double>(kRasterTilePixels))), 0, max_ty);
        ty2 = std::clamp(static_cast<int>(std::floor(bottom_px / static_cast<double>(kRasterTilePixels))), 0, max_ty);

        return tx1 <= tx2 && ty1 <= ty2;
    }

    QRectF raster_tile_world_rect(int tx, int ty) const {
        const double left = viz_.min_x + (static_cast<double>(tx) * static_cast<double>(kRasterTilePixels)) / zoom_;
        const double top = viz_.min_y + (static_cast<double>(ty) * static_cast<double>(kRasterTilePixels)) / zoom_;
        const double right = viz_.min_x + (static_cast<double>(tx + 1) * static_cast<double>(kRasterTilePixels)) / zoom_;
        const double bottom = viz_.min_y + (static_cast<double>(ty + 1) * static_cast<double>(kRasterTilePixels)) / zoom_;
        return QRectF(QPointF(left, top), QPointF(right, bottom));
    }

    QPointF world_to_raster_tile_local(const GraphVisualizer::WorldPoint& p, const QRectF& tile_world) const {
        return QPointF(
            (p.x - tile_world.left()) * zoom_,
            (p.y - tile_world.top()) * zoom_
        );
    }

    QPointF snap_to_pixel_center(const QPointF& p) const {
        return QPointF(std::round(p.x()) + 0.5, std::round(p.y()) + 0.5);
    }

    CachedRoadRasterTile render_road_raster_tile(const RoadTileLevel& level, int level_index, int tx, int ty) {
        CachedRoadRasterTile cached;
        cached.pixmap = QPixmap(kRasterTilePixels, kRasterTilePixels);
        cached.pixmap.fill(Qt::transparent);
        cached.last_used_frame = road_raster_frame_;

        const QRectF tile_world = raster_tile_world_rect(tx, ty);
        const double pixel_pad = 3.0 / std::max(zoom_, 1.0);
        const QRectF query_world = tile_world.adjusted(-pixel_pad, -pixel_pad, pixel_pad, pixel_pad);

        int gx1 = 0, gx2 = 0, gy1 = 0, gy2 = 0;
        if (!tile_range_for_view(query_world, level, gx1, gx2, gy1, gy2)) {
            return cached;
        }

        std::array<QVector<QLineF>, 5> lines_by_tier;
        for (QVector<QLineF>& lines : lines_by_tier) {
            lines.reserve(kRoadLineBatchSize);
        }

        std::unordered_set<std::uint32_t> seen_edges;
        seen_edges.reserve(4096);

        for (int gy = gy1; gy <= gy2; ++gy) {
            for (int gx = gx1; gx <= gx2; ++gx) {
                const std::size_t vector_tile = static_cast<std::size_t>(gy) * static_cast<std::size_t>(level.cols) +
                    static_cast<std::size_t>(gx);
                if (vector_tile >= level.tiles.size()) continue;

                const auto& tile_edges = level.tiles[vector_tile];
                for (std::uint32_t edge_index : tile_edges) {
                    if (edge_index >= road_edges_.size()) continue;
                    if (!seen_edges.insert(edge_index).second) continue;

                    const RoadEdgeItem& edge = road_edges_[edge_index];
                    if (edge.tier < level.min_tier) continue;
                    if (!edge_intersects_view(edge.u, edge.v, query_world)) continue;

                    const int tier = std::clamp(static_cast<int>(edge.tier), 0, 4);
                    lines_by_tier[static_cast<std::size_t>(tier)].push_back(QLineF(
                        snap_to_pixel_center(world_to_raster_tile_local(viz_.world_coords[edge.u], tile_world)),
                        snap_to_pixel_center(world_to_raster_tile_local(viz_.world_coords[edge.v], tile_world))
                    ));
                }
            }
        }

        QPainter tile_painter(&cached.pixmap);
        tile_painter.setRenderHint(QPainter::Antialiasing, false);
        tile_painter.setBrush(Qt::NoBrush);
        tile_painter.setClipRect(QRectF(0.0, 0.0,
            static_cast<double>(kRasterTilePixels),
            static_cast<double>(kRasterTilePixels)));

        // Preserve the exact visual ordering from the direct vector renderer:
        // weak roads first, important roads last.
        for (int tier = level.min_tier; tier <= 4; ++tier) {
            QVector<QLineF>& lines = lines_by_tier[static_cast<std::size_t>(tier)];
            if (lines.isEmpty()) continue;

            QPen pen(road_color_for_tier(tier));
            pen.setWidthF(road_width_for_tier(tier));
            pen.setCosmetic(true);
            pen.setCapStyle(Qt::FlatCap);
            pen.setJoinStyle(Qt::MiterJoin);
            tile_painter.setPen(pen);

            int offset = 0;
            const int line_count = static_cast<int>(lines.size());
            while (offset < line_count) {
                const int count = std::min(kRoadLineBatchSize, line_count - offset);
                tile_painter.drawLines(lines.constData() + offset, count);
                cached.roads_drawn += static_cast<std::uint64_t>(count);
                offset += count;
            }
        }

        return cached;
    }

    CachedRoadRasterTile& get_road_raster_tile(const RoadTileLevel& level, int level_index, int tx, int ty) {
        const RasterTileKey key{ level_index, tx, ty };
        auto it = road_raster_tile_cache_.find(key);
        if (it == road_raster_tile_cache_.end()) {
            CachedRoadRasterTile cached = render_road_raster_tile(level, level_index, tx, ty);
            it = road_raster_tile_cache_.emplace(key, std::move(cached)).first;
        }

        it->second.last_used_frame = road_raster_frame_;
        return it->second;
    }

    void prune_road_tile_cache() {
        if (road_raster_tile_cache_.size() <= kMaxCachedRasterTiles) return;

        std::vector<std::pair<int, RasterTileKey>> by_age;
        by_age.reserve(road_raster_tile_cache_.size());
        for (const auto& kv : road_raster_tile_cache_) {
            by_age.emplace_back(kv.second.last_used_frame, kv.first);
        }

        std::sort(by_age.begin(), by_age.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs.first < rhs.first;
            });

        const std::size_t erase_count = road_raster_tile_cache_.size() - kMaxCachedRasterTiles;
        for (std::size_t i = 0; i < erase_count && i < by_age.size(); ++i) {
            road_raster_tile_cache_.erase(by_age[i].second);
        }
    }

    void draw_roads(QPainter& painter) {
        last_drawn_roads_ = 0;
        last_candidate_roads_ = 0;
        last_total_visible_roads_ = 0;
        last_min_speed_kph_ = 0;
        last_tile_level_ = 0;

        if (road_edges_.empty()) return;

        const int level_index = std::clamp(tile_level_for_zoom(), 0, static_cast<int>(road_tile_levels_.size()) - 1);
        const RoadTileLevel& level = road_tile_levels_[static_cast<std::size_t>(level_index)];
        if (level.tiles.empty()) return;

        last_tile_level_ = level_index;
        last_min_speed_kph_ = min_kph_for_tier(level.min_tier);

        const QRectF view = visible_world_rect();

        int tx1 = 0, tx2 = 0, ty1 = 0, ty2 = 0;
        if (!raster_tile_range_for_view(view, tx1, tx2, ty1, ty2)) return;

        ++road_raster_frame_;
        painter.setRenderHint(QPainter::Antialiasing, false);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, false);

        for (int ty = ty1; ty <= ty2; ++ty) {
            for (int tx = tx1; tx <= tx2; ++tx) {
                CachedRoadRasterTile& cached = get_road_raster_tile(level, level_index, tx, ty);

                const QRectF tile_world = raster_tile_world_rect(tx, ty);
                const QPointF tile_screen = world_to_screen(GraphVisualizer::WorldPoint{
                    tile_world.left(),
                    tile_world.top()
                    });

                painter.drawPixmap(tile_screen, cached.pixmap);
                last_drawn_roads_ += cached.roads_drawn;
                last_total_visible_roads_ += cached.roads_drawn;
                last_candidate_roads_ += cached.roads_drawn;
            }
        }

        prune_road_tile_cache();
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
        if (viz_.routes.empty()) return;

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

            draw_arrow_on_line(painter, world_to_screen(viz_.world_coords[seg.a]),
                world_to_screen(viz_.world_coords[seg.b]));
        }

        draw_route_endpoint_markers(painter);
    }

    void draw_route_endpoint_markers(QPainter& painter) {
        // СПОСОБ 1: Идеальный. Рисуем маркеры по исходным координатам клиентов (waypoints).
        // Это игнорирует проблемы графа дорог, односторонних улиц и разбиения на леги.
        if (!viz_.waypoints.empty()) {

            // 1. СНАЧАЛА: Рисуем пунктирные линии "привязки" от зданий до дороги
            painter.save();
            painter.setPen(QPen(QColor(120, 120, 120, 180), 1.5, Qt::DashLine));

            for (std::size_t i = 0; i < viz_.waypoints.size(); ++i) {
                const Waypoint& wp = viz_.waypoints[i];
                double x = viz_.to_world(wp.lat, wp.lon).x;
                double y = viz_.to_world(wp.lat, wp.lon).y;
                QPointF wp_screen = world_to_screen(viz_.to_world(wp.lat, wp.lon));

                double min_dist = std::numeric_limits<double>::max();
                QPointF closest_road_screen = wp_screen;

                // Ищем ближайший узел дороги (таргет или старт/стоп маршрута)
                for (const Route& route : viz_.routes) {
                    if (!route.visible || route.nodes.empty()) continue;

                    std::vector<std::uint32_t> check_nodes;
                    if (!route.targets.empty()) {
                        for (const auto& t : route.targets) check_nodes.push_back(t.dense_id);
                    }
                    else {
                        check_nodes.push_back(route.nodes.front());
                        check_nodes.push_back(route.nodes.back());
                    }

                    for (std::uint32_t node : check_nodes) {
                        if (node >= viz_.graph.N) continue;
                        double p_worldx = viz_.world_coords[node].x;
                        double p_worldy = viz_.world_coords[node].y;

                        double dx = p_worldx - x;
                        double dy = p_worldy - y;
                        double dist = dx * dx + dy * dy;

                        if (dist < min_dist) {
                            min_dist = dist;
                            closest_road_screen = world_to_screen(viz_.world_coords[node]);
                        }
                    }
                }

                // Если дистанция адекватная, рисуем пунктир соединения
                if (min_dist < 1e-8 && wp_screen != closest_road_screen) {
                    painter.drawLine(wp_screen, closest_road_screen);
                }
            }
            painter.restore();

            // 2. ЗАТЕМ: Рисуем сами красивые большие маркеры
            for (std::size_t i = 0; i < viz_.waypoints.size(); ++i) {
                const Waypoint& wp = viz_.waypoints[i];
                QPointF screen_pos = world_to_screen(viz_.to_world(wp.lat, wp.lon));
                bool is_depot = (i == 0); // Нулевой индекс - синий маркер

                draw_marker(painter, screen_pos, marker_label(i), is_depot);
            }

            return; // Выходим, идеальные маркеры расставлены!
        }

        // СПОСОБ 2: Запасной (Fallback). Если waypoints не переданы, 
        // пытаемся склеить узлы графа, но с ГОРАЗДО бОльшим радиусом (около 150 метров).
        struct EndpointMarker {
            std::uint32_t node = 0;
            int label_index = 0;
            bool is_depot = false;
        };

        std::vector<EndpointMarker> markers;

        auto add_endpoint = [&](std::uint32_t node, bool is_depot) {
            if (node >= viz_.graph.N) return;
            const auto& p1 = viz_.world_coords[node];

            for (auto& m : markers) {
                const auto& p2 = viz_.world_coords[m.node];
                double dx = p1.x - p2.x;
                double dy = p1.y - p2.y;

                // Увеличенный порог: 5e-10 в радианах это примерно 140-150 метров.
                // Склеит точки въезда и выезда со двора в один логический маркер.
                if (dx * dx + dy * dy < 5e-10) {
                    if (is_depot) m.is_depot = true;
                    return;
                }
            }

            const int label_index = static_cast<int>(markers.size());
            markers.push_back(EndpointMarker{ node, label_index, is_depot });
            };

        for (const Route& route : viz_.routes) {
            if (!route.visible) continue;

            if (!route.targets.empty()) {
                for (const auto& target : route.targets) {
                    add_endpoint(target.dense_id, target.is_depot);
                }
            }
            else if (route.nodes.size() >= 2) {
                // Извлекаем только старт и финиш текущего сегмента
                add_endpoint(route.nodes.front(), true);
                add_endpoint(route.nodes.back(), false);
            }
        }

        for (const EndpointMarker& marker : markers) {
            draw_marker(painter,
                world_to_screen(viz_.world_coords[marker.node]),
                marker_label(marker.label_index),
                marker.is_depot);
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

    void draw_marker(QPainter& painter, const QPointF& p, const QString& text, bool is_depot = false) {
        const QColor target_color(230, 60, 60); // Красный для обычных точек
        const QColor depot_color(60, 150, 230); // Синий для депо

        painter.setPen(QPen(Qt::white, 2.0));

        // Устанавливаем цвет кисти в зависимости от флага is_depot
        painter.setBrush(is_depot ? depot_color : target_color);
        painter.drawEllipse(p, 8.5, 8.5);

        painter.setPen(Qt::white);
        const QRectF label_rect(p.x() - 8.5, p.y() - 9.0, 17.0, 17.0);
        painter.drawText(label_rect, Qt::AlignCenter, text);
    }

    void draw_arrow_on_line(QPainter& painter, const QPointF& from, const QPointF& to) {
        QLineF line(from, to);
        // Если сегмент на экране слишком короткий, не рисуем
        if (line.length() < 28.0) return;

        // Находим точку на 60% длины отрезка (чтобы стрелки не слипались в углах)
        QPointF mid = line.pointAt(0.6);

        double angle = std::atan2(line.dy(), line.dx());

        double arrow_size = 7.0;    // Размер наконечника
        double arrow_angle = 0.5;   // Угол разлета «ушек»

        
        QPointF arrow_p1 = mid - QPointF(std::cos(angle + arrow_angle), std::sin(angle + arrow_angle)) * arrow_size;
        QPointF arrow_p2 = mid - QPointF(std::cos(angle - arrow_angle), std::sin(angle - arrow_angle)) * arrow_size;

        // Точка "вдавленности" сзади наконечника
        QPointF arrow_base = mid - QPointF(std::cos(angle), std::sin(angle)) * (arrow_size * 0.6);

        // Короткий, аккуратный хвостик (линия)
        QPointF tail_end = mid - QPointF(std::cos(angle), std::sin(angle)) * (arrow_size * 1.8);

        painter.save();
        painter.setRenderHint(QPainter::Antialiasing, true);

        // 1. Отрисовка хвостика стрелки
        painter.setPen(QPen(Qt::black, 3.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(arrow_base, tail_end);
        painter.setPen(QPen(Qt::white, 1.2, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(arrow_base, tail_end);

        // 2. Отрисовка самого наконечника
        painter.setPen(QPen(Qt::black, 1.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(Qt::white);

        QPolygonF arrow_head;
        arrow_head << mid << arrow_p1 << arrow_base << arrow_p2;
        painter.drawPolygon(arrow_head);

        painter.restore();
    }

    void draw_overlay(QPainter& painter) {
        painter.setRenderHint(QPainter::Antialiasing, true);

        constexpr int base_overlay_height = 112;
        constexpr int route_row_height = 20;
        constexpr std::size_t max_route_rows_in_overlay = 12;

        const std::size_t route_count = viz_.routes.size();
        const std::size_t route_rows = route_count == 0
            ? 1
            : std::min<std::size_t>(route_count, max_route_rows_in_overlay);
        const bool routes_truncated = route_count > route_rows;
        const int extra_rows = static_cast<int>(route_rows) + (routes_truncated ? 1 : 0);
        const int overlay_height = base_overlay_height + extra_rows * route_row_height;

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 0, 0, 165));
        painter.drawRoundedRect(QRectF(10, 10, 980, overlay_height), 8, 8);

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

        const RoadTileLevel& overlay_level = road_tile_levels_[static_cast<std::size_t>(std::clamp(last_tile_level_, 0, 4))];

        painter.drawText(24, 58,
            QString("Renderer: raster tile cache LOD %1 (%2)    Road layer: %3    cached segments: %4")
            .arg(last_tile_level_)
            .arg(overlay_level.name)
            .arg(filter_text)
            .arg(static_cast<qulonglong>(last_total_visible_roads_))
        );

        painter.drawText(24, 82,
            "Mouse drag - pan | Wheel - zoom | R - fit route/map | shared routes use nested color bands"
        );

        if (route_count == 0) {
            painter.drawText(24, 106, "Routes: 0    No route was loaded from result.txt");
            return;
        }

        painter.drawText(24, 106, QString("Routes: %1").arg(route_count));

        int y = 128;
        for (std::size_t i = 0; i < route_rows; ++i) {
            const Route& r = viz_.routes[i];
            const QString label = r.label.empty()
                ? QString("Route %1").arg(i + 1)
                : QString::fromStdString(r.label);

            painter.setPen(Qt::NoPen);
            painter.setBrush(to_qcolor(route_palette_color(i)));
            painter.drawRoundedRect(QRectF(24, y - 9, 26, 7), 3, 3);

            // Формируем красивую строку времени
            QString time_info = format_time_hms(r.total_time_s); // Общая длительность
            if (!r.targets.empty()) {
                // Добавляем [Время_Старта -> Время_Финиша]
                if ((r.targets.back().arrival_time_s - r.targets.front().arrival_time_s) > 1e-12)
                    time_info += QString(" [%1 → %2]")
                        .arg(format_time_hms(r.targets.front().arrival_time_s))
                        .arg(format_time_hms(r.targets.back().arrival_time_s));
            }

            painter.setPen(QColor(245, 245, 245));
            painter.drawText(58, y,
                QString("%1: %2 nodes | %3 m | %4")
                .arg(label)
                .arg(r.nodes.size())
                .arg(r.total_dist_m, 0, 'f', 1)
                .arg(time_info) // Подставляем расширенную строку времени
            );

            y += route_row_height;
        }

        if (routes_truncated) {
            painter.setPen(QColor(210, 210, 210));
            painter.drawText(58, y, QString("... and %1 more routes").arg(route_count - route_rows));
        }
    }
};

QWidget* GraphVisualizer::createWidget(QWidget* parent) {
    if (graph.N == 0) return nullptr;

    m_activeWidget = new QtGraphWidget(*this, parent);
    return m_activeWidget;
}

// Метод для принудительного обновления экрана
void GraphVisualizer::updateVisuals() {
    if (m_activeWidget) {
        m_activeWidget->update(); // Вызывает paintEvent внутри Qt
    }
}

// Обновленный run() — теперь он не ломает программу, если QApplication уже создан главным окном
void GraphVisualizer::run() {
    if (graph.N == 0) return;

    int argc = 1;
    char app_name[] = "graph_builder";
    char* argv[] = { app_name, nullptr };

    // Проверяем, существует ли уже QApplication (если запустили из GUI)
    QApplication* app = qobject_cast<QApplication*>(QApplication::instance());
    bool own_app = false;

    if (!app) {
        app = new QApplication(argc, argv);
        own_app = true;
    }

    QtGraphWidget window(*this);
    window.show();

    if (own_app) {
        app->exec();
        delete app;
    }
}
