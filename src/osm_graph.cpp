#include "osm_graph.hpp"

#include <osmium/geom/haversine.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>

#include <unordered_set>
#include <string>
#include <iostream>

namespace {

    bool is_drivable_highway(const char* highway) {
        if (!highway) return false;
        static const std::unordered_set<std::string> ok = {
            "motorway","trunk","primary","secondary","tertiary",
            "unclassified","residential","service","living_street"
        };
        return ok.count(highway) > 0;
    }

    bool is_oneway(const osmium::TagList& tags) {
        const char* v = tags.get_value_by_key("oneway");
        if (!v) return false;
        const std::string s(v);
        return (s == "yes" || s == "1" || s == "true");
    }

    struct WayHandler : public osmium::handler::Handler {
        OsmGraph& g;
        explicit WayHandler(OsmGraph& graph) : g(graph) {}

        void way(const osmium::Way& w) {
            const char* highway = w.tags().get_value_by_key("highway");
            if (!is_drivable_highway(highway)) return;

            const bool oneway = is_oneway(w.tags());
            const auto& nr = w.nodes();
            if (nr.size() < 2) return;

            for (std::size_t i = 1; i < nr.size(); ++i) {
                const auto& a = nr[i - 1];
                const auto& b = nr[i];

                if (!a.location().valid() || !b.location().valid()) continue;

                const double dist_m = osmium::geom::haversine::distance(a.location(), b.location());

                const std::int64_t from = a.ref();
                const std::int64_t to = b.ref();

                g.nodes[from] = OsmNode{ from, a.location().lat(), a.location().lon() };
                g.nodes[to] = OsmNode{ to,   b.location().lat(), b.location().lon() };

                g.edges.push_back(OsmEdge{ from, to, dist_m, oneway });
                if (!oneway) {
                    g.edges.push_back(OsmEdge{ to, from, dist_m, oneway });
                }
            }
        }
    };

} // namespace

OsmGraph build_graph_from_pbf(const std::string& pbf_path) {
    OsmGraph g;

    osmium::io::Reader reader{ pbf_path };

    using index_type =
        osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>;

    using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

    index_type index;
    location_handler_type location_handler{ index };
    location_handler.ignore_errors();

    WayHandler way_handler{ g };
    osmium::apply(reader, location_handler, way_handler);


    reader.close();
    return g;
}
