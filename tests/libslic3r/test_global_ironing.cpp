#include <catch2/catch.hpp>
#include <memory>
#include <cmath>
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Surface.hpp"

using namespace Slic3r;

namespace {
Point mm(double x, double y) { return Point(scale_(x), scale_(y)); }
FillGlobalRectilinearIroning filler()
{
    FillGlobalRectilinearIroning f;
    f.spacing = 1.;
    f.angle = -float(M_PI / 2.);
    f.link_max_length = scale_(3.);
    return f;
}
FillParams params()
{
    FillParams p;
    p.density = 1.f;
    p.extrusion_role = erIroning;
    return p;
}
}

TEST_CASE("Global rectilinear ironing is registered and ordered", "[global_ironing]")
{
    std::unique_ptr<Fill> f(Fill::new_from_type(ipIroningGlobalRectilinear));
    REQUIRE(dynamic_cast<FillGlobalRectilinearIroning*>(f.get()) != nullptr);
    REQUIRE(f->no_sort());
    REQUIRE_FALSE(f->use_bridge_flow());
    std::unique_ptr<Fill> from_key(Fill::new_from_type("ironingglobalrectilinear"));
    REQUIRE(dynamic_cast<FillGlobalRectilinearIroning*>(from_key.get()) != nullptr);
}

TEST_CASE("Global rectilinear ironing sweeps a cutout without extruding across it", "[global_ironing]")
{
    const Polygon outer{mm(0,0), mm(20,0), mm(20,20), mm(0,20)};
    const Polygon hole{mm(5,5), mm(5,15), mm(15,15), mm(15,5)};
    const Surface s(stTop, ExPolygon(outer, hole));
    auto f = filler();
    const auto path = f.fill_surface(&s, params());
    REQUIRE_FALSE(path.empty());
    coord_t preceding_x = -scale_(1.);
    size_t hole_columns = 0, full_columns = 0;
    for (const auto &p : path) {
        REQUIRE(p.points.size() >= 2);
        REQUIRE(intersection_pl(Polylines{p}, hole).empty());
        for (size_t i = 1; i < p.points.size(); ++i) {
            const Point &a = p.points[i-1], &b = p.points[i];
            REQUIRE(b.x() >= a.x());
            if (a.x() == b.x()) {
                REQUIRE(a.x() >= preceding_x);
                preceding_x = a.x();
                if (a.x() > scale_(5.) && a.x() < scale_(15.)) ++hole_columns;
                else ++full_columns;
            }
        }
    }
    REQUIRE(hole_columns == 20);
    REQUIRE(full_columns == 10);
    REQUIRE(f.spacing == Approx(1.));
}

TEST_CASE("Global rectilinear ironing clips rotated strokes around multiple holes", "[global_ironing]")
{
    const Polygon outer{mm(-20,-15), mm(20,-15), mm(20,15), mm(-20,15)};
    const Polygon h1{mm(-12,-6), mm(-12,6), mm(-6,6), mm(-6,-6)};
    const Polygon h2{mm(5,-7), mm(5,3), mm(13,3), mm(13,-7)};
    ExPolygon area(outer, h1);
    area.holes.push_back(h2);
    const Surface s(stTop, std::move(area));
    auto f = filler();
    f.angle = float(M_PI / 4.);
    const auto path = f.fill_surface(&s, params());
    REQUIRE_FALSE(path.empty());
    for (const auto &p : path) {
        REQUIRE(p.points.size() >= 2);
        REQUIRE(intersection_pl(Polylines{p}, h1).empty());
        REQUIRE(intersection_pl(Polylines{p}, h2).empty());
    }
}

TEST_CASE("Global rectilinear ironing handles empty and narrow regions", "[global_ironing]")
{
    auto f = filler();
    const Surface empty(stTop, ExPolygon());
    REQUIRE(f.fill_surface(&empty, params()).empty());
    const Polygon narrow{mm(0,0), mm(.6,0), mm(.6,8), mm(0,8)};
    const Surface s(stTop, ExPolygon(narrow));
    REQUIRE_FALSE(f.fill_surface(&s, params()).empty());
    f.spacing = 0.;
    REQUIRE(f.fill_surface(&s, params()).empty());
}
