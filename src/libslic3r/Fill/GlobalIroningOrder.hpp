#pragma once

#include <algorithm>
#include <utility>

namespace Slic3r {
namespace detail {

// Input consists of clipped, straight, vertical segments in scan coordinates.
// Sort all spans in a column before moving to the next column. Only join
// adjacent columns when the caller proves that the link is safe to extrude.
template<class Paths, class Coord, class CanLink>
Paths order_global_ironing(Paths spans, Coord first_x, Coord step, CanLink can_link)
{
    Paths out;
    if (step <= 0)
        return out;
    spans.erase(std::remove_if(spans.begin(), spans.end(), [](const auto &p) {
        return p.points.size() < 2 || p.first_point() == p.last_point();
    }), spans.end());
    const auto low_y = [](const auto &p) {
        return std::min(p.first_point().y(), p.last_point().y());
    };
    const auto high_y = [](const auto &p) {
        return std::max(p.first_point().y(), p.last_point().y());
    };
    std::stable_sort(spans.begin(), spans.end(), [&](const auto &a, const auto &b) {
        const auto ax = a.first_point().x(), bx = b.first_point().x();
        if (ax != bx) return ax < bx;
        if (low_y(a) != low_y(b)) return low_y(a) < low_y(b);
        return high_y(a) < high_y(b);
    });
    Coord previous_x = first_x;
    for (auto begin = spans.begin(); begin != spans.end();) {
        const Coord x = begin->first_point().x();
        auto end = begin + 1;
        while (end != spans.end() && end->first_point().x() == x)
            ++end;
        const bool upward = ((x - first_x) / step) % 2 == 0;
        if (!upward)
            std::reverse(begin, end);
        for (auto it = begin; it != end; ++it) {
            if ((it->first_point().y() < it->last_point().y()) != upward)
                it->reverse();
            // Never join separate spans in the same column: there may be a hole.
            if (!out.empty() && x - previous_x == step &&
                can_link(out.back().last_point(), it->first_point())) {
                out.back().append(it->points);
            } else {
                out.emplace_back(std::move(*it));
            }
            previous_x = x;
        }
        begin = end;
    }
    return out;
}

} // namespace detail
} // namespace Slic3r
