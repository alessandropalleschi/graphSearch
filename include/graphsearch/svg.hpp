#pragma once
// Dependency-free SVG rendering of search results (header-only, no I/O).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "graph.hpp"
#include "heuristics.hpp"

namespace graphsearch::svg {

inline std::string escape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
            case '&': o += "&amp;"; break;
            case '<': o += "&lt;"; break;
            case '>': o += "&gt;"; break;
            case '"': o += "&quot;"; break;
            default: o += c;
        }
    }
    return o;
}

inline std::string num(double v, int prec = 2) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

inline std::string thousands(std::size_t v) {
    std::string s = std::to_string(v), o;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (i && (s.size() - i) % 3 == 0) o += ',';
        o += s[i];
    }
    return o;
}

constexpr int kBins = 16;

/// Shared stylesheet: 16-step blue ramp (early -> late expansion) and text styles.
inline std::string style() {
    std::ostringstream o;
    o << "<style>text{font-family:-apple-system,Segoe UI,Helvetica,Arial,sans-serif;fill:#1f2937}"
         ".h{font-size:18px;font-weight:600}.t{font-size:13px;font-weight:600}"
         ".s{font-size:11px;fill:#4b5563}.ax{font-size:11px;fill:#6b7280}";
    for (int i = 0; i < kBins; ++i) {
        const double f = double(i) / (kBins - 1);  // #dbeafe -> #2563eb
        const int r = int(0xdb + (0x25 - 0xdb) * f), g = int(0xea + (0x63 - 0xea) * f),
                  b = int(0xfe + (0xeb - 0xfe) * f);
        char col[16];
        std::snprintf(col, sizeof col, "#%02x%02x%02x", r, g, b);
        o << ".e" << i << "{fill:" << col << "}";
    }
    o << ".free{fill:#f3f4f6}.wall{fill:#1f2937}</style>";
    return o.str();
}

// ---------------------------------------------------------------------------------------------
// Grid panels
// ---------------------------------------------------------------------------------------------

struct GridPanel {
    std::string title;
    std::vector<NodeId> trace;  ///< expanded nodes in order
    std::vector<NodeId> path;
    bool found = false;
    double cost = 0;
    std::size_t expanded = 0;
};

inline std::string gridPanel(std::size_t w, std::size_t h, const std::vector<char>& blocked,
                             const GridPanel& p, double x0, double y0, double cell) {
    std::ostringstream o;
    const double gy = y0 + 44;
    o << "<text class=\"t\" x=\"" << x0 << "\" y=\"" << y0 + 14 << "\">" << escape(p.title) << "</text>";
    o << "<text class=\"s\" x=\"" << x0 << "\" y=\"" << y0 + 31 << "\">expanded "
      << thousands(p.expanded) << " · cost " << (p.found ? num(p.cost) : std::string("n/a"))
      << "</text>";
    o << "<rect class=\"free\" x=\"" << x0 << "\" y=\"" << gy << "\" width=\"" << w * cell
      << "\" height=\"" << h * cell << "\"/>";

    // Obstacles: one rect per horizontal run.
    for (std::size_t y = 0; y < h; ++y)
        for (std::size_t x = 0; x < w;) {
            if (!blocked[y * w + x]) { ++x; continue; }
            std::size_t e = x;
            while (e < w && blocked[y * w + e]) ++e;
            o << "<rect class=\"wall\" x=\"" << x0 + x * cell << "\" y=\"" << gy + y * cell
              << "\" width=\"" << (e - x) * cell << "\" height=\"" << cell << "\"/>";
            x = e;
        }
    // Expanded cells coloured by expansion order (first expansion wins).
    std::vector<signed char> bin(w * h, -1);
    for (std::size_t i = 0; i < p.trace.size(); ++i) {
        signed char& b = bin[p.trace[i]];
        if (b < 0) b = static_cast<signed char>(std::min<std::size_t>(kBins - 1, i * kBins / p.trace.size()));
    }
    for (std::size_t y = 0; y < h; ++y)
        for (std::size_t x = 0; x < w; ++x)
            if (bin[y * w + x] >= 0)
                o << "<rect class=\"e" << int(bin[y * w + x]) << "\" x=\"" << x0 + x * cell << "\" y=\""
                  << gy + y * cell << "\" width=\"" << cell << "\" height=\"" << cell << "\"/>";

    auto cx = [&](NodeId n) { return x0 + (n % w + 0.5) * cell; };
    auto cy = [&](NodeId n) { return gy + (n / w + 0.5) * cell; };
    if (p.found && p.path.size() > 1) {
        std::ostringstream pts;
        for (NodeId n : p.path) pts << num(cx(n)) << ',' << num(cy(n)) << ' ';
        for (const char* col : {"#ffffff", "#f97316"}) {  // white halo, then the path
            const bool halo = col[1] == 'f' && col[2] == 'f';
            o << "<polyline points=\"" << pts.str() << "\" fill=\"none\" stroke=\"" << col
              << "\" stroke-width=\"" << (halo ? cell * 0.9 : cell * 0.5)
              << "\" stroke-linejoin=\"round\" stroke-linecap=\"round\"/>";
        }
    }
    const double r = std::max(3.0, cell * 0.9);
    o << "<circle cx=\"" << cx(0) << "\" cy=\"" << cy(0) << "\" r=\"" << r
      << "\" fill=\"#16a34a\" stroke=\"#fff\" stroke-width=\"1.5\"/>";
    o << "<circle cx=\"" << cx(w * h - 1) << "\" cy=\"" << cy(w * h - 1) << "\" r=\"" << r
      << "\" fill=\"#dc2626\" stroke=\"#fff\" stroke-width=\"1.5\"/>";
    return o.str();
}

/// Side-by-side comparison of several algorithms on the same grid.
inline std::string gridComparison(std::size_t w, std::size_t h, const std::vector<char>& blocked,
                                  const std::vector<GridPanel>& panels, std::size_t columns,
                                  double cell, const std::string& heading,
                                  const std::string& subheading) {
    const double pad = 24, pw = w * cell, ph = h * cell + 44;
    const std::size_t rows = (panels.size() + columns - 1) / columns;
    const double W = pad + columns * (pw + pad), top = 62, H = top + rows * (ph + pad) + 34;
    std::ostringstream o;
    o << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << W << ' ' << H << "\" width=\"" << W
      << "\" height=\"" << H << "\" role=\"img\" aria-label=\"" << escape(heading) << "\">" << style()
      << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\" rx=\"8\"/>";
    o << "<text class=\"h\" x=\"" << pad << "\" y=\"28\">" << escape(heading) << "</text>";
    o << "<text class=\"s\" x=\"" << pad << "\" y=\"46\">" << escape(subheading) << "</text>";
    for (std::size_t i = 0; i < panels.size(); ++i)
        o << gridPanel(w, h, blocked, panels[i], pad + (i % columns) * (pw + pad),
                       top + (i / columns) * (ph + pad), cell);
    // Legend
    const double ly = H - 14;
    double lx = pad;
    auto item = [&](const char* cls, const char* fill, const char* label, bool circle) {
        if (circle) o << "<circle cx=\"" << lx + 5 << "\" cy=\"" << ly - 4 << "\" r=\"5\" fill=\"" << fill << "\"/>";
        else o << "<rect class=\"" << cls << "\" x=\"" << lx << "\" y=\"" << ly - 9 << "\" width=\"10\" height=\"10\"/>";
        o << "<text class=\"s\" x=\"" << lx + 15 << "\" y=\"" << ly << "\">" << label << "</text>";
        lx += 15 + 7.0 * std::string(label).size() + 14;
    };
    item("wall", "", "obstacle", false);
    item("e2", "", "expanded early", false);
    item("e13", "", "expanded late", false);
    o << "<rect x=\"" << lx << "\" y=\"" << ly - 6 << "\" width=\"14\" height=\"4\" rx=\"2\" fill=\"#f97316\"/>"
      << "<text class=\"s\" x=\"" << lx + 19 << "\" y=\"" << ly << "\">path</text>";
    lx += 19 + 7.0 * 4 + 14;
    item("", "#16a34a", "start", true);
    item("", "#dc2626", "goal", true);
    o << "</svg>\n";
    return o.str();
}

// ---------------------------------------------------------------------------------------------
// Graph with coordinates
// ---------------------------------------------------------------------------------------------

template <CostType Cost>
std::string graphDiagram(const Graph<Cost>& g, const std::vector<Point>& pts,
                         const std::vector<NodeId>& path, const std::string& heading) {
    constexpr double W = 760, M = 70, R = 17;
    double minx = 1e300, maxx = -1e300, miny = 1e300, maxy = -1e300;
    for (const auto& p : pts) {
        minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
    }
    const double sc = std::min((W - 2 * M) / std::max(1e-9, maxx - minx), 140.0);
    const double H = std::max(260.0, 2 * M + 30 + (maxy - miny) * sc);
    auto X = [&](NodeId n) { return M + (pts[n].x - minx) * sc; };
    auto Y = [&](NodeId n) { return H - M - (pts[n].y - miny) * sc; };  // y up

    std::set<std::pair<NodeId, NodeId>> onPath;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) onPath.insert({path[i], path[i + 1]});
    std::set<NodeId> nodesOnPath(path.begin(), path.end());

    std::ostringstream o;
    o << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << W << ' ' << H << "\" width=\"" << W
      << "\" height=\"" << H << "\" role=\"img\" aria-label=\"" << escape(heading) << "\">" << style()
      << "<defs><marker id=\"a\" viewBox=\"0 0 10 10\" refX=\"9\" refY=\"5\" markerUnits=\"userSpaceOnUse\" markerWidth=\"10\" markerHeight=\"10\" orient=\"auto\">"
         "<path d=\"M0,0 L10,5 L0,10 z\" fill=\"#9ca3af\"/></marker>"
         "<marker id=\"b\" viewBox=\"0 0 10 10\" refX=\"9\" refY=\"5\" markerUnits=\"userSpaceOnUse\" markerWidth=\"10\" markerHeight=\"10\" orient=\"auto\">"
         "<path d=\"M0,0 L10,5 L0,10 z\" fill=\"#f97316\"/></marker></defs>"
      << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\" rx=\"8\"/>"
      << "<text class=\"h\" x=\"24\" y=\"32\">" << escape(heading) << "</text>";

    // Draw ordinary edges first, then path edges on top.
    for (int pass = 0; pass < 2; ++pass)
        for (NodeId u = 0; u < g.size(); ++u)
            for (const auto& e : g.neighbours(u)) {
                const bool hot = onPath.count({u, e.to}) > 0;
                if (hot != (pass == 1)) continue;
                double dx = X(e.to) - X(u), dy = Y(e.to) - Y(u);
                const double len = std::max(1e-9, std::hypot(dx, dy));
                dx /= len; dy /= len;
                // Offset reciprocal edges sideways so both directions stay visible.
                double ox = -dy * 4, oy = dx * 4;
                const double x1 = X(u) + dx * R + ox, y1 = Y(u) + dy * R + oy;
                const double x2 = X(e.to) - dx * (R + 2) + ox, y2 = Y(e.to) - dy * (R + 2) + oy;
                o << "<line x1=\"" << num(x1) << "\" y1=\"" << num(y1) << "\" x2=\"" << num(x2) << "\" y2=\""
                  << num(y2) << "\" stroke=\"" << (hot ? "#f97316" : "#9ca3af") << "\" stroke-width=\""
                  << (hot ? 3 : 1.5) << "\" marker-end=\"url(#" << (hot ? 'b' : 'a') << ")\"/>";
                const double mx = (x1 + x2) / 2 + ox * 2.2, my = (y1 + y2) / 2 + oy * 2.2;
                o << "<text class=\"s\" x=\"" << num(mx) << "\" y=\"" << num(my + 4)
                  << "\" text-anchor=\"middle\" style=\"paint-order:stroke;stroke:#fff;stroke-width:3px;"
                  << (hot ? "fill:#c2410c;font-weight:600" : "") << "\">" << num(double(e.weight), 1) << "</text>";
            }
    for (NodeId n = 0; n < g.size(); ++n) {
        const bool hot = nodesOnPath.count(n) > 0;
        const char* fill = !path.empty() && n == path.front() ? "#16a34a"
                           : !path.empty() && n == path.back() ? "#dc2626"
                           : hot ? "#fed7aa" : "#fff";
        const bool ends = !path.empty() && (n == path.front() || n == path.back());
        o << "<circle cx=\"" << num(X(n)) << "\" cy=\"" << num(Y(n)) << "\" r=\"" << R << "\" fill=\"" << fill
          << "\" stroke=\"" << (hot ? "#f97316" : "#6b7280") << "\" stroke-width=\"2\"/>"
          << "<text class=\"t\" x=\"" << num(X(n)) << "\" y=\"" << num(Y(n) + 4.5)
          << "\" text-anchor=\"middle\"" << (ends ? " style=\"fill:#fff\"" : "") << ">" << n << "</text>";
    }
    o << "</svg>\n";
    return o.str();
}

// ---------------------------------------------------------------------------------------------
// Horizontal log-scale bar chart (benchmarks)
// ---------------------------------------------------------------------------------------------

struct Bar {
    std::string label;
    double value;     ///< milliseconds
    std::string note; ///< right-hand annotation, e.g. "839,070 expanded"
    int colour;       ///< index into the palette
};
struct BarGroup {
    std::string title;
    std::vector<Bar> bars;
};

inline std::string barChart(const std::vector<BarGroup>& groups, const std::string& heading,
                            const std::string& unitNote) {
    static const char* pal[] = {"#4c78a8", "#f58518", "#54a24b", "#b279a2", "#e45756", "#72b7b2", "#9d755d"};
    constexpr double W = 860, L = 150, R = 190, rowH = 24, pad = 22;
    double vmin = 1e300, vmax = 0;
    for (const auto& gr : groups)
        for (const auto& b : gr.bars) {
            vmin = std::min(vmin, b.value);
            vmax = std::max(vmax, b.value);
        }
    const double lo = std::floor(std::log10(std::max(vmin, 1e-4)) - 0.3), hi = std::ceil(std::log10(vmax) + 0.05);
    const double plotW = W - L - R;
    auto xOf = [&](double v) { return L + (std::log10(std::max(v, 1e-4)) - lo) / (hi - lo) * plotW; };

    double H = 70;
    for (const auto& gr : groups) H += 72 + gr.bars.size() * rowH;
    std::ostringstream o;
    o << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << W << ' ' << H << "\" width=\"" << W
      << "\" height=\"" << H << "\" role=\"img\" aria-label=\"" << escape(heading) << "\">" << style()
      << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\" rx=\"8\"/>"
      << "<text class=\"h\" x=\"24\" y=\"30\">" << escape(heading) << "</text>"
      << "<text class=\"s\" x=\"24\" y=\"48\">" << escape(unitNote) << "</text>";
    double y = 70;
    for (const auto& gr : groups) {
        o << "<text class=\"t\" x=\"24\" y=\"" << y + 12 << "\">" << escape(gr.title) << "</text>";
        const double top = y + 22, bottom = top + gr.bars.size() * rowH;
        for (double d = lo; d <= hi + 1e-9; d += 1) {  // decade gridlines
            const double x = L + (d - lo) / (hi - lo) * plotW;
            o << "<line x1=\"" << x << "\" y1=\"" << top - 4 << "\" x2=\"" << x << "\" y2=\"" << bottom
              << "\" stroke=\"#e5e7eb\"/><text class=\"ax\" x=\"" << x << "\" y=\"" << bottom + 13
              << "\" text-anchor=\"middle\">" << (d < 0 ? num(std::pow(10, d), int(-d)) : num(std::pow(10, d), 0)) << " ms</text>";
        }
        double by = top;
        for (const auto& b : gr.bars) {
            const double x = xOf(b.value);
            o << "<text class=\"s\" x=\"" << L - 8 << "\" y=\"" << by + 14 << "\" text-anchor=\"end\">" << escape(b.label)
              << "</text><rect x=\"" << L << "\" y=\"" << by + 3 << "\" width=\"" << std::max(2.0, x - L)
              << "\" height=\"" << rowH - 8 << "\" rx=\"2\" fill=\"" << pal[b.colour % 7] << "\"/>"
              << "<text class=\"s\" x=\"" << x + 6 << "\" y=\"" << by + 14 << "\">" << num(b.value, b.value < 10 ? 2 : 0)
              << " ms · " << escape(b.note) << "</text>";
            by += rowH;
        }
        y = bottom + 28 + pad;
    }
    o << "</svg>\n";
    return o.str();
}

}  // namespace graphsearch::svg
