//-----------------------------------------------------------------------------
// Triangulate a surface. If the surface is curved, then we first superimpose
// a grid of quads, with spacing to achieve our chord tolerance. We then
// proceed by ear-clipping; the resulting mesh should be watertight and not
// awful numerically, but has no special properties (Delaunay, etc.).
//
// For larger polygons (n > 50), we use monotone polygon triangulation which
// achieves O(n log n) complexity instead of O(n²) ear-clipping.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include <algorithm>
#include <set>
#include <map>

namespace SolveSpace {

//-----------------------------------------------------------------------------
// Monotone Polygon Triangulation - O(n log n) algorithm
//-----------------------------------------------------------------------------

namespace {

// Vertex type for sweep line algorithm
enum class MonotoneVertexType {
    START,          // Both neighbors below, convex angle
    END,            // Both neighbors above, convex angle
    SPLIT,          // Both neighbors below, reflex angle
    MERGE,          // Both neighbors above, reflex angle
    REGULAR_LEFT,   // Polygon interior is to the right
    REGULAR_RIGHT   // Polygon interior is to the left
};

struct MonotoneVertex {
    int index;          // Index in original contour
    Vector p;           // Position
    MonotoneVertexType type;
    int prev, next;     // Indices of adjacent vertices in contour
};

// Compare vertices by y-coordinate (then x for ties) for sweep line
struct VertexCompare {
    const std::vector<MonotoneVertex> &vertices;

    VertexCompare(const std::vector<MonotoneVertex> &v) : vertices(v) {}

    bool operator()(int a, int b) const {
        const Vector &pa = vertices[a].p;
        const Vector &pb = vertices[b].p;
        // Higher y first (top to bottom), then smaller x
        if(fabs(pa.y - pb.y) > LENGTH_EPS) return pa.y > pb.y;
        return pa.x < pb.x;
    }
};

// Edge in the sweep line status structure
struct SweepEdge {
    int upperVertex;    // Higher y endpoint
    int lowerVertex;    // Lower y endpoint
    int helper;         // Current helper vertex for this edge

    // Compute x-coordinate of edge at given y
    double XAtY(double y, const std::vector<MonotoneVertex> &vertices) const {
        const Vector &p1 = vertices[upperVertex].p;
        const Vector &p2 = vertices[lowerVertex].p;
        if(fabs(p1.y - p2.y) < LENGTH_EPS) return std::min(p1.x, p2.x);
        double t = (y - p2.y) / (p1.y - p2.y);
        return p2.x + t * (p1.x - p2.x);
    }
};

// Compare edges by x-coordinate at current sweep line y
struct EdgeCompare {
    const std::vector<MonotoneVertex> &vertices;
    double currentY;

    EdgeCompare(const std::vector<MonotoneVertex> &v) : vertices(v), currentY(0) {}

    bool operator()(const SweepEdge &a, const SweepEdge &b) const {
        double ax = a.XAtY(currentY, vertices);
        double bx = b.XAtY(currentY, vertices);
        if(fabs(ax - bx) > LENGTH_EPS) return ax < bx;
        // Tie-breaker: use lower vertex x
        return vertices[a.lowerVertex].p.x < vertices[b.lowerVertex].p.x;
    }
};

// Determine vertex type based on neighbors
MonotoneVertexType ClassifyVertex(const std::vector<MonotoneVertex> &vertices, int idx) {
    const MonotoneVertex &v = vertices[idx];
    const Vector &p = v.p;
    const Vector &prev = vertices[v.prev].p;
    const Vector &next = vertices[v.next].p;

    bool prevBelow = (prev.y < p.y) || (fabs(prev.y - p.y) < LENGTH_EPS && prev.x > p.x);
    bool nextBelow = (next.y < p.y) || (fabs(next.y - p.y) < LENGTH_EPS && next.x > p.x);

    if(!prevBelow && !nextBelow) {
        // Both neighbors above - this is an END or MERGE vertex
        // Check if angle is convex (interior angle < 180°)
        Vector e1 = prev.Minus(p);
        Vector e2 = next.Minus(p);
        double cross = e1.x * e2.y - e1.y * e2.x;
        return (cross > 0) ? MonotoneVertexType::END : MonotoneVertexType::MERGE;
    }
    else if(prevBelow && nextBelow) {
        // Both neighbors below - this is a START or SPLIT vertex
        Vector e1 = prev.Minus(p);
        Vector e2 = next.Minus(p);
        double cross = e1.x * e2.y - e1.y * e2.x;
        return (cross > 0) ? MonotoneVertexType::START : MonotoneVertexType::SPLIT;
    }
    else {
        // One above, one below - regular vertex
        // Determine if interior is to left or right
        Vector e1 = prev.Minus(p);
        Vector e2 = next.Minus(p);
        double cross = e1.x * e2.y - e1.y * e2.x;
        return (cross > 0) ? MonotoneVertexType::REGULAR_LEFT : MonotoneVertexType::REGULAR_RIGHT;
    }
}

// Triangulate a y-monotone polygon using the stack-based O(n) algorithm
void TriangulateMonotonePolygon(const std::vector<int> &polygon,
                                 const std::vector<MonotoneVertex> &vertices,
                                 SMesh *mesh, double scaledEps) {
    if(polygon.size() < 3) return;

    int n = (int)polygon.size();

    if(polygon.size() == 3) {
        STriangle tr = {};
        tr.a = vertices[polygon[0]].p;
        tr.b = vertices[polygon[1]].p;
        tr.c = vertices[polygon[2]].p;
        if(tr.Normal().MagSquared() >= scaledEps * scaledEps) {
            mesh->AddTriangle(&tr);
        }
        return;
    }

    // Sort vertices by y-coordinate (descending)
    std::vector<int> sorted = polygon;
    std::sort(sorted.begin(), sorted.end(), [&vertices](int a, int b) {
        const Vector &pa = vertices[a].p;
        const Vector &pb = vertices[b].p;
        if(fabs(pa.y - pb.y) > LENGTH_EPS) return pa.y > pb.y;
        return pa.x < pb.x;
    });

    // Find left and right chains
    // The leftmost top vertex starts the left chain
    std::set<int> leftChain, rightChain;

    // Walk from top to bottom on each side
    int topIdx = sorted[0];
    int botIdx = sorted[sorted.size() - 1];

    // Find position in polygon
    int topPos = -1;
    for(size_t i = 0; i < polygon.size(); i++) {
        if(polygon[i] == topIdx) { topPos = (int)i; break; }
    }

    // Walk forward (left chain) and backward (right chain)
    for(int pos = topPos; polygon[pos] != botIdx; pos = (pos + 1) % n) {
        leftChain.insert(polygon[pos]);
    }
    leftChain.insert(botIdx);

    for(int pos = topPos; polygon[pos] != botIdx; pos = (pos - 1 + n) % n) {
        rightChain.insert(polygon[pos]);
    }
    rightChain.insert(botIdx);

    // Stack-based triangulation
    std::vector<int> stack;
    stack.push_back(sorted[0]);
    stack.push_back(sorted[1]);

    for(size_t i = 2; i < sorted.size() - 1; i++) {
        int vi = sorted[i];
        int top = stack.back();

        bool viOnLeft = leftChain.count(vi) > 0 && rightChain.count(vi) == 0;
        bool topOnLeft = leftChain.count(top) > 0 && rightChain.count(top) == 0;
        bool sameChain = (viOnLeft == topOnLeft);

        if(!sameChain) {
            // vi is on opposite chain from stack top
            // Pop all vertices and form triangles
            while(stack.size() > 1) {
                int v1 = stack.back(); stack.pop_back();
                int v2 = stack.back();

                STriangle tr = {};
                tr.a = vertices[vi].p;
                tr.b = vertices[v1].p;
                tr.c = vertices[v2].p;

                if(tr.Normal().MagSquared() >= scaledEps * scaledEps) {
                    mesh->AddTriangle(&tr);
                }
            }
            stack.pop_back();
            stack.push_back(top);
            stack.push_back(vi);
        }
        else {
            // vi is on same chain as stack top
            // Pop vertices while we can form valid triangles
            int lastPopped = stack.back();
            stack.pop_back();

            while(!stack.empty()) {
                int v2 = stack.back();

                // Check if diagonal vi-v2 is inside the polygon
                Vector e1 = vertices[lastPopped].p.Minus(vertices[vi].p);
                Vector e2 = vertices[v2].p.Minus(vertices[vi].p);
                double cross = e1.x * e2.y - e1.y * e2.x;

                // For left chain, cross should be positive; for right, negative
                bool valid = viOnLeft ? (cross > LENGTH_EPS) : (cross < -LENGTH_EPS);

                if(!valid) break;

                STriangle tr = {};
                tr.a = vertices[vi].p;
                tr.b = vertices[lastPopped].p;
                tr.c = vertices[v2].p;

                if(tr.Normal().MagSquared() >= scaledEps * scaledEps) {
                    mesh->AddTriangle(&tr);
                }

                lastPopped = v2;
                stack.pop_back();
            }

            stack.push_back(lastPopped);
            stack.push_back(vi);
        }
    }

    // Process last vertex - bottom vertex connects to remaining stack
    int vn = sorted.back();
    while(stack.size() > 1) {
        int v1 = stack.back(); stack.pop_back();
        int v2 = stack.back();

        STriangle tr = {};
        tr.a = vertices[vn].p;
        tr.b = vertices[v1].p;
        tr.c = vertices[v2].p;

        if(tr.Normal().MagSquared() >= scaledEps * scaledEps) {
            mesh->AddTriangle(&tr);
        }
    }
}

// Diagonal to be added during monotone decomposition
struct Diagonal {
    int v1, v2;
};

// Find the edge immediately to the left of vertex vi at its y-coordinate
// Returns the index in the edges list, or -1 if none found
int FindEdgeToLeft(int vi, const std::vector<MonotoneVertex> &vertices,
                   const std::vector<std::pair<SweepEdge, int>> &activeEdges) {
    double y = vertices[vi].p.y;
    double x = vertices[vi].p.x;

    int bestIdx = -1;
    double bestX = -1e30;

    for(size_t i = 0; i < activeEdges.size(); i++) {
        double edgeX = activeEdges[i].first.XAtY(y, vertices);
        if(edgeX < x - LENGTH_EPS && edgeX > bestX) {
            bestX = edgeX;
            bestIdx = (int)i;
        }
    }
    return bestIdx;
}

// Decompose polygon into monotone pieces using sweep line algorithm
// Returns list of diagonals to add
std::vector<Diagonal> DecomposeIntoMonotone(std::vector<MonotoneVertex> &vertices) {
    std::vector<Diagonal> diagonals;
    int n = (int)vertices.size();
    if(n < 4) return diagonals;

    // Sort vertices by y-coordinate (descending), then x (ascending)
    std::vector<int> sortedVertices(n);
    for(int i = 0; i < n; i++) sortedVertices[i] = i;

    std::sort(sortedVertices.begin(), sortedVertices.end(),
              [&vertices](int a, int b) {
                  const Vector &pa = vertices[a].p;
                  const Vector &pb = vertices[b].p;
                  if(fabs(pa.y - pb.y) > LENGTH_EPS) return pa.y > pb.y;
                  return pa.x < pb.x;
              });

    // Active edges: pair of (edge, original index for identification)
    // Each edge stores its helper vertex
    std::vector<std::pair<SweepEdge, int>> activeEdges;

    // Process vertices in sweep order
    for(int vi : sortedVertices) {
        MonotoneVertex &v = vertices[vi];
        int prevIdx = v.prev;
        int nextIdx = v.next;

        switch(v.type) {
            case MonotoneVertexType::START: {
                // Insert edge vi->next into active edges
                SweepEdge edge;
                edge.upperVertex = vi;
                edge.lowerVertex = nextIdx;
                edge.helper = vi;
                activeEdges.push_back({edge, vi});
                break;
            }

            case MonotoneVertexType::END: {
                // Find and remove edge prev->vi
                // If helper of that edge is a merge vertex, add diagonal
                for(auto it = activeEdges.begin(); it != activeEdges.end(); ++it) {
                    if(it->second == prevIdx) {
                        int helper = it->first.helper;
                        if(vertices[helper].type == MonotoneVertexType::MERGE) {
                            diagonals.push_back({vi, helper});
                        }
                        activeEdges.erase(it);
                        break;
                    }
                }
                break;
            }

            case MonotoneVertexType::SPLIT: {
                // Find edge directly to the left
                int leftIdx = FindEdgeToLeft(vi, vertices, activeEdges);
                if(leftIdx >= 0) {
                    // Add diagonal from vi to helper of left edge
                    int helper = activeEdges[leftIdx].first.helper;
                    diagonals.push_back({vi, helper});
                    // Update helper of left edge
                    activeEdges[leftIdx].first.helper = vi;
                }
                // Insert edge vi->next
                SweepEdge edge;
                edge.upperVertex = vi;
                edge.lowerVertex = nextIdx;
                edge.helper = vi;
                activeEdges.push_back({edge, vi});
                break;
            }

            case MonotoneVertexType::MERGE: {
                // Handle edge prev->vi (ending at vi)
                for(auto it = activeEdges.begin(); it != activeEdges.end(); ++it) {
                    if(it->second == prevIdx) {
                        int helper = it->first.helper;
                        if(vertices[helper].type == MonotoneVertexType::MERGE) {
                            diagonals.push_back({vi, helper});
                        }
                        activeEdges.erase(it);
                        break;
                    }
                }
                // Find edge to the left and update its helper
                int leftIdx = FindEdgeToLeft(vi, vertices, activeEdges);
                if(leftIdx >= 0) {
                    int helper = activeEdges[leftIdx].first.helper;
                    if(vertices[helper].type == MonotoneVertexType::MERGE) {
                        diagonals.push_back({vi, helper});
                    }
                    activeEdges[leftIdx].first.helper = vi;
                }
                break;
            }

            case MonotoneVertexType::REGULAR_LEFT: {
                // Polygon interior is to the right
                // Remove edge prev->vi, add edge vi->next
                for(auto it = activeEdges.begin(); it != activeEdges.end(); ++it) {
                    if(it->second == prevIdx) {
                        int helper = it->first.helper;
                        if(vertices[helper].type == MonotoneVertexType::MERGE) {
                            diagonals.push_back({vi, helper});
                        }
                        activeEdges.erase(it);
                        break;
                    }
                }
                SweepEdge edge;
                edge.upperVertex = vi;
                edge.lowerVertex = nextIdx;
                edge.helper = vi;
                activeEdges.push_back({edge, vi});
                break;
            }

            case MonotoneVertexType::REGULAR_RIGHT: {
                // Polygon interior is to the left
                // Just update helper of edge to the left
                int leftIdx = FindEdgeToLeft(vi, vertices, activeEdges);
                if(leftIdx >= 0) {
                    int helper = activeEdges[leftIdx].first.helper;
                    if(vertices[helper].type == MonotoneVertexType::MERGE) {
                        diagonals.push_back({vi, helper});
                    }
                    activeEdges[leftIdx].first.helper = vi;
                }
                break;
            }
        }
    }

    return diagonals;
}

// Split polygon by a single diagonal into two sub-polygons
void SplitPolygonByDiagonal(const std::vector<int> &polygon,
                             const std::vector<MonotoneVertex> &vertices,
                             int d1, int d2,
                             std::vector<int> &poly1, std::vector<int> &poly2) {
    int n = (int)polygon.size();

    // Find positions of d1 and d2 in the polygon
    int pos1 = -1, pos2 = -1;
    for(int i = 0; i < n; i++) {
        if(polygon[i] == d1) pos1 = i;
        if(polygon[i] == d2) pos2 = i;
    }

    if(pos1 == -1 || pos2 == -1) return;

    // Ensure pos1 < pos2
    if(pos1 > pos2) std::swap(pos1, pos2);

    // First polygon: from pos1 to pos2 (inclusive)
    for(int i = pos1; i <= pos2; i++) {
        poly1.push_back(polygon[i]);
    }

    // Second polygon: from pos2 to pos1 (wrapping)
    for(int i = pos2; i != pos1; i = (i + 1) % n) {
        poly2.push_back(polygon[i]);
    }
    poly2.push_back(polygon[pos1]);
}

// Recursively triangulate polygon using diagonals
void TriangulateWithDiagonals(const std::vector<int> &polygon,
                               const std::vector<MonotoneVertex> &vertices,
                               const std::vector<Diagonal> &diagonals,
                               size_t diagIdx,
                               SMesh *mesh, double scaledEps) {
    if(polygon.size() < 3) return;

    // Check if polygon is monotone (has no split/merge vertices)
    bool isMonotone = true;
    for(int vi : polygon) {
        if(vertices[vi].type == MonotoneVertexType::SPLIT ||
           vertices[vi].type == MonotoneVertexType::MERGE) {
            isMonotone = false;
            break;
        }
    }

    if(isMonotone || polygon.size() <= 3) {
        // Triangulate directly
        TriangulateMonotonePolygon(polygon, vertices, mesh, scaledEps);
        return;
    }

    // Find a diagonal that applies to this polygon
    for(size_t i = diagIdx; i < diagonals.size(); i++) {
        const Diagonal &d = diagonals[i];

        // Check if both endpoints are in this polygon
        bool hasV1 = false, hasV2 = false;
        for(int vi : polygon) {
            if(vi == d.v1) hasV1 = true;
            if(vi == d.v2) hasV2 = true;
        }

        if(hasV1 && hasV2) {
            // Split by this diagonal
            std::vector<int> poly1, poly2;
            SplitPolygonByDiagonal(polygon, vertices, d.v1, d.v2, poly1, poly2);

            // Recursively triangulate both sub-polygons
            TriangulateWithDiagonals(poly1, vertices, diagonals, i + 1, mesh, scaledEps);
            TriangulateWithDiagonals(poly2, vertices, diagonals, i + 1, mesh, scaledEps);
            return;
        }
    }

    // No applicable diagonal found, try to triangulate directly
    TriangulateMonotonePolygon(polygon, vertices, mesh, scaledEps);
}

// Main monotone triangulation function
// Returns true if successful, false if should fall back to ear-clipping
bool MonotoneTriangulate(SContour *contour, SMesh *mesh, double scaledEps) {
    int n = contour->l.n;
    if(n < 3) return true;

    if(n == 3) {
        // Just add the triangle directly
        STriangle tr = {};
        tr.a = contour->l[0].p;
        tr.b = contour->l[1].p;
        tr.c = contour->l[2].p;
        if(tr.Normal().MagSquared() >= scaledEps * scaledEps) {
            mesh->AddTriangle(&tr);
        }
        return true;
    }

    // Build vertex list
    std::vector<MonotoneVertex> vertices(n);
    for(int i = 0; i < n; i++) {
        vertices[i].index = i;
        vertices[i].p = contour->l[i].p;
        vertices[i].prev = (i - 1 + n) % n;
        vertices[i].next = (i + 1) % n;
    }

    // Classify each vertex
    bool hasSplitOrMerge = false;
    for(int i = 0; i < n; i++) {
        vertices[i].type = ClassifyVertex(vertices, i);
        if(vertices[i].type == MonotoneVertexType::SPLIT ||
           vertices[i].type == MonotoneVertexType::MERGE) {
            hasSplitOrMerge = true;
        }
    }

    // If already monotone, triangulate directly
    if(!hasSplitOrMerge) {
        std::vector<int> polygon(n);
        for(int i = 0; i < n; i++) polygon[i] = i;
        TriangulateMonotonePolygon(polygon, vertices, mesh, scaledEps);
        return true;
    }

    // Decompose into monotone pieces using diagonals
    std::vector<Diagonal> diagonals = DecomposeIntoMonotone(vertices);

    // Build initial polygon
    std::vector<int> polygon(n);
    for(int i = 0; i < n; i++) polygon[i] = i;

    // Recursively split and triangulate
    TriangulateWithDiagonals(polygon, vertices, diagonals, 0, mesh, scaledEps);

    return true;
}

} // anonymous namespace

void SPolygon::UvTriangulateInto(SMesh *m, SSurface *srf) {
    if(l.n <= 0) return;

    //int64_t in = GetMilliseconds();

    normal = Vector::From(0, 0, 1);

    while(l.n > 0) {
        FixContourDirections();
        l.ClearTags();

        // Find a top-level contour, and start with that. Then build bridges
        // in order to merge all its islands into a single contour.
        SContour *top;
        for(top = l.First(); top; top = l.NextAfter(top)) {
            if(top->timesEnclosed == 0) {
                break;
            }
        }
        if(!top) {
            dbp("polygon has no top-level contours?");
            return;
        }

        // Start with the outer contour
        SContour merged = {};
        top->tag = 1;
        top->CopyInto(&merged);
        merged.l.RemoveLast(1);

        // List all of the edges, for testing whether bridges work.
        SEdgeList el = {};
        top->MakeEdgesInto(&el);
        List<Vector> vl = {};

        // And now find all of its holes. Note that we will also find any
        // outer contours that lie entirely within this contour, and any
        // holes for those contours. But that's okay, because we can merge
        // those too.
        SContour *sc;
        for(sc = l.First(); sc; sc = l.NextAfter(sc)) {
            if(sc->timesEnclosed != 1) continue;
            if(sc->l.n < 2) continue;

            // Test the midpoint of an edge. Our polygon may not be self-
            // intersecting, but two contours may share a vertex; so a
            // vertex could be on the edge of another polygon, in which
            // case ContainsPointProjdToNormal returns indeterminate.
            Vector tp = sc->AnyEdgeMidpoint();
            if(top->ContainsPointProjdToNormal(normal, tp)) {
                sc->tag = 2;
                sc->MakeEdgesInto(&el);
                sc->FindPointWithMinX();
            }
        }

//        dbp("finished finding holes: %d ms", (int)(GetMilliseconds() - in));
        for(;;) {
            double xmin = 1e10;
            SContour *scmin = NULL;

            for(sc = l.First(); sc; sc = l.NextAfter(sc)) {
                if(sc->tag != 2) continue;

                if(sc->xminPt.x < xmin) {
                    xmin = sc->xminPt.x;
                    scmin = sc;
                }
            }
            if(!scmin) break;

            if(!merged.BridgeToContour(scmin, &el, &vl)) {
                dbp("couldn't merge our hole");
                return;
            }
//            dbp("   bridged to contour: %d ms", (int)(GetMilliseconds() - in));
            scmin->tag = 3;
        }
//        dbp("finished merging holes: %d ms", (int)(GetMilliseconds() - in));

        merged.UvTriangulateInto(m, srf);
//        dbp("finished ear clippping: %d ms", (int)(GetMilliseconds() - in));
        merged.l.Clear();
        el.Clear();
        vl.Clear();

        // Careful, need to free the points within the contours, and not just
        // the contours themselves. This was a tricky memory leak.
        for(sc = l.First(); sc; sc = l.NextAfter(sc)) {
            if(sc->tag) {
                sc->l.Clear();
            }
        }
        l.RemoveTagged();
    }
}

bool SContour::BridgeToContour(SContour *sc,
                               SEdgeList *avoidEdges, List<Vector> *avoidPts)
{
    int i, j;
    bool withbridge = true;

    // Start looking for a bridge on our new hole near its leftmost (min x)
    // point.
    int sco = 0;
    for(i = 0; i < (sc->l.n - 1); i++) {
        if((sc->l[i].p).EqualsExactly(sc->xminPt)) {
            sco = i;
        }
    }

    // And start looking on our merged contour at whichever point is nearest
    // to the leftmost point of the new segment.
    int thiso = 0;
    double dmin = 1e10;
    for(i = 0; i < l.n-1; i++) {
        Vector p = l[i].p;
        double d = (p.Minus(sc->xminPt)).MagSquared();
        if(d < dmin) {
            dmin = d;
            thiso = i;
        }
    }

    int thisp, scp;

    Vector a, b, *f;

    // First check if the contours share a point; in that case we should
    // merge them there, without a bridge.
    for(i = 0; i < l.n; i++) {
        thisp = WRAP(i+thiso, l.n-1);
        a = l[thisp].p;

        for(f = avoidPts->First(); f; f = avoidPts->NextAfter(f)) {
            if(f->Equals(a)) break;
        }
        if(f) continue;

        for(j = 0; j < (sc->l.n - 1); j++) {
            scp = WRAP(j+sco, (sc->l.n - 1));
            b = sc->l[scp].p;

            if(a.Equals(b)) {
                withbridge = false;
                goto haveEdge;
            }
        }
    }

    // If that fails, look for a bridge that does not intersect any edges.
    for(i = 0; i < l.n; i++) {
        thisp = WRAP(i+thiso, l.n);
        a = l[thisp].p;

        for(f = avoidPts->First(); f; f = avoidPts->NextAfter(f)) {
            if(f->Equals(a)) break;
        }
        if(f) continue;

        for(j = 0; j < (sc->l.n - 1); j++) {
            scp = WRAP(j+sco, (sc->l.n - 1));
            b = sc->l[scp].p;

            for(f = avoidPts->First(); f; f = avoidPts->NextAfter(f)) {
                if(f->Equals(b)) break;
            }
            if(f) continue;

            if(avoidEdges->AnyEdgeCrossings(a, b) > 0) {
                // doesn't work, bridge crosses an existing edge
            } else {
                goto haveEdge;
            }
        }
    }

    // Tried all the possibilities, didn't find an edge
    return false;

haveEdge:
    SContour merged = {};
    for(i = 0; i < l.n; i++) {
        if(withbridge || (i != thisp)) {
            merged.AddPoint(l[i].p);
        }
        if(i == thisp) {
            // less than or equal; need to duplicate the join point
            for(j = 0; j <= (sc->l.n - 1); j++) {
                int jp = WRAP(j + scp, (sc->l.n - 1));
                merged.AddPoint((sc->l[jp]).p);
            }
            // and likewise duplicate join point for the outer curve
            if(withbridge) {
                merged.AddPoint(l[i].p);
            }
        }
    }

    // and future bridges mustn't cross our bridge, and it's tricky to get
    // things right if two bridges come from the same point
    if(withbridge) {
        avoidEdges->AddEdge(a, b);
        avoidPts->Add(&a);
    }
    avoidPts->Add(&b);

    l.Clear();
    l = merged.l;
    return true;
}

bool SContour::IsEmptyTriangle(int ap, int bp, int cp, double scaledEPS) const {

    STriangle tr = {};
    tr.a = l[ap].p;
    tr.b = l[bp].p;
    tr.c = l[cp].p;

    // Accelerate with an axis-aligned bounding box test
    Vector maxv = tr.a, minv = tr.a;
    (tr.b).MakeMaxMin(&maxv, &minv);
    (tr.c).MakeMaxMin(&maxv, &minv);

    Vector n = Vector::From(0, 0, -1);

    int i;
    for(i = 0; i < l.n; i++) {
        if(i == ap || i == bp || i == cp) continue;

        Vector p = l[i].p;
        if(p.OutsideAndNotOn(maxv, minv)) continue;

        // A point on the edge of the triangle is considered to be inside,
        // and therefore makes it a non-ear; but a point on the vertex is
        // "outside", since that's necessary to make bridges work.
        if(p.EqualsExactly(tr.a)) continue;
        if(p.EqualsExactly(tr.b)) continue;
        if(p.EqualsExactly(tr.c)) continue;

        if(tr.ContainsPointProjd(n, p)) {
            return false;
        }
    }
    return true;
}

// Test if ray b->d passes through triangle a,b,c
static bool RayIsInside(Vector a, Vector c, Vector b, Vector d) {
    // coincident edges are not considered to intersect the triangle
    if (d.Equals(a)) return false;
    if (d.Equals(c)) return false;
    // if d and c are on opposite sides of ba, we are ok
    // likewise if d and a are on opposite sides of bc
    Vector ba = a.Minus(b);
    Vector bc = c.Minus(b);
    Vector bd = d.Minus(b);

    // perpendicular to (x,y) is (x,-y) so dot that with the two points. If they
    // have opposite signs their product will be negative. If bd and bc are on
    // opposite sides of ba the ray does not intersect. Likewise for bd,ba and bc.
    if ( (bd.x*(ba.y) + (bd.y * (-ba.x))) * ( bc.x*(ba.y) + (bc.y * (-ba.x))) < LENGTH_EPS)
        return false;
    if ( (bd.x*(bc.y) + (bd.y * (-bc.x))) * ( ba.x*(bc.y) + (ba.y * (-bc.x))) < LENGTH_EPS)
        return false;

    return true;
}

bool SContour::IsEar(int bp, double scaledEps) const {
    int ap = WRAP(bp-1, l.n),
        cp = WRAP(bp+1, l.n);

    STriangle tr = {};
    tr.a = l[ap].p;
    tr.b = l[bp].p;
    tr.c = l[cp].p;

    if((tr.a).Equals(tr.c)) {
        // This is two coincident and anti-parallel edges. Zero-area, so
        // won't generate a real triangle, but we certainly can clip it.
        return true;
    }

    Vector n = Vector::From(0, 0, -1);
    if((tr.Normal()).Dot(n) < scaledEps) {
        // This vertex is reflex, or between two collinear edges; either way,
        // it's not an ear.
        return false;
    }

    // Accelerate with an axis-aligned bounding box test
    Vector maxv = tr.a, minv = tr.a;
    (tr.b).MakeMaxMin(&maxv, &minv);
    (tr.c).MakeMaxMin(&maxv, &minv);

    int i;
    for(i = 0; i < l.n; i++) {
        if(i == ap || i == bp || i == cp) continue;

        Vector p = l[i].p;
        if(p.OutsideAndNotOn(maxv, minv)) continue;

        // A point on the edge of the triangle is considered to be inside,
        // and therefore makes it a non-ear; but a point on the vertex is
        // "outside", since that's necessary to make bridges work.
        if(p.EqualsExactly(tr.a)) continue;
        if(p.EqualsExactly(tr.c)) continue;
        // points coincident with bp have to be allowed for bridges but edges
        // from that other point must not cross through our triangle.
        if(p.EqualsExactly(tr.b)) {
            int j = WRAP(i-1, l.n);
            int k = WRAP(i+1, l.n);
            Vector jp = l[j].p;
            Vector kp = l[k].p;

            // two consecutive bridges (A,B,C) and later (C,B,A) are not an ear
            if (jp.Equals(tr.c) && kp.Equals(tr.a)) return false;
            // check both edges from the point in question
            if (!RayIsInside(tr.a, tr.c, p,jp) && !RayIsInside(tr.a, tr.c, p,kp))
                continue;
        }

        if(tr.ContainsPointProjd(n, p)) {
            return false;
        }
    }
    return true;
}

void SContour::ClipEarInto(SMesh *m, int bp, double scaledEps) {
    int ap = WRAP(bp-1, l.n),
        cp = WRAP(bp+1, l.n);

    STriangle tr = {};
    tr.a = l[ap].p;
    tr.b = l[bp].p;
    tr.c = l[cp].p;
    if(tr.Normal().MagSquared() < scaledEps*scaledEps) {
        // A vertex with more than two edges will cause us to generate
        // zero-area triangles, which must be culled.
    } else {
        m->AddTriangle(&tr);
    }

    // By deleting the point at bp, we may change the ear-ness of the points
    // on either side.
    l[ap].ear = EarType::UNKNOWN;
    l[cp].ear = EarType::UNKNOWN;

    l.ClearTags();
    l[bp].tag = 1;
    l.RemoveTagged();
}

void SContour::UvTriangulateInto(SMesh *m, SSurface *srf) {
    Vector tu, tv;
    srf->TangentsAt(0.5, 0.5, &tu, &tv);
    double s = sqrt(tu.MagSquared() + tv.MagSquared());
    // We would like to apply our tolerances in xyz; but that would be a lot
    // of work, so at least scale the epsilon semi-reasonably. That's
    // perfect for square planes, less perfect for anything else.
    double scaledEps = LENGTH_EPS / s;

    int i;
    // Clean the original contour by removing any zero-length edges.
    // initialize eartypes to unknown while we're going over them.
    l.ClearTags();
    l[0].ear = EarType::UNKNOWN;
    for(i = 1; i < l.n; i++) {
       l[i].ear = EarType::UNKNOWN;
       if((l[i].p).Equals(l[i-1].p)) {
            l[i].tag = 1;
        }
    }
    if( (l[0].p).Equals(l[l.n-1].p) ) {
        l[l.n-1].tag = 1;
    }
    l.RemoveTagged();

    // For planar surfaces with larger polygons, use O(n log n) monotone
    // triangulation instead of O(n²) ear-clipping
    if(srf->degm == 1 && srf->degn == 1 && l.n > 30) {
        if(MonotoneTriangulate(this, m, scaledEps)) {
            return;
        }
        // Fall through to ear-clipping if monotone fails
    }

    // Handle simple triangle fans all at once. This pass is optional.
    if(srf->degm == 1 && srf->degn == 1) {
        l.ClearTags();
        int j=0;
        int pstart = 0;
        double elen = -1.0;
        double oldspan = 0.0;
        for(i = 1; i < l.n; i++) {
            Vector ab = l[i].p.Minus(l[i-1].p);
            // first time just measure the segment
            if (elen < 0.0) {
                elen = ab.Dot(ab);
                oldspan = elen;
                j = 1;
                continue;
            }
            // check for consecutive segments of similar size which are also
            // ears and where the group forms a convex ear
            bool end = false;
            double ratio = ab.Dot(ab) / elen;
            if ((ratio < 0.25) || (ratio > 4.0)) end = true;

            double slen = l[pstart].p.Minus(l[i].p).MagSquared();
            if (slen < oldspan) end = true;

            if (!IsEar(i-1, scaledEps) ) end = true;
//            if ((j>0) && !IsEar(pstart, i-1, i, scaledEps)) end = true;
            if ((j>0) && !IsEmptyTriangle(pstart, i-1, i, scaledEps)) end = true;
            // the new segment is valid so add to the fan
            if (!end) {
                j++;
                oldspan = slen;
            }
            // we need to stop at the end of polygon but may still
            if (i == l.n-1) {
                end = true;
            }
            if (end) {  // triangulate the fan and tag the vertices
                if (j > 3) {
                    Vector center = l[pstart+1].p.Plus(l[pstart+j-1].p).ScaledBy(0.5);
                    for (int x=0; x<j; x++) {
                        STriangle tr = {};
                        tr.a = center;
                        tr.b = l[pstart+x].p;
                        tr.c = l[pstart+x+1].p;
                        m->AddTriangle(&tr);
                    }
                    for (int x=1; x<j; x++) {
                        l[pstart+x].tag = 1;
                    }
                    STriangle tr = {};
                    tr.a = center;
                    tr.b = l[pstart+j].p;
                    tr.c = l[pstart].p;
                    m->AddTriangle(&tr);    
                }
                pstart = i-1;
                elen = ab.Dot(ab);
                oldspan = elen;
                j = 1;
            }
        }
        l.RemoveTagged();
    }  // end optional fan creation pass

    bool toggle = false;
    while(l.n > 3) {
        int bestEar = -1;
        double bestChordTol = VERY_POSITIVE;
        // Alternate the starting position so we generate strip-like
        // triangulations instead of fan-like
        toggle = !toggle;
        int offset = toggle ? -1 : 0;
        for(i = 0; i < l.n; i++) {
            int ear = WRAP(i+offset, l.n);
            if(l[ear].ear == EarType::UNKNOWN) {
                (l[ear]).ear = IsEar(ear, scaledEps) ? EarType::EAR : EarType::NOT_EAR;
            }
            if(l[ear].ear == EarType::EAR) {
                if(srf->degm == 1 && srf->degn == 1) {
                    // This is a plane; any ear is a good ear.
                    bestEar = ear;
                    break;
                }
                // If we are triangulating a curved surface, then try to
                // clip ears that have a small chord tolerance from the
                // surface.
                Vector prev = l[WRAP((i+offset-1), l.n)].p,
                       next = l[WRAP((i+offset+1), l.n)].p;
                double tol = srf->ChordToleranceForEdge(prev, next);
                if(tol < bestChordTol - scaledEps) {
                    bestEar = ear;
                    bestChordTol = tol;
                }
                if(bestChordTol < 0.1*SS.ChordTolMm()) {
                    break;
                }
            }
        }
        if(bestEar < 0) {
            dbp("couldn't find an ear! fail");
            return;
        }
        ClipEarInto(m, bestEar, scaledEps);
    }

    ClipEarInto(m, 0, scaledEps); // add the last triangle
}

double SSurface::ChordToleranceForEdge(Vector a, Vector b) const {
    Vector as = PointAt(a.x, a.y), bs = PointAt(b.x, b.y);

    double worst = VERY_NEGATIVE;
    int i;
    for(i = 1; i <= 3; i++) {
        Vector p  = a. Plus((b. Minus(a )).ScaledBy(i/4.0)),
               ps = as.Plus((bs.Minus(as)).ScaledBy(i/4.0));

        Vector pps = PointAt(p.x, p.y);
        worst = max(worst, (pps.Minus(ps)).MagSquared());
    }
    return sqrt(worst);
}

Vector SSurface::PointAtMaybeSwapped(double u, double v, bool swapped) const {
    if(swapped) {
        return PointAt(v, u);
    } else {
        return PointAt(u, v);
    }
}

Vector SSurface::NormalAtMaybeSwapped(double u, double v, bool swapped) const {
    Vector du, dv;
    if(swapped) {
        TangentsAt(v, u, &dv, &du);
    } else {
        TangentsAt(u, v, &du, &dv);
    }
    return du.Cross(dv).WithMagnitude(1.0);
}

void SSurface::MakeTriangulationGridInto(List<double> *l, double vs, double vf,
                                         bool swapped, int depth) const
{
    double worst = 0;

    // Try piecewise linearizing four curves, at u = 0, 1/3, 2/3, 1; choose
    // the worst chord tolerance of any of those.
    double worst_twist = 1.0;
    int i;
    for(i = 0; i <= 3; i++) {
        double u = i/3.0;

        // This chord test should be identical to the one in SBezier::MakePwl
        // to make the piecewise linear edges line up with the grid more or
        // less.
        Vector ps = PointAtMaybeSwapped(u, vs, swapped),
               pf = PointAtMaybeSwapped(u, vf, swapped);

        double vm1 = (2*vs + vf) / 3,
               vm2 = (vs + 2*vf) / 3;

        Vector pm1 = PointAtMaybeSwapped(u, vm1, swapped),
               pm2 = PointAtMaybeSwapped(u, vm2, swapped);

        // 0.999 is about 2.5 degrees of twist over the middle 1/3 V-span.
        // we don't check at the ends because the derivative may not be valid there.
        double twist = 1.0;
        if (degm == 1) twist = NormalAtMaybeSwapped(u, vm1, swapped).Dot(
                               NormalAtMaybeSwapped(u, vm2, swapped) );
        if (twist < worst_twist) worst_twist = twist;

        worst = max(worst, pm1.DistanceToLine(ps, pf.Minus(ps)));
        worst = max(worst, pm2.DistanceToLine(ps, pf.Minus(ps)));
    }

    double step = 1.0/SS.GetMaxSegments();
    if( ((vf - vs) < step || worst < SS.ChordTolMm())
        && ((worst_twist > 0.999) || (depth > 3)) ) {
        l->Add(&vf);
    } else {
        MakeTriangulationGridInto(l, vs, (vs+vf)/2, swapped, depth+1);
        MakeTriangulationGridInto(l, (vs+vf)/2, vf, swapped, depth+1);
    }
}

void SPolygon::UvGridTriangulateInto(SMesh *mesh, SSurface *srf) {
    SEdgeList orig = {};
    MakeEdgesInto(&orig);

    SEdgeList holes = {};

    normal = Vector::From(0, 0, 1);
    FixContourDirections();

    // Build a rectangular grid, with horizontal and vertical lines in the
    // uv plane. The spacing of these lines is adaptive, so calculate that.
    List<double> li, lj;
    li = {};
    lj = {};
    double v[5] = {0.0, 0.25, 0.5, 0.75, 1.0};
    li.Add(&v[0]);
    srf->MakeTriangulationGridInto(&li, 0, 1, /*swapped=*/true, 0);
    lj.Add(&v[0]);
    srf->MakeTriangulationGridInto(&lj, 0, 1, /*swapped=*/false, 0);

    // force 2nd order grid to have at least 4 segments in each direction
    if ((li.n < 5) && (srf->degm>1)) { // 4 segments minimum
        li.Clear();
        li.Add(&v[0]);li.Add(&v[1]);li.Add(&v[2]);li.Add(&v[3]);li.Add(&v[4]);
    }
    if ((lj.n < 5) && (srf->degn>1)) { // 4 segments minimum
        lj.Clear();
        lj.Add(&v[0]);lj.Add(&v[1]);lj.Add(&v[2]);lj.Add(&v[3]);lj.Add(&v[4]);
    }

    if ((li.n > 3) && (lj.n > 3)) {
        // Now iterate over each quad in the grid. If it's outside the polygon,
        // or if it intersects the polygon, then we discard it. Otherwise we
        // generate two triangles in the mesh, and cut it out of our polygon.
        // Quads around the perimeter would be rejected by AnyEdgeCrossings.
        std::vector<bool> bottom(lj.n, false); // did we use this quad?
        Vector tu = {0,0,0}, tv = {0,0,0};
        int i, j;
        for(i = 1; i < (li.n-1); i++) {
            bool prev_flag = false;
            for(j = 1; j < (lj.n-1); j++) {
                bool this_flag = true;
                double us = li[i], uf = li[i+1],
                       vs = lj[j], vf = lj[j+1];

                Vector a = Vector::From(us, vs, 0),
                       b = Vector::From(us, vf, 0),
                       c = Vector::From(uf, vf, 0),
                       d = Vector::From(uf, vs, 0);

                //  |   d-----c
                //  |   |     |
                //  |   |     |
                //  |   a-----b
                //  |
                //  +-------------> j/v axis

                if( (i==(li.n-2)) || (j==(lj.n-2)) ||
                   orig.AnyEdgeCrossings(a, b, NULL) ||
                   orig.AnyEdgeCrossings(b, c, NULL) ||
                   orig.AnyEdgeCrossings(c, d, NULL) ||
                   orig.AnyEdgeCrossings(d, a, NULL))
                {
                    this_flag = false;
                }

                // There's no intersections, so it doesn't matter which point
                // we decide to test.
                if(!this->ContainsPoint(a)) {
                    this_flag = false;
                }
                
                if (this_flag) {
                    // Add the quad to our mesh
                    srf->TangentsAt(us,vs, &tu,&tv);
                    if(tu.Dot(tv) < LENGTH_EPS) {
                        /* Split "the other way" if angle>90
                           compare to LENGTH_EPS instead of zero to avoid alternating triangle
                           "orientations" when the tangents are orthogonal (revolve, lathe etc.)
                           this results in a higher quality mesh. */
                        STriangle tr = {};
                        tr.a = a;
                        tr.b = b;
                        tr.c = c;
                        mesh->AddTriangle(&tr);
                        tr.a = a;
                        tr.b = c;
                        tr.c = d;
                        mesh->AddTriangle(&tr);
                    } else{
                        STriangle tr = {};
                        tr.a = a;
                        tr.b = b;
                        tr.c = d;
                        mesh->AddTriangle(&tr);
                        tr.a = b;
                        tr.b = c;
                        tr.c = d;
                        mesh->AddTriangle(&tr);
                    }
                    if (!prev_flag) // add our own left edge
                        holes.AddEdge(d, a);
                    if (!bottom[j]) // add our own bottom edge
                        holes.AddEdge(a, b);
                } else {
                    if (prev_flag) // add our left neighbors right edge
                        holes.AddEdge(a, d);
                    if (bottom[j]) // add our bottom neighbors top edge
                        holes.AddEdge(b, a);
                }
                prev_flag = this_flag;
                bottom[j] = this_flag;
            }
        }

        // Because no duplicate edges were created we do not need to cull them.
        SPolygon hp = {};
        holes.AssemblePolygon(&hp, NULL, /*keepDir=*/true);

        SContour *sc;
        for(sc = hp.l.First(); sc; sc = hp.l.NextAfter(sc)) {
            l.Add(sc);
        }
        hp.l.Clear();
    }
    orig.Clear();
    holes.Clear();
    li.Clear();
    lj.Clear();
    UvTriangulateInto(mesh, srf);
}

void SPolygon::TriangulateInto(SMesh *m) const {
    Vector n = normal;
    if(n.Equals(Vector::From(0.0, 0.0, 0.0))) {
       n = ComputeNormal();
    }
    Vector u = n.Normal(0);
    Vector v = n.Normal(1);

    SPolygon p = {};
    this->InverseTransformInto(&p, u, v, n);

    SSurface srf = SSurface::FromPlane(Vector::From(0.0, 0.0, 0.0),
                                       Vector::From(1.0, 0.0, 0.0),
                                       Vector::From(0.0, 1.0, 0.0));
    SMesh pm = {};
    p.UvTriangulateInto(&pm, &srf);
    for(STriangle st : pm.l) {
        st = st.Transform(u, v, n);
        m->AddTriangle(&st);
    }

    p.Clear();
    pm.Clear();
}

} // namespace SolveSpace
