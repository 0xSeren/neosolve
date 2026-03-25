//-----------------------------------------------------------------------------
// FaceBuilder implementation: Convert SolveSpace geometry to OCC.
//
//-----------------------------------------------------------------------------

// Include solvespace.h BEFORE OCC headers to ensure Platform namespace is defined
// before any Windows headers are included that might interfere
#include "solvespace.h"
#include "facebuilder.h"

#ifdef HAVE_OPENCASCADE

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS_Builder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax2.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <gp_Pln.hxx>
#include <BRepBuilderAPI_FindPlane.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <cmath>

namespace SolveSpace {

FaceBuilder::FaceBuilder() {
    TopoDS_Builder builder;
    builder.MakeCompound(compound);
}

TopoDS_Edge FaceBuilder::BezierToEdge(const SBezier *sb) const {
    if(sb->deg == 1) {
        // Line segment
        gp_Pnt p0 = OccUtil::ToOccPoint(sb->ctrl[0]);
        gp_Pnt p1 = OccUtil::ToOccPoint(sb->ctrl[1]);
        return BRepBuilderAPI_MakeEdge(p0, p1);
    }
    else if(sb->deg == 2) {
        // Check if this is a circular arc
        Vector center;
        double radius;
        if(sb->IsCircle(normal, &center, &radius)) {
            // Build as a true circle arc using three points to ensure correct direction
            // Use start, middle, and end points to guarantee correct arc orientation
            gp_Pnt p0 = OccUtil::ToOccPoint(sb->Start());
            gp_Pnt pMid = OccUtil::ToOccPoint(sb->PointAt(0.5));
            gp_Pnt p1 = OccUtil::ToOccPoint(sb->Finish());

            GC_MakeArcOfCircle arcMaker(p0, pMid, p1);
            if(arcMaker.IsDone()) {
                return BRepBuilderAPI_MakeEdge(arcMaker.Value());
            }
            // Fall through to bezier handling if arc construction failed
        }

        // Quadratic Bezier - elevate to cubic for OCC
        // Cubic control points from quadratic: P0, P0+2/3*(P1-P0), P2+2/3*(P1-P2), P2
        TColgp_Array1OfPnt poles(1, 4);
        Vector q0 = sb->ctrl[0];
        Vector q1 = sb->ctrl[1];
        Vector q2 = sb->ctrl[2];

        Vector c1 = q0.Plus(q1.Minus(q0).ScaledBy(2.0/3.0));
        Vector c2 = q2.Plus(q1.Minus(q2).ScaledBy(2.0/3.0));

        poles(1) = OccUtil::ToOccPoint(q0);
        poles(2) = OccUtil::ToOccPoint(c1);
        poles(3) = OccUtil::ToOccPoint(c2);
        poles(4) = OccUtil::ToOccPoint(q2);

        Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
        return BRepBuilderAPI_MakeEdge(curve);
    }
    else if(sb->deg == 3) {
        // Cubic Bezier
        TColgp_Array1OfPnt poles(1, 4);
        poles(1) = OccUtil::ToOccPoint(sb->ctrl[0]);
        poles(2) = OccUtil::ToOccPoint(sb->ctrl[1]);
        poles(3) = OccUtil::ToOccPoint(sb->ctrl[2]);
        poles(4) = OccUtil::ToOccPoint(sb->ctrl[3]);

        Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
        return BRepBuilderAPI_MakeEdge(curve);
    }

    // Fallback: treat as line
    gp_Pnt p0 = OccUtil::ToOccPoint(sb->Start());
    gp_Pnt p1 = OccUtil::ToOccPoint(sb->Finish());
    return BRepBuilderAPI_MakeEdge(p0, p1);
}

TopoDS_Wire FaceBuilder::BezierLoopToWire(const SBezierLoop *loop) const {
    BRepBuilderAPI_MakeWire wireBuilder;

    for(int i = 0; i < loop->l.n; i++) {
        const SBezier *sb = &loop->l[i];

        // Check for full circle (single degree-2 bezier that closes on itself)
        if(loop->l.n == 1 && sb->deg == 2) {
            Vector center;
            double radius;
            if(sb->IsCircle(normal, &center, &radius)) {
                // Check if it's actually a closed circle (start == finish)
                Vector startPt = sb->Start();
                Vector endPt = sb->Finish();
                if(startPt.Equals(endPt, LENGTH_EPS)) {
                    // It's a full circle
                    gp_Pnt occCenter = OccUtil::ToOccPoint(center);
                    gp_Dir occNormal = OccUtil::ToOccDir(normal);
                    gp_Circ circle(gp_Ax2(occCenter, occNormal), radius);
                    TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(circle);
                    wireBuilder.Add(edge);
                    continue;
                }
                // Otherwise fall through to create an arc edge
            }
        }

        TopoDS_Edge edge = BezierToEdge(sb);
        wireBuilder.Add(edge);
    }

    return wireBuilder.Wire();
}

TopoDS_Wire FaceBuilder::WireFromBezierList(const SBezierList *sbl, Vector normal) {
    BRepBuilderAPI_MakeWire wireBuilder;

    for(int i = 0; i < sbl->l.n; i++) {
        const SBezier *sb = &sbl->l[i];
        TopoDS_Edge edge;

        if(sb->deg == 1) {
            // Line segment
            gp_Pnt p0 = OccUtil::ToOccPoint(sb->ctrl[0]);
            gp_Pnt p1 = OccUtil::ToOccPoint(sb->ctrl[1]);
            edge = BRepBuilderAPI_MakeEdge(p0, p1);
        }
        else if(sb->deg == 2) {
            // Check if this is a circular arc
            Vector center;
            double radius;
            if(sb->IsCircle(normal, &center, &radius)) {
                gp_Pnt p0 = OccUtil::ToOccPoint(sb->Start());
                gp_Pnt pMid = OccUtil::ToOccPoint(sb->PointAt(0.5));
                gp_Pnt p1 = OccUtil::ToOccPoint(sb->Finish());

                GC_MakeArcOfCircle arcMaker(p0, pMid, p1);
                if(arcMaker.IsDone()) {
                    edge = BRepBuilderAPI_MakeEdge(arcMaker.Value());
                } else {
                    // Fallback: quadratic bezier elevated to cubic
                    TColgp_Array1OfPnt poles(1, 4);
                    Vector q0 = sb->ctrl[0], q1 = sb->ctrl[1], q2 = sb->ctrl[2];
                    Vector c1 = q0.Plus(q1.Minus(q0).ScaledBy(2.0/3.0));
                    Vector c2 = q2.Plus(q1.Minus(q2).ScaledBy(2.0/3.0));
                    poles(1) = OccUtil::ToOccPoint(q0);
                    poles(2) = OccUtil::ToOccPoint(c1);
                    poles(3) = OccUtil::ToOccPoint(c2);
                    poles(4) = OccUtil::ToOccPoint(q2);
                    Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
                    edge = BRepBuilderAPI_MakeEdge(curve);
                }
            } else {
                // Quadratic bezier elevated to cubic
                TColgp_Array1OfPnt poles(1, 4);
                Vector q0 = sb->ctrl[0], q1 = sb->ctrl[1], q2 = sb->ctrl[2];
                Vector c1 = q0.Plus(q1.Minus(q0).ScaledBy(2.0/3.0));
                Vector c2 = q2.Plus(q1.Minus(q2).ScaledBy(2.0/3.0));
                poles(1) = OccUtil::ToOccPoint(q0);
                poles(2) = OccUtil::ToOccPoint(c1);
                poles(3) = OccUtil::ToOccPoint(c2);
                poles(4) = OccUtil::ToOccPoint(q2);
                Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
                edge = BRepBuilderAPI_MakeEdge(curve);
            }
        }
        else if(sb->deg == 3) {
            // Cubic Bezier
            TColgp_Array1OfPnt poles(1, 4);
            poles(1) = OccUtil::ToOccPoint(sb->ctrl[0]);
            poles(2) = OccUtil::ToOccPoint(sb->ctrl[1]);
            poles(3) = OccUtil::ToOccPoint(sb->ctrl[2]);
            poles(4) = OccUtil::ToOccPoint(sb->ctrl[3]);
            Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
            edge = BRepBuilderAPI_MakeEdge(curve);
        }
        else {
            // Fallback: treat as line
            gp_Pnt p0 = OccUtil::ToOccPoint(sb->Start());
            gp_Pnt p1 = OccUtil::ToOccPoint(sb->Finish());
            edge = BRepBuilderAPI_MakeEdge(p0, p1);
        }

        wireBuilder.Add(edge);
    }

    if(wireBuilder.IsDone()) {
        return wireBuilder.Wire();
    }
    return TopoDS_Wire();  // Return null wire if failed
}

void FaceBuilder::BuildFaces(const SBezierLoopSet *sbls) {
    if(sbls->l.n == 0) return;

    normal = sbls->normal;
    planePoint = sbls->point;

    // Create a plane with the correct normal direction
    // This ensures the face orientation matches the SolveSpace normal
    gp_Pnt planePt = OccUtil::ToOccPoint(planePoint);
    gp_Dir planeDir = OccUtil::ToOccDir(normal);
    gp_Pln plane(planePt, planeDir);

    // Simple case: single loop = single face
    if(sbls->l.n == 1) {
        TopoDS_Wire wire = BezierLoopToWire(&sbls->l[0]);
        wires.push_back(wire);
        outerWire = wire;

        // Build face on the plane with correct normal direction
        BRepBuilderAPI_MakeFace faceBuilder(plane, wire, Standard_True);
        if(faceBuilder.IsDone()) {
            TopoDS_Face face = faceBuilder.Face();

            // Check if face normal matches expected normal, reverse if not
            BRepAdaptor_Surface surf(face);
            gp_Pnt pnt;
            gp_Vec d1u, d1v;
            surf.D1(surf.FirstUParameter() + (surf.LastUParameter() - surf.FirstUParameter()) / 2,
                    surf.FirstVParameter() + (surf.LastVParameter() - surf.FirstVParameter()) / 2,
                    pnt, d1u, d1v);
            gp_Vec faceNormal = d1u.Crossed(d1v);
            double dotProduct = faceNormal.Dot(gp_Vec(planeDir));

            // Compare with expected normal
            if(dotProduct < 0) {
                face.Reverse();
            }

            TopoDS_Builder builder;
            builder.Add(compound, face);
            faceCount = 1;
        }
        return;
    }

    // Multiple loops: need to determine which are outer contours and which are holes
    // In SolveSpace, loops are ordered with outer contours having positive area
    // and holes having negative area (or vice versa depending on normal direction)

    // For now, use a simple heuristic: largest area loop is outer, rest are holes
    // This matches how SolveSpace typically organizes profiles

    int outerIndex = 0;
    double maxArea = 0;

    for(int i = 0; i < sbls->l.n; i++) {
        const SBezierLoop *loop = &sbls->l[i];

        // Approximate area by converting to polygon
        SContour contour = {};
        loop->MakePwlInto(&contour, SS.ChordTolMm());
        double area = std::abs(contour.SignedAreaProjdToNormal(normal));
        contour.l.Clear();

        if(area > maxArea) {
            maxArea = area;
            outerIndex = i;
        }
    }

    // Build outer wire
    outerWire = BezierLoopToWire(&sbls->l[outerIndex]);
    wires.push_back(outerWire);

    // Build face on the plane with correct normal direction
    BRepBuilderAPI_MakeFace faceBuilder(plane, outerWire, Standard_True);

    // Add holes
    for(int i = 0; i < sbls->l.n; i++) {
        if(i == outerIndex) continue;

        TopoDS_Wire holeWire = BezierLoopToWire(&sbls->l[i]);
        wires.push_back(holeWire);
        faceBuilder.Add(holeWire);
        hasHoles = true;
    }

    if(faceBuilder.IsDone()) {
        TopoDS_Face face = faceBuilder.Face();

        // Check if face normal matches expected normal, reverse if not
        BRepAdaptor_Surface surf(face);
        gp_Pnt pnt;
        gp_Vec d1u, d1v;
        surf.D1(surf.FirstUParameter() + (surf.LastUParameter() - surf.FirstUParameter()) / 2,
                surf.FirstVParameter() + (surf.LastVParameter() - surf.FirstVParameter()) / 2,
                pnt, d1u, d1v);
        gp_Vec faceNormal = d1u.Crossed(d1v);

        if(faceNormal.Dot(gp_Vec(planeDir)) < 0) {
            face.Reverse();
        }

        TopoDS_Builder builder;
        builder.Add(compound, face);
        faceCount = 1;
    }
}

FaceBuilder FaceBuilder::FromBezierLoopSet(const SBezierLoopSet *sbls) {
    FaceBuilder fb;
    if(sbls) {
        fb.BuildFaces(sbls);
    }
    return fb;
}

const TopoDS_Wire &FaceBuilder::GetOuterWire() const {
    return outerWire;
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
