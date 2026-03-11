//-----------------------------------------------------------------------------
// FaceBuilder: Convert SolveSpace SBezierLoopSet to OpenCascade TopoDS_Face.
// This is the key bridge between the constraint solver output and OCC geometry.
//
//-----------------------------------------------------------------------------

#ifndef SOLVESPACE_OCC_FACEBUILDER_H
#define SOLVESPACE_OCC_FACEBUILDER_H

#ifdef HAVE_OPENCASCADE

#include "occutil.h"
#include "srf/surface.h"
#include <TopoDS_Compound.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <list>

namespace SolveSpace {

class FaceBuilder {
public:
    // Build faces from a SBezierLoopSet (sketch profile)
    // The loops should be in a common plane with the given normal
    static FaceBuilder FromBezierLoopSet(const SBezierLoopSet *sbls);

    // Build a wire from a bezier list (can be open or closed path)
    static TopoDS_Wire WireFromBezierList(const SBezierList *sbl, Vector normal);

    // Get the resulting compound of faces
    const TopoDS_Compound &GetFaces() const { return compound; }

    // Get the outer wire (for loft operations)
    const TopoDS_Wire &GetOuterWire() const;

    // Get all wires
    const std::list<TopoDS_Wire> &GetWires() const { return wires; }

    // Check if the profile contains holes
    bool HasHoles() const { return hasHoles; }

    // Number of faces built
    unsigned int GetFaceCount() const { return faceCount; }

    // Check if build succeeded
    bool IsValid() const { return faceCount > 0; }

private:
    FaceBuilder();

    // Convert a single SBezier to an OCC edge
    TopoDS_Edge BezierToEdge(const SBezier *sb) const;

    // Convert a SBezierLoop to an OCC wire
    TopoDS_Wire BezierLoopToWire(const SBezierLoop *loop) const;

    // Build faces from loops, detecting outer contours and holes
    void BuildFaces(const SBezierLoopSet *sbls);

    TopoDS_Compound compound;
    std::list<TopoDS_Wire> wires;
    TopoDS_Wire outerWire;
    unsigned int faceCount = 0;
    bool hasHoles = false;

    // Normal and point for the sketch plane
    Vector normal;
    Vector planePoint;
};

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
#endif // SOLVESPACE_OCC_FACEBUILDER_H
