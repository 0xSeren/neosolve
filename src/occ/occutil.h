//-----------------------------------------------------------------------------
// OpenCascade utility functions and common includes.
//
//-----------------------------------------------------------------------------

#ifndef SOLVESPACE_OCC_UTIL_H
#define SOLVESPACE_OCC_UTIL_H

#ifdef HAVE_OPENCASCADE

#include <Standard_Version.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>

#include "dsc.h"

namespace SolveSpace {
namespace OccUtil {

// Convert SolveSpace Vector to OCC gp_Pnt
inline gp_Pnt ToOccPoint(const Vector &v) {
    return gp_Pnt(v.x, v.y, v.z);
}

// Convert OCC gp_Pnt to SolveSpace Vector
inline Vector FromOccPoint(const gp_Pnt &p) {
    return Vector::From(p.X(), p.Y(), p.Z());
}

// Convert SolveSpace Vector to OCC gp_Vec
inline gp_Vec ToOccVec(const Vector &v) {
    return gp_Vec(v.x, v.y, v.z);
}

// Convert OCC gp_Vec to SolveSpace Vector
inline Vector FromOccVec(const gp_Vec &v) {
    return Vector::From(v.X(), v.Y(), v.Z());
}

// Convert SolveSpace Vector to OCC gp_Dir (normalized)
inline gp_Dir ToOccDir(const Vector &v) {
    Vector n = v.WithMagnitude(1.0);
    return gp_Dir(n.x, n.y, n.z);
}

// Check if OCC version is >= 7.6 for newer API
#if OCC_VERSION_MAJOR >= 7 && OCC_VERSION_MINOR >= 6
#define SOLVESPACE_OCC_NEW_API
#endif

// Chord tolerance for triangulation (in mm)
constexpr double DEFAULT_MESH_TOLERANCE = 0.1;
constexpr double DEFAULT_MESH_ANGLE = 0.5; // radians (~28 degrees)

} // namespace OccUtil
} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
#endif // SOLVESPACE_OCC_UTIL_H
