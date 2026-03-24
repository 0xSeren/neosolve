//-----------------------------------------------------------------------------
// OpenCascade-based solid model implementation.
//
//-----------------------------------------------------------------------------

#include "solidmodel.h"
#include "solvespace.h"

#ifdef HAVE_OPENCASCADE

#include <BRepMesh_IncrementalMesh.hxx>
#include <IMeshTools_Parameters.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly.hxx>
#include <Precision.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IGESControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <list>

namespace SolveSpace {

void SolidModelOcc::Clear() {
    shape.Nullify();
    shapeAcc.Nullify();
    displayMesh.Clear();
    edges.clear();
    faces.clear();
    meshCacheValid = false;
}

// Copy constructor - deep copy the mesh
SolidModelOcc::SolidModelOcc(const SolidModelOcc &other)
    : shape(other.shape),
      shapeAcc(other.shapeAcc),
      cachedShapeHash(other.cachedShapeHash),
      cachedChordTol(other.cachedChordTol),
      meshCacheValid(other.meshCacheValid),
      edges(other.edges),
      faces(other.faces)
{
    displayMesh.MakeFromCopyOf(const_cast<SMesh*>(&other.displayMesh));
}

// Copy assignment - deep copy the mesh
SolidModelOcc& SolidModelOcc::operator=(const SolidModelOcc &other) {
    if(this != &other) {
        shape = other.shape;
        shapeAcc = other.shapeAcc;
        cachedShapeHash = other.cachedShapeHash;
        cachedChordTol = other.cachedChordTol;
        meshCacheValid = other.meshCacheValid;
        edges = other.edges;
        faces = other.faces;
        displayMesh.Clear();
        displayMesh.MakeFromCopyOf(const_cast<SMesh*>(&other.displayMesh));
    }
    return *this;
}

// Move constructor - transfer ownership of the mesh
SolidModelOcc::SolidModelOcc(SolidModelOcc &&other) noexcept
    : shape(std::move(other.shape)),
      shapeAcc(std::move(other.shapeAcc)),
      cachedShapeHash(other.cachedShapeHash),
      cachedChordTol(other.cachedChordTol),
      meshCacheValid(other.meshCacheValid),
      edges(std::move(other.edges)),
      faces(std::move(other.faces))
{
    // Transfer the mesh data - steal the pointer
    displayMesh.l = other.displayMesh.l;
    displayMesh.flipNormal = other.displayMesh.flipNormal;
    displayMesh.keepInsideOtherShell = other.displayMesh.keepInsideOtherShell;
    displayMesh.keepCoplanar = other.displayMesh.keepCoplanar;
    displayMesh.atLeastOneDiscarded = other.displayMesh.atLeastOneDiscarded;
    displayMesh.isTransparent = other.displayMesh.isTransparent;

    // Null out the source to prevent double-free
    other.displayMesh.l = {};
    other.meshCacheValid = false;
}

// Move assignment - transfer ownership of the mesh
SolidModelOcc& SolidModelOcc::operator=(SolidModelOcc &&other) noexcept {
    if(this != &other) {
        // Free our current mesh
        displayMesh.Clear();

        shape = std::move(other.shape);
        shapeAcc = std::move(other.shapeAcc);
        cachedShapeHash = other.cachedShapeHash;
        cachedChordTol = other.cachedChordTol;
        meshCacheValid = other.meshCacheValid;
        edges = std::move(other.edges);
        faces = std::move(other.faces);

        // Transfer the mesh data
        displayMesh.l = other.displayMesh.l;
        displayMesh.flipNormal = other.displayMesh.flipNormal;
        displayMesh.keepInsideOtherShell = other.displayMesh.keepInsideOtherShell;
        displayMesh.keepCoplanar = other.displayMesh.keepCoplanar;
        displayMesh.atLeastOneDiscarded = other.displayMesh.atLeastOneDiscarded;
        displayMesh.isTransparent = other.displayMesh.isTransparent;

        // Null out the source
        other.displayMesh.l = {};
        other.meshCacheValid = false;
    }
    return *this;
}

bool SolidModelOcc::IsEmpty() const {
    return shapeAcc.IsNull();
}

void SolidModelOcc::GetBoundingBox(Vector *minPt, Vector *maxPt) const {
    *minPt = Vector::From(0, 0, 0);
    *maxPt = Vector::From(0, 0, 0);

    const TopoDS_Shape &s = shape.IsNull() ? shapeAcc : shape;
    if(s.IsNull()) return;

    Bnd_Box box;
    BRepBndLib::Add(s, box);

    if(box.IsVoid()) return;

    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    *minPt = Vector::From(xMin, yMin, zMin);
    *maxPt = Vector::From(xMax, yMax, zMax);
}

void SolidModelOcc::InitializeAccumulator() {
    shapeAcc = shape;
}

void SolidModelOcc::ApplyBoolean(Operation op, const SolidModelOcc *other) {
    if(!other || other->shapeAcc.IsNull()) return;
    if(shapeAcc.IsNull()) {
        shapeAcc = other->shapeAcc;
        return;
    }

    try {
        switch(op) {
            case Operation::UNION:
                shapeAcc = BRepAlgoAPI_Fuse(shapeAcc, other->shapeAcc).Shape();
                break;
            case Operation::DIFFERENCE:
                shapeAcc = BRepAlgoAPI_Cut(shapeAcc, other->shapeAcc).Shape();
                break;
            case Operation::INTERSECTION:
                shapeAcc = BRepAlgoAPI_Common(shapeAcc, other->shapeAcc).Shape();
                break;
        }
    } catch(const Standard_Failure &e) {
        dbp("OCC Boolean operation failed: %s", e.GetMessageString());
    }
}

void SolidModelOcc::UpdateAccumulator(Operation op, const SolidModelOcc *previous) {
    if(shape.IsNull()) {
        shapeAcc.Nullify();
        return;
    }

    if(!previous || previous->shapeAcc.IsNull()) {
        shapeAcc = shape;
        return;
    }

    try {
        switch(op) {
            case Operation::UNION: {
                BRepAlgoAPI_Fuse fuser(previous->shapeAcc, shape);
                fuser.Build();
                if(fuser.HasErrors()) {
                    // Fall back to creating a compound if fuse fails
                    BRep_Builder builder;
                    TopoDS_Compound compound;
                    builder.MakeCompound(compound);
                    builder.Add(compound, previous->shapeAcc);
                    builder.Add(compound, shape);
                    shapeAcc = compound;
                } else {
                    shapeAcc = fuser.Shape();
                }
                break;
            }
            case Operation::DIFFERENCE:
                shapeAcc = BRepAlgoAPI_Cut(previous->shapeAcc, shape).Shape();
                break;
            case Operation::INTERSECTION:
                shapeAcc = BRepAlgoAPI_Common(previous->shapeAcc, shape).Shape();
                break;
        }
    } catch(const Standard_Failure &e) {
        dbp("OCC Boolean operation failed: %s", e.GetMessageString());
        shapeAcc = shape;
    }
}

// Helper to process faces recursively
static void ProcessFace(const TopoDS_Face &face, SMesh &mesh, RgbaColor color,
                        uint32_t faceEntityHandle = 0) {
    if(face.IsNull()) return;

    TopLoc_Location loc;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
    if(triangulation.IsNull()) return;

    Poly::ComputeNormals(triangulation);

    // Get transformation from face location
    gp_Trsf tr = loc.Transformation();

    // Face orientation: REVERSED means outward normal is opposite to surface parametric normal
    bool reversed = (face.Orientation() == TopAbs_REVERSED);

    // Process triangles
    for(int i = 1; i <= triangulation->NbTriangles(); i++) {
        int n1, n2, n3;
#if OCC_VERSION_MAJOR >= 7 && OCC_VERSION_MINOR >= 6
        triangulation->Triangle(i).Get(n1, n2, n3);
#else
        triangulation->Triangles()(i).Get(n1, n2, n3);
#endif

        // Get vertices and transform them
        gp_Pnt p1 = triangulation->Node(n1).Transformed(tr);
        gp_Pnt p2 = triangulation->Node(n2).Transformed(tr);
        gp_Pnt p3 = triangulation->Node(n3).Transformed(tr);

        Vector v1 = Vector::From(p1.X(), p1.Y(), p1.Z());
        Vector v2 = Vector::From(p2.X(), p2.Y(), p2.Z());
        Vector v3 = Vector::From(p3.X(), p3.Y(), p3.Z());

        // Get OCC-computed normals and transform them
        // These are surface parametric normals, not face outward normals
        gp_Dir norm1 = triangulation->Normal(n1);
        gp_Dir norm2 = triangulation->Normal(n2);
        gp_Dir norm3 = triangulation->Normal(n3);

        norm1 = norm1.Transformed(tr);
        norm2 = norm2.Transformed(tr);
        norm3 = norm3.Transformed(tr);

        Vector vn1 = Vector::From(norm1.X(), norm1.Y(), norm1.Z());
        Vector vn2 = Vector::From(norm2.X(), norm2.Y(), norm2.Z());
        Vector vn3 = Vector::From(norm3.X(), norm3.Y(), norm3.Z());

        STriangle tri = {};

        // For REVERSED faces, the outward normal is opposite to the surface normal,
        // so we need to flip both the winding AND the normals
        if(reversed) {
            // Swap v2 and v3 to reverse winding, and negate normals
            tri.a = v1;
            tri.b = v3;
            tri.c = v2;
            tri.an = vn1.Negated();
            tri.bn = vn3.Negated();
            tri.cn = vn2.Negated();
        } else {
            tri.a = v1;
            tri.b = v2;
            tri.c = v3;
            tri.an = vn1;
            tri.bn = vn2;
            tri.cn = vn3;
        }

        tri.meta.face = faceEntityHandle;  // Set face entity for selection
        tri.meta.color = color;
        mesh.AddTriangle(&tri);
    }
}

// Use TopExp_Explorer to properly compose orientations through the shape hierarchy
static void ProcessShape(const TopoDS_Shape &shape, SMesh &mesh, RgbaColor color) {
    // TopExp_Explorer properly computes the composed orientation of each face
    // relative to the top-level shape (solid/shell/compound)
    TopExp_Explorer explorer(shape, TopAbs_FACE);
    while(explorer.More()) {
        TopoDS_Face face = TopoDS::Face(explorer.Current());
        ProcessFace(face, mesh, color);
        explorer.Next();
    }
}

// Process shape with face entity handles for selection support
static void ProcessShapeWithFaces(const TopoDS_Shape &shape, SMesh &mesh, RgbaColor color,
                                   const std::map<uint32_t, SolidModelOcc::FaceInfo> &faceMap) {
    std::list<TopoDS_Shape> seenFaces;
    uint32_t faceIndex = 0;

    TopExp_Explorer explorer(shape, TopAbs_FACE);
    while(explorer.More()) {
        TopoDS_Face face = TopoDS::Face(explorer.Current());

        // Skip duplicate faces (same logic as ExtractFaces)
        bool duplicate = false;
        for(const auto &seen : seenFaces) {
            if(seen.IsSame(face)) {
                duplicate = true;
                break;
            }
        }

        if(!duplicate) {
            seenFaces.push_back(face);

            // Look up the entity handle for this face
            uint32_t entityHandle = 0;
            auto it = faceMap.find(faceIndex);
            if(it != faceMap.end()) {
                entityHandle = it->second.entityHandle;
            }

            ProcessFace(face, mesh, color, entityHandle);
        }

        explorer.Next();
        faceIndex++;
    }
}

void SolidModelOcc::Triangulate(double chordTol) {
    if(shapeAcc.IsNull()) {
        displayMesh.Clear();
        meshCacheValid = false;
        return;
    }

    // Ensure tolerance is reasonable
    if(chordTol <= 0 || chordTol > 1000) {
        chordTol = OccUtil::DEFAULT_MESH_TOLERANCE;
    }

    // Check if we can use cached mesh
    // Use TShape pointer as hash - it's unique per shape definition
    const TopoDS_TShape* tshape = shapeAcc.TShape().get();
    size_t shapeHash = tshape ? reinterpret_cast<size_t>(tshape) : 0;

    if(meshCacheValid &&
       cachedShapeHash == shapeHash &&
       fabs(cachedChordTol - chordTol) < 1e-9 &&
       !displayMesh.l.IsEmpty()) {
        // Cache hit - mesh is still valid
        return;
    }

    displayMesh.Clear();

    try {
        // Generate mesh with specified tolerance
        // Using relative deflection mode for better results on imported models
        IMeshTools_Parameters params;
        params.Deflection = chordTol;
        params.Angle = OccUtil::DEFAULT_MESH_ANGLE;
        params.Relative = Standard_True;
        params.InParallel = Standard_True;

        BRepMesh_IncrementalMesh meshGen(shapeAcc, params);
        meshGen.Perform();

        if(!meshGen.IsDone()) {
            meshCacheValid = false;
            return;
        }

        // Use a default gray color - actual color will be set by Group
        RgbaColor color = RgbaColor::FromFloat(0.6f, 0.6f, 0.6f, 1.0f);

        // Use face-aware processing if faces have been extracted (for selection support)
        if(!faces.empty()) {
            ProcessShapeWithFaces(shapeAcc, displayMesh, color, faces);
        } else {
            ProcessShape(shapeAcc, displayMesh, color);
        }

        // Update cache state
        cachedShapeHash = shapeHash;
        cachedChordTol = chordTol;
        meshCacheValid = true;

    } catch(const Standard_Failure &e) {
        dbp("OCC Triangulation failed: %s", e.GetMessageString());
        meshCacheValid = false;
    }
}

void SolidModelOcc::ExtractEdges() {
    edges.clear();

    if(shapeAcc.IsNull()) return;

    std::list<TopoDS_Shape> seenEdges;
    uint32_t edgeIndex = 0;

    TopExp_Explorer explorer(shapeAcc, TopAbs_EDGE);
    while(explorer.More()) {
        TopoDS_Edge edge = TopoDS::Edge(explorer.Current());

        // Skip duplicate edges
        bool duplicate = false;
        for(const auto &seen : seenEdges) {
            if(seen.IsSame(edge)) {
                duplicate = true;
                break;
            }
        }

        if(!duplicate) {
            seenEdges.push_back(edge);

            EdgeInfo info;
            info.index = edgeIndex;

            try {
                BRepAdaptor_Curve curve(edge);
                GCPnts_TangentialDeflection discretizer(curve, M_PI / 16.0, 1e3);

                if(discretizer.NbPoints() > 0) {
                    for(int i = 1; i <= discretizer.NbPoints(); i++) {
                        gp_Pnt pnt = discretizer.Value(i);
                        info.points.push_back(Vector::From(pnt.X(), pnt.Y(), pnt.Z()));
                    }
                }
            } catch(const Standard_Failure &) {
                // Skip edges that can't be discretized
            }

            edges[edgeIndex] = std::move(info);
        }

        explorer.Next();
        edgeIndex++;
    }
}

int SolidModelOcc::ExtractFaces() {
    faces.clear();

    if(shapeAcc.IsNull()) return 0;

    std::list<TopoDS_Shape> seenFaces;
    uint32_t faceIndex = 0;

    TopExp_Explorer explorer(shapeAcc, TopAbs_FACE);
    while(explorer.More()) {
        TopoDS_Face face = TopoDS::Face(explorer.Current());

        // Skip duplicate faces
        bool duplicate = false;
        for(const auto &seen : seenFaces) {
            if(seen.IsSame(face)) {
                duplicate = true;
                break;
            }
        }

        if(!duplicate) {
            seenFaces.push_back(face);

            FaceInfo info;
            // Entity handle will be set by CreateOccFaceEntities after extraction
            info.entityHandle = 0;

            try {
                // Get face center using mass properties
                GProp_GProps props;
                BRepGProp::SurfaceProperties(face, props);
                gp_Pnt center = props.CentreOfMass();
                info.point = Vector::From(center.X(), center.Y(), center.Z());

                // Get face normal at center point
                BRepAdaptor_Surface surface(face);
                Standard_Real u = (surface.FirstUParameter() + surface.LastUParameter()) / 2.0;
                Standard_Real v = (surface.FirstVParameter() + surface.LastVParameter()) / 2.0;
                gp_Pnt pnt;
                gp_Vec du, dv;
                surface.D1(u, v, pnt, du, dv);
                gp_Vec normal = du.Crossed(dv);
                if(normal.Magnitude() > Precision::Confusion()) {
                    normal.Normalize();
                    // Adjust for face orientation
                    if(face.Orientation() == TopAbs_REVERSED) {
                        normal.Reverse();
                    }
                    info.normal = Vector::From(normal.X(), normal.Y(), normal.Z());
                } else {
                    info.normal = Vector::From(0, 0, 1);  // Fallback
                }

                faces[faceIndex] = info;
            } catch(const Standard_Failure &) {
                // Skip faces that can't be processed
            }
        }

        explorer.Next();
        faceIndex++;
    }

    return (int)faces.size();
}

void SolidModelOcc::MakeEdgesInto(SEdgeList *sel) const {
    for(const auto &pair : edges) {
        const EdgeInfo &info = pair.second;
        for(size_t i = 0; i + 1 < info.points.size(); i++) {
            sel->AddEdge(info.points[i], info.points[i + 1]);
        }
    }
}

void SolidModelOcc::ExportSTL(const Platform::Path &path) const {
    // TODO: Implement STL export when TKStl is linked
    if(shapeAcc.IsNull()) return;
    dbp("STL export not yet implemented for OCC solid models");
}

// Template implementation for finding selected edges
template<typename SelectionList>
void SolidModelOcc::FindSelectedEdges(const SelectionList *selection,
                                       std::vector<uint32_t> *outEdges) const {
    if(!selection || !outEdges) return;
    outEdges->clear();

    if(shapeAcc.IsNull()) return;

    // Collect selected line segment endpoints
    std::vector<std::pair<Vector, Vector>> selectedLines;

    for(int i = 0; i < selection->n; i++) {
        const auto &sel = (*selection)[i];
        if(sel.entity.v == 0) continue;

        Entity *e = SK.GetEntity(sel.entity);
        if(!e) continue;

        // Check if this is a line segment
        if(e->type == Entity::Type::LINE_SEGMENT) {
            Vector a = SK.GetEntity(e->point[0])->PointGetNum();
            Vector b = SK.GetEntity(e->point[1])->PointGetNum();
            selectedLines.push_back({a, b});
        }
    }

    if(selectedLines.empty()) return;

    // Match selected lines against OCC edges
    double tol = 1e-3;  // 1 micron tolerance
    std::list<TopoDS_Shape> seenEdges;
    uint32_t edgeIndex = 0;

    TopExp_Explorer explorer(shapeAcc, TopAbs_EDGE);
    while(explorer.More()) {
        TopoDS_Edge edge = TopoDS::Edge(explorer.Current());

        // Skip duplicate edges
        bool duplicate = false;
        for(const auto &seen : seenEdges) {
            if(seen.IsSame(edge)) {
                duplicate = true;
                break;
            }
        }

        if(!duplicate) {
            seenEdges.push_back(edge);

            try {
                BRepAdaptor_Curve curve(edge);
                gp_Pnt p1 = curve.Value(curve.FirstParameter());
                gp_Pnt p2 = curve.Value(curve.LastParameter());
                Vector ea = OccUtil::FromOccPoint(p1);
                Vector eb = OccUtil::FromOccPoint(p2);

                // Check if this edge matches any selected line
                for(const auto &line : selectedLines) {
                    Vector la = line.first;
                    Vector lb = line.second;

                    // Match if endpoints are close (in either order)
                    bool match = (ea.Minus(la).Magnitude() < tol && eb.Minus(lb).Magnitude() < tol) ||
                                 (ea.Minus(lb).Magnitude() < tol && eb.Minus(la).Magnitude() < tol);

                    if(match) {
                        outEdges->push_back(edgeIndex);
                        break;
                    }
                }
            } catch(const Standard_Failure &) {
                // Skip edges that can't be processed
            }

            edgeIndex++;
        }

        explorer.Next();
    }
}

// Explicit instantiation for List<GraphicsWindow::Selection>
template void SolidModelOcc::FindSelectedEdges(
    const List<GraphicsWindow::Selection> *selection,
    std::vector<uint32_t> *outEdges) const;

// Template implementation for finding selected faces
template<typename SelectionList>
void SolidModelOcc::FindSelectedFaces(const SelectionList *selection,
                                       std::vector<uint32_t> *outFaces) const {
    if(!selection || !outFaces) return;
    outFaces->clear();

    if(shapeAcc.IsNull() || faces.empty()) return;

    // Collect selected face entity handles
    std::vector<uint32_t> selectedHandles;

    for(int i = 0; i < selection->n; i++) {
        const auto &sel = (*selection)[i];
        if(sel.entity.v == 0) continue;

        Entity *e = SK.entity.FindByIdNoOops(sel.entity);
        if(!e) continue;

        // Check if this is a face entity (FACE_OCC or other face types)
        if(e->IsFace()) {
            selectedHandles.push_back(sel.entity.v);
        }
    }

    if(selectedHandles.empty()) return;

    // Match selected handles against extracted faces
    for(const auto &kv : faces) {
        uint32_t faceIndex = kv.first;
        const FaceInfo &info = kv.second;

        for(uint32_t handle : selectedHandles) {
            if(info.entityHandle == handle) {
                outFaces->push_back(faceIndex);
                break;
            }
        }
    }
}

// Explicit instantiation for List<GraphicsWindow::Selection>
template void SolidModelOcc::FindSelectedFaces(
    const List<GraphicsWindow::Selection> *selection,
    std::vector<uint32_t> *outFaces) const;

//-----------------------------------------------------------------------------
// STEP/BREP/IGES Export
//-----------------------------------------------------------------------------

bool SolidModelOcc::ExportSTEP(const Platform::Path &path) const {
    if(shapeAcc.IsNull()) return false;

    try {
        STEPControl_Writer writer;
        Interface_Static::SetCVal("write.step.unit", "MM");

        if(writer.Transfer(shapeAcc, STEPControl_AsIs) != IFSelect_RetDone) {
            dbp("STEP export: Transfer failed");
            return false;
        }

        std::string pathStr = path.raw;
        if(writer.Write(pathStr.c_str()) != IFSelect_RetDone) {
            dbp("STEP export: Write failed");
            return false;
        }

        return true;
    } catch(const Standard_Failure &e) {
        dbp("STEP export error: %s", e.GetMessageString());
        return false;
    }
}

bool SolidModelOcc::ExportBREP(const Platform::Path &path) const {
    if(shapeAcc.IsNull()) return false;

    try {
        std::string pathStr = path.raw;
        return BRepTools::Write(shapeAcc, pathStr.c_str());
    } catch(const Standard_Failure &e) {
        dbp("BREP export error: %s", e.GetMessageString());
        return false;
    }
}

//-----------------------------------------------------------------------------
// STEP/BREP/IGES Import
//-----------------------------------------------------------------------------

SolidModelOcc SolidModelOcc::ImportSTEP(const Platform::Path &path, bool *success) {
    SolidModelOcc result;
    *success = false;

    try {
        STEPControl_Reader reader;
        std::string pathStr = path.raw;

        if(reader.ReadFile(pathStr.c_str()) != IFSelect_RetDone) {
            dbp("STEP import: Failed to read file");
            return result;
        }

        reader.TransferRoots();
        int numShapes = reader.NbShapes();

        if(numShapes == 0) {
            dbp("STEP import: No shapes found");
            return result;
        }

        if(numShapes == 1) {
            result.shape = reader.Shape(1);
        } else {
            TopoDS_Compound compound;
            BRep_Builder builder;
            builder.MakeCompound(compound);
            for(int i = 1; i <= numShapes; i++) {
                builder.Add(compound, reader.Shape(i));
            }
            result.shape = compound;
        }

        result.shapeAcc = result.shape;
        *success = true;
    } catch(const Standard_Failure &e) {
        dbp("STEP import error: %s", e.GetMessageString());
    }

    return result;
}

SolidModelOcc SolidModelOcc::ImportBREP(const Platform::Path &path, bool *success) {
    SolidModelOcc result;
    *success = false;

    try {
        BRep_Builder builder;
        std::string pathStr = path.raw;

        if(!BRepTools::Read(result.shape, pathStr.c_str(), builder)) {
            dbp("BREP import: Failed to read file");
            return result;
        }

        result.shapeAcc = result.shape;
        *success = true;
    } catch(const Standard_Failure &e) {
        dbp("BREP import error: %s", e.GetMessageString());
    }

    return result;
}

SolidModelOcc SolidModelOcc::ImportIGES(const Platform::Path &path, bool *success) {
    SolidModelOcc result;
    *success = false;

    try {
        IGESControl_Reader reader;
        std::string pathStr = path.raw;

        if(reader.ReadFile(pathStr.c_str()) != IFSelect_RetDone) {
            dbp("IGES import: Failed to read file");
            return result;
        }

        reader.TransferRoots();
        int numShapes = reader.NbShapes();

        if(numShapes == 0) {
            dbp("IGES import: No shapes found");
            return result;
        }

        if(numShapes == 1) {
            result.shape = reader.Shape(1);
        } else {
            TopoDS_Compound compound;
            BRep_Builder builder;
            builder.MakeCompound(compound);
            for(int i = 1; i <= numShapes; i++) {
                builder.Add(compound, reader.Shape(i));
            }
            result.shape = compound;
        }

        result.shapeAcc = result.shape;
        *success = true;
    } catch(const Standard_Failure &e) {
        dbp("IGES import error: %s", e.GetMessageString());
    }

    return result;
}

// Static cache for imported solids
std::map<std::string, SolidModelOcc::CachedImport> SolidModelOcc::importCache;
static std::mutex importCacheMutex;

SolidModelOcc SolidModelOcc::ImportCached(const Platform::Path &path, bool *success) {
    SolidModelOcc result;
    *success = false;

    std::string pathStr = path.raw;

    // Check cache first (with lock)
    {
        std::lock_guard<std::mutex> lock(importCacheMutex);
        auto it = importCache.find(pathStr);
        if(it != importCache.end()) {
            result.shape = it->second.shape;
            result.shapeAcc = it->second.shape;
            result.displayMesh.MakeFromCopyOf(it->second.displayMesh.get());
            result.edges = it->second.edges;
            *success = true;
            return result;
        }
    }

    // Not in cache - import based on extension (outside lock - this is slow)
    if(path.HasExtension("step") || path.HasExtension("stp")) {
        result = ImportSTEP(path, success);
    } else if(path.HasExtension("brep") || path.HasExtension("brp")) {
        result = ImportBREP(path, success);
    } else if(path.HasExtension("iges") || path.HasExtension("igs")) {
        result = ImportIGES(path, success);
    }

    // Triangulate and cache the result if successful
    if(*success) {
        // Triangulate and extract edges before caching (outside lock - this is slow)
        result.Triangulate(OccUtil::DEFAULT_MESH_TOLERANCE);
        result.ExtractEdges();

        CachedImport cached;
        cached.shape = result.shape;
        cached.displayMesh->MakeFromCopyOf(&result.displayMesh);
        cached.edges = result.edges;
        result.GetBoundingBox(&cached.bboxMin, &cached.bboxMax);

        // Add to cache (with lock)
        {
            std::lock_guard<std::mutex> lock(importCacheMutex);
            importCache.emplace(pathStr, std::move(cached));
        }
    }

    return result;
}

const SMesh *SolidModelOcc::GetCachedMesh(const Platform::Path &path) {
    std::lock_guard<std::mutex> lock(importCacheMutex);
    std::string pathStr = path.raw;
    auto it = importCache.find(pathStr);
    if(it != importCache.end()) {
        return it->second.displayMesh.get();
    }
    return nullptr;
}

bool SolidModelOcc::GetCachedBoundingBox(const Platform::Path &path, Vector *minPt, Vector *maxPt) {
    std::lock_guard<std::mutex> lock(importCacheMutex);
    std::string pathStr = path.raw;
    auto it = importCache.find(pathStr);
    if(it != importCache.end()) {
        *minPt = it->second.bboxMin;
        *maxPt = it->second.bboxMax;
        return true;
    }
    return false;
}

bool SolidModelOcc::GetCachedEdgesInto(const Platform::Path &path, SEdgeList *sel) {
    std::lock_guard<std::mutex> lock(importCacheMutex);
    std::string pathStr = path.raw;
    auto it = importCache.find(pathStr);
    if(it != importCache.end()) {
        for(const auto &pair : it->second.edges) {
            const EdgeInfo &info = pair.second;
            for(size_t i = 0; i + 1 < info.points.size(); i++) {
                sel->AddEdge(info.points[i], info.points[i + 1]);
            }
        }
        return true;
    }
    return false;
}

void SolidModelOcc::ClearImportCache() {
    importCache.clear();
}

// Static storage for async imports
std::map<std::string, SolidModelOcc::AsyncImportState> SolidModelOcc::asyncImports;
std::mutex SolidModelOcc::asyncImportMutex;

void SolidModelOcc::StartAsyncImport(const Platform::Path &path) {
    std::string pathStr = path.raw;

    std::lock_guard<std::mutex> lock(asyncImportMutex);

    // Check if already cached
    if(importCache.find(pathStr) != importCache.end()) {
        return; // Already in cache, nothing to do
    }

    // Check if already importing
    auto it = asyncImports.find(pathStr);
    if(it != asyncImports.end() && !it->second.completed) {
        return; // Already importing
    }

    // Start async import
    Platform::Path pathCopy = path;
    AsyncImportState state;
    state.completed = false;
    state.success = false;
    state.future = std::async(std::launch::async, [pathCopy]() -> bool {
        bool success = false;
        // This runs in background thread
        SolidModelOcc::ImportCached(pathCopy, &success);
        return success;
    });

    asyncImports[pathStr] = std::move(state);
}

bool SolidModelOcc::IsImportInProgress(const Platform::Path &path) {
    std::string pathStr = path.raw;

    std::lock_guard<std::mutex> lock(asyncImportMutex);

    // If already cached, not in progress
    if(importCache.find(pathStr) != importCache.end()) {
        return false;
    }

    auto it = asyncImports.find(pathStr);
    if(it == asyncImports.end()) {
        return false;
    }

    // Check if future is ready without blocking
    if(!it->second.completed) {
        auto status = it->second.future.wait_for(std::chrono::milliseconds(0));
        if(status == std::future_status::ready) {
            it->second.completed = true;
            it->second.success = it->second.future.get();
        }
    }

    return !it->second.completed;
}

bool SolidModelOcc::CheckAsyncImportComplete(const Platform::Path &path, bool *success) {
    std::string pathStr = path.raw;

    std::lock_guard<std::mutex> lock(asyncImportMutex);

    // If already cached, import is complete
    if(importCache.find(pathStr) != importCache.end()) {
        *success = true;
        return true;
    }

    auto it = asyncImports.find(pathStr);
    if(it == asyncImports.end()) {
        *success = false;
        return true; // Not started means "complete" with failure
    }

    // Check if future is ready
    if(!it->second.completed) {
        auto status = it->second.future.wait_for(std::chrono::milliseconds(0));
        if(status == std::future_status::ready) {
            it->second.completed = true;
            it->second.success = it->second.future.get();
        }
    }

    if(it->second.completed) {
        *success = it->second.success;
        return true;
    }

    return false; // Still in progress
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
