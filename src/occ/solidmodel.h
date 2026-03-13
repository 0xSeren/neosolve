//-----------------------------------------------------------------------------
// OpenCascade-based solid model storage and operations.
// Replaces the NURBS-based SShell for solid modeling operations.
//
//-----------------------------------------------------------------------------

#ifndef SOLVESPACE_OCC_SOLIDMODEL_H
#define SOLVESPACE_OCC_SOLIDMODEL_H

#ifdef HAVE_OPENCASCADE

#include "occutil.h"
#include "polygon.h"
#include "platform/platform.h"
#include <TopoDS_Shape.hxx>
#include <map>
#include <vector>
#include <future>
#include <mutex>
#include <memory>

namespace SolveSpace {

// Forward declaration
class GraphicsWindow;

class SolidModelOcc {
public:
    // The shape from the current operation (extrude, revolve, etc.)
    TopoDS_Shape shape;

    // The accumulated shape after boolean operations with previous groups
    TopoDS_Shape shapeAcc;

    // Triangulated mesh for display
    SMesh displayMesh;

    // Mesh cache validation
    size_t cachedShapeHash = 0;
    double cachedChordTol = 0.0;
    bool meshCacheValid = false;

    // Edge information for selection (fillet/chamfer)
    struct EdgeInfo {
        uint32_t index;
        std::vector<Vector> points; // Discretized edge points for display
    };
    std::map<uint32_t, EdgeInfo> edges;

    // Constructor/destructor for proper memory management
    SolidModelOcc() = default;
    ~SolidModelOcc() { displayMesh.Clear(); }

    // Copy operations - deep copy the mesh to avoid sharing raw pointers
    SolidModelOcc(const SolidModelOcc &other);
    SolidModelOcc& operator=(const SolidModelOcc &other);

    // Move operations
    SolidModelOcc(SolidModelOcc &&other) noexcept;
    SolidModelOcc& operator=(SolidModelOcc &&other) noexcept;

    // Generate triangle mesh from shapeAcc for rendering
    void Triangulate(double chordTol = OccUtil::DEFAULT_MESH_TOLERANCE);

    // Extract edges from shapeAcc for selection/display
    void ExtractEdges();

    // Clear all data
    void Clear();

    // Check if the model is empty
    bool IsEmpty() const;

    // Get bounding box (min and max corners)
    void GetBoundingBox(Vector *minPt, Vector *maxPt) const;

    // Boolean operations
    enum class Operation {
        UNION,
        DIFFERENCE,
        INTERSECTION
    };

    // Perform boolean operation: result = this->shapeAcc OP other->shapeAcc
    // Store result in this->shapeAcc
    void ApplyBoolean(Operation op, const SolidModelOcc *other);

    // Set shapeAcc directly from shape (when there's no previous solid)
    void InitializeAccumulator();

    // Apply boolean against a previous group's solid model
    void UpdateAccumulator(Operation op, const SolidModelOcc *previous);

    // Export functions
    void ExportSTL(const Platform::Path &path) const;
    bool ExportSTEP(const Platform::Path &path) const;
    bool ExportBREP(const Platform::Path &path) const;

    // Import functions (static factory methods)
    static SolidModelOcc ImportSTEP(const Platform::Path &path, bool *success);
    static SolidModelOcc ImportBREP(const Platform::Path &path, bool *success);
    static SolidModelOcc ImportIGES(const Platform::Path &path, bool *success);

    // Cached import - returns cached solid if available, otherwise imports and caches
    static SolidModelOcc ImportCached(const Platform::Path &path, bool *success);

    // Get pointer to cached mesh (avoids copying) - returns nullptr if not cached
    static const SMesh *GetCachedMesh(const Platform::Path &path);

    // Get cached bounding box - returns false if not cached
    static bool GetCachedBoundingBox(const Platform::Path &path, Vector *minPt, Vector *maxPt);

    // Get cached edges directly into SEdgeList - returns false if not cached
    static bool GetCachedEdgesInto(const Platform::Path &path, SEdgeList *sel);

    // Clear the import cache (call when closing document or freeing memory)
    static void ClearImportCache();

    // Async import - starts background thread for import
    static void StartAsyncImport(const Platform::Path &path);

    // Check if async import is in progress for a path
    static bool IsImportInProgress(const Platform::Path &path);

    // Check if async import completed (and get success status)
    static bool CheckAsyncImportComplete(const Platform::Path &path, bool *success);

    // Get edges for display as SEdgeList
    void MakeEdgesInto(SEdgeList *sel) const;

    // Find which OCC edges match the selected entities
    // Stores matching edge indices in outEdges
    template<typename SelectionList>
    void FindSelectedEdges(const SelectionList *selection, std::vector<uint32_t> *outEdges) const;

private:
    // Cache for imported solids (keyed by absolute file path)
    // Uses unique_ptr for SMesh to ensure proper memory management
    // (SMesh contains List which has no destructor and shallow copy semantics)
    struct CachedImport {
        TopoDS_Shape shape;
        std::unique_ptr<SMesh> displayMesh;
        std::map<uint32_t, EdgeInfo> edges;
        Vector bboxMin, bboxMax;

        CachedImport() : displayMesh(new SMesh()) {}
        ~CachedImport() {
            if(displayMesh) displayMesh->Clear();
        }
        // Non-copyable, move-only
        CachedImport(const CachedImport&) = delete;
        CachedImport& operator=(const CachedImport&) = delete;
        CachedImport(CachedImport&&) = default;
        CachedImport& operator=(CachedImport&&) = default;
    };
    static std::map<std::string, CachedImport> importCache;

    // Async import tracking
    struct AsyncImportState {
        std::future<bool> future;
        bool completed;
        bool success;
    };
    static std::map<std::string, AsyncImportState> asyncImports;
    static std::mutex asyncImportMutex;
};

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
#endif // SOLVESPACE_OCC_SOLIDMODEL_H
