#include "solvespace.h"

#include "harness.h"

TEST_CASE(normal_roundtrip) {
    CHECK_LOAD("normal.slvs");
    CHECK_RENDER_ISO("normal.png");
    CHECK_SAVE("normal.slvs");
}

TEST_CASE(normal_migrate_from_v22) {
    CHECK_LOAD("normal_v22.slvs");
    CHECK_SAVE("normal.slvs");
}

TEST_CASE(normal_inters) {
    CHECK_LOAD("normal.slvs");

    // Get the last group safely
    Group *g = nullptr;
    for(int i = SK.groupOrder.n - 1; i >= 0; i--) {
        g = SK.group.FindByIdNoOops(SK.groupOrder[i]);
        if(g) break;
    }
    CHECK_TRUE(g != nullptr);
    g->GenerateDisplayItems();
    SMesh *m = &g->displayMesh;

    SEdgeList el = {};
    bool inters, leaks;
    SKdNode::From(m)->MakeCertainEdgesInto(&el,
        EdgeKind::SELF_INTER, /*coplanarIsInter=*/false, &inters, &leaks);
    el.Clear();

    // The assembly is supposed to interfere.
    CHECK_TRUE(inters);
}
