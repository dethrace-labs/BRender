#include "priminfo.h"
#include <stdarg.h>
#include "fpsetup.h"

float                            temp;
struct workspace_t               workspace;
struct ArbitraryWidthWorkspace_t workspaceA;

void TriangleRender_ZT_I8_D16(brp_block *block, ...)
{
    brp_vertex *v0; // [esp+18h] [ebp+Ch]
    brp_vertex *v1; // [esp+1Ch] [ebp+10h]
    brp_vertex *v2; // [esp+20h] [ebp+14h]
    va_list     va; // [esp+24h] [ebp+18h] BYREF
    va_start(va, block);
    v0 = va_arg(va, brp_vertex *);
    v1 = va_arg(va, brp_vertex *);
    v2 = va_arg(va, brp_vertex *);
    va_end(va);

    workspace.v0 = v0;
    workspace.v1 = v1;
    workspace.v2 = v2;

    TriangleSetup_ZT_ARBITRARY(v0, v1, v2);
}
