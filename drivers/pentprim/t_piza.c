#include <brender.h>
#include <limits.h>
#include "work.h"

#define add_carry(x, y) (x += y, (br_uint_32)x < (br_uint_32)y)

#define scan_inc(carry, wrap)                                      \
    {                                                              \
        work.pz.current += work.pz.d_##carry;                      \
        if(use_light)                                              \
            work.pi.current += work.pi.d_##carry;                  \
        if(add_carry(work.awsl.u_current, work.awsl.du_##carry)) { \
            work.awsl.source_current += bpp;                       \
            if(wrap)                                               \
                work.awsl.u_int_current++;                         \
        }                                                          \
        work.awsl.source_current += work.awsl.dsource_##carry;     \
        if(wrap)                                                   \
            work.awsl.u_int_current += work.awsl.du_int_##carry;   \
        if(add_carry(work.awsl.v_current, work.awsl.dv_##carry))   \
            work.awsl.source_current += work.awsl.texture_stride;  \
    }

br_size_t noffset;

static inline void __TriangleRenderZ2(br_boolean use_light, br_boolean use_bump, br_boolean use_transparency, br_uint_32 bpp)
{
}

void BR_ASM_CALL TrapezoidRenderPIZ2TA()
{
    __TriangleRenderZ2(BR_FALSE, BR_FALSE, BR_TRUE, 1);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TA_RGB_555()
{
    __TriangleRenderZ2(BR_FALSE, BR_FALSE, BR_TRUE, 2);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TA_RGB_888()
{
    __TriangleRenderZ2(BR_FALSE, BR_FALSE, BR_TRUE, 3);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TAN()
{
    __TriangleRenderZ2(BR_FALSE, BR_TRUE, BR_TRUE, 1);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TAN_RGB_555()
{
    __TriangleRenderZ2(BR_FALSE, BR_TRUE, BR_TRUE, 2);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TAN_RGB_888()
{
    __TriangleRenderZ2(BR_FALSE, BR_TRUE, BR_TRUE, 3);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TIA()
{
    __TriangleRenderZ2(BR_TRUE, BR_FALSE, BR_TRUE, 1);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TIA_RGB_555()
{
    __TriangleRenderZ2(BR_TRUE, BR_FALSE, BR_TRUE, 2);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TIA_RGB_888()
{
    __TriangleRenderZ2(BR_TRUE, BR_FALSE, BR_TRUE, 3);
}

void BR_ASM_CALL TrapezoidRenderPIZ2TIANT()
{
    __TriangleRenderZ2(BR_TRUE, BR_FALSE, BR_FALSE, 1);
}
