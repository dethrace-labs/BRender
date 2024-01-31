#include <brender.h>
#include <limits.h>
#include <stdint.h>
#include "work.h"
#include <stdio.h>
#include "fpsetup.h"

#define _BYTE  uint8_t
#define _WORD  uint16_t
#define _DWORD uint32_t
#define _QWORD uint64_t

#if defined(__BYTE_ORDER) && __BYTE_ORDER == __BIG_ENDIAN
#define LOW_IND(x, part_type)  LAST_IND(x, part_type)
#define HIGH_IND(x, part_type) 0
#else
#define HIGH_IND(x, part_type) LAST_IND(x, part_type)
#define LOW_IND(x, part_type)  0
#endif

// Some convenience macros to make partial accesses nicer
#define LAST_IND(x, part_type) (sizeof(x) / sizeof(part_type) - 1)

// first unsigned macros:
#define BYTEn(x, n)     (*((_BYTE *)&(x) + n))
#define WORDn(x, n)     (*((_WORD *)&(x) + n))
#define DWORDn(x, n)    (*((_DWORD *)&(x) + n))

#define LOBYTE(x)       BYTEn(x, LOW_IND(x, _BYTE))
#define LOWORD(x)       WORDn(x, LOW_IND(x, _WORD))
#define LODWORD(x)      DWORDn(x, LOW_IND(x, _DWORD))
#define HIBYTE(x)       BYTEn(x, HIGH_IND(x, _BYTE))
#define HIWORD(x)       WORDn(x, HIGH_IND(x, _WORD))
#define HIDWORD(x)      DWORDn(x, HIGH_IND(x, _DWORD))

#define SDWORDn(x, n)   (*((int32_t *)&(x) + n))
#define SLOBYTE(x)      SBYTEn(x, LOW_IND(x, int8))
#define SLOWORD(x)      SWORDn(x, LOW_IND(x, int16))
#define SLODWORD(x)     SDWORDn(x, LOW_IND(x, int32))

#define add_carry(x, y) (x += y, (br_uint_32)x < (br_uint_32)y)

static int __CFADD__(uint32_t x, uint32_t y)
{
    return x > x + y;
}

static int __SETS__(uint32_t x)
{
    return (int32_t)x < 0;
}

static int __OFADD__(uint32_t x, uint32_t y)
{
    uint32_t y2 = y;
    int8_t   sx = __SETS__(x);
    return ((1 ^ sx) ^ __SETS__(y2)) & (sx ^ __SETS__(x + y2));
}

struct temp_vertex_fixed {
};

float                            temp;
struct workspace_t               workspace;
struct ArbitraryWidthWorkspace_t workspaceA;

extern int BR_ASM_CALL SafeFixedMac2Div(int, int, int, int, int);

#define swap(type, a, b) \
    {                    \
        type _;          \
        _ = a;           \
        a = b;           \
        b = _;           \
    }

static br_uint_32 temp_b      = 0;
static br_uint_32 temp_g      = 0;
static br_uint_32 temp_r      = 0;
static br_uint_32 temp_colour = 0;

static br_uint_32 p0_offset_x = 0;
static br_uint_32 p0_offset_y = 0;

static br_uint_32 temp_i = 0;
static br_uint_32 temp_u = 0;
static br_uint_32 temp_v = 0;

static screen_scalar vertex_0[2];
static screen_scalar vertex_1[2];
static screen_scalar vertex_2[2];

static br_int_32 g_divisor = 0;

static inline void __TRAPEZOID_PIZ2(struct scan_edge *edge, br_boolean is_forward, br_fixed_ls *z_val,
                                    br_int_32 *fb_index, br_uint_8 colour)
{
}

static inline void __TRAPEZOID_PIZ2I(struct scan_edge *edge, br_boolean is_forward, br_fixed_ls *z_val, br_int_32 *fb_index)
{
}

static inline void __TRAPEZOID_PIZ2T(struct scan_edge *edge, br_boolean is_forward, br_fixed_ls *z_val, br_int_32 *fb_index)
{
}

static inline void __TRAPEZOID_PIZ2TI(struct scan_edge *edge, br_boolean is_forward, br_fixed_ls *z_val, br_int_32 *fb_index)
{
}

static inline void __SETUP_PI(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

static inline void __PARAM_PI_DIRN(screen_scalar a, screen_scalar b, screen_scalar c, struct scan_parameter *param)
{
}

void BR_ASM_CALL RawTriangle_PIZ2(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL RawTriangle_PIZ2I(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL RawTriangle_PIZ2T(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL RawTriangle_PIZ2TI(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL TriangleRenderPIZ2(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL TriangleRenderPIZ2I(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL TriangleRenderPIZ2T(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void BR_ASM_CALL TriangleRenderPIZ2TI(struct temp_vertex_fixed *v0, struct temp_vertex_fixed *v1, struct temp_vertex_fixed *v2)
{
}

void divzero()
{
    // no op
}

/*
// not really a function, this is a macro in asm
// eax=v0 ecx=v1 edx=v2
void SETUP_FLOAT(brp_vertex *v0, brp_vertex *v1, brp_vertex *v2, brp_vertex *vt)
{

    // double       dx2;                   // st7
    // double       dx1;                   // st6
    // double       dy2;                   // st5
    // double       dy1;                   // st4
    // int          _EBX;                  // ebx
    // double       half_area;             // st3
    // uint32_t     old_flip;              // esi
    // brp_vertex  *vt;                    // eax
    // brp_vertex  *vm;                    // edx
    // brp_vertex  *vb;                    // ebx
    // unsigned int y_t;                   // ebp
    // unsigned int y_m;                   // esi
    // unsigned int y_b;                   // edi
    // double       v16;                   // st4
    // double       v17;                   // st2
    // double       bottom_minus_top_y;    // st5
    // double       v19;                   // st2
    // double       v20;                   // st1
    // double       middle_minus_bottom_y; // st6
    // double       v22;                   // st0
    // double       middle_minus_top_y;    // st7
    // double       bottom_minus_top_x;    // st3
    // double       v25;                   // st2
    // double       v26;                   // st1
    // double       middle_minus_bottom_x; // st3
    // double       v28;                   // st2
    // double       v29;                   // rt2
    // double       middle_minus_top_x;    // st1
    // double       v31;                   // st2
    // double       v32;                   // st7
    // double       v33;                   // st3
    // double       v34;                   // st4
    // double       v35;                   // rt1
    // double       v36;                   // st2
    // double       v37;                   // st6
    // double       v38;                   // st3
    // double       v39;                   // st2
    // double       v40;                   // st7
    // double       v41;                   // st5
    // double       v42;                   // st6
    // float        xm;                    // edx
    // float        x1;                    // esi
    // float        x2;                    // edi
    // double       v46;                   // st6
    // double       v47;                   // st7
    // double       v48;                   // st2
    // double       v49;                   // st2
    // double       v50;                   // st7
    // double       v51;                   // st6
    // double       v52;                   // st4
    // double       v53;                   // st2
    // double       v54;                   // rt0
    // double       v55;                   // st1
    // double       v56;                   // st3
    // double       v57;                   // st4
    // float        s_z;                   // esi
    // float        d_z_x;                 // edi
    // double       v60;                   // st7
    // double       v61;                   // st6
    // double       v62;                   // st4
    // double       v63;                   // st2
    // double       v64;                   // rt1
    // double       v65;                   // st1
    // double       v66;                   // st3
    // double       v67;                   // st4
    // uint32_t     s_u;                   // esi
    // uint32_t     d_u_x;                 // edi
    // double       v70;                   // st7
    // double       v71;                   // st6
    // double       v72;                   // st4
    // double       v73;                   // st2
    // double       v74;                   // rt2
    // double       v75;                   // st1
    // double       v76;                   // st3
    // double       v77;                   // st4
    // uint32_t     s_v;                   // esi
    // uint32_t     d_v_x;                 // edi
    // int          v80;                   // esi
    // double       v81;                   // st7
    // double       v82;                   // st6
    // double       v83;                   // st5
    // double       v84;                   // st3
    // double       v85;                   // st4
    // int          v86;                   // ebp
    // uint32_t     v87;                   // eax
    // int          v88;                   // ebp
    // uint32_t     d_u_y_0;               // eax
    // int          v90;                   // ebp
    // uint32_t     d_u_y_1;               // eax
    // int          v92;                   // ebp
    // uint32_t     v93;                   // eax
    // int          v94;                   // ebp
    // uint32_t     d_v_y_0;               // eax
    // int          v96;                   // ebp
    // uint32_t     d_v_y_1;               // eax
    // double       width_p;               // st7
    // double       height;                // st7
    // double       stride_b;              // st3

    // dx2  = v2->comp_f[C_SX] - v0->comp_f[C_SX];
    // dx1  = v1->comp_f[C_SX] - v0->comp_f[C_SX];
    // dy2  = v2->comp_f[C_SY] - v0->comp_f[C_SY];
    // dy1  = v1->comp_f[C_SX] - v0->comp_f[C_SX];
    // _EBX = 0;
    // // cmp ecx, eax
    // // __asm { rcl     ebx, 1 }
    // if(v1->comp_f[C_SY] < v0->comp_f[C_SY]) {
    //     _EBX |= 1;
    // }
    // half_area = fp_one / (dx1 * dy2 - dx2 * dy1);
    // // cmp edx, eax
    // if(v2->comp_f[C_SY] < v0->comp_f[C_SY]) {
    //     _EBX << 1;
    //     _EBX |= 1;
    // }
    // // cmp edx, ecx
    // if(v2->comp_f[C_SY] < v1->comp_f[C_SY]) {
    //     _EBX << 1;
    //     _EBX |= 1;
    // }

    // old_flip       = flip_table[_EBX];
    // eax_           = *(brp_vertex **)((char *)&workspace.v0 + sort_table_0[_EBX]);
    // edx_           = *(brp_vertex **)((char *)&workspace.v0 + sort_table_2[_EBX]);
    // ebx_           = *(brp_vertex **)((char *)&workspace.v0 + sort_table_1[_EBX]);
    // workspace.flip = old_flip;
    // ebp_           = (eax_->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - eax_->comp_x[6]) >> 23);
    // esi_           = (ebx_->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - ebx_->comp_x[6]) >> 23);
    // edi_           = (edx_->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - edx_->comp_x[6]) >> 23);
    // if(ebp_ == esi_) {
    //     if(ebp_ == edi_) {
    //         workspace.topCount    = -1;
    //         workspace.bottomCount = -1;
    //         return;
    //     }
    //     v16                   = dy1 * half_area;
    //     v48                   = dy2;
    //     bottom_minus_top_y    = fp_one;
    //     v19                   = v48 * half_area;
    //     v20                   = dx1;
    //     middle_minus_bottom_y = edx_->comp[6] - ebx_->comp[6];
    // } else if(esi_ == edi_) {
    //     v16                   = dy1 * half_area;
    //     v49                   = dy2;
    //     bottom_minus_top_y    = ebx_->comp[6] - eax_->comp[6];
    //     v19                   = v49 * half_area;
    //     v20                   = dx1;
    //     middle_minus_bottom_y = fp_one;
    // } else {
    //     v16                   = dy1 * half_area;
    //     v17                   = dy2;
    //     bottom_minus_top_y    = ebx_->comp[6] - eax_->comp[6];
    //     v19                   = v17 * half_area;
    //     v20                   = dx1;
    //     middle_minus_bottom_y = edx_->comp[6] - ebx_->comp[6];
    // }
    // v22                        = dx2;
    // middle_minus_top_y         = edx_->comp[6] - eax_->comp[6];
    // *(float *)&workspace.iarea = half_area;
    // *(float *)&workspace.dx1_a = v20 * half_area;
    // *(float *)&workspace.dy2_a = v19;
    // *(float *)&workspace.dx2_a = v22 * half_area;
    // *(float *)&workspace.dy1_a = v16;
    // bottom_minus_top_x         = ebx_->comp[5] - eax_->comp[5];
    // v25                        = middle_minus_top_y * middle_minus_bottom_y * bottom_minus_top_y;
    // temp                       = v25;
    // if((LOWORD(temp) & 0x7FFF) == 0)
    //     divzero();
    // v26                          = bottom_minus_top_x;
    // middle_minus_bottom_x        = edx_->comp[5] - ebx_->comp[5];
    // v28                          = fp_one / v25;
    // workspace.t_y                = ebp_ + 1;
    // workspace.topCount           = esi_ - (ebp_ + 1);
    // workspace.t_dy               = esi_ + 1;
    // workspace.bottomCount        = edi_ - (esi_ + 1);
    // workspace.flip               = (workspace.iarea ^ workspace.flip) >> 31;
    // v29                          = middle_minus_top_y * middle_minus_bottom_y * v26 * v28;
    // middle_minus_top_x           = edx_->comp[5] - eax_->comp[5];
    // v31                          = bottom_minus_top_y * v28;
    // v32                          = middle_minus_top_y * middle_minus_bottom_x * v31;
    // v33                          = (double)(int)(ebp_ + 1) - eax_->comp[6];
    // v34                          = ((double)(int)(esi_ + 1) - ebx_->comp[6]) * v32;
    // *(float *)&workspace.t_dy    = v33;
    // v35                          = v33 * v29 + eax_->comp[5];
    // v36                          = middle_minus_bottom_y * middle_minus_top_x * v31;
    // v37                          = v33 * v36 + eax_->comp[5];
    // v38                          = v36 + fp_conv_d16;
    // v39                          = v32;
    // v40                          = v34 + ebx_->comp[5] + *(&fconv_d16_12 + workspace.flip);
    // *(double *)&workspace.x1     = v29 + fp_conv_d16;
    // *(double *)&workspace.xm     = v38;
    // *(double *)&workspace.x2     = v39 + fp_conv_d16;
    // v41                          = v37;
    // v42                          = v35 + *(&fconv_d16_12 + workspace.flip);
    // xm                           = workspace.xm;
    // x1                           = workspace.x1;
    // x2                           = workspace.x2;
    // *(double *)&workspace.xm     = v41 + *(&fconv_d16_m + workspace.flip);
    // *(double *)&workspace.x1     = v42;
    // workspace.d_xm               = xm;
    // workspace.d_x1               = x1;
    // workspace.t_dx               = (int)workspace.xm >> 16;
    // workspace.xstep_0            = xm >> 16;
    // workspace.xstep_1            = (xm >> 16) + 1;
    // v46                          = v40;
    // v47                          = (double)((int)workspace.xm >> 16) - eax_->comp[5];
    // *(double *)&workspace.x2     = v46;
    // *(float *)&workspace.t_dx    = v47;
    // workspace.d_x2               = x2;
    // *(float *)&workspace.xstep_0 = (float)(xm >> 16);
    // *(float *)&workspace.xstep_1 = (float)(int)workspace.xstep_1; // end of SETUP_FLOAT
    // v50                          = workspace.v2->comp[7] - workspace.v0->comp[7];
    // v51                          = workspace.v1->comp[7] - workspace.v0->comp[7];
    // v52                          = v51 * *(float *)&workspace.dy2_a - v50 * *(float *)&workspace.dy1_a;
    // v53                          = v52 * *(float *)&workspace.xstep_1;
    // v54                          = v52;
    // v55                          = v50 * *(float *)&workspace.dx1_a - v51 * *(float *)&workspace.dx2_a;
    // v56                          = v52 * *(float *)&workspace.xstep_0 + v55;
    // v57                          = *(float *)&workspace.t_dx * v52 + eax_->comp[7] + *(float *)&workspace.t_dy * v55;
    // *(double *)&workspace.s_z    = v53 + v55 + fp_conv_d16;
    // *(double *)&workspace.d_z_x  = v56 + fp_conv_d16;
    // s_z                          = workspace.s_z;
    // d_z_x                        = workspace.d_z_x;
    // *(double *)&workspace.s_z    = v57 + fp_conv_d16;
    // *(double *)&workspace.d_z_x  = v54 + fp_conv_d16;
    // workspace.d_z_y_1            = s_z;
    // workspace.d_z_y_0            = d_z_x;
    // workspace.s_z ^= 0x80000000;
    // v60                             = workspace.v2->comp[8] - workspace.v0->comp[8];
    // v61                             = workspace.v1->comp[8] - workspace.v0->comp[8];
    // v62                             = v61 * *(float *)&workspace.dy2_a - v60 * *(float *)&workspace.dy1_a;
    // v63                             = v62 * *(float *)&workspace.xstep_1;
    // v64                             = v62;
    // v65                             = v60 * *(float *)&workspace.dx1_a - v61 * *(float *)&workspace.dx2_a;
    // v66                             = v62 * *(float *)&workspace.xstep_0 + v65;
    // v67                             = *(float *)&workspace.t_dx * v62 + eax_->comp[8] + *(float *)&workspace.t_dy * v65;
    // *(double *)&workspace.s_u       = v63 + v65 + fp_conv_d24;
    // *(double *)&workspace.d_u_x     = v66 + fp_conv_d24;
    // s_u                             = workspace.s_u;
    // d_u_x                           = workspace.d_u_x;
    // *(double *)&workspace.s_u       = v67 + fp_conv_d24;
    // *(double *)&workspace.d_u_x     = v64 + fp_conv_d24;
    // workspace.d_u_y_1               = s_u;
    // workspace.d_u_y_0               = d_u_x;
    // v70                             = workspace.v2->comp[9] - workspace.v0->comp[9];
    // v71                             = workspace.v1->comp[9] - workspace.v0->comp[9];
    // v72                             = v71 * *(float *)&workspace.dy2_a - v70 * *(float *)&workspace.dy1_a;
    // v73                             = v72 * *(float *)&workspace.xstep_1;
    // v74                             = v72;
    // v75                             = v70 * *(float *)&workspace.dx1_a - v71 * *(float *)&workspace.dx2_a;
    // v76                             = v72 * *(float *)&workspace.xstep_0 + v75;
    // v77                             = *(float *)&workspace.t_dx * v72 + eax_->comp[9] + *(float *)&workspace.t_dy * v75;
    // *(double *)&workspace.s_v       = v73 + v75 + fp_conv_d24;
    // *(double *)&workspace.d_v_x     = v76 + fp_conv_d24;
    // s_v                             = workspace.s_v;
    // d_v_x                           = workspace.d_v_x;
    // *(double *)&workspace.s_v       = v77 + fp_conv_d24;
    // *(double *)&workspace.d_v_x     = v74 + fp_conv_d24;
    // workspace.d_v_y_1               = s_v;
    // workspace.d_v_y_0               = d_v_x;
    // v80                             = 2;
    // v81                             = workspace.v2->comp[8] + fp_conv_d16;
    // v82                             = workspace.v0->comp[8] + fp_conv_d16;
    // v83                             = workspace.v1->comp[8] + fp_conv_d16;
    // v84                             = workspace.v0->comp[9] + fp_conv_d16;
    // v85                             = workspace.v1->comp[9] + fp_conv_d16;
    // *(double *)&workspace.scratch1  = workspace.v2->comp[9] + fp_conv_d16;
    // *(double *)&workspace.scratch3  = v84;
    // *(double *)&workspace.scratch5  = v85;
    // *(double *)&workspace.scratch7  = v83;
    // *(double *)&workspace.scratch9  = v82;
    // *(double *)&workspace.scratch11 = v81;
    // if((workspace.scratch7 & 0xFFFF0000) == (workspace.scratch9 & 0xFFFF0000) &&
    //    (workspace.scratch7 & 0xFFFF0000) == (workspace.scratch11 & 0xFFFF0000) &&
    //    (workspace.scratch1 & 0xFFFF0000) == (workspace.scratch3 & 0xFFFF0000) &&
    //    (workspace.scratch1 & 0xFFFF0000) == (workspace.scratch5 & 0xFFFF0000)) {
    //     v80 = 0;
    // }
    // workspaceA.flags = v80 | workspace.flip;
    // workspace.s_u &= 0xFFFFFFu;
    // workspace.s_v &= 0xFFFFFFu;
    // v86 = -16777216;
    // v87 = workspace.d_u_x;
    // if((workspace.d_u_x & 0x80000000) == 0) {
    //     v86 = 0xFFFFFF;
    //     v87 = workspace.d_u_x & 0xFFFFFF;
    // }
    // workspace.d_u_x = v86 & 0xFF000000 | v87;
    // v88             = -16777216;
    // d_u_y_0         = workspace.d_u_y_0;
    // if((workspace.d_u_y_0 & 0x80000000) == 0) {
    //     v88     = 0xFFFFFF;
    //     d_u_y_0 = workspace.d_u_y_0 & 0xFFFFFF;
    // }
    // workspace.d_u_y_0 = v88 & 0xFF000000 | d_u_y_0;
    // v90               = -16777216;
    // d_u_y_1           = workspace.d_u_y_1;
    // if((workspace.d_u_y_1 & 0x80000000) == 0) {
    //     v90     = 0xFFFFFF;
    //     d_u_y_1 = workspace.d_u_y_1 & 0xFFFFFF;
    // }
    // workspace.d_u_y_1 = v90 & 0xFF000000 | d_u_y_1;
    // v92               = -16777216;
    // v93               = workspace.d_v_x;
    // if((workspace.d_v_x & 0x80000000) == 0) {
    //     v92 = 0xFFFFFF;
    //     v93 = workspace.d_v_x & 0xFFFFFF;
    // }
    // workspace.d_v_x = v92 & 0xFF000000 | v93;
    // v94             = -16777216;
    // d_v_y_0         = workspace.d_v_y_0;
    // if((workspace.d_v_y_0 & 0x80000000) == 0) {
    //     v94     = 0xFFFFFF;
    //     d_v_y_0 = workspace.d_v_y_0 & 0xFFFFFF;
    // }
    // workspace.d_v_y_0 = v94 & 0xFF000000 | d_v_y_0;
    // v96               = -16777216;
    // d_v_y_1           = workspace.d_v_y_1;
    // if((workspace.d_v_y_1 & 0x80000000) == 0) {
    //     v96     = 0xFFFFFF;
    //     d_v_y_1 = workspace.d_v_y_1 & 0xFFFFFF;
    // }
    // workspace.d_v_y_1         = v96 & 0xFF000000 | d_v_y_1;
    // width_p                   = (double)(int)work.texture.width_p;
    // workspaceA.su             = (double)(int)workspace.s_u * width_p + fp_conv_d8r;
    // workspaceA.dux            = (double)(int)workspace.d_u_x * width_p + fp_conv_d8r;
    // workspaceA.duy1           = (double)(int)workspace.d_u_y_1 * width_p + fp_conv_d8r;
    // workspaceA.duy0           = (double)(int)workspace.d_u_y_0 * width_p + fp_conv_d8r;
    // height                    = (double)(int)work.texture.height;
    // workspaceA.sv             = (double)(int)workspace.s_v * height + fp_conv_d8r;
    // workspaceA.dvx            = (double)(int)workspace.d_v_x * height + fp_conv_d8r;
    // workspaceA.dvy1           = (double)(int)workspace.d_v_y_1 * height + fp_conv_d8r;
    // workspaceA.dvy0           = (double)(int)workspace.d_v_y_0 * height + fp_conv_d8r;
    // LODWORD(workspaceA.svf)   = LODWORD(workspaceA.sv) << 16;
    // LODWORD(workspaceA.dvxf)  = LODWORD(workspaceA.dvx) << 16;
    // LODWORD(workspaceA.dvy0f) = LODWORD(workspaceA.dvy0) << 16;
    // LODWORD(workspaceA.dvy1f) = LODWORD(workspaceA.dvy1) << 16;
    // SLODWORD(workspaceA.sv) >>= 16;
    // SLODWORD(workspaceA.dvx) >>= 16;
    // SLODWORD(workspaceA.dvy0) >>= 16;
    // SLODWORD(workspaceA.dvy1) >>= 16;
    // stride_b                        = (double)work.texture.stride_b;
    // workspaceA.sv                   = (double)SLODWORD(workspaceA.sv) * stride_b + fp_conv_d;
    // workspaceA.dvx                  = (double)SLODWORD(workspaceA.dvx) * stride_b + fp_conv_d;
    // workspaceA.dvy1                 = (double)SLODWORD(workspaceA.dvy1) * stride_b + fp_conv_d;
    // workspaceA.dvy0                 = (double)SLODWORD(workspaceA.dvy0) * stride_b + fp_conv_d;
    // LODWORD(workspaceA.dvy0c)       = work.texture.stride_b + LODWORD(workspaceA.dvy0);
    // LODWORD(workspaceA.dvy1c)       = work.texture.stride_b + LODWORD(workspaceA.dvy1);
    // LODWORD(workspaceA.dvxc)        = work.texture.stride_b + LODWORD(workspaceA.dvx);
    // LODWORD(workspaceA.uUpperBound) = work.texture.width_p << 16;
    // LODWORD(workspaceA.vUpperBound) = (char *)work.texture.base + work.texture.size;

    /*
        old_flip       = flip_table[_EBX];
        vt             = *(brp_vertex **)((char *)&workspace.v0 + sort_table_0[_EBX]);
        vm             = *(brp_vertex **)((char *)&workspace.v0 + sort_table_2[_EBX]);
        vb             = *(brp_vertex **)((char *)&workspace.v0 + sort_table_1[_EBX]);
        workspace.flip = old_flip;
        y_t            = (int)vt->comp_f[C_SY];
        y_m            = (vb->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - vb->comp_x[6]) >> 23);
        y_b            = (vm->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - vm->comp_x[6]) >> 23);
        if(y_t == y_m) {
            if(y_t == y_b) {
                workspace.topCount    = -1;
                workspace.bottomCount = -1;
                return;
            }
            v16                   = dy1 * half_area;
            v48                   = dy2;
            bottom_minus_top_y    = fp_one;
            v19                   = v48 * half_area;
            v20                   = dx1;
            middle_minus_bottom_y = vm->comp[6] - vb->comp[6];
        } else if(y_m == y_b) {
            v16                   = dy1 * half_area;
            v49                   = dy2;
            bottom_minus_top_y    = vb->comp[6] - vt->comp[6];
            v19                   = v49 * half_area;
            v20                   = dx1;
            middle_minus_bottom_y = fp_one;
        } else {
            v16                   = dy1 * half_area;
            v17                   = dy2;
            bottom_minus_top_y    = vb->comp[6] - vt->comp[6];
            v19                   = v17 * half_area;
            v20                   = dx1;
            middle_minus_bottom_y = vm->comp[6] - vb->comp[6];
        }
        v22                = dx2;
        middle_minus_top_y = vm->comp[6] - vt->comp[6];
        workspace.iarea    = half_area;
        workspace.dx1_a    = v20 * half_area;
        workspace.dy2_a    = v19;
        workspace.dx2_a    = v22 * half_area;
        workspace.dy1_a    = v16;
        bottom_minus_top_x = vb->comp[5] - vt->comp[5];
        v25                = middle_minus_top_y * middle_minus_bottom_y * bottom_minus_top_y;
        temp               = v25;
        if((LOWORD(temp) & 0x7FFF) == 0)
            divzero();
        v26                        = bottom_minus_top_x;
        middle_minus_bottom_x      = vm->comp[5] - vb->comp[5];
        v28                        = fp_one / v25;
        workspace.t_y              = y_t + 1;
        workspace.topCount         = y_m - (y_t + 1);
        LODWORD(workspace.t_dy)    = y_m + 1;
        workspace.bottomCount      = y_b - (y_m + 1);
        workspace.flip             = (LODWORD(workspace.iarea) ^ workspace.flip) >> 31;
        v29                        = middle_minus_top_y * middle_minus_bottom_y * v26 * v28;
        middle_minus_top_x         = vm->comp[5] - vt->comp[5];
        v31                        = bottom_minus_top_y * v28;
        v32                        = middle_minus_top_y * middle_minus_bottom_x * v31;
        v33                        = (double)(int)(y_t + 1) - vt->comp[6];
        v34                        = ((double)(int)(y_m + 1) - vb->comp[6]) * v32;
        workspace.t_dy             = v33;
        v35                        = v33 * v29 + vt->comp[5];
        v36                        = middle_minus_bottom_y * middle_minus_top_x * v31;
        v37                        = v33 * v36 + vt->comp[5];
        v38                        = v36 + fp_conv_d16;
        v39                        = v32;
        v40                        = v34 + vb->comp[5] + *(&fconv_d16_12 + workspace.flip);
        *(double *)&workspace.x1   = v29 + fp_conv_d16;
        *(double *)&workspace.xm   = v38;
        *(double *)&workspace.x2   = v39 + fp_conv_d16;
        v41                        = v37;
        v42                        = v35 + *(&fconv_d16_12 + workspace.flip);
        xm                         = workspace.xm;
        x1                         = workspace.x1;
        x2                         = workspace.x2;
        *(double *)&workspace.xm   = v41 + *(&fconv_d16_m + workspace.flip);
        *(double *)&workspace.x1   = v42;
        workspace.d_xm             = xm;
        workspace.d_x1             = x1;
        LODWORD(workspace.t_dx)    = SLODWORD(workspace.xm) >> 16;
        LODWORD(workspace.xstep_0) = SLODWORD(xm) >> 16;
        LODWORD(workspace.xstep_1) = (SLODWORD(xm) >> 16) + 1;
        v46                        = v40;
        v47                        = (double)(SLODWORD(workspace.xm) >> 16) - vt->comp[5];
        *(double *)&workspace.x2   = v46;
        workspace.t_dx             = v47;
        workspace.d_x2             = x2;
        workspace.xstep_0          = (float)(SLODWORD(xm) >> 16);
        workspace.xstep_1          = (float)SLODWORD(workspace.xstep_1);

        double   dx1;       // st7
        double   dx2;       // st6
        double   dy1;       // st5
        double   dy2;       // st4
        double   half_area; // st3
        uint32_t old_flip;  // esi
        // todo brp_vertex *vt;         // eax
        brp_vertex *vb;         // edx
        brp_vertex *vm;         // ebx
        int         y_t;        // ebp
        int         y_m;        // esi
        int         y_b;        // edi
        double      var_st4;    // st4
        double      var_st2222; // st2
        double      var_st5;    // st5
        double      var_st22;   // st2
        double      var_st1;    // st1
        double      var_st6;    // st6
        double      v22;        // st0
        double      v23;        // st7
        double      v24;        // st3
        double      v25;        // st2
        double      v26;        // st1
        double      v27;        // st3
        double      v28;        // st2
        double      gm;         // rt2
        double      v30;        // st1
        double      v31;        // st2
        double      v32;        // st7
        double      t_dy;       // st3
        double      m_dy;       // st4
        double      x_major;    // rt1
        double      g1;         // st2
        double      x_minor1;   // st6
        double      v38;        // st3
        double      v39;        // st2
        double      x_minor2;   // st7
        double      v41;        // st5
        double      v42;        // st6
        float       xm;         // edx
        float       x1;         // esi
        float       x2;         // edi
        double      v46;        // st6
        double      v47;        // st7
        double      var_st2;    // st2
        double      var_st222;  // st2
        double      dp1;        // st7
        double      dp2;        // st6
        double      pdy;        // rt0
        double      pdx;        // st1
        double      v3;         // st6
        float       s_z;        // esi
        float       d_z_x;      // edi

}
*/

#define SETUP_FLOAT_PARAM(comp, param, s_p, d_p_x, conv, unsigned)                                 \
    {                                                                                              \
        dp2               = workspace.v2->comp_f[comp] - workspace.v0->comp_f[comp];               \
        dp1               = workspace.v1->comp_f[comp] - workspace.v0->comp_f[comp];               \
        pdx               = dp1 * workspace.dy2_a - dp2 * workspace.dy1_a;                         \
        pdy               = dp2 * workspace.dx1_a - dp1 * workspace.dx2_a;                         \
        pstart            = workspace.t_dx * pdx + vt->comp_f[comp] + workspace.t_dy * pdy + conv; \
        *(double *)&s_p   = pdx * workspace.xstep_1 + pdy + conv;                                  \
        *(double *)&d_p_x = pdx * workspace.xstep_0 + pdy + conv;                                  \
        s_p_tmp           = s_p;                                                                   \
        d_p_x_tmp         = d_p_x;                                                                 \
        *(double *)&s_p   = pstart;                                                                \
        printf("hi\n");                                                                            \
        *(double *)&d_p_x        = pdx + conv;                                                     \
        *(((float *)&s_p) + 1)   = s_p_tmp;                                                        \
        *(((float *)&d_p_x) + 1) = d_p_x_tmp;                                                      \
        if(unsigned) {                                                                             \
            LODWORD(s_p) ^= 0x80000000;                                                            \
        }                                                                                          \
    }

// void TriangleSetup_ZT_ARBITRARY(brp_vertex *v0, brp_vertex *v1, brp_vertex *v2)
// {

//     double dp1;
//     double dp2;
//     double pdy;
//     double pdx;
//     double pstart;
//     float  s_p_tmp;
//     float  d_p_x_tmp;

//     // SETUP_FLOAT(v0, v1, v2, vt);

//     // SETUP_FLOAT_PARAM(C_SZ, _z, workspace.s_z, workspace.d_z_x, fp_conv_d16, 1);
//     // SETUP_FLOAT_PARAM(C_U, _u, workspace.s_u, workspace.d_u_x, fp_conv_d24, 0);
//     // SETUP_FLOAT_PARAM(C_V, _v, workspace.s_v, workspace.d_v_x, fp_conv_d24, 0);

//     double       dx2;                   // st7
//     double       dx1;                   // st6
//     double       dy2;                   // st5
//     double       dy1;                   // st4
//     int          _EBX;                  // ebx
//     double       half_area;             // st3
//     uint32_t     old_flip;              // esi
//     brp_vertex  *vt;                    // eax
//     brp_vertex  *vm;                    // edx
//     brp_vertex  *vb;                    // ebx
//     unsigned int y_t;                   // ebp
//     unsigned int y_m;                   // esi
//     unsigned int y_b;                   // edi
//     double       v16;                   // st4
//     double       v17;                   // st2
//     double       bottom_minus_top_y;    // st5
//     double       v19;                   // st2
//     double       v20;                   // st1
//     double       middle_minus_bottom_y; // st6
//     double       v22;                   // st0
//     double       middle_minus_top_y;    // st7
//     double       bottom_minus_top_x;    // st3
//     double       v25;                   // st2
//     double       v26;                   // st1
//     double       middle_minus_bottom_x; // st3
//     double       v28;                   // st2
//     double       v29;                   // rt2
//     double       middle_minus_top_x;    // st1
//     double       v31;                   // st2
//     double       v32;                   // st7
//     double       v33;                   // st3
//     double       v34;                   // st4
//     double       v35;                   // rt1
//     double       v36;                   // st2
//     double       v37;                   // st6
//     double       v38;                   // st3
//     double       v39;                   // st2
//     double       v40;                   // st7
//     double       v41;                   // st5
//     double       v42;                   // st6
//     float        xm;                    // edx
//     float        x1;                    // esi
//     float        x2;                    // edi
//     double       v46;                   // st6
//     double       v47;                   // st7
//     double       v48;                   // st2
//     double       v49;                   // st2
//     double       v50;                   // st7
//     double       v51;                   // st6
//     double       v52;                   // st4
//     double       v53;                   // st2
//     double       v54;                   // rt0
//     double       v55;                   // st1
//     double       v56;                   // st3
//     double       v57;                   // st4
//     float        s_z;                   // esi
//     float        d_z_x;                 // edi
//     double       v60;                   // st7
//     double       v61;                   // st6
//     double       v62;                   // st4
//     double       v63;                   // st2
//     double       v64;                   // rt1
//     double       v65;                   // st1
//     double       v66;                   // st3
//     double       v67;                   // st4
//     uint32_t     s_u;                   // esi
//     uint32_t     d_u_x;                 // edi
//     double       v70;                   // st7
//     double       v71;                   // st6
//     double       v72;                   // st4
//     double       v73;                   // st2
//     double       v74;                   // rt2
//     double       v75;                   // st1
//     double       v76;                   // st3
//     double       v77;                   // st4
//     uint32_t     s_v;                   // esi
//     uint32_t     d_v_x;                 // edi
//     int          v80;                   // esi
//     double       v81;                   // st7
//     double       v82;                   // st6
//     double       v83;                   // st5
//     double       v84;                   // st3
//     double       v85;                   // st4
//     int          v86;                   // ebp
//     uint32_t     v87;                   // eax
//     int          v88;                   // ebp
//     uint32_t     d_u_y_0;               // eax
//     int          v90;                   // ebp
//     uint32_t     d_u_y_1;               // eax
//     int          v92;                   // ebp
//     uint32_t     v93;                   // eax
//     int          v94;                   // ebp
//     uint32_t     d_v_y_0;               // eax
//     int          v96;                   // ebp
//     uint32_t     d_v_y_1;               // eax
//     double       width_p;               // st7
//     double       height;                // st7
//     double       stride_b;              // st3

//     brp_vertex *eax_;
//     brp_vertex *edx_;
//     brp_vertex *ebx_;

//     unsigned int ebp_, esi_, edi_;

//     dx2  = v2->comp_f[C_SX] - v0->comp_f[C_SX];
//     dx1  = v1->comp_f[C_SX] - v0->comp_f[C_SX];
//     dy2  = v2->comp_f[C_SY] - v0->comp_f[C_SY];
//     dy1  = v1->comp_f[C_SX] - v0->comp_f[C_SX];
//     _EBX = 0;
//     // cmp ecx, eax
//     // __asm { rcl     ebx, 1 }
//     if(v1->comp_f[C_SY] < v0->comp_f[C_SY]) {
//         _EBX |= 1;
//     }
//     half_area = fp_one / (dx1 * dy2 - dx2 * dy1);
//     // cmp edx, eax
//     if(v2->comp_f[C_SY] < v0->comp_f[C_SY]) {
//         _EBX = _EBX << 1;
//         _EBX |= 1;
//     }
//     // cmp edx, ecx
//     if(v2->comp_f[C_SY] < v1->comp_f[C_SY]) {
//         _EBX = _EBX << 1;
//         _EBX |= 1;
//     }

//     old_flip       = flip_table[_EBX];
//     eax_           = *(brp_vertex **)((char *)&workspace.v0 + sort_table_0[_EBX]);
//     edx_           = *(brp_vertex **)((char *)&workspace.v0 + sort_table_2[_EBX]);
//     ebx_           = *(brp_vertex **)((char *)&workspace.v0 + sort_table_1[_EBX]);
//     workspace.flip = old_flip;
//     ebp_           = (eax_->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - eax_->comp_x[6]) >> 23);
//     esi_           = (ebx_->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - ebx_->comp_x[6]) >> 23);
//     edi_           = (edx_->comp_x[6] & 0x7FFFFF | 0x800000u) >> ((unsigned int)(1266679807 - edx_->comp_x[6]) >> 23);
//     if(ebp_ == esi_) {
//         if(ebp_ == edi_) {
//             workspace.topCount    = -1;
//             workspace.bottomCount = -1;
//             return;
//         }
//         v16                   = dy1 * half_area;
//         v48                   = dy2;
//         bottom_minus_top_y    = fp_one;
//         v19                   = v48 * half_area;
//         v20                   = dx1;
//         middle_minus_bottom_y = edx_->comp[6] - ebx_->comp[6];
//     } else if(esi_ == edi_) {
//         v16                   = dy1 * half_area;
//         v49                   = dy2;
//         bottom_minus_top_y    = ebx_->comp[6] - eax_->comp[6];
//         v19                   = v49 * half_area;
//         v20                   = dx1;
//         middle_minus_bottom_y = fp_one;
//     } else {
//         v16                   = dy1 * half_area;
//         v17                   = dy2;
//         bottom_minus_top_y    = ebx_->comp[6] - eax_->comp[6];
//         v19                   = v17 * half_area;
//         v20                   = dx1;
//         middle_minus_bottom_y = edx_->comp[6] - ebx_->comp[6];
//     }
//     v22                        = dx2;
//     middle_minus_top_y         = edx_->comp[6] - eax_->comp[6];
//     *(float *)&workspace.iarea = half_area;
//     *(float *)&workspace.dx1_a = v20 * half_area;
//     *(float *)&workspace.dy2_a = v19;
//     *(float *)&workspace.dx2_a = v22 * half_area;
//     *(float *)&workspace.dy1_a = v16;
//     bottom_minus_top_x         = ebx_->comp[5] - eax_->comp[5];
//     v25                        = middle_minus_top_y * middle_minus_bottom_y * bottom_minus_top_y;
//     temp                       = v25;
//     if((LOWORD(temp) & 0x7FFF) == 0)
//         divzero();
//     v26                          = bottom_minus_top_x;
//     middle_minus_bottom_x        = edx_->comp[5] - ebx_->comp[5];
//     v28                          = fp_one / v25;
//     workspace.t_y                = ebp_ + 1;
//     workspace.topCount           = esi_ - (ebp_ + 1);
//     workspace.t_dy               = esi_ + 1;
//     workspace.bottomCount        = edi_ - (esi_ + 1);
//     workspace.flip               = (workspace.iarea ^ workspace.flip) >> 31;
//     v29                          = middle_minus_top_y * middle_minus_bottom_y * v26 * v28;
//     middle_minus_top_x           = edx_->comp[5] - eax_->comp[5];
//     v31                          = bottom_minus_top_y * v28;
//     v32                          = middle_minus_top_y * middle_minus_bottom_x * v31;
//     v33                          = (double)(int)(ebp_ + 1) - eax_->comp[6];
//     v34                          = ((double)(int)(esi_ + 1) - ebx_->comp[6]) * v32;
//     *(float *)&workspace.t_dy    = v33;
//     v35                          = v33 * v29 + eax_->comp[5];
//     v36                          = middle_minus_bottom_y * middle_minus_top_x * v31;
//     v37                          = v33 * v36 + eax_->comp[5];
//     v38                          = v36 + fp_conv_d16;
//     v39                          = v32;
//     v40                          = v34 + ebx_->comp[5] + *(&fconv_d16_12 + workspace.flip);
//     *(double *)&workspace.x1     = v29 + fp_conv_d16;
//     *(double *)&workspace.xm     = v38;
//     *(double *)&workspace.x2     = v39 + fp_conv_d16;
//     v41                          = v37;
//     v42                          = v35 + *(&fconv_d16_12 + workspace.flip);
//     xm                           = workspace.xm;
//     x1                           = workspace.x1;
//     x2                           = workspace.x2;
//     *(double *)&workspace.xm     = v41 + *(&fconv_d16_m + workspace.flip);
//     *(double *)&workspace.x1     = v42;
//     workspace.d_xm               = xm;
//     workspace.d_x1               = x1;
//     workspace.t_dx               = (int)workspace.xm >> 16;
//     workspace.xstep_0            = (int)xm >> 16;
//     workspace.xstep_1            = ((int)xm >> 16) + 1;
//     v46                          = v40;
//     v47                          = (double)((int)workspace.xm >> 16) - eax_->comp[5];
//     *(double *)&workspace.x2     = v46;
//     *(float *)&workspace.t_dx    = v47;
//     workspace.d_x2               = x2;
//     *(float *)&workspace.xstep_0 = (float)((int)xm >> 16);
//     *(float *)&workspace.xstep_1 = (float)(int)workspace.xstep_1; // end of SETUP_FLOAT
//     v50                          = workspace.v2->comp[7] - workspace.v0->comp[7];
//     v51                          = workspace.v1->comp[7] - workspace.v0->comp[7];
//     v52                          = v51 * *(float *)&workspace.dy2_a - v50 * *(float *)&workspace.dy1_a;
//     v53                          = v52 * *(float *)&workspace.xstep_1;
//     v54                          = v52;
//     v55                          = v50 * *(float *)&workspace.dx1_a - v51 * *(float *)&workspace.dx2_a;
//     v56                          = v52 * *(float *)&workspace.xstep_0 + v55;
//     v57                          = *(float *)&workspace.t_dx * v52 + eax_->comp[7] + *(float *)&workspace.t_dy * v55;
//     *(double *)&workspace.s_z    = v53 + v55 + fp_conv_d16;
//     *(double *)&workspace.d_z_x  = v56 + fp_conv_d16;
//     s_z                          = workspace.s_z;
//     d_z_x                        = workspace.d_z_x;
//     *(double *)&workspace.s_z    = v57 + fp_conv_d16;
//     *(double *)&workspace.d_z_x  = v54 + fp_conv_d16;
//     workspace.d_z_y_1            = s_z;
//     workspace.d_z_y_0            = d_z_x;
//     workspace.s_z ^= 0x80000000;
//     v60                             = workspace.v2->comp[8] - workspace.v0->comp[8];
//     v61                             = workspace.v1->comp[8] - workspace.v0->comp[8];
//     v62                             = v61 * *(float *)&workspace.dy2_a - v60 * *(float *)&workspace.dy1_a;
//     v63                             = v62 * *(float *)&workspace.xstep_1;
//     v64                             = v62;
//     v65                             = v60 * *(float *)&workspace.dx1_a - v61 * *(float *)&workspace.dx2_a;
//     v66                             = v62 * *(float *)&workspace.xstep_0 + v65;
//     v67                             = *(float *)&workspace.t_dx * v62 + eax_->comp[8] + *(float *)&workspace.t_dy * v65;
//     *(double *)&workspace.s_u       = v63 + v65 + fp_conv_d24;
//     *(double *)&workspace.d_u_x     = v66 + fp_conv_d24;
//     s_u                             = workspace.s_u;
//     d_u_x                           = workspace.d_u_x;
//     *(double *)&workspace.s_u       = v67 + fp_conv_d24;
//     *(double *)&workspace.d_u_x     = v64 + fp_conv_d24;
//     workspace.d_u_y_1               = s_u;
//     workspace.d_u_y_0               = d_u_x;
//     v70                             = workspace.v2->comp[9] - workspace.v0->comp[9];
//     v71                             = workspace.v1->comp[9] - workspace.v0->comp[9];
//     v72                             = v71 * *(float *)&workspace.dy2_a - v70 * *(float *)&workspace.dy1_a;
//     v73                             = v72 * *(float *)&workspace.xstep_1;
//     v74                             = v72;
//     v75                             = v70 * *(float *)&workspace.dx1_a - v71 * *(float *)&workspace.dx2_a;
//     v76                             = v72 * *(float *)&workspace.xstep_0 + v75;
//     v77                             = *(float *)&workspace.t_dx * v72 + eax_->comp[9] + *(float *)&workspace.t_dy * v75;
//     *(double *)&workspace.s_v       = v73 + v75 + fp_conv_d24;
//     *(double *)&workspace.d_v_x     = v76 + fp_conv_d24;
//     s_v                             = workspace.s_v;
//     d_v_x                           = workspace.d_v_x;
//     *(double *)&workspace.s_v       = v77 + fp_conv_d24;
//     *(double *)&workspace.d_v_x     = v74 + fp_conv_d24;
//     workspace.d_v_y_1               = s_v;
//     workspace.d_v_y_0               = d_v_x;
//     v80                             = 2;
//     v81                             = workspace.v2->comp[8] + fp_conv_d16;
//     v82                             = workspace.v0->comp[8] + fp_conv_d16;
//     v83                             = workspace.v1->comp[8] + fp_conv_d16;
//     v84                             = workspace.v0->comp[9] + fp_conv_d16;
//     v85                             = workspace.v1->comp[9] + fp_conv_d16;
//     *(double *)&workspace.scratch1  = workspace.v2->comp[9] + fp_conv_d16;
//     *(double *)&workspace.scratch3  = v84;
//     *(double *)&workspace.scratch5  = v85;
//     *(double *)&workspace.scratch7  = v83;
//     *(double *)&workspace.scratch9  = v82;
//     *(double *)&workspace.scratch11 = v81;
//     if((workspace.scratch7 & 0xFFFF0000) == (workspace.scratch9 & 0xFFFF0000) &&
//        (workspace.scratch7 & 0xFFFF0000) == (workspace.scratch11 & 0xFFFF0000) &&
//        (workspace.scratch1 & 0xFFFF0000) == (workspace.scratch3 & 0xFFFF0000) &&
//        (workspace.scratch1 & 0xFFFF0000) == (workspace.scratch5 & 0xFFFF0000)) {
//         v80 = 0;
//     }
//     workspaceA.flags = v80 | workspace.flip;
//     workspace.s_u &= 0xFFFFFFu;
//     workspace.s_v &= 0xFFFFFFu;
//     v86 = -16777216;
//     v87 = workspace.d_u_x;
//     if((workspace.d_u_x & 0x80000000) == 0) {
//         v86 = 0xFFFFFF;
//         v87 = workspace.d_u_x & 0xFFFFFF;
//     }
//     workspace.d_u_x = v86 & 0xFF000000 | v87;
//     v88             = -16777216;
//     d_u_y_0         = workspace.d_u_y_0;
//     if((workspace.d_u_y_0 & 0x80000000) == 0) {
//         v88     = 0xFFFFFF;
//         d_u_y_0 = workspace.d_u_y_0 & 0xFFFFFF;
//     }
//     workspace.d_u_y_0 = v88 & 0xFF000000 | d_u_y_0;
//     v90               = -16777216;
//     d_u_y_1           = workspace.d_u_y_1;
//     if((workspace.d_u_y_1 & 0x80000000) == 0) {
//         v90     = 0xFFFFFF;
//         d_u_y_1 = workspace.d_u_y_1 & 0xFFFFFF;
//     }
//     workspace.d_u_y_1 = v90 & 0xFF000000 | d_u_y_1;
//     v92               = -16777216;
//     v93               = workspace.d_v_x;
//     if((workspace.d_v_x & 0x80000000) == 0) {
//         v92 = 0xFFFFFF;
//         v93 = workspace.d_v_x & 0xFFFFFF;
//     }
//     workspace.d_v_x = v92 & 0xFF000000 | v93;
//     v94             = -16777216;
//     d_v_y_0         = workspace.d_v_y_0;
//     if((workspace.d_v_y_0 & 0x80000000) == 0) {
//         v94     = 0xFFFFFF;
//         d_v_y_0 = workspace.d_v_y_0 & 0xFFFFFF;
//     }
//     workspace.d_v_y_0 = v94 & 0xFF000000 | d_v_y_0;
//     v96               = -16777216;
//     d_v_y_1           = workspace.d_v_y_1;
//     if((workspace.d_v_y_1 & 0x80000000) == 0) {
//         v96     = 0xFFFFFF;
//         d_v_y_1 = workspace.d_v_y_1 & 0xFFFFFF;
//     }
//     workspace.d_v_y_1               = v96 & 0xFF000000 | d_v_y_1;
//     width_p                         = (double)(int)work.texture.width_p;
//     *(double *)&workspaceA.su       = (double)(int)workspace.s_u * width_p + fp_conv_d8r;
//     *(double *)&workspaceA.dux      = (double)(int)workspace.d_u_x * width_p + fp_conv_d8r;
//     *(double *)&workspaceA.duy1     = (double)(int)workspace.d_u_y_1 * width_p + fp_conv_d8r;
//     *(double *)&workspaceA.duy0     = (double)(int)workspace.d_u_y_0 * width_p + fp_conv_d8r;
//     height                          = (double)(int)work.texture.height;
//     *(double *)&workspaceA.sv_setup = (double)(int)workspace.s_v * height + fp_conv_d8r;
//     *(double *)&workspaceA.dvx      = (double)(int)workspace.d_v_x * height + fp_conv_d8r;
//     *(double *)&workspaceA.dvy1     = (double)(int)workspace.d_v_y_1 * height + fp_conv_d8r;
//     *(double *)&workspaceA.dvy0     = (double)(int)workspace.d_v_y_0 * height + fp_conv_d8r;
//     workspaceA.svf                  = workspaceA.sv_setup << 16;
//     workspaceA.dvxf                 = workspaceA.dvx << 16;
//     workspaceA.dvy0f                = workspaceA.dvy0 << 16;
//     workspaceA.dvy1f                = workspaceA.dvy1 << 16;
//     workspaceA.sv_setup             = (signed int)workspaceA.sv_setup >> 16;
//     workspaceA.dvx                  = (signed int)workspaceA.dvx >> 16;
//     workspaceA.dvy0                 = (signed int)workspaceA.dvy0 >> 16;
//     workspaceA.dvy1                 = (signed int)workspaceA.dvy1 >> 16;
//     stride_b                        = (double)work.texture.stride_b;
//     *(double *)&workspaceA.sv_setup = (double)(int)workspaceA.sv_setup * stride_b + fp_conv_d;
//     *(double *)&workspaceA.dvx      = (double)(int)workspaceA.dvx * stride_b + fp_conv_d;
//     *(double *)&workspaceA.dvy1     = (double)(int)workspaceA.dvy1 * stride_b + fp_conv_d;
//     *(double *)&workspaceA.dvy0     = (double)(int)workspaceA.dvy0 * stride_b + fp_conv_d;
//     workspaceA.dvy0c                = work.texture.stride_b + workspaceA.dvy0;
//     workspaceA.dvy1c                = work.texture.stride_b + workspaceA.dvy1;
//     workspaceA.dvxc                 = work.texture.stride_b + workspaceA.dvx;
//     workspaceA.uUpperBound          = work.texture.width_p << 16;
//     workspaceA.vUpperBound          = work.texture.base + work.texture.size;
// }

// #include "priminfo.h"
// void TriangleRender_ZT_I8_D16(brp_block *block, ...)
// {
//     float flt_100240A0 = 1.0f;
//     float flt_100240A4 = 2.0f;
//     float flt_100240A8 = 6.7553994e15f;

//     double      st5_1;   // st5
//     uint32_t    x1;      // ebx
//     uint32_t    xm;      // ecx
//     signed int  v4;      // ebx
//     signed int  v5;      // ecx
//     char       *v6;      // edi
//     char        v7;      // cc
//     int         v8;      // ecx
//     _BYTE      *sv;      // esi
//     uint32_t    su_high; // eax
//     char       *v11;     // ebp
//     char        v12;     // bl
//     int         v13;     // cf  // was bool
//     signed int  i;       // eax
//     int         v15;     // edi
//     int         v17;     // ebp
//     signed int  v18;     // edx
//     char       *v19;     // eax
//     uint32_t    x2;      // ebx
//     uint32_t    v21;     // ecx
//     signed int  v22;     // ebx
//     signed int  v23;     // ecx
//     char       *v24;     // edi
//     int         v25;     // ecx
//     _BYTE      *v26;     // esi
//     uint32_t    v27;     // eax
//     char       *v28;     // ebp
//     char        v29;     // bl
//     signed int  j;       // eax
//     int         v31;     // edi
//     int         v33;     // ebp
//     signed int  v34;     // edx
//     char       *v35;     // eax
//     uint32_t    v36;     // ebx
//     uint32_t    v37;     // ecx
//     signed int  v38;     // ebx
//     signed int  v39;     // ecx
//     char       *v40;     // edi
//     int         v41;     // ecx
//     _BYTE      *v42;     // esi
//     uint32_t    v43;     // eax
//     char       *v44;     // ebp
//     char        v45;     // bl
//     int         v46;     // edx
//     signed int  k;       // eax
//     int         v48;     // edi
//     int         v50;     // ebp
//     signed int  v51;     // edx
//     char       *v52;     // eax
//     uint32_t    v53;     // ebx
//     uint32_t    v54;     // ecx
//     signed int  v55;     // ebx
//     signed int  v56;     // ecx
//     char       *v57;     // edi
//     int         v58;     // ecx
//     _BYTE      *v59;     // esi
//     uint32_t    v60;     // eax
//     char       *v61;     // ebp
//     char        v62;     // bl
//     int         v63;     // edx
//     signed int  m;       // eax
//     int         v65;     // edi
//     int         v67;     // ebp
//     signed int  v68;     // edx
//     char       *v69;     // eax
//     uint32_t    v70;     // ebx
//     uint32_t    v71;     // ecx
//     signed int  v72;     // ebx
//     signed int  v73;     // ecx
//     char       *v74;     // edi
//     int         v75;     // ecx
//     _BYTE      *v76;     // esi
//     uint32_t    v77;     // eax
//     char       *v78;     // ebp
//     char        v79;     // bl
//     signed int  n;       // eax
//     int         v81;     // edi
//     int         v83;     // ebp
//     signed int  v84;     // edx
//     char       *v85;     // eax
//     uint32_t    v86;     // ebx
//     uint32_t    v87;     // ecx
//     signed int  v88;     // ebx
//     signed int  v89;     // ecx
//     char       *v90;     // edi
//     int         v91;     // ecx
//     _BYTE      *v92;     // esi
//     uint32_t    v93;     // eax
//     char       *v94;     // ebp
//     char        v95;     // bl
//     signed int  ii;      // eax
//     int         v97;     // edi
//     int         v99;     // ebp
//     signed int  v100;    // edx
//     char       *v101;    // eax
//     uint32_t    v102;    // ebx
//     uint32_t    v103;    // ecx
//     signed int  v104;    // ebx
//     signed int  v105;    // ecx
//     char       *v106;    // edi
//     int         v107;    // ecx
//     _BYTE      *v108;    // esi
//     uint32_t    v109;    // eax
//     char       *v110;    // ebp
//     char        v111;    // bl
//     int         v112;    // edx
//     signed int  jj;      // eax
//     int         v114;    // edi
//     int         v116;    // ebp
//     signed int  v117;    // edx
//     char       *v118;    // eax
//     uint32_t    v119;    // ebx
//     uint32_t    v120;    // ecx
//     signed int  v121;    // ebx
//     signed int  v122;    // ecx
//     char       *v123;    // edi
//     int         v124;    // ecx
//     _BYTE      *v125;    // esi
//     uint32_t    v126;    // eax
//     char       *v127;    // ebp
//     char        v128;    // bl
//     int         v129;    // edx
//     signed int  kk;      // eax
//     int         v131;    // edi
//     int         v133;    // ebp
//     signed int  v134;    // edx
//     char       *v135;    // eax
//     brp_vertex *v0;      // [esp+18h] [ebp+Ch]
//     brp_vertex *v1;      // [esp+1Ch] [ebp+10h]
//     brp_vertex *v2;      // [esp+20h] [ebp+14h]
//     va_list     va;      // [esp+24h] [ebp+18h] BYREF

//     va_start(va, block);
//     v0 = va_arg(va, brp_vertex *);
//     v1 = va_arg(va, brp_vertex *);
//     v2 = va_arg(va, brp_vertex *);
//     va_end(va);

//     workspace.v0 = v0;
//     workspace.v1 = v1;
//     workspace.v2 = v2;
//     TriangleSetup_ZT_ARBITRARY(v0, v1, v2);

//     st5_1           = (double)workspace.t_y - flt_100240A0;
//     double offs     = ((double)work.colour.stride_b * st5_1 - flt_100240A0 + flt_100240A8);
//     int    x        = ((long)offs & 0xffffffff);
//     char  *scanadd1 = &work.colour.base[x];
//     char  *scanadd2 = work.colour.base + x;
//     char  *scanadd3;
//     *(double *)&scanadd3 = (double)work.colour.stride_b * st5_1 + (double)(intptr_t)work.colour.base - flt_100240A0 + flt_100240A8;

//     workspace.depthAddress = &work.depth.base[(int)(work.depth.stride_b * st5_1 - flt_100240A4 + flt_100240A8)];

//     *(double *)&workspace.scanAddress = (double)work.colour.stride_b * st5_1 + (double)(intptr_t)work.colour.base -
//                                         flt_100240A0 + flt_100240A8;
//     *(double *)&workspace.depthAddress = (double)work.depth.stride_b * st5_1 + (double)(intptr_t)work.depth.base -
//                                          flt_100240A4 + flt_100240A8;

//     // workspaceA.sv += work.texture.base;
//     workspaceA.sv    = workspaceA.sv_setup + work.texture.base;
//     workspace.xm_f   = workspace.xm << 16;
//     workspace.d_xm_f = workspace.d_xm << 16;
//     switch(workspaceA.flags) {
//         case 0u:
//             // workspaceA.retAddress = (char *)&loc_10008060;
//             if((workspace.topCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 x1            = workspace.x1;
//                 xm            = workspace.xm;
//                 do {
//                     v4 = HIWORD(x1);
//                     v5 = HIWORD(xm);
//                     v6 = &workspace.scanAddress[v4];
//                     v7 = v5 <= v4;
//                     v8 = v5 - v4;
//                     if(v7) {
//                         sv      = workspaceA.sv;
//                         su_high = HIWORD(workspaceA.su);
//                         v11     = &workspace.depthAddress[2 * v4];
//                         do {

//                             *v6 = '\1';

//                             // if(HIWORD(workspace.c_z) <= *(_WORD *)&v11[2 * v8]) {
//                             //     v12 = sv[su_high];
//                             //     if(v12) {
//                             //         *(_WORD *)&v11[2 * v8] = HIWORD(workspace.c_z);
//                             //         v6[v8]                 = v12;
//                             //     }
//                             // }
//                             v13 = __CFADD__(workspaceA.dvxf, workspace.c_v);
//                             workspace.c_v += workspaceA.dvxf;
//                             for(sv += *(&workspaceA.dvx - 2 * v13); sv < work.texture.base; sv += work.texture.size)
//                                 ;
//                             while(sv >= workspaceA.vUpperBound)
//                                 sv -= work.texture.size;
//                             for(i = workspaceA.dux + workspace.c_u; i < 0; i += workspaceA.uUpperBound)
//                                 ;
//                             while(i >= workspaceA.uUpperBound)
//                                 i -= workspaceA.uUpperBound;
//                             workspace.c_u = i;
//                             su_high       = i >> 16;
//                             workspace.c_z += workspace.d_z_x;
//                             v7 = (v8 + 1 < 0) ^ __OFADD__(1, v8) | (v8 == -1);
//                             ++v8;
//                         } while(v7);
//                     }
//                     v15 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     // sbb edi, edi

//                     int _ESI = v15;
//                     v17      = *(&workspace.d_z_y_0 + 2 * v15);
//                     v18      = *(&workspaceA.duy0 + 2 * v15) + workspaceA.su;
//                     int cf2  = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v15));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;
//                     workspaceA.svf += *(&workspaceA.dvy0f + v15);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v17;
//                     workspace.c_z = workspace.s_z;
//                     v19           = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     x1            = workspace.d_x1 + workspace.x1;
//                     while(v19 < work.texture.base)
//                         v19 += work.texture.size;
//                     while(v19 >= workspaceA.vUpperBound)
//                         v19 -= work.texture.size;
//                     while(v18 < 0)
//                         v18 += workspaceA.uUpperBound;
//                     while(v18 >= workspaceA.uUpperBound)
//                         v18 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v19;
//                     workspaceA.su = v18;
//                     workspace.c_u = v18;
//                     workspace.x1 += workspace.d_x1;
//                     xm = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.topCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.topCount;
//                 } while(!v7);
//             }
//             // workspaceA.retAddress = (char *)&loc_1000828B;
//             if((workspace.bottomCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 x2            = workspace.x2;
//                 v21           = workspace.xm;
//                 do {
//                     v22 = HIWORD(x2);
//                     v23 = HIWORD(v21);
//                     v24 = &workspace.scanAddress[v22];
//                     v7  = v23 <= v22;
//                     v25 = v23 - v22;
//                     if(v7) {
//                         v26 = (_BYTE *)workspaceA.sv;
//                         v27 = HIWORD(workspaceA.su);
//                         v28 = &workspace.depthAddress[2 * v22];
//                         do {
//                             if(HIWORD(workspace.c_z) <= *(_WORD *)&v28[2 * v25]) {
//                                 v29 = v26[v27];
//                                 if(v29) {
//                                     *(_WORD *)&v28[2 * v25] = HIWORD(workspace.c_z);
//                                     v24[v25]                = v29;
//                                 }
//                             }
//                             v13 = __CFADD__(workspaceA.dvxf, workspace.c_v);
//                             workspace.c_v += workspaceA.dvxf;
//                             for(v26 += *(&workspaceA.dvx - 2 * v13); v26 < work.texture.base; v26 += work.texture.size)
//                                 ;
//                             while(v26 >= workspaceA.vUpperBound)
//                                 v26 -= work.texture.size;
//                             for(j = workspaceA.dux + workspace.c_u; j < 0; j += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(j >= (int)workspaceA.uUpperBound)
//                                 j -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = j;
//                             v27           = j >> 16;
//                             workspace.c_z += workspace.d_z_x;
//                             v7 = (v25 + 1 < 0) ^ __OFADD__(1, v25) | (v25 == -1);
//                             ++v25;
//                         } while(v7);
//                     }
//                     v31 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v31;
//                     v33      = *(&workspace.d_z_y_0 + 2 * v31);
//                     v34      = *(&workspaceA.duy0 + 2 * v31) + workspaceA.su;

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v31));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v31);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v33;
//                     workspace.c_z = workspace.s_z;
//                     v35           = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     x2            = workspace.d_x2 + workspace.x2;
//                     while(v35 < work.texture.base)
//                         v35 += work.texture.size;
//                     while(v35 >= workspaceA.vUpperBound)
//                         v35 -= work.texture.size;
//                     while(v34 < 0)
//                         v34 += (signed int)workspaceA.uUpperBound;
//                     while(v34 >= (int)workspaceA.uUpperBound)
//                         v34 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v35;
//                     workspaceA.su = v34;
//                     workspace.c_u = v34;
//                     workspace.x2 += workspace.d_x2;
//                     v21 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.bottomCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.bottomCount;
//                 } while(!v7);
//             }
//             break;
//         case 1u:
//             // workspaceA.retAddress = (char *)&loc_100084BB;
//             if((workspace.topCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 v36           = workspace.x1;
//                 v37           = workspace.xm;
//                 do {
//                     v38 = HIWORD(v36);
//                     v39 = HIWORD(v37);
//                     v40 = &workspace.scanAddress[v38];
//                     v7  = v39 < v38;
//                     v41 = v39 - v38;
//                     if(!v7) {
//                         v42 = (_BYTE *)workspaceA.sv;
//                         v43 = HIWORD(workspaceA.su);
//                         v44 = &workspace.depthAddress[2 * v38];
//                         do {
//                             // if(HIWORD(workspace.c_z) <= *(_WORD *)&v44[2 * v41]) {
//                             //     v45 = v42[v43];
//                             //     if(v45) {
//                             //         *(_WORD *)&v44[2 * v41] = HIWORD(workspace.c_z);
//                             //         v40[v41]                = v45;
//                             //     }
//                             // }
//                             v46 = (workspace.c_v - (uint64_t)workspaceA.dvxf) >> 32;
//                             workspace.c_v -= workspaceA.dvxf;
//                             for(v42 -= *(&workspaceA.dvx + 2 * v46); v42 < work.texture.base; v42 += work.texture.size)
//                                 ;
//                             while(v42 >= workspaceA.vUpperBound)
//                                 v42 -= work.texture.size;
//                             for(k = workspace.c_u - workspaceA.dux; k < 0; k += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(k >= (int)workspaceA.uUpperBound)
//                                 k -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = k;
//                             v43           = k >> 16;
//                             workspace.c_z -= workspace.d_z_x;
//                             v7 = v41-- < 1;
//                         } while(!v7);
//                     }
//                     v48 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v48;
//                     v50      = *(&workspace.d_z_y_0 + 2 * v48);
//                     v51      = *(&workspaceA.duy0 + 2 * v48) + workspaceA.su;
//                     //__asm { rcl     esi, 1 }

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v48));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v48);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v50;
//                     workspace.c_z = workspace.s_z;
//                     v52           = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     v36           = workspace.d_x1 + workspace.x1;
//                     while(v52 < work.texture.base)
//                         v52 += work.texture.size;
//                     while(v52 >= workspaceA.vUpperBound)
//                         v52 -= work.texture.size;
//                     while(v51 < 0)
//                         v51 += (signed int)workspaceA.uUpperBound;
//                     while(v51 >= (int)workspaceA.uUpperBound)
//                         v51 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v52;
//                     workspaceA.su = v51;
//                     workspace.c_u = v51;
//                     workspace.x1 += workspace.d_x1;
//                     v37 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.topCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.topCount;
//                 } while(!v7);
//             }
//             // workspaceA.retAddress = (char *)&loc_100086E6;
//             if((workspace.bottomCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 v53           = workspace.x2;
//                 v54           = workspace.xm;
//                 do {
//                     v55 = HIWORD(v53);
//                     v56 = HIWORD(v54);
//                     v57 = &workspace.scanAddress[v55];
//                     v7  = v56 < v55;
//                     v58 = v56 - v55;
//                     if(!v7) {
//                         v59 = (_BYTE *)workspaceA.sv;
//                         v60 = HIWORD(workspaceA.su);
//                         v61 = &workspace.depthAddress[2 * v55];
//                         do {
//                             // if(HIWORD(workspace.c_z) <= *(_WORD *)&v61[2 * v58]) {
//                             //     v62 = v59[v60];
//                             //     if(v62) {
//                             //         *(_WORD *)&v61[2 * v58] = HIWORD(workspace.c_z);
//                             //         v57[v58]                = v62;
//                             //     }
//                             // }
//                             v63 = (workspace.c_v - (uint64_t)workspaceA.dvxf) >> 32;
//                             workspace.c_v -= workspaceA.dvxf;
//                             for(v59 -= *(&workspaceA.dvx + 2 * v63); v59 < work.texture.base; v59 += work.texture.size)
//                                 ;
//                             while(v59 >= workspaceA.vUpperBound)
//                                 v59 -= work.texture.size;
//                             for(m = workspace.c_u - workspaceA.dux; m < 0; m += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(m >= (int)workspaceA.uUpperBound)
//                                 m -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = m;
//                             v60           = m >> 16;
//                             workspace.c_z -= workspace.d_z_x;
//                             v7 = v58-- < 1;
//                         } while(!v7);
//                     }
//                     v65 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v65;
//                     v67      = *(&workspace.d_z_y_0 + 2 * v65);
//                     v68      = *(&workspaceA.duy0 + 2 * v65) + workspaceA.su;

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v65));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v65);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v67;
//                     workspace.c_z = workspace.s_z;
//                     v69           = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     v53           = workspace.d_x2 + workspace.x2;
//                     while(v69 < work.texture.base)
//                         v69 += work.texture.size;
//                     while(v69 >= workspaceA.vUpperBound)
//                         v69 -= work.texture.size;
//                     while(v68 < 0)
//                         v68 += (signed int)workspaceA.uUpperBound;
//                     while(v68 >= (int)workspaceA.uUpperBound)
//                         v68 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v69;
//                     workspaceA.su = v68;
//                     workspace.c_u = v68;
//                     workspace.x2 += workspace.d_x2;
//                     v54 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.bottomCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.bottomCount;
//                 } while(!v7);
//             }
//             break;
//         case 2u:
//             // workspaceA.retAddress = (char *)&loc_10008916;
//             if((workspace.topCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 v70           = workspace.x1;
//                 v71           = workspace.xm;
//                 do {
//                     v72 = HIWORD(v70);
//                     v73 = HIWORD(v71);
//                     v74 = &workspace.scanAddress[v72];
//                     v7  = v73 <= v72;
//                     v75 = v73 - v72;
//                     if(v7) {
//                         v76 = (_BYTE *)workspaceA.sv;
//                         v77 = HIWORD(workspaceA.su);
//                         v78 = &workspace.depthAddress[2 * v72];
//                         do {
//                             // if(HIWORD(workspace.c_z) <= *(_WORD *)&v78[2 * v75]) {
//                             //     v79 = v76[v77];
//                             //     if(v79) {
//                             //         *(_WORD *)&v78[2 * v75] = HIWORD(workspace.c_z);
//                             //         v74[v75]                = v79;
//                             //     }
//                             // }

//                             v13 = __CFADD__(workspaceA.dvxf, workspace.c_v);
//                             workspace.c_v += workspaceA.dvxf;
//                             for(v76 += *(&workspaceA.dvx - 2 * v13); v76 < work.texture.base; v76 += work.texture.size)
//                                 ;
//                             while(v76 >= workspaceA.vUpperBound)
//                                 v76 -= work.texture.size;
//                             for(n = workspaceA.dux + workspace.c_u; n < 0; n += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(n >= (int)workspaceA.uUpperBound)
//                                 n -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = n;
//                             v77           = n >> 16;
//                             workspace.c_z += workspace.d_z_x;
//                             v7 = (v75 + 1 < 0) ^ __OFADD__(1, v75) | (v75 == -1);
//                             ++v75;
//                         } while(v7);
//                     }
//                     v81 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v81;
//                     v83      = *(&workspace.d_z_y_0 + 2 * v81);
//                     v84      = *(&workspaceA.duy0 + 2 * v81) + workspaceA.su;

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v81));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v81);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v83;
//                     workspace.c_z = workspace.s_z;
//                     v85           = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     v70           = workspace.d_x1 + workspace.x1;
//                     while(v85 < work.texture.base)
//                         v85 += work.texture.size;
//                     while(v85 >= workspaceA.vUpperBound)
//                         v85 -= work.texture.size;
//                     while(v84 < 0)
//                         v84 += (signed int)workspaceA.uUpperBound;
//                     while(v84 >= (int)workspaceA.uUpperBound)
//                         v84 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v85;
//                     workspaceA.su = v84;
//                     workspace.c_u = v84;
//                     workspace.x1 += workspace.d_x1;
//                     v71 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.topCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.topCount;
//                 } while(!v7);
//             }
//             // workspaceA.retAddress = (char *)&loc_10008B41;
//             if((workspace.bottomCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 v86           = workspace.x2;
//                 v87           = workspace.xm;
//                 do {
//                     v88 = HIWORD(v86);
//                     v89 = HIWORD(v87);
//                     v90 = &workspace.scanAddress[v88];
//                     v7  = v89 <= v88;
//                     v91 = v89 - v88;
//                     if(v7) {
//                         v92 = (_BYTE *)workspaceA.sv;
//                         v93 = HIWORD(workspaceA.su);
//                         v94 = &workspace.depthAddress[2 * v88];
//                         do {
//                             if(HIWORD(workspace.c_z) <= *(_WORD *)&v94[2 * v91]) {
//                                 v95 = v92[v93];
//                                 if(v95) {
//                                     *(_WORD *)&v94[2 * v91] = HIWORD(workspace.c_z);
//                                     v90[v91]                = v95;
//                                 }
//                             }
//                             v13 = __CFADD__(workspaceA.dvxf, workspace.c_v);
//                             workspace.c_v += workspaceA.dvxf;
//                             for(v92 += *(&workspaceA.dvx - 2 * v13); v92 < work.texture.base; v92 += work.texture.size)
//                                 ;
//                             while(v92 >= workspaceA.vUpperBound)
//                                 v92 -= work.texture.size;
//                             for(ii = workspaceA.dux + workspace.c_u; ii < 0; ii += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(ii >= (int)workspaceA.uUpperBound)
//                                 ii -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = ii;
//                             v93           = ii >> 16;
//                             workspace.c_z += workspace.d_z_x;
//                             v7 = (v91 + 1 < 0) ^ __OFADD__(1, v91) | (v91 == -1);
//                             ++v91;
//                         } while(v7);
//                     }
//                     v97 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v97;
//                     v99      = *(&workspace.d_z_y_0 + 2 * v97);
//                     v100     = *(&workspaceA.duy0 + 2 * v97) + workspaceA.su;
//                     //__asm { rcl     esi, 1 }

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v97));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v97);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v99;
//                     workspace.c_z = workspace.s_z;
//                     v101          = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     v86           = workspace.d_x2 + workspace.x2;
//                     while(v101 < work.texture.base)
//                         v101 += work.texture.size;
//                     while(v101 >= workspaceA.vUpperBound)
//                         v101 -= work.texture.size;
//                     while(v100 < 0)
//                         v100 += (signed int)workspaceA.uUpperBound;
//                     while(v100 >= (int)workspaceA.uUpperBound)
//                         v100 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v101;
//                     workspaceA.su = v100;
//                     workspace.c_u = v100;
//                     workspace.x2 += workspace.d_x2;
//                     v87 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.bottomCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.bottomCount;
//                 } while(!v7);
//             }
//             break;
//         case 3u:
//             // workspaceA.retAddress = (char *)&loc_10008D71;
//             if((workspace.topCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 v102          = workspace.x1;
//                 v103          = workspace.xm;
//                 do {
//                     v104 = HIWORD(v102);
//                     v105 = HIWORD(v103);
//                     v106 = &workspace.scanAddress[v104];
//                     v7   = v105 < v104;
//                     v107 = v105 - v104;
//                     if(!v7) {
//                         v108 = (_BYTE *)workspaceA.sv;
//                         v109 = HIWORD(workspaceA.su);
//                         v110 = &workspace.depthAddress[2 * v104];
//                         do {
//                             if(HIWORD(workspace.c_z) <= *(_WORD *)&v110[2 * v107]) {
//                                 v111 = v108[v109];
//                                 if(v111) {
//                                     *(_WORD *)&v110[2 * v107] = HIWORD(workspace.c_z);
//                                     v106[v107]                = v111;
//                                 }
//                             }
//                             v112 = (workspace.c_v - (uint64_t)workspaceA.dvxf) >> 32;
//                             workspace.c_v -= workspaceA.dvxf;
//                             for(v108 -= *(&workspaceA.dvx + 2 * v112); v108 < work.texture.base; v108 += work.texture.size)
//                                 ;
//                             while(v108 >= workspaceA.vUpperBound)
//                                 v108 -= work.texture.size;
//                             for(jj = workspace.c_u - workspaceA.dux; jj < 0; jj += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(jj >= (int)workspaceA.uUpperBound)
//                                 jj -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = jj;
//                             v109          = jj >> 16;
//                             workspace.c_z -= workspace.d_z_x;
//                             v7 = v107-- < 1;
//                         } while(!v7);
//                     }
//                     v114 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v114;
//                     v116     = *(&workspace.d_z_y_0 + 2 * v114);
//                     v117     = *(&workspaceA.duy0 + 2 * v114) + workspaceA.su;

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v114));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v114);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v116;
//                     workspace.c_z = workspace.s_z;
//                     v118          = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     v102          = workspace.d_x1 + workspace.x1;
//                     while(v118 < work.texture.base)
//                         v118 += work.texture.size;
//                     while(v118 >= workspaceA.vUpperBound)
//                         v118 -= work.texture.size;
//                     while(v117 < 0)
//                         v117 += (signed int)workspaceA.uUpperBound;
//                     while(v117 >= (int)workspaceA.uUpperBound)
//                         v117 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = v118;
//                     workspaceA.su = v117;
//                     workspace.c_u = v117;
//                     workspace.x1 += workspace.d_x1;
//                     v103 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.topCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.topCount;
//                 } while(!v7);
//             }
//             // workspaceA.retAddress = (char *)&loc_10008F9C;
//             if((workspace.bottomCount & 0x80000000) == 0) {
//                 workspace.c_u = workspaceA.su;
//                 workspace.c_v = workspaceA.svf;
//                 workspace.c_z = workspace.s_z;
//                 v119          = workspace.x2;
//                 v120          = workspace.xm;
//                 do {
//                     v121 = HIWORD(v119);
//                     v122 = HIWORD(v120);
//                     v123 = &workspace.scanAddress[v121];
//                     v7   = v122 < v121;
//                     v124 = v122 - v121;
//                     if(!v7) {
//                         v125 = (_BYTE *)workspaceA.sv;
//                         v126 = HIWORD(workspaceA.su);
//                         v127 = &workspace.depthAddress[2 * v121];
//                         do {
//                             if(HIWORD(workspace.c_z) <= *(_WORD *)&v127[2 * v124]) {
//                                 v128 = v125[v126];
//                                 if(v128) {
//                                     *(_WORD *)&v127[2 * v124] = HIWORD(workspace.c_z);
//                                     v123[v124]                = v128;
//                                 }
//                             }
//                             v129 = (workspace.c_v - (uint64_t)workspaceA.dvxf) >> 32;
//                             workspace.c_v -= workspaceA.dvxf;
//                             for(v125 -= *(&workspaceA.dvx + 2 * v129); v125 < work.texture.base; v125 += work.texture.size)
//                                 ;
//                             while(v125 >= workspaceA.vUpperBound)
//                                 v125 -= work.texture.size;
//                             for(kk = workspace.c_u - workspaceA.dux; kk < 0; kk += (signed int)workspaceA.uUpperBound)
//                                 ;
//                             while(kk >= (int)workspaceA.uUpperBound)
//                                 kk -= (signed int)workspaceA.uUpperBound;
//                             workspace.c_u = kk;
//                             v126          = kk >> 16;
//                             workspace.c_z -= workspace.d_z_x;
//                             v7 = v124-- < 1;
//                         } while(!v7);
//                     }
//                     v131 = -__CFADD__(workspace.d_xm_f, workspace.xm_f);
//                     workspace.xm_f += workspace.d_xm_f;
//                     int _ESI = v131;
//                     v133     = *(&workspace.d_z_y_0 + 2 * v131);
//                     v134     = *(&workspaceA.duy0 + 2 * v131) + workspaceA.su;

//                     int cf2 = __CFADD__(workspaceA.svf, *(&workspaceA.dvy0f + v131));
//                     //__asm { rcl     esi, 1 }
//                     _ESI = _ESI << 1;
//                     _ESI = _ESI | cf2;

//                     workspaceA.svf += *(&workspaceA.dvy0f + v131);
//                     workspace.c_v = workspaceA.svf;
//                     workspace.s_z += v133;
//                     workspace.c_z = workspace.s_z;
//                     v135          = (char *)(*(&workspaceA.dvy0 + 2 * _ESI) + workspaceA.sv);
//                     v119          = workspace.d_x2 + workspace.x2;
//                     while(v135 < work.texture.base)
//                         v135 += work.texture.size;
//                     while(v135 >= workspaceA.vUpperBound)
//                         v135 -= work.texture.size;
//                     while(v134 < 0)
//                         v134 += (signed int)workspaceA.uUpperBound;
//                     while(v134 >= (int)workspaceA.uUpperBound)
//                         v134 -= (signed int)workspaceA.uUpperBound;
//                     workspaceA.sv = (uint32_t)v135;
//                     workspaceA.su = v134;
//                     workspace.c_u = v134;
//                     workspace.x2 += workspace.d_x2;
//                     v120 = workspace.d_xm + workspace.xm;
//                     workspace.xm += workspace.d_xm;
//                     workspace.scanAddress += work.colour.stride_b;
//                     v7 = (signed int)workspace.bottomCount < 1;
//                     workspace.depthAddress += work.depth.stride_b;
//                     --workspace.bottomCount;
//                 } while(!v7);
//             }
//             break;
//     }
// }
