#include <stdint.h>
#include "../softrend/ddi/priminfo.h"
#include "x86emu.h"
#include "fpsetup.h"

double fconv_d16_12 = 1.03079215104e11;
double fconv_d16_m  = 1.03079215105e11;
float  fp_one       = 1.0f;
float  fp_conv_d    = 6.7553994e15;
float  fp_conv_d8   = 2.6388279e13;
float  fp_conv_d8r  = 1.7293823e18;
float  fp_conv_d16  = 1.0307922e11;
float  fp_conv_d24  = 4.0265318e8;
float  fp_conv_d32  = 1572864.0;

uint16_t fp_single_cw   = 0x107f;
uint16_t fp_double_cw   = 0x127f;
uint16_t fp_extended_cw = 0x137f;

int      sort_table_1[] = {1, 2, 0, 0, 0, 0, 2, 1};
int      sort_table_0[] = {0, 0, 0, 2, 1, 0, 1, 2};
int      sort_table_2[] = {2, 1, 0, 1, 2, 0, 0, 0};
uint32_t flip_table[8]  = {0x000000000, 0x080000000, 0x080000000, 0x000000000,
                           0x080000000, 0x000000000, 0x000000000, 0x080000000};

struct workspace_t workspace;

void TriangleSetup_ZT_ARBITRARY(brp_vertex *v0, brp_vertex *v1, brp_vertex *v2)
{
    SETUP_FLOAT(v0, v1, v2);
    // SETUP_FLOAT_PARAM C_SZ,_z,workspace.s_z,workspace.d_z_x,fp_conv_d16,1
    // SETUP_FLOAT_PARAM C_U,_u,workspace.s_u,workspace.d_u_x,fp_conv_d24
    // SETUP_FLOAT_PARAM C_V,_v,workspace.s_v,workspace.d_v_x,fp_conv_d24
    // ARBITRARY_SETUP
}

void SETUP_FLOAT(brp_vertex *v0, brp_vertex *v1, brp_vertex *v2)
{
    x86emu_init();

    eax->ptr_val = v0;
    ecx->ptr_val = v1;
    edx->ptr_val = v2;
    // local count_cont,exit,top_zero,bottom_zero,empty_triangle

    // assume eax: ptr brp_vertex, /*ebx: ptr brp_vertex,*/ ecx: ptr brp_vertex, edx: ptr brp_vertex

    //; Calculate area of triangle and generate dx1/2area, dx1/2area, dx1/2area and dx1/2area
    //;
    //; Also sort the vertices in Y order whilst divide is happening
    //;
    //;	0		1		2		3		4		5		6		7
    fld(x87_op_f(((brp_vertex *)edx->ptr_val)->comp_f[C_SX])); //	x2
    fsub(((brp_vertex *)eax->ptr_val)->comp_f[C_SX]);          //	dx2
    fld(x87_op_f(((brp_vertex *)ecx->ptr_val)->comp_f[C_SX])); //	x1		dx2
    fsub(((brp_vertex *)eax->ptr_val)->comp_f[C_SX]);          //	dx1		dx2
    fld(x87_op_f(((brp_vertex *)edx->ptr_val)->comp_f[C_SY])); //	y2		dx1		dx2
    fsub(((brp_vertex *)eax->ptr_val)->comp_f[C_SY]);          //	dy2		dx1		dx2
    fld(x87_op_f(((brp_vertex *)ecx->ptr_val)->comp_f[C_SY])); //	y1		dy2		dx1		dx2
    fsub(((brp_vertex *)eax->ptr_val)->comp_f[C_SY]);          //	dy1		dy2		dx1		dx2

    fld(x87_op_i(2)); //	dx1		dy1		dy2		dx1		dx2

    fmul_2(x87_op_i(0), x87_op_i(2)); //	dx1*dy2	dy1		dy2		dx1		dx2

    fld(x87_op_i(4));                 //	dx2		dx1*dy2	dy1		dy2		dx1		dx2
    fmul_2(x87_op_i(0), x87_op_i(2)); //	dx2*dy1	dx1*dy2	dy1		dy2		dx1		dx2

    mov(x86_op_reg(eax), x86_op_mem32(&((brp_vertex *)eax->ptr_val)->comp_f[C_SY]));

    mov(x86_op_reg(ecx), x86_op_mem32(&((brp_vertex *)ecx->ptr_val)->comp_f[C_SY]));

    fsubp_2(x87_op_i(1), x87_op_i(0)); //	2area	dy1		dy2		dx1		dx2

    xor_(x86_op_reg(ebx), x86_op_reg(ebx));
    cmp(x86_op_reg(ecx), x86_op_reg(eax));

    rcl(x86_op_reg(ebx), 1);
    mov(x86_op_reg(edx), x86_op_mem32(&((brp_vertex *)edx->ptr_val)->comp_f[C_SY]));

    fdivr(fp_one); //	1/2area	dy1		dy2		dx1		dx2

    cmp(x86_op_reg(edx), x86_op_reg(eax));

    rcl(x86_op_reg(ebx), 1);
    cmp(x86_op_reg(edx), x86_op_reg(ecx));

    rcl(x86_op_reg(ebx), 1); // ebx now has 3 bit number characterising the order of the vertices.

    mov(x86_op_reg(eax), x86_op_mem32(&sort_table_0[ebx->uint_val]));
    mov(x86_op_reg(edx), x86_op_mem32(&sort_table_2[ebx->uint_val]));

    mov(x86_op_reg(esi), x86_op_mem32(&flip_table[ebx->uint_val]));
    mov(x86_op_reg(ebx), x86_op_mem32(&sort_table_1[ebx->uint_val]));

    mov(x86_op_reg(eax), x86_op_ptr(&workspace.v0_array[eax->uint_val]));
    mov(x86_op_reg(edx), x86_op_ptr(&workspace.v0_array[edx->uint_val]));
    mov(x86_op_reg(ebx), x86_op_ptr(&workspace.v0_array[ebx->uint_val]));

    mov(x86_op_mem32(&workspace.flip), x86_op_reg(esi));
    int a = 0;
}
