#include "priminfo.h"
#include <stdarg.h>
#include "fpsetup.h"

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

    //     ; Floating point address calculation - 20 cycles, (Integer=26)
    // ;										st(0)		st(1)		st(2)		st(3)		st(4)		st(5) st(6) st(7)

    // 	fild work.colour.base			;	cb
    // 	fild workspace.t_y				;	ty			cb
    // 	fild work.depth.base			;	db			ty			cb
    // 	fild work.colour.stride_b		;	cs			db			ty			cb
    // 	fild work.depth.stride_b		;	ds			cs			db			ty			cb
    // 	fxch st(4)						;	cb			cs			db			ty			ds
    // 	fsub fp_one						;	cb-1		cs			db			ty			ds
    // 	 fxch st(3)						;	ty			cs			db			cb-1		ds
    // 	fsub fp_one						;	ty-1		cs			db			cb-1		ds
    // 	 fxch st(2)						;	db			cs			ty-1		cb-1		ds
    // 	fsub fp_two						;	db-2		cs			ty-1		cb-1		ds
    // 	 fxch st(3)						;	cb-1		cs			ty-1		db-2		ds
    // 	fadd fp_conv_d					;	cb-1I		cs			ty-1		db-2		ds
    // 	 fxch st(1)						;	cs			cb-1I		ty-1		db-2		ds
    // 	fmul st,st(2)					;	csy			cb-1I		ty-1		db-2		ds
    // 	 fxch st(3)						;	db-2		cb-1I		ty-1		csy			ds
    // 	fadd fp_conv_d					;	db-2I		cb-1I		ty-1		csy			ds
    // 	 fxch st(2)						;	ty-1		cb-1I		db-2I		csy			ds
    // 	fmulp st(4),st					;	cb-1I		db-2I		csy			dsy
    // 	faddp st(2),st					;	db-2I		ca			dsy
    // 	;stall
    // 	faddp st(2),st					;	ca			da
    // 	fstp qword ptr workspace.scanAddress
    // 	fstp qword ptr workspace.depthAddress

    // 	mov eax,work.texture.base
    // 	mov ebx,workspaceA.sv

    // 	add ebx,eax
    // 	mov eax,workspace.xm

    // 	shl eax,16
    // 	mov workspaceA.sv,ebx

    // 	mov	edx,workspaceA.flags
    // 	mov ebx,workspace.d_xm

    // 	mov ecx,[jumpTable_ZT_I8+4*edx]

    // 	shl ebx,16
    // 	mov workspace.xm_f,eax

    // 	mov workspace.d_xm_f,ebx
    // 	jmp ecx
}
