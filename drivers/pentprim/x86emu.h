#ifndef X86_EMU
#define X86_EMU

#include <stdint.h>

#define st

enum X86_OP {
    X86_OP_REG,
    X86_OP_MEM32,
    X86_OP_PTR
};

enum {
    X87_OP_FLOAT,
    X87_OP_DOUBLE,
    X87_OP_ST,
};

typedef struct x86_reg {
    union {
        uint32_t      uint_val;
        float         float_val;
        unsigned char bytes[8];
        void         *ptr_val;
    };

} x86_reg;

typedef union x86_mem {
    uint32_t      uint_val;
    float         float_val;
    void         *ptr_val;
    unsigned char bytes[8];

} x86_mem;

typedef struct x86_operand {
    union {
        x86_reg *reg;
        x86_mem  mem;
        void    *ptr;
    };
    char type;
} x86_operand;

typedef struct x87_operand {
    struct { // union?
        float         float_val;
        double        double_val;
        int           st_index;
        unsigned char bytes[8];
    };
    char type;
} x87_operand;

x87_operand x87_op_f(float f);
x87_operand x87_op_i(int i);
x86_operand x86_op_reg(x86_reg *r);
x86_operand x86_op_mem32(void *bytes);
x86_operand x86_op_ptr(void *ptr);

extern x86_reg *eax, *ebx, *ecx, *edx, *esi;
void            x86emu_init();

void fld(x87_operand op);
void fsub(float val);
void fsub_2(x87_operand dest, x87_operand src);
void fsubp_2(x87_operand dest, x87_operand src);
void fmul(float);
void fmul_2(x87_operand dest, x87_operand src);
void fdivr(float f);

void mov(x86_operand dest, x86_operand src);
void xor_(x86_operand dest, x86_operand src);
void cmp(x86_operand dest, x86_operand src);
void rcl(x86_operand dest, int count);

#endif
