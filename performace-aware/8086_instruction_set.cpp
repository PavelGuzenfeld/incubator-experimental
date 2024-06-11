#include <cinttypes>
#include <string>

class InstructionSet8086
{

public:
    std::string mov(int16_t code)
    {
        std::string instruction = "MOV ";
        switch (code)
        {
        case 0x00:
            instruction += "AL, ";
            break;
        case 0x01:
            instruction += "AX, ";
            break;
        case 0x02:
            instruction += "CL, ";
            break;
        case 0x03:
            instruction += "CX, ";
            break;
        case 0x04:
            instruction += "DL, ";
            break;
        case 0x05:
            instruction += "DX, ";
            break;
        case 0x06:
            instruction += "BL, ";
            break;
        case 0x07:
            instruction += "BX, ";
            break;
        case 0x08:
            instruction += "AH, ";
            break;
        case 0x09:
            instruction += "SP, ";
            break;
        case 0x0A:
            instruction += "CH, ";
            break;
        case 0x0B:
            instruction += "BP, ";
            break;
        case 0x0C:
            instruction += "DH, ";
            break;
        case 0x0D:
            instruction += "SI, ";
            break;
        case 0x0E:
            instruction += "BH, ";
            break;
        case 0x0F:
            instruction += "DI, ";
            break;
        case 0x10:
            instruction += "AX, ";
            break;
        case 0x11:
            instruction += "BX, ";
            break;
        case 0x12:
            instruction += "CX, ";
            break;
        case 0x13:
            instruction += "DX, ";
            break;
        case 0x14:
            instruction += "SP, ";
            break;
        case 0x15:

            instruction += "BP, ";
            break;
        case 0x16:

            instruction += "SI, ";
            break;
        case 0x17:

            instruction += "DI, ";
            break;
        case 0x18:

            instruction += "AL, ";
            break;
        case 0x19:

            instruction += "AX, ";
            break;
        case 0x1A:

            instruction += "AL, ";
            break;
        case 0x1B:

            instruction += "AX, ";
            break;

        case 0x1C:

            instruction += "AL, ";
            break;
        case 0x1D:

            instruction += "AX, ";
            break;
        case 0x1E:

            instruction += "AL, ";
            break;
        case 0x1F:

            instruction += "AX, ";
            break;
        case 0x20:

            instruction += "AL, ";
            break;
        case 0x21:

            instruction += "AX, ";
            break;
        case 0x22:

            instruction += "AL, ";
            break;
        case 0x23:

            instruction += "AX, ";
            break;
        case 0x24:

            instruction += "AL, ";
            break;
        case 0x25:
            
                instruction += "AX, ";
                break;
        case 0x26:

            instruction += "AL, ";
            break;
        case 0x27:
            
                instruction += "AX, ";
                break;
                
        }

        void add();
        void sub();
        void mul();
        void div();
        void inc();
        void dec();
        void cmp();
        void jmp();
        void je();
        void jne();
        void jg();
        void jge();
        void jl();
        void jle();
        void call();
        void ret();
        void push();
        void pop();

        void lea();
        void nop();
        void hlt();
        void int();
        void iret();
        void in();
        void out();
        void xchg();
        void cwd();
        void cbw();
        void clc();
        void stc();
        void cld();
        void std();
        void sar();
        void shr();
        void sal();
        void shl();
        void ror();
        void rol();
        void rcr();
        void rcl();
        void not();
        void and ();
        void or ();
        void xor ();
        void neg();
        void test();
        void xlat();
        void aaa();
        void aas();
        void aam();
        void aad();
        void daa();
        void das();
        void cbw();
        void cwd();
        void wait();
        void lock();
        void rep();
        void repe();
        void repne();
        void repnz();
        void repz();
        void esc();
        void loop();
        void loope();
        void loopne();
        void loopnz();
        void loopz();
        void jcxz();
        void jc();
        void jnc();
        void jb();
        void jnb();
        void jnae();
        void jae();
        void jnc();
        void jnb();
        void jbe();
        void ja();
        void jnbe();
        void js();
        void jns();
        void jp();
        void jpe();
        void jnp();
        void jpo();
        void jo();
        void jno();
        void jnz();
        void jz();
        void jg();
        void jge();
        void jl();
        void jle();
        void jna();
        void ja();
        void jnge();
        void jnl();
        void jnle();
        void jng();
        void jnle();
        void jnge();
        void jnl();
        void jnbe();
        void jnae();
        void jnb();
        void jnc();
        void jno();
        void jnp();
        void jpo();
        void jns();
        void jnz();
        void jpe();
        void jp();
        void js();
        void jz();
        void jcxz();
        void jmp();
        void jcxz();
        void jecxz();
        void jrcxz();
    };

    void InstructionSet8086::add()
    {
        // Add instruction
    }