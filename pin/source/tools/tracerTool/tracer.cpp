/*BEGIN_LEGAL 
Intel Open Source License 

Copyright (c) 2002-2018 Intel Corporation. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.  Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.  Neither the name of
the Intel Corporation nor the names of its contributors may be used to
endorse or promote products derived from this software without
specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
END_LEGAL */
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <stdio.h>
#include <fstream>
#include <set>
#include <types.h>
#include <vector>
#include <iterator>
#include "pin.H"
#include <string>

#define NUM_REGS 15

enum EdgeType {Direct, Indirect, Conditional, Syscall, Return, Regular, Unknown};
static const std::string EDGE_TYPE_STR[7] = {
    "Direct", "Indirect", "Conditional", "Syscall", "Return", "Regular", "Unknown"
};

struct MemoryField {
    UINT64 address;
    UINT32 size;
    UINT64 value;
};

struct MemoryData {
    MemoryField last_addr  = {0, 0, 0};
    MemoryField min_addr   = {UINT64_MAX, 0, 0};
    MemoryField max_addr   = {0, 0, 0};
    MemoryField last_value = {0, 0, 0};
    MemoryField min_value  = {0, 0, UINT64_MAX};
    MemoryField max_value  = {0, 0, 0};

    std::string to_string() const {
        std::ostringstream ss;
        ss << "{\"last_address\":" << last_addr.address << ",";
        ss << "\"min_address\":"   << min_addr.address  << ",";
        ss << "\"max_address\":"   << max_addr.address  << ",";
        ss << "\"last_value\":" << last_value.value << ",";
        ss << "\"min_value\":"   << min_value.value  << ",";
        ss << "\"max_value\":"   << max_value.value  << "}";
        return ss.str();
    }
};

struct InstructionData {
  UINT64 count;
  std::string disas;
  UINT64 min_val[NUM_REGS];
  UINT64 max_val[NUM_REGS];
  UINT64 last_val[NUM_REGS];
  std::set<ADDRINT> successors;
  std::set<ADDRINT> predecessors;
  MemoryData mem;
public:
    InstructionData(std::string disas){
        count = 0;
        this->disas = disas;
        for (int i = 0;i < NUM_REGS;i++) {
            min_val[i] = UINT64_MAX;
            max_val[i] = 0;
            last_val[i] = 0;
        }
    }
    InstructionData(){
        count = 0;
        this->disas = "undefined";
        for (int i = 0;i < NUM_REGS;i++) {
            min_val[i] = UINT64_MAX;
            max_val[i] = 0;
            last_val[i] = 0;
        }
    }
};

static const REG REGISTERS[NUM_REGS] = {
    REG_RAX, REG_RBX, REG_RCX, REG_RDX, REG_RSI, REG_RDI,
    REG_R8, REG_R9, REG_R10, REG_R11, REG_R12, REG_R13, 
    REG_R14, REG_R15,REG_GFLAGS
};
static const std::string REG_NAMES[NUM_REGS] = {
    "rax", "rbx", "rcx", "rdx", "rsi", "rdi",
    "r8", "r9", "r10", "r11", "r12", "r13", 
    "r14", "r15", "eflags"
};

static FILE * g_trace_file;
static std::map<ADDRINT, InstructionData> g_instruction_map;
static ADDRINT g_prev_ins_addr = 0;
static ADDRINT g_load_offset;
static ADDRINT g_low_address;
static ADDRINT g_first_ins_addr;
static PIN_LOCK g_lock;

/**
 * Update globally tracked register state and append written or modified values to address
 */
VOID update_reg_state(ADDRINT ins_addr, const CONTEXT * ctxt) {
    PIN_REGISTER temp;
    InstructionData * tuple = &g_instruction_map[ins_addr];
    for (UINT32 i = 0; i < NUM_REGS; i++) {
        PIN_GetContextRegval(ctxt, REGISTERS[i], reinterpret_cast<UINT8*>(&temp));
        if (tuple->min_val[i] >= *(temp.qword)) tuple->min_val[i] = *(temp.qword);
        if (tuple->max_val[i] <= *(temp.qword)) tuple->max_val[i] = *(temp.qword);
        // store "last-seen" value
        tuple->last_val[i] = *(temp.qword);
    }
}


/**
 * Update register state, save edge, and update global information based on current instruction
 */
VOID ins_save_state(ADDRINT ins_addr, const std::string& ins_disas, const CONTEXT * ctxt) {
    PIN_GetLock(&g_lock, ins_addr);
    // if first occurence, add instruction to map
    if (g_instruction_map.find(ins_addr) == g_instruction_map.end()) g_instruction_map[ins_addr] = InstructionData(ins_disas);
    // increase visited count
    g_instruction_map[ins_addr].count += 1;
    // update which registers where changed during execution of the instruction
    update_reg_state(ins_addr, ctxt);
    // if a predecessor exists, save the edge
    if (g_prev_ins_addr) {
        g_instruction_map[g_prev_ins_addr].successors.insert(ins_addr);
        g_instruction_map[ins_addr].predecessors.insert(g_prev_ins_addr);
    }
    // data for next instruction to act upon
    g_prev_ins_addr = ins_addr;
    PIN_ReleaseLock(&g_lock);
}
UINT64 read_from_addr(ADDRINT mem_addr, ADDRINT size, ADDRINT ins_addr) {
    switch(size) {
        case 1:
        {
            uint8_t * value = reinterpret_cast<uint8_t *>(mem_addr);
            return static_cast<uint64_t>(*value);
        }
        case 2:
        {
            uint16_t * value = reinterpret_cast<uint16_t *>(mem_addr);
            return static_cast<uint64_t>(*value);
        }
        case 4:
        {
            uint32_t * value = reinterpret_cast<uint32_t *>(mem_addr);
            return static_cast<uint64_t>(*value);
        }
        case 8:
        {
            uint64_t * value = reinterpret_cast<uint64_t *>(mem_addr);
            return static_cast<uint64_t>(*value);
        }
        default:
            LOG ("[E] Unhandled memory access size " + decstr(size) + " (" + decstr(size*8)
                 + " bits). Value set to 0 for " + StringFromAddrint(ins_addr) + "\n");
    }
    return 0;
}
EdgeType get_edge_type(INS ins) {
    if (INS_IsRet(ins)) return Return;
    if (INS_IsCall(ins) || INS_IsBranch(ins)) {
        if (INS_Category(ins) == XED_CATEGORY_COND_BR) return Conditional;
        if (INS_IsIndirectControlFlow(ins)) return Indirect;
        if (INS_IsDirectControlFlow(ins)) return Direct;
        return Unknown;
    }
    if (INS_IsSyscall(ins)) return Syscall;
    return Regular;
}

VOID ins_save_memory_access(ADDRINT ins_addr, ADDRINT mem_addr, UINT32 size) {
    // Disregard everything with more than 8 bytes (we are not interested in floating point stuff)
    if (size > 8) {
        return;
    }
    PIN_GetLock(&g_lock, ins_addr);
    MemoryData* mem_data = &g_instruction_map[ins_addr].mem;
    if (mem_data->last_addr.size && mem_data->last_addr.size != size) {
        LOG("[E] Memory operand has different memory access sizes at " + StringFromAddrint(ins_addr) + "\n");
        assert(mem_data->last_addr.size == size && "Memory operand has different memory access sizes");
    }
    MemoryField access = {mem_addr, size, 0};
    access.value = read_from_addr(mem_addr, size, ins_addr);
    if (mem_data->max_addr.address <= access.address) mem_data->max_addr = access;
    if (mem_data->min_addr.address >= access.address) mem_data->min_addr = access;
    mem_data->last_addr = access;
    if (mem_data->max_value.value <= access.value) mem_data->max_value = access;
    if (mem_data->min_value.value >= access.value) mem_data->min_value = access;
    mem_data->last_value = access;
    PIN_ReleaseLock(&g_lock);
}

// Pin calls this function every time a new instruction is encountered
VOID Instruction(INS ins, VOID *v) {
    // Skip instructions outside main exec
    PIN_LockClient();
    const IMG image = IMG_FindByAddress(INS_Address(ins));
    PIN_UnlockClient();
    if (IMG_Valid(image) && IMG_IsMainExecutable(image)) {
        if (INS_IsHalt(ins)) {
            LOG("[W] Skipping instruction: " + StringFromAddrint(INS_Address(ins)) + " : "
                + INS_Disassemble(ins) + "\n");
            return;
        }
        // Check whether the instruction is a branch | call | ret | ...
        EdgeType type = get_edge_type(ins);
        // For regular edges, put insertion point after execution else (calls/ret/(cond) branches) before
        IPOINT ipoint = (type == Regular ? IPOINT_AFTER : IPOINT_BEFORE);
        INS_InsertCall(ins,
            ipoint, (AFUNPTR)ins_save_state,
            IARG_ADDRINT, INS_Address(ins),
            IARG_PTR, new std::string(INS_Disassemble(ins)),
            IARG_CONST_CONTEXT,
            IARG_END
        );

                if (!(INS_HasExplicitMemoryReference(ins) || INS_Stutters(ins)) || type != Regular) {
            return;
        }
        // Ignore non-typical operations such as vscatter/vgather
        if (!INS_IsStandardMemop(ins)) {
            LOG("[W] Non-standard memory operand encountered: " + StringFromAddrint(INS_Address(ins))
                + " : " + INS_Disassemble(ins) + "\n");
            return;
        }

        // Iterate over all memory operands of the instruction
        UINT32 mem_operands = INS_MemoryOperandCount(ins);
        for (UINT32 mem_op = 0; mem_op < mem_operands; mem_op++) {
            // Ensure that we can determine the size
            if (!INS_hasKnownMemorySize(ins)) {
                LOG("[W] Memory operand with unknown size encountered: " + StringFromAddrint(INS_Address(ins))
                    + " : " + INS_Disassemble(ins) + "\n");
                continue;
            }
            // Instrument only when we *write* to memory
            if (INS_MemoryOperandIsWritten(ins, mem_op)) {
                // Instrument only when the instruction is executed (conditional mov)
                INS_InsertPredicatedCall(
                    ins, IPOINT_AFTER, (AFUNPTR)ins_save_memory_access,
                    IARG_INST_PTR,
                    IARG_MEMORYOP_EA, mem_op,
                    IARG_MEMORYWRITE_SIZE,
                    IARG_END
                );
            }
        }

    }
}



/**
 *  Extract metadata from main executable. Includes image base, load offset,
 *  first executed instruction address, and stack + heap ranges
 */
VOID parse_image(IMG img, VOID *v) {
    LOG("[+] Called parse_image on " + IMG_Name(img) + "\n");
    if (IMG_IsMainExecutable(img)) {
        g_load_offset  = IMG_LoadOffset(img);
        g_low_address  = IMG_LowAddress(img);
        LOG("[*] Image base: " + StringFromAddrint(g_low_address) + "\n");
        LOG("[*] Load offset: " + StringFromAddrint(g_load_offset) + "\n");
        ADDRINT img_entry_addr = IMG_EntryAddress(img);
        LOG("[*] Image entry address: " + StringFromAddrint(img_entry_addr) + "\n");
        g_first_ins_addr = g_load_offset + img_entry_addr;
        LOG("[*] First instruction address: " + StringFromAddrint(g_first_ins_addr) + "\n");
    }
}

/**
 *  Convert an array of REGISTER : data to JSON string
 */

/**
 *  Write data as JSON to output file upon application exit
 */
VOID Fini(INT32 code, VOID *v) {
    std::ostringstream title;
    title << "Addr,Disassemble";
    for (int i = 0;i < NUM_REGS; i++) {
        title << "," << REG_NAMES[i] << "_max";
        title << "," << REG_NAMES[i] << "_min";
    }
    // title << ",mem_max,mem_min";
    title << ",predecessors";
    title << ",successors";
    title << ",hit_count";
    fprintf(g_trace_file, "%s\n", title.str().data());
    for (std::pair<ADDRINT, InstructionData> temp: g_instruction_map) {
        std::string alias = temp.second.disas;
        UINT64 predecessors = temp.second.predecessors.size();
        UINT64 successors = temp.second.successors.size();
        UINT64 count = temp.second.count;
        std::ostringstream ss;
        ss << std::hex << "0x" <<temp.first << std::dec <<",\"" << alias << "\"";
        for (int j = 0;j < NUM_REGS; j++) {
            ss << "," << temp.second.max_val[j];
            ss << "," << temp.second.min_val[j];
        }
        // if(temp.second.mem.last_addr.size != 0) {
        //     ss << "," << temp.second.mem.max_value.value;
        //     ss << "," << temp.second.mem.min_value.value;
        // } else {
        //     ss << ",0";
        //     ss << ",0";
        // }
        ss << "," << predecessors;
        ss << "," << successors;
        ss << "," << count;
        fprintf(g_trace_file, "%s\n", ss.str().data());
    }
    fclose(g_trace_file);
    LOG("[=] Completed trace.\n");
}


// Allow renaming output file via -o switch
KNOB<std::string> KnobOutputFile(KNOB_MODE_WRITEONCE, "pintool",
    "o", "itrace.out", "specify output file name");


/* ===================================================================== */
/* Print Help Messages                                                   */
/* ===================================================================== */

INT32 Usage() {
    PIN_ERROR("This Pintool traces each instruction, dumping their addresses and additional state.\n"
              + KNOB_BASE::StringKnobSummary() + "\n");
    return -1;
}


INT32 Aslr() {
    PIN_ERROR("Disable ASLR before running this tool: echo 0 | sudo tee /proc/sys/kernel/randomize_va_space");
    return -1;
}


/* ===================================================================== */
/* Main                                                                  */
/* ===================================================================== */

int main(int argc, char * argv[]) {
    // Check if ASLR is disabled
    std::ifstream infile("/proc/sys/kernel/randomize_va_space");
    int aslr;
    if (!infile) {
        PIN_ERROR("Unable to check whether ASLR is enabled or not. Failed to open /proc/sys/kernel/randomize_va_space");
        return -1;
    }
    infile >> aslr;
    infile.close();
    if (aslr != 0) return Aslr();

    // Initialize pin
    if (PIN_Init(argc, argv)) return Usage();

    g_trace_file = fopen(KnobOutputFile.Value().c_str(), "w");


    // get image base address
    IMG_AddInstrumentFunction(parse_image, 0);

    // Register Instruction to be called to instrument instructions
    INS_AddInstrumentFunction(Instruction, 0);
    // Register Fini to be called when the application exits
    PIN_AddFiniFunction(Fini, 0);

    LOG("[*] Pintool: " + std::string(PIN_ToolFullPath()) + "\n");
    LOG("[*] Target:  " + std::string(PIN_VmFullPath()) + "\n");

    // Start the program, never returns
    PIN_StartProgram();

    return 0;
}

