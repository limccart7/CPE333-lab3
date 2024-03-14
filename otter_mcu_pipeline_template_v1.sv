`timescale 1ns / 1ps

typedef enum logic [6:0] {
    LUI      = 7'b0110111,
    AUIPC    = 7'b0010111,
    JAL      = 7'b1101111,
    JALR     = 7'b1100111,
    BRANCH   = 7'b1100011,
    LOAD     = 7'b0000011,
    STORE    = 7'b0100011,
    OP_IMM   = 7'b0010011,
    OP       = 7'b0110011,
    SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                 input INTR,
                 input RESET,
                 input [31:0] IOBUS_IN,
                 output [31:0] IOBUS_OUT,
                 output [31:0] IOBUS_ADDR,
                 output logic IOBUS_WR
                 );     
                       
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    logic br_lt,br_eq,br_ltu;
              
//==== Instruction Fetch ===========================================

    logic [31:0] if_de_pc;
     
    always_ff @(posedge CLK) begin
        if_de_pc <= pc;
    end
     
    assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
    assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
    assign next_pc = pc + 4;
     
    //PC instantiation
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                 .PC_DIN(pc_value), .PC_COUNT(pc));   
    
    // Creates a 4-to-1 multiplexor used to select the source of the next PC
    Mult4to1 PCdatasrc ( .In1(next_pc), .In2(jalr_pc), .In3(branch_pc), .In4(jump_pc), .Sel(pc_sel), .Out(pc_value) );
     
     
//==== Instruction Decode ===========================================
    
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

    instr_t de_ex_inst, de_inst;
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.opcode=OPCODE;
   
    assign de_inst.rs1_used=    de_inst.rs1_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
                                
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(opcode), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), 
             .CU_BR_EQ(br_eq),.CU_BR_LT(br_lt),.CU_BR_LTU(br_ltu),.CU_PCSOURCE(pc_sel),
             .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel),.intTaken(intTaken));
             
    // Creates a RISC-V register file
    OTTER_registerFile RF (IR[19:15], IR[24:20], IR[11:7], rfIn, regWrite, A, B, CLK); // Register file
	
	
//==== Execute ======================================================
    
    logic [31:0] ex_mem_rs2;
    logic ex_mem_aluRes = 0;
    instr_t ex_mem_inst;
    logic [31:0] opA_forwarded;
    logic [31:0] opB_forwarded;
     
    // Creates a RISC-V ALU
    OTTER_ALU ALU (de_ex_inst.alu_fun, de_ex_opA, de_ex_opB, aluResult); // the ALU
     

//==== Memory ======================================================
     
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc),.MEM_ADDR2(aluResult),.MEM_DIN2(B),
                               .MEM_WRITE2(memWrite),.MEM_READ1(memRead1),.MEM_READ2(memRead2),
                               .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(IR[13:12]),.MEM_SIGN(IR[14]));
 
 
 
     
//==== Write Back ==================================================
     


 
 

       
            
endmodule
