/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();
int checkExc();

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

#define sext_imm5(x)  (((x) | ((x) >> 4) * 0xFFFFFFE0))
#define sext_off6(x)  (((x) | ((x) >> 5) * 0xFFFFFFC0))
#define sext_off9(x)  (((x) | ((x) >> 8) * 0xFFFFFE00)) 
#define sext_off11(x) (((x) | ((x) >> 9) * 0xFFFFF800)) 
#define sext_byte(x)  (((x) | ((x) >> 7) * 0xFF00)) //Automatically omits upper 16 bits

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    LD_SSP,
    LD_USP,
    LD_VEC,
    LD_PRIV,
    GATE_SP,
    GATE_VEC,
    GATE_PSR,
    GATE_PC2,
    COND2,
    DRMUX1,
    SR1MUX1,
    SPMUX1, SPMUX0,
    VECMUX,
    PSRMUX,
    CONTROL_STORE_BITS
} CS_BITS;

enum COND {
    UNCO,       //unconditional
    MEMR,       //memory ready
    BRAN,       //branch
    ADRM,       //addressing mode
    CINT        //check interrupt
};

enum ALUK {
    ADD,
    AND,
    XOR,
    PASSA
};

enum ADDR2MUX{
    ZERO,
    OFF6,
    OFF9,
    OFF11,
};

enum SPMUX{
    USP,
    SSP,
    SPp2,
    SPm2
};

enum TSB_EN{  //values for concatenation of tristate buffer enables
    NONE = 0,
    G_PC2 = 0x100,
    G_PSR = 0x80,
    G_VEC = 0x40,
    G_SP = 0x20,
    G_ALU = 0x10,
    G_MDR = 0x8,
    G_SHF = 0x4,
    G_MARMUX = 0x2,
    G_PC = 0x1
};

enum PCMUX{
    INCR_PC,
    BUS_PC,
    ADDER
};

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND2] << 2) + (x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }
int GetLD_SSP(int *x)        { return(x[LD_SSP]); }
int GetLD_USP(int *x)        { return(x[LD_USP]); }
int GetLD_VEC(int *x)        { return(x[LD_VEC]); }
int GetLD_PRIV(int *x)       { return(x[LD_PRIV]); }
int GetGATE_SP(int *x)       { return(x[GATE_SP]); }
int GetGATE_VEC(int *x)      { return(x[GATE_VEC]); }
int GetGATE_PSR(int *x)      { return(x[GATE_PSR]); }
int GetGATE_PC2(int *x)      { return(x[GATE_PC2]); }
int GetSPMUX(int *x)         { return((x[SPMUX1] << 1) + x[SPMUX0]); }
int GetVECMUX(int *x)        { return(x[VECMUX]); }
int GetPSRMUX(int *x)        { return(x[PSRMUX]); }
int GetSR1MUX1(int *x)       { return (x[SR1MUX1]); }
int GetDRMUX1(int *x)        { return (x[DRMUX1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. write0 is used for 
   the least significant byte of a word. write1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/
#define getUcode()  CONTROL_STORE[CURRENT_LATCHES.STATE_NUMBER]
#define setState(x) NEXT_LATCHES.STATE_NUMBER = (x)
#define getOpcode() ((CURRENT_LATCHES.IR & 0xF000) >> 12)

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */
int MEM_EN;
int MEM_CYC;

//Tristate buffer inputs
int GateALUIn;
int GateMDRIn;
int GateSHFIn;
int GateMARMuxIn;
int GatePCIn;
int GateSPIn;
int GateVECIn;
int GatePSRIn;
int GatePC2In;

int ADDEROut; //Address/PC calculation adder output
int MEMMUXOut;
int SR1MUXOut;
int INT;
int EXCV;

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit PSR[2]*/
    Z,		/* z condition bit PSR[1]*/
    P,		/* p condition bit PSR[0]*/
    PRIV,   /* privilege mode PSR[15]*/
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 

/* For lab 4 */
int VEC; /* Interrupt/Exception vector register */
int SSP; /* Initial value of system stack pointer */
int USP; /* User Stack Pointer */

} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                
  if(CYCLE_COUNT == 300){
      INT = 1; 
  }
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();
  eval_micro_sequencer();   

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%0.4x\n", BUS);
    printf("MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%0.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : init_memory                                     */
/*                                                             */
/* Purpose   : Zero out the memory array                       */
/*                                                             */
/***************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *argv[], int num_prog_files) { 
    int i;
    init_control_store(argv[1]);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(argv[i + 2]);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.PRIV = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv, argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code, except for the places indicated 
   with a "MODIFY:" comment.

   Do not modify the rdump and mdump functions.

   You are allowrited to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

/* 
* Evaluate the address of the next state according to the 
* micro sequencer logic. Latch the next microinstruction.
*/
void eval_micro_sequencer() {
    int nextState = -1;

    // 优先处理异常
    if (checkExc()) {
        nextState = 10;
    }
    // 如果 IRD 位被设置，使用操作码作为下一个状态
    else if (GetIRD(getUcode())) {
        nextState = getOpcode();
    }
    // 根据条件选择下一个状态
    else {
        int condition = GetCOND(getUcode());
        int j_bits = GetJ(getUcode());

        if (condition == MEMR) {
            // 内存就绪条件
            if (CURRENT_LATCHES.READY) {
                nextState = j_bits | (1 << 1); // READY 为 1
            } else {
                nextState = j_bits;
            }
        }
        else if (condition == BRAN) {
            // 分支条件
            if (CURRENT_LATCHES.BEN) {
                nextState = j_bits | (1 << 2); // BEN 为 1
            } else {
                nextState = j_bits;
            }
        }
        else if (condition == ADRM) {
            // 寻址模式条件
            int addressing_bit = (CURRENT_LATCHES.IR & 0x0800) >> 11;
            nextState = j_bits | addressing_bit;
        }
        else if (condition == CINT) {
            // 中断条件
            if (INT) {
                nextState = j_bits | (1 << 3); // INT 为 1
                INT = 0; // 清除中断标志
            } else {
                nextState = j_bits;
            }
        }
        else {
            // 默认情况
            nextState = j_bits;
        }
    }

    // 更新状态
    setState(nextState);
    memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[nextState], sizeof(int) * CONTROL_STORE_BITS);

    // 调试输出
    printf("Current State: %d -> Next State: %d\n", CURRENT_LATCHES.STATE_NUMBER, NEXT_LATCHES.STATE_NUMBER);
}

int checkExc() {
    int opcode = (CURRENT_LATCHES.IR & 0xF000) >> 12;
    int is_privileged = CURRENT_LATCHES.PRIV;
    int ld_mar = GetLD_MAR(getUcode());
    int bus_address = Low16bits(BUS);
    int data_size = GetDATA_SIZE(getUcode());
    int is_write = GetLD_MAR(getUcode());

    // 检查保护异常
    if (opcode != 0xF && is_privileged && ld_mar && bus_address < 0x3000) {
        EXCV = 0x02;
        return 1;
    }

    // 检查未对齐访问异常
    if (data_size && is_write && (bus_address & 0x1) && is_privileged) {
        EXCV = 0x03;
        return 1;
    }

    // 检查未知操作码异常
    int upper_bits = (CURRENT_LATCHES.IR >> 13) & 0x7; // 获取 IR 的最高3位
    if (upper_bits == 0b101) {
        EXCV = 0x04;
        return 1;
    }

    return 0;
}



  /* 
   * This function emulates memory and the write logic. 
   * Keep track of which cycle of MEMEN write are dealing with.  
   * If fourth, write need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */
void cycle_memory() {
    // 获取当前微指令中的 MIO_EN 控制位
    MEM_EN = GetMIO_EN(getUcode());

    // 更新内存访问周期计数器
    if (MEM_EN) {
        MEM_CYC += 1;
    } else {
        MEM_CYC = 0;
    }

    // 在第4个周期时，设置 READY 信号
    if (MEM_CYC == 4) {
        NEXT_LATCHES.READY = 1;
    }

    // 如果当前 READY 信号被置位，执行内存操作
    if (CURRENT_LATCHES.READY) {
        int is_write = GetR_W(getUcode());
        int data_size = GetDATA_SIZE(getUcode());
        int mar = CURRENT_LATCHES.MAR;
        int mdr = CURRENT_LATCHES.MDR;
        int address = mar >> 1;

        if (is_write) { // 写操作
            int write0 = 0, write1 = 0;

            if (data_size) { // 16位写
                write0 = 1;
                write1 = 1;
            }
            else { // 字节写
                if (mar & 0x1) { // 奇地址
                    write1 = 1;
                }
                else { // 偶地址
                    write0 = 1;
                }
            }

            // 写入内存
            if (write1) {
                MEMORY[address][1] = (mdr & 0xFF00) >> 8;
            }
            if (write0) {
                MEMORY[address][0] = mdr & 0x00FF;
            }
        }
        else { // 读操作
            MEMMUXOut = (MEMORY[address][1] << 8) | MEMORY[address][0];
        }

        // 重置内存周期计数器和 READY 信号
        MEM_CYC = 0;
        NEXT_LATCHES.READY = 0;
    }
}


  /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */    
void eval_bus_drivers() {
    int instr = CURRENT_LATCHES.IR;

    // 计算 GatePC 的输入
    GatePCIn = CURRENT_LATCHES.PC;
    GatePC2In = CURRENT_LATCHES.PC - 2;

    // 计算 GatePSR 的输入
    GatePSRIn = (CURRENT_LATCHES.PRIV << 15) | (CURRENT_LATCHES.N << 2) |
                (CURRENT_LATCHES.Z << 1) | CURRENT_LATCHES.P;

    // 计算 GateVEC 的输入
    GateVECIn = (0x02 << 8) | (CURRENT_LATCHES.VEC << 1);

    // 计算 GateMDR 的输入
    if (GetDATA_SIZE(getUcode())) {
        GateMDRIn = CURRENT_LATCHES.MDR;
    } else {
        // 字节访问，根据 MAR 的奇偶性选择高字节或低字节，并进行符号扩展
        if (CURRENT_LATCHES.MAR & 0x1) { // 奇地址
            GateMDRIn = sext_byte((CURRENT_LATCHES.MDR & 0xFF00) >> 8);
        } else { // 偶地址
            GateMDRIn = sext_byte(CURRENT_LATCHES.MDR & 0x00FF);
        }
    }

    // 计算 SR1MUXOut
    int sr1_mux = GetSR1MUX(getUcode());
    int sr1_mux1 = GetSR1MUX1(getUcode());
    int sr1 = sr1_mux ? ((instr & 0x01C0) >> 6) : ((instr & 0x0E00) >> 9);
    if (sr1_mux1) {
        sr1 = 6;
    }
    SR1MUXOut = CURRENT_LATCHES.REGS[sr1];

    // 计算 SR2MUXOut
    int sr2_mux = (instr & 0x0020) ? sext_imm5(instr & 0x001F) : CURRENT_LATCHES.REGS[instr & 0x0007];

    // 计算 GateALUIn
    int aluk = GetALUK(getUcode());
    if (aluk == ADD) {
        GateALUIn = Low16bits(SR1MUXOut + sr2_mux);
    } else if (aluk == AND) {
        GateALUIn = Low16bits(SR1MUXOut & sr2_mux);
    } else if (aluk == XOR) {
        GateALUIn = Low16bits(SR1MUXOut ^ sr2_mux);
    } else if (aluk == PASSA) {
        GateALUIn = Low16bits(SR1MUXOut);
    }

    // 计算 GateSHFIn
    GateSHFIn = SR1MUXOut;
    int shift_amount = instr & 0x000F;
    if (instr & 0x0010) { // 右移
        if (instr & 0x0020) { // 算术右移
            int sign = GateSHFIn & 0x8000;
            for (int i = 0; i < shift_amount; i++) {
                GateSHFIn = (sign ? 0x8000 : 0x0000) | (GateSHFIn >> 1);
            }
        } else { // 逻辑右移
            GateSHFIn >>= shift_amount;
        }
    } else { // 左移
        GateSHFIn <<= shift_amount;
    }

    // 计算 ADDEROut
    int addr1_mux = GetADDR1MUX(getUcode()) ? SR1MUXOut : CURRENT_LATCHES.PC;
    int addr2_mux_type = GetADDR2MUX(getUcode());
    int addr2_value = 0;

    if (addr2_mux_type == OFF11) {
        addr2_value = sext_off11(instr & 0x07FF);
    } else if (addr2_mux_type == OFF9) {
        addr2_value = sext_off9(instr & 0x01FF);
    } else if (addr2_mux_type == OFF6) {
        addr2_value = sext_off6(instr & 0x003F);
    } else if (addr2_mux_type == ZERO) {
        addr2_value = 0;
    }

    int lshf1 = GetLSHF1(getUcode()) ? (addr2_value << 1) : addr2_value;
    ADDEROut = Low16bits(lshf1 + addr1_mux);

    // 计算 GateMARMuxIn
    if (GetMARMUX(getUcode())) {
        GateMARMuxIn = ADDEROut;
    } else {
        GateMARMuxIn = Low16bits((instr & 0x00FF) << 1);
    }

    // 计算 GateSPIn
    int sp_mux = GetSPMUX(getUcode());
    if (sp_mux == USP) {
        GateSPIn = CURRENT_LATCHES.USP;
    } else if (sp_mux == SSP) {
        GateSPIn = CURRENT_LATCHES.SSP;
    } else if (sp_mux == SPp2) {
        GateSPIn = Low16bits(SR1MUXOut + 2);
    } else if (sp_mux == SPm2) {
        GateSPIn = Low16bits(SR1MUXOut - 2);
    } else {
        // 未定义的 SPMUX 类型，设置为 0 并打印错误信息
        GateSPIn = 0;
        printf("Error: Undefined SPMUX type.\n");
    }
}

  /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */
void drive_bus() {
    // 定义三态缓冲使能的宏
    #define TSB_NONE      0x000
    #define TSB_G_PC2     0x100
    #define TSB_G_PSR     0x080
    #define TSB_G_VEC     0x040
    #define TSB_G_SP      0x020
    #define TSB_G_ALU     0x010
    #define TSB_G_MDR     0x008
    #define TSB_G_SHF     0x004
    #define TSB_G_MARMUX  0x002
    #define TSB_G_PC      0x001

    // 计算三态缓冲使能的组合值
    int bus_enable = 0;
    if (GetGATE_PC2(getUcode())) {
        bus_enable |= TSB_G_PC2;
    }
    if (GetGATE_PSR(getUcode())) {
        bus_enable |= TSB_G_PSR;
    }
    if (GetGATE_VEC(getUcode())) {
        bus_enable |= TSB_G_VEC;
    }
    if (GetGATE_SP(getUcode())) {
        bus_enable |= TSB_G_SP;
    }
    if (GetGATE_ALU(getUcode())) {
        bus_enable |= TSB_G_ALU;
    }
    if (GetGATE_MDR(getUcode())) {
        bus_enable |= TSB_G_MDR;
    }
    if (GetGATE_SHF(getUcode())) {
        bus_enable |= TSB_G_SHF;
    }
    if (GetGATE_MARMUX(getUcode())) {
        bus_enable |= TSB_G_MARMUX;
    }
    if (GetGATE_PC(getUcode())) {
        bus_enable |= TSB_G_PC;
    }

    // 根据使能位设置总线值
    if (bus_enable == TSB_NONE) {
        BUS = 0;
    }
    else {
        int drivers = 0;
        if (bus_enable & TSB_G_PC2) {
            drivers++;
            BUS = Low16bits(GatePC2In);
        }
        if (bus_enable & TSB_G_PSR) {
            drivers++;
            BUS = Low16bits(GatePSRIn);
        }
        if (bus_enable & TSB_G_VEC) {
            drivers++;
            BUS = Low16bits(GateVECIn);
        }
        if (bus_enable & TSB_G_SP) {
            drivers++;
            BUS = Low16bits(GateSPIn);
        }
        if (bus_enable & TSB_G_ALU) {
            drivers++;
            BUS = Low16bits(GateALUIn);
        }
        if (bus_enable & TSB_G_MDR) {
            drivers++;
            BUS = Low16bits(GateMDRIn);
        }
        if (bus_enable & TSB_G_SHF) {
            drivers++;
            BUS = Low16bits(GateSHFIn);
        }
        if (bus_enable & TSB_G_MARMUX) {
            drivers++;
            BUS = Low16bits(GateMARMuxIn);
        }
        if (bus_enable & TSB_G_PC) {
            drivers++;
            BUS = Low16bits(GatePCIn);
        }

        if (drivers > 1) {
            printf("Error: Multiple bus drivers are active.\n");
            BUS = 0; 
        }
    }

    // 清除宏定义
    #undef TSB_NONE
    #undef TSB_G_PC2
    #undef TSB_G_PSR
    #undef TSB_G_VEC
    #undef TSB_G_SP
    #undef TSB_G_ALU
    #undef TSB_G_MDR
    #undef TSB_G_SHF
    #undef TSB_G_MARMUX
    #undef TSB_G_PC
}

  /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */       
void latch_datapath_values() {
    // 锁存 IR
    if (GetLD_IR(getUcode())) {
        NEXT_LATCHES.IR = BUS;
    }

    // 锁存条件码（CC）
    if (GetLD_CC(getUcode())) {
        if (GetPSRMUX(getUcode())) {
            NEXT_LATCHES.N = (BUS & 0x4) >> 2;
            NEXT_LATCHES.Z = (BUS & 0x2) >> 1;
            NEXT_LATCHES.P = BUS & 0x1;
        }
        else {
            // 根据 BUS 值设置条件码
            if (BUS == 0) {
                NEXT_LATCHES.N = 0;
                NEXT_LATCHES.Z = 1;
                NEXT_LATCHES.P = 0;
            }
            else if ((BUS & 0x8000) != 0) { // 负数
                NEXT_LATCHES.N = 1;
                NEXT_LATCHES.Z = 0;
                NEXT_LATCHES.P = 0;
            }
            else { // 正数
                NEXT_LATCHES.N = 0;
                NEXT_LATCHES.Z = 0;
                NEXT_LATCHES.P = 1;
            }
        }
    }

    // 锁存 PC
    if (GetLD_PC(getUcode())) {
        int pcmux_sel = GetPCMUX(getUcode());

        if (pcmux_sel == INCR_PC) {
            NEXT_LATCHES.PC = Low16bits(CURRENT_LATCHES.PC + 2);
        }
        else if (pcmux_sel == BUS_PC) {
            NEXT_LATCHES.PC = BUS;
        }
        else if (pcmux_sel == ADDER) {
            NEXT_LATCHES.PC = ADDEROut;
        }
        else {
            printf("Error: Undefined PCMUX selection.\n");
            NEXT_LATCHES.PC = 0;
        }
    }

    // 锁存 MAR
    if (GetLD_MAR(getUcode())) {
        NEXT_LATCHES.MAR = BUS;
    }

    // 锁存 MDR
    if (GetLD_MDR(getUcode())) {
        if (GetMIO_EN(getUcode())) {
            NEXT_LATCHES.MDR = Low16bits(MEMMUXOut);
        }
        else {
            if (CURRENT_LATCHES.MAR & 0x1) { // 奇地址
                NEXT_LATCHES.MDR = (BUS << 8) | (BUS & 0x00FF);
            }
            else { // 偶地址
                NEXT_LATCHES.MDR = Low16bits(BUS);
            }
        }
    }

    // 锁存 BEN
    if (GetLD_BEN(getUcode())) {
        int instr = CURRENT_LATCHES.IR;
        NEXT_LATCHES.BEN = ((CURRENT_LATCHES.N && (instr & 0x0800)) ||
                            (CURRENT_LATCHES.Z && (instr & 0x0400)) ||
                            (CURRENT_LATCHES.P && (instr & 0x0200)));
    }

    // 锁存寄存器
    if (GetLD_REG(getUcode())) {
        int dr_mux = GetDRMUX(getUcode());
        int dr_mux1 = GetDRMUX1(getUcode());
        int dr;

        if (dr_mux1) {
            dr = 6;
        }
        else {
            dr = (CURRENT_LATCHES.IR & 0x0E00) >> 9;
            if (dr_mux) {
                dr = 7;
            }
        }

        if (dr >= 0 && dr < LC_3b_REGS) {
            NEXT_LATCHES.REGS[dr] = BUS;
        }
        else {
            printf("Error: Invalid register number %d.\n", dr);
        }
    }

    // 锁存 PRIV
    if (GetLD_PRIV(getUcode())) {
        if (GetPSRMUX(getUcode())) {
            NEXT_LATCHES.PRIV = (BUS & 0x8000) >> 15;
        }
        else {
            NEXT_LATCHES.PRIV = 0;
        }
    }

    // 锁存 VEC
    if (GetLD_VEC(getUcode())) {
        if (GetVECMUX(getUcode())) {
            NEXT_LATCHES.VEC = EXCV;
        }
        else {
            NEXT_LATCHES.VEC = 0x01;
        }
    }

    // 锁存 USP
    if (GetLD_USP(getUcode())) {
        NEXT_LATCHES.USP = SR1MUXOut;
    }

    // 锁存 SSP
    if (GetLD_SSP(getUcode())) {
        NEXT_LATCHES.SSP = SR1MUXOut;
    }
}
