/*
    Name 1: Xin Siyang 
    UTEID 1: xsy59
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N - Lab 5                                           */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         pagetable    page table in LC-3b machine language   */
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

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
//#define Low16bits(x) ((x) & 0xFFFF)

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
    COND2, COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    LD_SSP,
    LD_USP,
    LD_PSR,
    LD_PSRM,
    LD_VA,
    LD_MDRR,
    LD_MDRM,
    LD_TMPPSR,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    GATE_SSP,
    GATE_SPS,
    GATE_VEC,
    GATE_PSR,
    GATE_DPC,
    GATE_PTBR,
    GATE_VA,
    GATE_TMPPSR,
    PCMUX1, PCMUX0,
    DRMUX1,DRMUX0,
    SR1MUX1,SR1MUX0,
    RMUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    USPMUX,
    VECMUX,
    CCMUX,
    LD_VEC,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    J_Gate,
    CONTROL_STORE_BITS,
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND2] << 2)+(x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5]<<5) + (x[J4]<<4) + (x[J3]<<3) + (x[J2]<<2) + (x[J1]<<1) + x[J0]);}
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetLD_SSP(int *x)        { return(x[LD_SSP]); }
int GetLD_USP(int *x)        { return(x[LD_USP]); }
int GetLD_PSR(int *x)        { return(x[LD_PSR]); }
int GetLD_PSRM(int *x)       { return(x[LD_PSRM]); }
int GetLD_VA(int *x)         { return(x[LD_VA]); }
int GetLD_MDRR(int *x)       { return(x[LD_MDRR]); }
int GetLD_MDRM(int *x)       { return(x[LD_MDRM]); }
int GetLD_TMPPSR(int *x)       { return(x[LD_TMPPSR]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetGATE_SSP(int *x)      { return(x[GATE_SSP]); }
int GetGATE_SPS(int *x)      { return(x[GATE_SPS]); }
int GetGATE_VEC(int *x)      { return(x[GATE_VEC]); }
int GetGATE_PSR(int *x)      { return(x[GATE_PSR]); }
int GetGATE_PTBR(int *x)     { return(x[GATE_PTBR]); }
int GetGATE_VA(int *x)       { return(x[GATE_VA]); }
int GetGATE_TMPPSR(int *x)       { return(x[GATE_TMPPSR]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX0]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX0]); }
int GetDRMUX1(int *x)        { return(x[DRMUX1]); }
int GetSR1MUX1(int *x)       { return(x[SR1MUX1]); }
int GetRMUX(int *x)          { return(x[RMUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetUSPMUX(int *x)        { return(x[USPMUX]);}
int GetCCMUX(int* x)         { return(x[CCMUX]);}
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); }
int GetLSHF1(int *x)         { return(x[LSHF1]); }
int GetGATE_DPC(int* x)      { return(x[GATE_DPC]); }
int GetVECMUX(int *x)        { return(x[VECMUX]); }
int GetLD_VEC(int *x)        { return(x[LD_VEC]); }
int GetJGate(int *x)         { return(x[J_Gate]); }
int16_t getSR1(int* cStore);
int16_t getSR2();
int16_t getADDR1(int* cStore) ;
int16_t getADDR2(int* cStore);
int16_t evalADDR(int* cStore);
int16_t evalMARMUX(int* cStore);
int16_t evalALU(int* cStore);
int16_t latchPC(int* cStore);
int16_t latchCC(int* cStore);
int16_t latchREG(int* cStore);
int16_t latchBEN(int* cStore);
int16_t latchIR(int* cStore);
int16_t latchMAR(int* cStore);
int16_t latchMDR(int* cStore);
int16_t latchPSR(int* cStore);
int16_t latchSSP(int* cStore);
int16_t latchUSP(int* cStore);
int16_t latchVEC(int* cStore);
int16_t latchVA(int* cStore);
int16_t latchTMPPSR(int* cStore);
int16_t evalShift(int* cStore);
int16_t evalMDR(int* cStore);
int16_t evalVECMUX(int* cStore);
int16_t evalSPMUX(int* cStore);
/* Grabbing Bits*/
int16_t getBits(int reg, int numBits, int start, int sext);

int16_t checkException(int* cStore);


/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x2000 /* 32 frames */ 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

    int PC,		/* program counter */
    PSR,
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */
    int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/
    int REGS[LC_3b_REGS]; /* register file. */
    int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */
    int STATE_NUMBER; /* Current State Number - Provided for debugging */
/* For lab 4 */
    int INTV; /* Interrupt vector register */
    int EXCV; /* Exception vector register */
    int SSP; /* Initial value of system stack pointer */
    int INTS;
    int VEC;
    int USP;
/* For lab 5 */
    int PTBR; /* This is initialized when we load the page table */
    int VA;   /* Temporary VA register */
    int JSave;
    int TMPPSR;
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/* For lab 5 */
#define PAGE_NUM_BITS 9
#define PTE_PFN_MASK 0x3E00
#define PTE_VALID_MASK 0x0004
#define PAGE_OFFSET_MASK 0x1FF

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

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

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
void load_program(char *program_filename, int is_virtual_base) {                   
    FILE * prog;
    int ii, word, program_base, pte, virtual_pc;

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

    if (is_virtual_base) {
      if (CURRENT_LATCHES.PTBR == 0) {
	printf("Error: Page table base not loaded %s\n", program_filename);
	exit(-1);
      }

      /* convert virtual_base to physical_base */
      virtual_pc = program_base << 1;
      pte = (MEMORY[(CURRENT_LATCHES.PTBR + (((program_base << 1) >> PAGE_NUM_BITS) << 1)) >> 1][1] << 8) | 
	     MEMORY[(CURRENT_LATCHES.PTBR + (((program_base << 1) >> PAGE_NUM_BITS) << 1)) >> 1][0];

      printf("virtual base of program: %04x\npte: %04x\n", program_base << 1, pte);
		if ((pte & PTE_VALID_MASK) == PTE_VALID_MASK) {
	      program_base = (pte & PTE_PFN_MASK) | ((program_base << 1) & PAGE_OFFSET_MASK);
   	   printf("physical base of program: %x\n\n", program_base);
	      program_base = program_base >> 1; 
		} else {
   	   printf("attempting to load a program into an invalid (non-resident) page\n\n");
			exit(-1);
		}
    }
    else {
      /* is page table */
     CURRENT_LATCHES.PTBR = program_base << 1;
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
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0 && is_virtual_base) 
      CURRENT_LATCHES.PC = virtual_pc;

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine         */
/*                                                             */
/***************************************************************/
void initialize(char *argv[], int num_prog_files) { 
    int i;
    init_control_store(argv[1]);

    init_memory();
    load_program(argv[2],0);
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(argv[i + 3],1);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */

    CURRENT_LATCHES.INTS = 0;
    CURRENT_LATCHES.PSR = 1<<15;
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
    if (argc < 4) {
	printf("Error: usage: %s <micro_code_file> <page table file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv, argc - 3);

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
   You are allowed to use the following global variables in your
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



void eval_micro_sequencer() {
    int currState = CURRENT_LATCHES.STATE_NUMBER;
    int outState = GetJ(CONTROL_STORE[currState]);
    int conds = GetCOND(CONTROL_STORE[currState]);

    if (conds == 0) {
    } else if (conds == 1) {
        outState |= ((CURRENT_LATCHES.READY != 0) << 1);
    } else if (conds == 2) {
        outState |= ((CURRENT_LATCHES.BEN & 1) << 2);
    } else if (conds == 3) {
        outState |= ((CURRENT_LATCHES.IR >> 11) & 1);
    } else if (conds == 6) {
        outState |= CURRENT_LATCHES.INTS << 4;
        if (CURRENT_LATCHES.INTS) NEXT_LATCHES.INTS = 0;
    } else if (conds == 7) {
        outState |= ((CURRENT_LATCHES.PSR & 0x8000) >> 12);
    } else {
        printf("microsequencer error");
    }

    NEXT_LATCHES.STATE_NUMBER = outState;

    int condition = 0;
    if (GetLD_VA(CONTROL_STORE[currState])) {
        condition = 1;
    } else if (checkException(CONTROL_STORE[currState])) {
        condition = 2;
    } else if (GetIRD(CONTROL_STORE[currState])) {
        condition = 3;
    } else if (GetJGate(CONTROL_STORE[currState])) {
        condition = 4;
    }

    switch (condition) {
        case 1:
            NEXT_LATCHES.STATE_NUMBER = 44;
            NEXT_LATCHES.JSave = outState;
            break;
        case 2:
            NEXT_LATCHES.STATE_NUMBER = 11;
            break;
        case 3:
            NEXT_LATCHES.STATE_NUMBER = ((uint8_t)(CURRENT_LATCHES.IR >> 12)) & 0xF;
            break;
        case 4:
            NEXT_LATCHES.STATE_NUMBER = CURRENT_LATCHES.JSave;
            break;
        default:
            // 保持 NEXT_LATCHES.STATE_NUMBER 不变
            break;
    }
}

int16_t checkException(int* cStore){
    int exceptionType = 0;
    // 简化条件判断，确定异常类型
    if (((CURRENT_LATCHES.IR & 0xF000) >> 13) == 5) {
        exceptionType = 5; // 未知操作码
    } else if (NEXT_LATCHES.STATE_NUMBER == 45) {
        if ((CURRENT_LATCHES.MAR % 2) && (CURRENT_LATCHES.JSave != 24) && (CURRENT_LATCHES.JSave != 29)) {
            exceptionType = 3; // 未对齐的访问
        }
    } else if (NEXT_LATCHES.STATE_NUMBER == 48) {
        if (!(CURRENT_LATCHES.MDR & 0x8) && ((CURRENT_LATCHES.IR & 0xF000) != 0xF000) && (CURRENT_LATCHES.PSR & 0x8000)) {
            exceptionType = 4; // 保护异常
        } else if (!(CURRENT_LATCHES.MDR & 0x4)) {
            exceptionType = 2; // 页面缺失
        }
    }
    // 使用 switch-case 结构处理异常类型
    switch (exceptionType) {
        case 2:
            NEXT_LATCHES.EXCV = 2;
            return 1;
        case 3:
            NEXT_LATCHES.EXCV = 3;
            return 1;
        case 4:
            NEXT_LATCHES.EXCV = 4;
            return 1;
        case 5:
            NEXT_LATCHES.EXCV = 5;
            return 0;
        default:
            return 0;
    }
}

/*
 * This function emulates memory and the WE logic.
 * Keep track of which cycle of MEMEN we are dealing with.
 * If fourth, we need to latch Ready bit at the end of
 * cycle to prepare microsequencer for the fifth cycle.
 */
int memCycleCount = 1;

void cycle_memory() {
    int* cStore = CONTROL_STORE[CURRENT_LATCHES.STATE_NUMBER];

    if (CYCLE_COUNT == 299) {
        NEXT_LATCHES.INTS = 1;
        NEXT_LATCHES.INTV = 1;
    }

    switch (GetMIO_EN(cStore)) {
        case 0:
            return;
        default:
            break;
    }

    switch (((memCycleCount + 1) % MEM_CYCLES) == 0) {
        case 1:
            NEXT_LATCHES.READY = 1;
            break;
        default:
            break;
    }

    memCycleCount = (memCycleCount + 1) % MEM_CYCLES;

    switch (CURRENT_LATCHES.READY) {
        case 0:
            return;
        default:
            break;
    }

    switch (GetR_W(cStore)) {
        case 1:  // 写操作
            NEXT_LATCHES.READY = 0;
            memCycleCount = 1;

            // 将内部的 if-else 结构也改为 switch-case
            switch (GetDATA_SIZE(cStore)) {
                case 1:  // 写字
                    MEMORY[CURRENT_LATCHES.MAR / 2][1] = (CURRENT_LATCHES.MDR >> 8) & 0xFF;
                    MEMORY[CURRENT_LATCHES.MAR / 2][0] = CURRENT_LATCHES.MDR & 0xFF;
                    break;
                case 0:  // 写字节
                    MEMORY[CURRENT_LATCHES.MAR / 2][CURRENT_LATCHES.MAR % 2] = CURRENT_LATCHES.MDR & 0xFF;
                    break;
            }
            break;

        case 0:  // 读操作
            NEXT_LATCHES.READY = 0;
            memCycleCount = 1;
            NEXT_LATCHES.MDR = (MEMORY[CURRENT_LATCHES.MAR / 2][0] & 0xFF) | ((MEMORY[CURRENT_LATCHES.MAR / 2][1] & 0xFF) << 8);
            break;
    }
}

/*
 * Datapath routine emulating operations before driving the bus.
 * Evaluate the input of tristate drivers
 *             Gate_MARMUX,
 *       Gate_PC,
 *       Gate_ALU,
 *       Gate_SHF,
 *       Gate_MDR.
 */
void eval_bus_drivers() {
    int* cStore = CONTROL_STORE[CURRENT_LATCHES.STATE_NUMBER];
    int16_t gates = 0;
    gates |=  GetGATE_ALU(cStore) != 0;
    gates |= (GetGATE_PC (cStore) != 0)    << 1;
    gates |= (GetGATE_MDR(cStore) != 0)    << 2;
    gates |= (GetGATE_MARMUX(cStore) != 0) << 3;
    gates |= (GetGATE_SHF(cStore) != 0)    << 4;
    gates |= (GetGATE_PSR(cStore) != 0)    << 5;
    gates |= (GetGATE_SPS(cStore) != 0)    << 6;
    gates |= (GetGATE_SSP(cStore) != 0)    << 7;
    gates |= (GetGATE_VEC(cStore)!= 0)     << 8;
    gates |= (GetGATE_DPC(cStore) != 0)    << 9;
    gates |= (GetGATE_PTBR(cStore)!=0)     <<10;
    gates |= (GetGATE_VA(cStore)!=0)       <<11;
    gates |= (GetGATE_TMPPSR(cStore)!=0)   <<12;

    if (gates == 0x0) {
        BUS = 0;
    } else if (gates == 0x1) {
        evalALU(cStore);
    } else if (gates == 0x2) {
        BUS = CURRENT_LATCHES.PC & 0xFFFF;
    } else if (gates == 0x4) {
        evalMDR(cStore);
    } else if (gates == 0x8) {
        evalMARMUX(cStore);
    } else if (gates == 0x10) {
        evalShift(cStore);
    } else if (gates == 0x20) {
        BUS = CURRENT_LATCHES.PSR;
    } else if (gates == 0x40) {
        evalSPMUX(cStore);
    } else if (gates == 0x80) {
        BUS = (CURRENT_LATCHES.SSP - 2);
    } else if (gates == 0x100) {
        evalVECMUX(cStore);
    } else if (gates == 0x200) {
        BUS = CURRENT_LATCHES.PC - 2;
    } else if (gates == 0x400) {
        BUS = (CURRENT_LATCHES.PTBR & 0xFF00) + ((CURRENT_LATCHES.MAR & 0xFE00) >> 8);
    } else if (gates == 0x800) {
        BUS = (CURRENT_LATCHES.MDR & 0x3E00) + (CURRENT_LATCHES.VA & 0x00FF);
    } else if (gates == 0x1000) {
        BUS = CURRENT_LATCHES.TMPPSR;
    } else {
        printf("gate error");
    }
    BUS = BUS & 0xFFFF;
}

/*
 * Datapath routine for driving the bus from one of the 5 possible
 * tristate drivers.
 */
void drive_bus() {}

/*
 * Datapath routine for computing all functions that need to latch
 * values in the data path at the end of this cycle.  Some values
 * require sourcing the bus; therefore, this routine has to come
 * after drive_bus.
 */
void latch_datapath_values() {
    int* cStore = CONTROL_STORE[CURRENT_LATCHES.STATE_NUMBER];
    latchMAR(cStore);
    latchMDR(cStore);
    latchIR(cStore);
    latchBEN(cStore);
    latchREG(cStore);
    latchCC(cStore);
    latchPC(cStore);
    latchPSR(cStore);
    latchSSP(cStore);
    latchUSP(cStore);
    latchVEC(cStore);
    latchVA(cStore);
    latchTMPPSR(cStore);
}

/* Latching Nodes */
int16_t latchPC(int* cStore){
    if(GetLD_PC(cStore)){
        int pcMUX = GetPCMUX(cStore);
        if (pcMUX == 0) {
            NEXT_LATCHES.PC = CURRENT_LATCHES.PC + 2;
        } else if (pcMUX == 1) {
            NEXT_LATCHES.PC = BUS & 0xFFFF;
        } else if (pcMUX == 2) {
            NEXT_LATCHES.PC = evalADDR(cStore) & 0xFFFF;
        } else {
            printf("pc latch issue");
        }
    }
}

int16_t latchCC(int* cStore){
    if(GetLD_CC(cStore)){
        if(!GetCCMUX(cStore)) {
            NEXT_LATCHES.PSR &= 0xFFF8;
            int condition = 0;
            if (BUS & (1 << 15)) {
                condition = 1;
            } else if (BUS == 0) {
                condition = 2;
            } else {
                condition = 3;
            }
            switch (condition) {
                case 1:
                    NEXT_LATCHES.N = 1;
                    NEXT_LATCHES.Z = 0;
                    NEXT_LATCHES.P = 0;
                    NEXT_LATCHES.PSR += 4;
                    break;
                case 2:
                    NEXT_LATCHES.N = 0;
                    NEXT_LATCHES.Z = 1;
                    NEXT_LATCHES.P = 0;
                    NEXT_LATCHES.PSR += 2;
                    break;
                case 3:
                    NEXT_LATCHES.N = 0;
                    NEXT_LATCHES.Z = 0;
                    NEXT_LATCHES.P = 1;
                    NEXT_LATCHES.PSR += 1;
                    break;
                default:
                    // 不做任何操作
                    break;
            }
        } else {
            NEXT_LATCHES.N = BUS & 0x4;
            NEXT_LATCHES.Z = BUS & 0x2;
            NEXT_LATCHES.P = BUS & 0x1;
        }
    }
}

int16_t latchREG(int* cStore){
    if(GetLD_REG(cStore)){
        int DR;
        int drmux_value = 0;
        if (GetDRMUX(cStore) == 1) {
            drmux_value = 1;
        } else if (GetDRMUX1(cStore) == 1) {
            drmux_value = 2;
        } else {
            drmux_value = 0;
        }
        switch (drmux_value) {
            case 1:
                DR = 7;
                break;
            case 2:
                DR = 6;
                break;
            default:
                DR = getBits(CURRENT_LATCHES.IR, 3, 9, 0);
                break;
        }
        int regIn;
        if(GetRMUX(cStore)) {
            regIn = CURRENT_LATCHES.USP;
        } else {
            regIn = BUS;
        }
        NEXT_LATCHES.REGS[DR] = regIn & 0xFFFF;
    }
}

int16_t latchBEN(int* cStore){
    if(GetLD_BEN(cStore)){
        NEXT_LATCHES.BEN = ((CURRENT_LATCHES.N && (CURRENT_LATCHES.IR & (1 << 11))) ||
                            (CURRENT_LATCHES.Z && (CURRENT_LATCHES.IR & (1 << 10))) ||
                            (CURRENT_LATCHES.P && (CURRENT_LATCHES.IR & (1 << 9))));
    }
}

int16_t latchVA(int* cStore){
    if(GetLD_VA(cStore)){
        NEXT_LATCHES.VA = BUS & 0xFFFF;
    }
}

int16_t latchIR(int* cStore){
    if(GetLD_IR(cStore)){
        NEXT_LATCHES.IR = BUS & 0xFFFF;
    }
}

int16_t latchMAR(int* cStore){
    if(GetLD_MAR(cStore)){
        NEXT_LATCHES.MAR = BUS & 0xFFFF;
    }
}

int16_t latchMDR(int* cStore){
    int MDRMUX = GetMIO_EN(cStore);
    int wordLoad = GetDATA_SIZE(CONTROL_STORE[CURRENT_LATCHES.STATE_NUMBER]);
    if(GetLD_MDR(cStore) && (MDRMUX == 0)){
        switch (wordLoad) {
            case 1:
                NEXT_LATCHES.MDR = BUS & 0xFFFF;
                break;
            case 0:
                NEXT_LATCHES.MDR = (BUS & 0x00FF) | ((BUS & 0x00FF) << 8);
                break;
        }
    }
    if(GetLD_MDRR(cStore)){
        NEXT_LATCHES.MDR |= 0x1;
    }
    if(GetLD_MDRM(cStore)){
        NEXT_LATCHES.MDR &= ~0x2;
        int tmp = (CURRENT_LATCHES.IR & 0xF000) >> 12;
        int condition = 0;
        if (CURRENT_LATCHES.JSave == 61 || CURRENT_LATCHES.JSave == 56 ||
            CURRENT_LATCHES.JSave == 23 || CURRENT_LATCHES.JSave == 24) {
            condition = 1;
        }
        switch (condition) {
            case 1:
                NEXT_LATCHES.MDR |= 0x2;
                break;
            default:
                // 不做任何操作
                break;
        }
    }
}

int16_t latchPSR(int* cStore){
    if(GetLD_PSR(cStore)){
        NEXT_LATCHES.PSR = BUS & 0xFFFF;
    }
    if(GetLD_PSRM(cStore)){
        NEXT_LATCHES.PSR &= 0x7FFF;
    }
}

int16_t latchSSP(int* cStore){
    if(GetLD_SSP(cStore)){
        NEXT_LATCHES.SSP = BUS & 0xFFFF;
    }
}

int16_t latchUSP(int* cStore){
    if(GetLD_USP(cStore)){
        NEXT_LATCHES.USP = CURRENT_LATCHES.REGS[6];
    }
}

int16_t latchVEC(int* cStore){
    if(GetLD_VEC(cStore)){
        switch (GetVECMUX(cStore)) {
            case 1:
                NEXT_LATCHES.VEC = CURRENT_LATCHES.EXCV;
                break;
            default:
                NEXT_LATCHES.VEC = CURRENT_LATCHES.INTV;
                break;
        }
    }
}

int16_t latchTMPPSR(int* cStore){
    if(GetLD_TMPPSR(cStore)){
        NEXT_LATCHES.TMPPSR = BUS;
    }
}

/* Evaluating BUS Pushes */
int16_t evalALU(int* cStore){
    if(!GetGATE_ALU(cStore)){
        return 0;
    }
    int16_t arg1 = getSR1(cStore);
    int16_t arg2 = getSR2();
    int out = 0;
    int aluk = GetALUK(cStore);
    if (aluk == 0) {
        out = (arg1 + arg2) & 0xFFFF;
    } else if (aluk == 1) {
        out = (arg1 & arg2) & 0xFFFF;
    } else if (aluk == 2) {
        out = (arg1 ^ arg2) & 0xFFFF;
    } else if (aluk == 3) {
        out = arg1 & 0xFFFF;
    } else {
        printf("alu error");
    }
    BUS = out & 0xFFFF;
}

int16_t evalMARMUX(int* cStore){
    if(GetGATE_MARMUX(cStore)){
        int marmux = GetMARMUX(cStore);
        switch (marmux) {
            case 0:
                BUS = (getBits(CURRENT_LATCHES.IR, 8, 0, 0) << 1) & 0xFFFF;
                break;
            case 1:
                BUS = evalADDR(cStore) & 0xFFFF;
                break;
        }
    }
}

int16_t evalShift(int* cStore){
    uint8_t rY = (uint8_t)getBits(CURRENT_LATCHES.IR, 4, 0, 0);
    int16_t rX = getSR1(cStore), out;
    int shift_type = 0;
    if (!(CURRENT_LATCHES.IR & 16)) {
        shift_type = 0; // 左移
    } else {
        if (!(CURRENT_LATCHES.IR & 32)) {
            shift_type = 1; // 逻辑右移
        } else {
            shift_type = 2; // 算术右移
        }
    }
    switch (shift_type) {
        case 0:
            out = rX << rY;
            break;
        case 1:
            out = ((uint16_t)rX) >> rY;
            break;
        case 2:
            out = ((int16_t)rX) >> rY;
            break;
        default:
            // 不做任何操作
            break;
    }
    if(GetGATE_SHF(cStore)){
        BUS = out & 0xFFFF;
    }
}

int16_t evalMDR(int* cStore){
    if(!GetGATE_MDR(cStore)){
        return 0;
    }
    int data_size = GetDATA_SIZE(cStore);
    switch (data_size) {
        case 1:
            BUS = CURRENT_LATCHES.MDR & 0xFFFF;
            break;
        case 0:
            if (CURRENT_LATCHES.MAR % 2) {
                BUS = (CURRENT_LATCHES.MDR & 0xFF00) >> 8;
            } else {
                BUS = CURRENT_LATCHES.MDR & 0x00FF;
            }
            if (BUS & (1 << 7)) {
                BUS |= 0xFF00;
            }
            break;
    }
}

int16_t evalSPMUX(int* cStore){
    if(GetUSPMUX(cStore)){
        BUS = (CURRENT_LATCHES.REGS[6] - 2) & 0xFFFF;
    } else {
        BUS = (CURRENT_LATCHES.REGS[6] + 2) & 0xFFFF;
    }
}

int16_t evalVECMUX(int* cStore){
    if(GetGATE_VEC(cStore)){
        BUS = (0x0200 + 2 * CURRENT_LATCHES.VEC) & 0xFFFF;
    }
}

/* Address Adder Helpers */
int16_t evalADDR(int* cStore){
    return getADDR1(cStore) + getADDR2(cStore);
}

int16_t getADDR1(int* cStore) {
    switch (GetADDR1MUX(cStore)) {
        case 0:
            return (int16_t)CURRENT_LATCHES.PC;
        case 1:
            return getSR1(cStore);
        default:
            printf("ADDR1MUX issue");
            return 0;
    }
}

int16_t getADDR2(int* cStore){
    int addr2mux = GetADDR2MUX(cStore);
    if (addr2mux == 0) {
        return 0;
    } else if (addr2mux == 1) {
        return getBits(CURRENT_LATCHES.IR, 6, 0, 1) << GetLSHF1(cStore);
    } else if (addr2mux == 2) {
        return getBits(CURRENT_LATCHES.IR, 9, 0, 1) << GetLSHF1(cStore);
    } else if (addr2mux == 3) {
        return getBits(CURRENT_LATCHES.IR, 11, 0, 1) << GetLSHF1(cStore);
    } else {
        printf("addr2 issue");
        return 0;
    }
}

/* Grabbing Arguments */
int16_t getSR1(int* cStore){
    int sr1mux1 = GetSR1MUX1(cStore);
    int sr1mux0 = GetSR1MUX(cStore);
    int sr1;
    switch (sr1mux1) {
        case 1:
            sr1 = CURRENT_LATCHES.REGS[6];
            break;
        default:
            switch (sr1mux0) {
                case 0:
                    sr1 = (int16_t)CURRENT_LATCHES.REGS[getBits(CURRENT_LATCHES.IR, 3, 9, 0)];
                    break;
                case 1:
                    sr1 = (int16_t)CURRENT_LATCHES.REGS[getBits(CURRENT_LATCHES.IR, 3, 6, 0)];
                    break;
                default:
                    printf("SR1MUX issue");
                    sr1 = 0;
                    break;
            }
            break;
    }
    return sr1;
}

int16_t getSR2(){
    int SR2MUX = (CURRENT_LATCHES.IR >> 5) & 1;
    switch (SR2MUX) {
        case 0:
            return (int16_t)CURRENT_LATCHES.REGS[getBits(CURRENT_LATCHES.IR, 3, 0, 0)];
        case 1:
            return (int16_t)getBits(CURRENT_LATCHES.IR, 5, 0, 1);
        default:
            printf("SR2MUX issue");
            return 0;
    }
}

int16_t getBits(int reg, int numBits, int start, int sext){
    int16_t out = 0;
    reg=reg>>start;
    for(int i = 0; i<numBits; i++) out += reg&(1<<i);
    if(!sext) return (int16_t) (out & 0xFFFF);
    for(int i = numBits; i<16; i++) out |= (((out&(1<<(numBits-1)))!=0)<<i);
    return (int16_t) (out & 0xFFFF);
}
