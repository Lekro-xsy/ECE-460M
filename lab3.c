
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONTROL_STORE_ROWS 64
#define CONTROL_STORE_BITS 35

// Global variables for simulation
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];
int MEMORY[0xFFFF];  // Simulating memory space
int MEMORY_READY = 0;
int MEMORY_CYCLE_COUNT = 0;
int BUS = 0;

// Structures to hold the current and next state
struct {
    int STATE_NUMBER;
    int MICROINSTRUCTION[CONTROL_STORE_BITS];
    int PC, MDR, MAR, IR, N, Z, P;
    int REGS[8];
    int BEN;
} CURRENT_LATCHES, NEXT_LATCHES;

// Function to load microcode from file
void load_microcode(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Error: Unable to open microcode file: %s\n", filename);
        exit(1);
    }
    char line[36];
    int state = 0;
    while (fgets(line, sizeof(line), file)) {
        for (int i = 0; i < 35; i++) {
            CONTROL_STORE[state][i] = line[i] - '0';
        }
        state++;
    }
    fclose(file);
}

// State transition function
void eval_micro_sequencer() {
    int next_state;
    if (CURRENT_LATCHES.MICROINSTRUCTION[0] == 1) {  // IRD bit
        next_state = (CURRENT_LATCHES.IR & 0xF000) >> 12;
    } else {
        int j = CURRENT_LATCHES.MICROINSTRUCTION[1];
        int cond = CURRENT_LATCHES.MICROINSTRUCTION[2];
        int ben = CURRENT_LATCHES.BEN;
        int ready = MEMORY_READY;

        switch (cond) {
            case 0: next_state = j; break;
            case 1: next_state = ready ? j : CURRENT_LATCHES.STATE_NUMBER; break;
            case 2: next_state = ben ? j : CURRENT_LATCHES.STATE_NUMBER; break;
            default: next_state = CURRENT_LATCHES.STATE_NUMBER;
        }
    }
    NEXT_LATCHES.STATE_NUMBER = next_state;
}

// Memory cycle simulation
void cycle_memory() {
    if (CURRENT_LATCHES.MICROINSTRUCTION[3] == 1) {  // MIO.EN bit
        if (MEMORY_CYCLE_COUNT < 4) {
            MEMORY_CYCLE_COUNT++;
        } else {
            MEMORY_READY = 1;
            int mar = CURRENT_LATCHES.MAR >> 1;
            if (CURRENT_LATCHES.MICROINSTRUCTION[4] == 0) {  // R/W bit
                NEXT_LATCHES.MDR = MEMORY[mar];
            } else {
                MEMORY[mar] = CURRENT_LATCHES.MDR;
            }
        }
    } else {
        MEMORY_CYCLE_COUNT = 0;
        MEMORY_READY = 0;
    }
}

// Bus driver functions
void eval_bus_drivers() {
    BUS = 0;
    if (CURRENT_LATCHES.MICROINSTRUCTION[5]) BUS = CURRENT_LATCHES.PC;
    if (CURRENT_LATCHES.MICROINSTRUCTION[6]) BUS = CURRENT_LATCHES.MDR;
    if (CURRENT_LATCHES.MICROINSTRUCTION[7]) BUS = CURRENT_LATCHES.REGS[0];  // Example for ALU result
}

// Drive bus and latch values
void drive_bus() {
    if (CURRENT_LATCHES.MICROINSTRUCTION[8]) NEXT_LATCHES.MAR = BUS;
    if (CURRENT_LATCHES.MICROINSTRUCTION[9]) NEXT_LATCHES.MDR = BUS;
    if (CURRENT_LATCHES.MICROINSTRUCTION[10]) NEXT_LATCHES.PC = BUS;
    if (CURRENT_LATCHES.MICROINSTRUCTION[11]) {
        int dr = (CURRENT_LATCHES.IR & 0x0E00) >> 9;
        NEXT_LATCHES.REGS[dr] = BUS;
    }
}

void latch_datapath_values() {
    if (CURRENT_LATCHES.MICROINSTRUCTION[12]) {  // LD.CC
        NEXT_LATCHES.N = (BUS & 0x8000) ? 1 : 0;
        NEXT_LATCHES.Z = (BUS == 0) ? 1 : 0;
        NEXT_LATCHES.P = (!NEXT_LATCHES.N && !NEXT_LATCHES.Z) ? 1 : 0;
    }
    if (CURRENT_LATCHES.MICROINSTRUCTION[13]) {  // LD.IR
        NEXT_LATCHES.IR = BUS;
    }
}

// Shell command processing
void process_command() {
    char command[20];
    int cycles, low, high;
    printf("LC-3b Simulator> ");
    scanf("%s", command);

    if (strcmp(command, "go") == 0) {
        // Implement run until HALT
    } else if (strcmp(command, "run") == 0) {
        scanf("%d", &cycles);
        // Implement run for cycles
    } else if (strcmp(command, "mdump") == 0) {
        scanf("%x %x", &low, &high);
        // Implement memory dump
    } else if (strcmp(command, "rdump") == 0) {
        // Implement register dump
    } else if (strcmp(command, "quit") == 0) {
        exit(0);
    } else {
        printf("Invalid command.\n");
    }
}

// Main function
int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <microcode_file>\n", argv[0]);
        return 1;
    }
    load_microcode(argv[1]);
    while (1) {
        process_command();
    }
    return 0;
}
