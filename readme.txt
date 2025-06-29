# Cycle-Accurate Pipeline Simulator

This project is a lightweight, cycle-accurate simulator for a generic 5-stage processor pipeline, written in C++. It is designed as a foundational framework for computer architecture research and education, allowing for the simulation of instruction execution on a cycle-by-cycle basis.

## Current State & Functionality

This is an early alpha version and serves as a bare-bones architectural framework.

-   **5-Stage Pipeline:** Implements a classic five-stage pipeline: Instruction Fetch (IF), Instruction Decode (ID), Execute (EXE), Memory Access (MEM), and Write-Back (WB).
-   **Modular Design:** Each pipeline stage is implemented as a separate C++ class, inheriting from an abstract `Module` base class.
-   **Instruction Loading:** The simulator loads machine code from an Altera-style Memory Initialization File (`.mif`).
-   **Sequential Execution:** It correctly simulates the flow of simple, sequential instructions and basic procedure blocks.

### Current Limitations

-   **No Hazard Handling:** The simulator does not currently implement data forwarding or handle data/control hazards.
-   **No Branch/Jump Logic:** The pipeline does not support branching or jumping instructions. The program counter (`PC`) simply increments.
-   **Testing:** This version has undergone minimal testing and is intended as a proof-of-concept.

## Dependencies

To compile and run this simulator, you will need:

-   A modern C++ compiler (C++17 or later)
-   The **{fmt}** library for formatted output.
-   The **Boost C++ Libraries** (specifically for `boost::circular_buffer`).

## How to Use & Configure

1.  **Set Instruction File Path:**
    To load instructions, you must manually change the hardcoded file path in `setup.hpp` at line 127.
    ```cpp
    // in setup.hpp
    void loadMem(std::vector<uint32_t>& memory, const std::string& filePath = "/path/to/your/instructions.mif") { ... }
    ```

2.  **Adjust Simulation Cycles:**
    To change the number of simulation cycles, modify the loop parameter in `main.cpp` at line 16.
    ```cpp
    // in main.cpp
    for (int cycle = 0; cycle < 10; ++cycle) { // Change '10' to the desired number of cycles
        // ...
    }
    ```

3.  **Debugging Stage Vectors:**
    For a more detailed view of the input and output vectors of each pipeline stage, you can add `stage[i]->printVectors();` within the main simulation loop in the `Pipeline::tick()` function.

## Building and Running

You can compile the project using a C++ compiler like g++. Make sure to link the `{fmt}` library.

```bash
# Example compilation command
g++ main.cpp -o simulator

# Run the simulator
./simulator
