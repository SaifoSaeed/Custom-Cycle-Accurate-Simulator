#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include <boost/circular_buffer.hpp>
#include <fstream>
#include <fmt/format.h>
#include <fmt/core.h>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <stdlib.h>

//Abstract Module Class.
class Module {
protected:
    std::vector<uint32_t> inputs;
    std::vector<uint32_t> outputs;

public:
    virtual void tick() = 0;                //Executes one clock cycle.
    virtual std::string name() const = 0;   // Returns the module's name

    virtual void reset() {                  //Resets the module state.
        inputs.assign(inputs.size(), 0);
        outputs.assign(outputs.size(), 0);
    }               
    
    //Getters.
    std::vector<uint32_t>& getInputVector() {
        return inputs;
    }
    
    std::vector<uint32_t>& getOutputVector() {
        return outputs;
    }

    //Prints I/O vectors.
void printVectors() const {
    fmt::print("\n[Module: {}]\n", name());

    // Print inputs horizontally in chunks of 10
    fmt::print("Inputs:\n");
    for (size_t start = 0; start < inputs.size(); start += 10) {
        size_t end = std::min(start + 10, inputs.size());

        // Print input values
        for (size_t i = start; i < end; ++i) {
            fmt::print(" 0x{:08X}", inputs[i]);
        }
        fmt::print("\n");

        // Print input indices
        for (size_t i = start; i < end; ++i) {
            fmt::print("      [{}]", i);
        }
        fmt::print("\n");
    }

    // Print outputs horizontally in chunks of 10
    fmt::print("Outputs:\n");
    for (size_t start = 0; start < outputs.size(); start += 10) {
        size_t end = std::min(start + 10, outputs.size());

        // Print output values
        for (size_t i = start; i < end; ++i) {
            fmt::print(" 0x{:08X}", outputs[i]);
        }
        fmt::print("\n");

        // Print output indices
        for (size_t i = start; i < end; ++i) {
            fmt::print("      [{}]", i);
        }
        fmt::print("\n");
    }
}

    //Destructor.
    virtual ~Module() = default;
};

//IF Stage Module.
class IFStage : public Module {
    uint32_t    currPC, instruction, jumpAddr,
                opcode, rs, rt, rd, shamt, funct;
    int32_t     imm;

    std::vector<uint32_t> insMem;

public:
    IFStage() : currPC(0) {
        inputs.resize(1); //currPC
        outputs.resize(21);
        insMem.resize(256);
    }

    void tick() override {
        currPC = inputs[0];
        instruction = insMem[currPC];  //Instruction Fetch.
        decode();                       //Instruction Decode.
        sendSignals();                  //Control Signals

        printState();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    //Reset module.
    void reset() override {
        currPC, instruction, jumpAddr, imm, opcode, rs, rt , rd, shamt, funct = 0;
        inputs.assign(inputs.size(), 0);
        outputs.assign(outputs.size(), 0);
        insMem.assign(insMem.size(),0);
        loadMem(insMem);
    }

    //Name getter.
    std::string name() const override {
        return "IFStage";
    }

    //Loading instruction memory with instructions.
    void loadMem(std::vector<uint32_t>& memory, const std::string& filePath = "/mnt/c/Users/jmorp/OneDrive/Desktop/MIF.mif") {
	
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error(fmt::format("File in use!\nFile path: {}",filePath));
        }

        std::string line;
        bool inContentSection = false; //Instruction section flag.

        while (std::getline(file, line)) {
            if (!inContentSection && (line.find("CONTENT BEGIN") != std::string::npos)) {
                inContentSection = true;
                continue;
            }

            if (line.find("END;") != std::string::npos) {
                break;
            }

            //Extract string (instruction) after colon.
            size_t colonPos = line.find(':');
            if (colonPos == std::string::npos) {
                continue;
            }

            std::string dataString = line.substr(colonPos + 1);

            dataString.erase(std::remove_if(dataString.begin(), dataString.end(), ::isspace), dataString.end());

            if (dataString.length() > 9) {
                throw std::runtime_error(fmt::format("Invalid instruction format: '{}'", dataString));
            }

            //Convert hexadecimal string to type uint32_t.
            uint32_t instruction = std::stoul(dataString, nullptr, 16);

            std::string addressString = line.substr(0, colonPos);
            addressString.erase(std::remove_if(addressString.begin(), addressString.end(), ::isspace), addressString.end());

            if(addressString[0] == '['){

                for (int i = std::stoi(addressString.substr(1,2)) ; i<std::stoi(addressString.substr(addressString.size()-2,addressString.size()-1)) ; ++i)
                {
                    memory[i] = 0;
                }
                break;
                
            }

            uint32_t address = std::stoul(addressString);

            //Assumes instruction memory address range = {0,255}
            if (address >= 256) throw std::runtime_error(fmt::format("Address {} exceeds instruction memory size", address));

            else memory[address] = instruction;
        }

        file.close();
    }

    //Decode instruction.
    void decode(){
        opcode = (instruction >> 26) & 0x3F;
        rs = (instruction >> 21) & 0x1F;
        rt = (instruction >> 16) & 0x1F;
        rd = (instruction >> 11) & 0x1F;
        shamt = (instruction >> 6) & 0x1F;
        funct = instruction & 0x3F;
        imm = static_cast<int32_t>(static_cast<int16_t>(instruction & 0xFFFF));
        jumpAddr = instruction & 0x3FFFFFF;

        outputs[13] = rs & 0x1F;
        outputs[14] = rt & 0x1F;
        outputs[15] = rd & 0x1F;
        outputs[16] = shamt & 0x1F;
        outputs[17] = static_cast<uint32_t>(imm);
        outputs[18] = jumpAddr & 0x3FFFFFF;
        outputs[19] = opcode & 0x3F;
        outputs[20] = currPC+1;
        return;
    }

    //Send control signals.
    void sendSignals(){
        uint32_t jump, jal, jr, beq,
                bne, memRead, memToReg,
                aluOp, memWrite, aluSrc, regWriteEn,
                dstReg, writeSrc;
        
        jump = jal = jr = dstReg = beq = bne = memRead = memToReg = memWrite = aluSrc = regWriteEn = aluOp = 0;
		writeSrc = 1;
        
        switch (opcode) {
            case 0: { // R-TYPE
                dstReg = 1;
                regWriteEn = 1;
                switch (funct) {
                    case 0: aluOp = 7; break; // SLL
                    case 2: aluOp = 8; break; // SRL
                    case 8: jr = 1;   break; // JR
                    case 32: aluOp = 0; break; // ADD
                    case 34: aluOp = 1; break; // SUB
                    case 36: aluOp = 2; break; // AND
                    case 37: aluOp = 4; break; // XOR
                    case 38: aluOp = 3; break; // OR
                    case 39: aluOp = 5; break; // NOR
                    case 42: aluOp = 6; break; // SLT
                    default: aluOp = 15;       // ERR
                }
                break;
            }
            case 2: { // J
                jump = 1;
                break;
            }
            case 3: { // JAL
                jal = 1;
                dstReg = 2;
                regWriteEn = 1;
                writeSrc = 0;
                break;
            }
            case 4: { // BEQ
                aluOp = 1;
                beq = 1;
                break;
            }
            case 5: { // BNE
                aluOp = 1;
                bne = 1;
                break;
            }
            case 8: { // ADDI
                aluOp = 0;
                aluSrc = 1;
                regWriteEn = 1;
                break;
            }
            case 10: { // SLTI
                aluOp = 6;
                aluSrc = 1;
                regWriteEn = 1;
                break;
            }
            case 12: { // ANDI
                aluOp = 2;
                aluSrc = 1;
                regWriteEn = 1;
                break;
            }
            case 13: { // ORI
                aluOp = 3;
                aluSrc = 1;
                regWriteEn = 1;
                break;
            }
            case 14: { // XORI
                aluOp = 4;
                aluSrc = 1;
                regWriteEn = 1;
                break;
            }
            case 35: { // LW
                aluOp = 0;
                memRead = 1;
                memToReg = 1;
                aluSrc = 1;
                regWriteEn = 1;
                break;
            }
            case 43: { // SW
                aluOp = 0;
                memWrite = 1;
                aluSrc = 1;
                break;
            }
        }
        
        outputs[0] = jump & 0x1;
        outputs[1] = jal & 0x1;
        outputs[2] = jr & 0x1;
        outputs[3] = beq & 0x1;
        outputs[4] = bne & 0x1;
        outputs[5] = memRead & 0x1;
        outputs[6] = memToReg & 0x1;
        outputs[7] = memWrite & 0x1;
        outputs[8] = aluSrc & 0x1;
        outputs[9] = regWriteEn & 0x1;
        outputs[10] = writeSrc & 0x1;
        outputs[11] = aluOp & 0x3;
        outputs[12] = dstReg & 0x3;

        //fmt::print("\nDstReg = {}, Outputs[12] = {}\n", dstReg, outputs[12]);
        return;
    }
    
    //Prints Stage State.
    void printState() {
        fmt::print("\n[IFStage State]\n");
        fmt::print("PC Next: {}\n", currPC);
        fmt::print("Instruction: 0x{:08X}\n", instruction);
        fmt::print("Jump Address: 0x{:08X}\n", jumpAddr);
        fmt::print("Immediate: 0x{:08X}\n", imm);
        fmt::print("Opcode: 0x{:02X}\n", opcode);
        fmt::print("RS: 0x{:02X}, RT: 0x{:02X}, RD: 0x{:02X}\n", rs, rt, rd);
        fmt::print("Shamt: 0x{:02X}, Funct: 0x{:02X}\n", shamt, funct);
    }

    //Prints the next n instructions in the memory. (Does not account for branches)
    void nextInstructions(int n) {
        fmt::print("\tThe next {} Instructions:\n", n);
        for (size_t i = currPC + 1 ; i <= currPC+n && i < insMem.size(); ++i) {
            fmt::print("[{}]: 0x{:08X}\n", i, insMem[i]);
        }
    }
};

//ID Stage Module.
class IDStage : public Module {
    uint32_t    nextPC, writeReg, flush,
                aluOp, aluSrc, memRead,
                memWrite, regWriteEn, writeSrc,
                jump, jal, jr, bne, beq, dstReg,
                rs, rt, rd, shamt, jumpAddr, 
                opcode, currPC, writeBackEn, writeBackReg, dataToWB;

    int32_t  rsData, rtData, immSignExtend;
    std::vector<int32_t> regFile;
public:
    IDStage() : rsData(0), rtData(0), immSignExtend(0), 
                flush(0), aluOp(0), aluSrc(0), memRead(0),
                memWrite(0), regWriteEn(0), writeSrc(1), 
                jump(0), jal(0), jr(0), beq(0), bne(0),
                dstReg(0), rs(0), rt(0), rd(0), shamt(0),
                jumpAddr(0), opcode(0), currPC(0), writeBackEn(0),
                writeBackReg(0), dataToWB(0) {
        inputs.resize(24);
        outputs.resize(14);
        regFile.resize(32);
    }

    void tick() override {
        iniVar();
        writeBack();
        readData();
        nextPC = findNextPC();
        writeReg = findWriteReg();
        prepareOutputs();
        printState();
        printRegisterFile();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void reset() override {
        inputs.assign(inputs.size(), 0);
        outputs.assign(outputs.size(), 0);
        regFile.assign(regFile.size(),0);
    }

    std::string name() const override {
        return "IDStage";
    }

    //Initialize variables for readability.
    void iniVar() {
        jump = inputs[0];
        jal = inputs[1];
        jr = inputs[2];
        beq = inputs[3];
        bne = inputs[4];
        memRead = inputs[5];       
        memWrite = inputs[6];      
        regWriteEn = inputs[7];    
        writeSrc = inputs[8];      
        flush = inputs[9];
        aluSrc = inputs[10];
        aluOp = inputs[11];
        dstReg = inputs[12];
        rs = inputs[13];
        rt = inputs[14];
        rd = inputs[15];
        shamt = inputs[16];
        immSignExtend = static_cast<int32_t>(inputs[17]);
        jumpAddr = inputs[18];
        opcode = inputs[19];
        currPC = inputs[20];
        writeBackEn = inputs[21];
        writeBackReg = inputs[22];
        dataToWB = inputs[23];
    }

    //Read RS and RT.
    void readData(){
        rsData = regFile[rs];
        rtData = regFile[rt];
    }

    //Perform the Write-Back operation.
    void writeBack(){
        if(writeBackEn){
            if(writeBackReg > 31 || writeBackReg < 0){
               throw std::runtime_error(fmt::format("Invalid Register Index!\n Write-Back Register Index: {}", writeBackReg)); 
            }
            else if (writeBackReg == 0) return;

            else regFile[writeBackReg] = dataToWB;
            
        }    
        else return;
    }
    
    //Find write register for current instruction.
    uint32_t findWriteReg(){
        if (inputs[9] == 1) {
            switch (dstReg){
                case 0: {
                    return rt;
                    }
                case 1: return rd;
                case 2: return 31;

                default: throw std::runtime_error(fmt::format("Destination Register Invalid\nDestination Register Value: {}",inputs[12]));
            }
        }
        return 0;
    }

    //Compute next PC address.
    uint32_t findNextPC(){
        //If JR.
        if (jr) {
            flush = 1;
            return rsData;
        }
        
        //If JUMP or JAL.
        else if (jump || jal){
            flush = 1;
            //Return Jump Address.
            uint32_t pcUpper = inputs[20] & 0xFC000000;
            return (pcUpper | (jumpAddr << 2));
        }

        //If BEQ or BNE.
        else if ((beq && (rsData == rtData)) || (bne && (rsData != rtData))){
            flush = 1;
            int32_t offset = immSignExtend;
            return static_cast<int32_t>(currPC) + offset;  //PC + Offset
        }

        //Normal increment.
        else {
            flush = 0;
            return currPC + 1;
        }
    }

    //Prepare values for output vector. 
    void prepareOutputs(){
        //MemRead + MemToReg + MemWrite + ALUSrc + RegWriteEn + WriteSrc + ALUOp
        int i = 0;
        for(i; i < 7; ++i){
            outputs[i] = inputs[i+5];
        }
        outputs[i] = shamt;
        outputs[i+1] = writeReg;
        outputs[i+2] = inputs[17]; //Immediate value.
        outputs[i+3] = inputs[20] + 1; //PC Incremented.
        outputs[i+4] = static_cast<uint32_t>(rsData);
        outputs[i+5] = static_cast<uint32_t>(rtData);
        outputs[i+6] = nextPC;
    }
    
    //Prints Register File in chunks of 10
    void printRegisterFile() const {
        fmt::print("Register File:\n");
        for (size_t start = 0; start < regFile.size(); start += 10) {
            size_t end = std::min(start + 10, regFile.size());

            // Print register values
            for (size_t i = start; i < end; ++i) {
                fmt::print(" R[{}]: 0x{:08X}", i, regFile[i]);
            }
            fmt::print("\n");
        }
    }
    
    //Prints Stage State.
    void printState() {
        fmt::print("\n[IDStage State]\n");
        fmt::print("RS Data: 0x{:08X}\n", rsData);
        fmt::print("RT Data: 0x{:08X}\n", rtData);
        fmt::print("Next PC: 0x{:08X}\n", nextPC);
        fmt::print("Write Register: 0x{:02X}\n", writeReg);
        fmt::print("Immediate (Sign-Extended): 0x{:08X}\n", inputs[17]);
        fmt::print("Flush: {}\n", flush);

        //printRegisterFile();
    }
};

//EXE Stage Module.
class EXEStage : public Module {
    int32_t result, firstOperand, secondOperand, rtData, rsData;
    uint32_t aluOp, aluSrc, shiftAmount, regWriteEn, writeSrc, writeReg, incrPC;

public:
    EXEStage() : result(0), aluOp(0), aluSrc(0), shiftAmount(0), regWriteEn(0), writeSrc(0), writeReg(0), incrPC(0), rtData(0) {
        inputs.resize(14);
        outputs.resize(9);
    }

    void tick() override {
        iniVar();
        firstOperand = static_cast<int32_t>(inputs[11]);
        secondOperand = findSecondOperand();
        result = calculate();
        prepareOutputs();
        printState();
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Simulate delay
    }

    void reset() override {
        result = firstOperand = secondOperand = 0;
        aluOp = aluSrc = shiftAmount = regWriteEn = writeSrc = writeReg = incrPC = rtData = 0;
        inputs.assign(inputs.size(), 0);
        outputs.assign(outputs.size(), 0);
    }

    std::string name() const override {
        return "EXEStage";
    }

    void iniVar() {
        firstOperand = rsData;
        aluOp = inputs[6];
        aluSrc = inputs[3];
        shiftAmount = inputs[7];
        regWriteEn = inputs[4];
        writeSrc = inputs[5];
        writeReg = inputs[8];
        incrPC = inputs[10];
        rsData = static_cast<int32_t>(inputs[11]);
        rtData = static_cast<int32_t>(inputs[12]);
    }

    //Finding the second ALU operand.
    int32_t findSecondOperand() {
        switch (aluSrc) {                    // ALUSrc
            case 0: 
                return rtData;
            case 1: 
                return static_cast<int32_t>(inputs[9]);
            default:
                throw std::runtime_error(fmt::format("Invalid ALUSrc value: {}", aluSrc));
        }
    }

    //Calculates result.
    int32_t calculate() {
        switch (aluOp) {      // ALUOp
        //ADD
        case 0:
            return firstOperand + secondOperand;
        //SUB
        case 1: 
            return firstOperand - secondOperand;
        //AND
        case 2: 
            return firstOperand & secondOperand;
        //OR
        case 3:
            return firstOperand | secondOperand;
        //XOR
        case 4:
            return firstOperand ^ secondOperand;
        //NOR
        case 5:
            return ~(firstOperand | secondOperand);
        //SLT
        case 6:
            return firstOperand < secondOperand;
        //SLL
        case 7:
            return secondOperand << shiftAmount; // shamt
        //SRL
        case 8:
            return secondOperand >> shiftAmount;
        default:
            throw std::runtime_error(fmt::format("Invalid ALUOp value: {}", aluOp));
        }
    }

    //Prepare outputs for EXE stage.
    void prepareOutputs(){
        int i = 0;

        //MemRead + MemToReg + MemWrite
        for(i ; i < 3 ; ++i){
            outputs[i] = inputs[i];
        }

        //RegWriteEn + WriteSrc
        for (i ; i < 5; ++i)
        {
            outputs[i] = inputs[i+1];
        }
        outputs[i] = writeReg; // WriteReg
        outputs[i+1] = static_cast<uint32_t>(result); 
        outputs[i+2] = static_cast<uint32_t>(rtData); // RT Data
        outputs[i+3] = incrPC; // nextPC
    }

    //Shows EXE Stage state after tick().
    void printState() {
    fmt::print("\n[EXEStage State]\n");
    fmt::print("First Operand: 0x{:08X} ({})\n", firstOperand, firstOperand);
    fmt::print("Second Operand: 0x{:08X} ({})\n", secondOperand, secondOperand);
    fmt::print("Result: 0x{:08X} ({})\n", result, result);
    fmt::print("ALU Operation (ALUOp): {}\n", aluOp);
    fmt::print("Shift Amount (shamt): {}\n", shiftAmount);
    fmt::print("ALUSrc: {}\n", aluSrc);
}

};

//MEM Stage Module.
class MEMStage : public Module {
    uint32_t    address, memRead, memToReg,
                memWrite, regWriteEn, writeSrc,
                writeReg, currPC, dataToWB;

    int32_t     memOut, rtData;

    std::vector<int32_t> dataMem;

public:
    MEMStage() : address(0), memRead(0), memToReg(0), memWrite(0), regWriteEn(0), writeSrc(0), writeReg(0), currPC(0), dataToWB(0) {
        inputs.resize(8);
        outputs.resize(3);
        dataMem.resize(256); //Assumes fixed size of 256.
    }

    void tick() override {
        iniVar();
        memOut = readMem();
        writeVal();
        dataToWB = findWBData();
        prepareOutputs();
        printState();
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Simulate delay
    }

    void reset() override {
        inputs.assign(inputs.size(), 0);
        outputs.assign(outputs.size(), 0);
        dataMem.assign(dataMem.size(),0);
    }

    std::string name() const override {
        return "MEMStage";
    }

    //Initialise variables for readability.
    void iniVar(){
        memRead = inputs[0];
        memToReg = inputs[1];
        memWrite = inputs[2];
        regWriteEn = inputs[3];
        writeSrc = inputs[4];
        writeReg = inputs[5];
        address = inputs[6];
        rtData = static_cast<int32_t>(inputs[7]);
        currPC = inputs[8];
        return;
    }

    //Read from memory address.
    int32_t readMem(){
        if(memRead){
            if(address < dataMem.size()) return dataMem[address];
            else std::runtime_error(fmt::format("Invalid Data Memory Address\nAddress Value: {}",address));
        }
        
        else return 0;

    }

    //Write to the data memory.
    void writeVal(){
        if(memWrite){
            if(address < dataMem.size()) dataMem[address] = rtData;
            else std::runtime_error(fmt::format("Invalid Data Memory Address\nAddress Value: {}",address));
        }
        else return;
    }

    //Find write-back data.
    uint32_t findWBData(){
        if(writeSrc){
            switch (memOut)  {
                case 0:
                    return address;
                
                case 1:
                    return memOut;
                
                default: throw std::runtime_error(fmt::format("Invalid WriteSource or MemToReg value!\nWriteSrc: {}, MemToReg: {}\n", writeSrc, memToReg));
            }
        }
        else return currPC; //INCREMENT IF ISSUE WITH JAL
        
    }

    //Prepare output vector.
    void prepareOutputs(){
        outputs[0] = inputs[3]; //RegWriteEn
        outputs[1] = inputs[5]; //WriteReg
        outputs[2] = dataToWB;  //Write-back Data
    }

    //Displays Memory Stage State.
    void printState() {
        fmt::print("\n[MEMStage State]\n");
        fmt::print("Memory Address: 0x{:08X} ({})\n", address, address);
        fmt::print("Memory Read: {}\n", memRead);
        fmt::print("Memory To Register (MemToReg): {}\n", memToReg);
        fmt::print("Memory Write: {}\n", memWrite);
        fmt::print("Register Write Enable (RegWriteEn): {}\n", regWriteEn);
        fmt::print("Write Source (WriteSrc): {}\n", writeSrc);
        fmt::print("Write Register: 0x{:02X}\n", writeReg);
        fmt::print("Current PC: 0x{:08X}\n", currPC);
        fmt::print("Data to Write-Back: 0x{:08X} ({})\n", dataToWB, dataToWB);
        fmt::print("RT Data: 0x{:08X} ({})\n", rtData, rtData);
        fmt::print("Memory Output (MemOut): 0x{:08X} ({})\n", memOut, memOut);
    }

    //Displays the first n blocks in the data memory.
    void memHeader(const int& n) const {
        fmt::print("Data Memory (First 10 locations):\n");
        for (size_t i = 0; i < n && i < dataMem.size(); ++i) {
            fmt::print("  [0x{:02X}] = 0x{:08X}\n", i, dataMem[i]);
        }
    }

};

//WB Stage Module.
class WBStage : public Module {
    uint32_t writeReg, dataToWB, writeBackEn;

public:
    WBStage() : writeReg(0), dataToWB(0), writeBackEn(0) {
        inputs.resize(3);
        outputs.resize(3);
    }

    void tick() override {
        iniVar();
        prepareOutputs();
        printState();
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Simulate delay
    }

    void reset() override {
        writeReg = dataToWB = 0;
        inputs.assign(inputs.size(), 0);
        outputs.assign(outputs.size(), 0);
    }

    std::string name() const override {
        return "WBStage";
    }

    //Initialise variables.
    void iniVar(){
        writeBackEn = inputs[0];
        writeReg = inputs[1];
        dataToWB = inputs[2];
    }

    void prepareOutputs(){
        outputs[0] = writeBackEn;
        outputs[1] = writeReg;
        outputs[2] = dataToWB;
    }

    void printState() {
    fmt::print("\n[WBStage State]\n");
    fmt::print("Write Back Enable: {}\n", writeBackEn);
    fmt::print("Write Register: 0x{:02X}\n", writeReg);
    fmt::print("Data to Write-Back: 0x{:08X} ({})\n", dataToWB, dataToWB);
}

};

//Pipeline class
class Pipeline {
private:
    boost::circular_buffer<std::unique_ptr<Module>> stages;

public:
    Pipeline(size_t size) : stages(size) {}

    void addStage(std::unique_ptr<Module> stage) {
        stages.push_back(std::move(stage));
    }

    void tick() {
        //Data transfer and processing.
        for (size_t i = 0; i < stages.size(); ++i) {
            stages[i]->tick();
            if(i>0) {
                stages[i]->getInputVector().assign(
                    stages[i - 1]->getOutputVector().begin(),
                    stages[i - 1]->getOutputVector().end()
                );
            }
        
        }
        stages[0]->getInputVector()[0] = stages[1]->getOutputVector().back();
        
        for(int i = 21 ; i < 24 ; ++i){
            stages[1]->getInputVector()[i] = stages[4]->getOutputVector()[i-21];
        }
    
    }

    void reset() {
        for (auto& stage : stages) {
            if (stage) {
                stage->reset();
            }
        }
    }
};

#endif