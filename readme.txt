Welcome to Quack Attack's MIPS-based Cycle Accurate Simulator!

Notes on usage:
	1. To load from a specific instruction file, adjust the filepath variable in line 127 in setup.hpp .
	
	2. To decide number of cycle runs, adjust the loop parameter in line 16 in main.cpp .
	
	3. To monitor further about the input and output vectors of every stage object in the pipeline, add "stage[i]->printVectors();" in the loop inside the tick() function of the Pipeline class .

Functionality:
	- This CAS has been developed in three days due to mid-term exams and quizzes . 
	
	- It should function properly for sequential instructions and simple procedure blocks .
	
	- It is not capable of dealing with branching or jumping. It is not capable of forwarding, so no dependencies either .
	
	- It has not been tested AT ALL. It is meant to be a very early Alpha version of bare-bone framework architecture .
	
	- It shall become more robust by the end of the next phase, when we hopefully qualify 😎 .
	
Enjoy!
	