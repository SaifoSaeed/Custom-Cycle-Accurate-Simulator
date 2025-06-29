#include "setup.hpp"

int main() {
    Pipeline pipeline(5);

    pipeline.addStage(std::make_unique<IFStage>());
    pipeline.addStage(std::make_unique<IDStage>());
    pipeline.addStage(std::make_unique<EXEStage>());
    pipeline.addStage(std::make_unique<MEMStage>());
    pipeline.addStage(std::make_unique<WBStage>());

    fmt::print("Resetting pipeline...\n");
    pipeline.reset();

    fmt::print("Starting pipeline simulation...\n");
    for (int cycle = 0; cycle < 6; ++cycle) {
        fmt::print("Cycle: {}\n", cycle + 1);
        pipeline.tick();
        fmt::print("---------------------\n");
    }

    return 0;
}
