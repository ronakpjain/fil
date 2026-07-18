#include "fil/devices/can_bus.hpp"
#include "fil/cli/cli.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../test_support.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

class TempWorldConfigs {
public:
    TempWorldConfigs()
        : root_(std::filesystem::temp_directory_path() / "fil-world-tests") {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
        std::filesystem::create_directories(root_, error);
    }

    ~TempWorldConfigs() {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
    }

    [[nodiscard]] std::filesystem::path writeBoard(
        const std::string_view file_name,
        const std::string_view board_name,
        const std::string_view bus_name = "vehicle"
    ) const {
        const std::filesystem::path mcu =
            std::filesystem::path(FIL_SOURCE_DIR) / "configs/mcus/stm32g474retx.json";
        const std::filesystem::path elf =
            std::filesystem::path(FIL_SOURCE_DIR) / "tests/fixtures/elf/split_image.elf";
        std::ostringstream json;
        json << "{\n"
             << "  \"schema_version\": 1,\n"
             << "  \"name\": \"" << board_name << "\",\n"
             << "  \"mcu\": \"" << mcu.string() << "\",\n"
             << "  \"elf\": \"" << elf.string() << "\",\n"
             << "  \"vector_base\": \"0x08000000\",\n"
             << "  \"can\": {\"FDCAN1\": {\"bus\": \"" << bus_name
             << "\", \"loopback\": false}}\n"
             << "}\n";

        const std::filesystem::path path = root_ / file_name;
        std::ofstream output(path, std::ios::binary);
        output << json.str();
        return path;
    }

    [[nodiscard]] fil::config::NetworkConfig network(
        std::vector<std::filesystem::path> boards
    ) const {
        fil::config::NetworkConfig config;
        config.name = "fixture-network";
        config.source_path = root_ / "network.json";
        config.buses.push_back({"vehicle", 500000U});
        config.board_paths = std::move(boards);
        return config;
    }

    [[nodiscard]] std::filesystem::path writeNetwork(
        const std::vector<std::filesystem::path>& boards
    ) const {
        const std::filesystem::path path = root_ / "network.json";
        std::ofstream output(path, std::ios::binary);
        output << "{\n  \"schema_version\": 1,\n  \"name\": \"fixture-network\",\n"
               << "  \"buses\": {\"vehicle\": {\"type\": \"can\", \"bitrate\": 500000}},\n"
               << "  \"boards\": [";
        for (std::size_t index = 0; index < boards.size(); ++index) {
            if (index != 0U) output << ", ";
            output << '"' << boards[index].string() << '"';
        }
        output << "]\n}\n";
        return path;
    }

private:
    std::filesystem::path root_;
};

void loadsSharedCanFabric() {
    TempWorldConfigs files;
    auto network = files.network({
        files.writeBoard("alpha.json", "alpha"),
        files.writeBoard("beta.json", "beta"),
    });
    auto world = fil::sim::World::load(network);
    fil::test::check(world.hasValue(), "loads a two-board simulation world");
    if (!world) return;

    fil::test::check(world.value()->boardCount() == 2U, "owns every configured world board");
    fil::test::check(world.value()->board("alpha") != nullptr, "finds a board by name");
    fil::test::check(world.value()->canBusCount() == 1U, "owns every configured CAN bus");
    const auto* bus = world.value()->canBus("vehicle");
    fil::test::check(bus != nullptr && bus->nodeCount() == 2U, "attaches both FDCAN nodes to the shared bus");
}

void dispatchesBoardsInFixedOrder() {
    TempWorldConfigs files;
    auto network = files.network({
        files.writeBoard("alpha.json", "alpha"),
        files.writeBoard("beta.json", "beta"),
    });
    auto world = fil::sim::World::load(network);
    if (!world) {
        fil::test::check(false, "loads world for scheduler test");
        return;
    }

    fil::sim::WorldRunOptions options;
    options.max_instructions_per_board = 10U;
    options.duration_ns = 0U;
    options.instruction_quantum = 1U;
    options.trace_instructions = true;
    const auto result = world.value()->run(options);
    fil::test::check(result.hasValue(), "runs a fixed-quantum world");
    if (!result) return;

    fil::test::check(
        result.value().reason == fil::sim::WorldStopReason::all_boards_stopped,
        "stops when both fixture boards reach BKPT"
    );
    fil::test::check(result.value().instructions == 6U, "aggregates instruction counts across boards");
    fil::test::check(result.value().dispatches == 6U, "dispatches once per one-instruction slice");
    fil::test::check(
        result.value().boards.size() == 2U
            && result.value().boards[0].result.instructions == 3U
            && result.value().boards[1].result.instructions == 3U,
        "keeps independent per-board counters"
    );
    fil::test::check(
        result.value().end_time_ns == world.value()->eventLoop().now(),
        "reports the shared simulated clock"
    );

    std::vector<std::string> instruction_sources;
    for (const fil::sim::TraceRecord& record : world.value()->trace().records()) {
        if (record.type == "instr") instruction_sources.push_back(record.source);
    }
    fil::test::check(
        instruction_sources == std::vector<std::string>{
            "alpha", "beta", "alpha", "beta", "alpha", "beta",
        },
        "traces instruction dispatches in stable network order"
    );
}

void qualifiesSameNamedPeripheralTraceSources() {
    TempWorldConfigs files;
    auto network = files.network({
        files.writeBoard("alpha.json", "alpha"),
        files.writeBoard("beta.json", "beta"),
    });
    auto world = fil::sim::World::load(network);
    fil::test::check(world.hasValue(), "loads two boards for peripheral trace qualification");
    if (!world) return;

    fil::sim::Board* alpha = world.value()->board("alpha");
    fil::sim::Board* beta = world.value()->board("beta");
    auto* alpha_gpio = alpha == nullptr ? nullptr : alpha->peripherals().gpio("GPIOA");
    auto* beta_gpio = beta == nullptr ? nullptr : beta->peripherals().gpio("GPIOA");
    fil::test::check(alpha_gpio != nullptr && beta_gpio != nullptr,
                     "finds the same GPIO instance on both boards");
    if (alpha_gpio == nullptr || beta_gpio == nullptr) return;

    world.value()->trace().clear();
    fil::test::check(
        alpha_gpio->write(0x18U, fil::mem::AccessSize::word, 1U, {}).hasValue()
            && beta_gpio->write(0x18U, fil::mem::AccessSize::word, 1U, {}).hasValue(),
        "drives the same GPIO pin on both boards"
    );

    std::vector<std::string> gpio_sources;
    for (const fil::sim::TraceRecord& record : world.value()->trace().records()) {
        if (record.type == "gpio_output") gpio_sources.push_back(record.source);
    }
    fil::test::check(
        gpio_sources == std::vector<std::string>{"alpha.GPIOA", "beta.GPIOA"},
        "shared trace qualifies same-named peripherals with stable board prefixes"
    );
}

void enforcesBudgetsAndValidatesTopology() {
    TempWorldConfigs files;
    const std::filesystem::path alpha = files.writeBoard("alpha.json", "alpha");
    auto network = files.network({alpha});
    auto world = fil::sim::World::load(network);
    if (!world) {
        fil::test::check(false, "loads world for budget test");
        return;
    }

    fil::sim::WorldRunOptions options;
    options.max_instructions_per_board = 2U;
    options.duration_ns = 0U;
    options.instruction_quantum = 1U;
    const auto exhausted = world.value()->run(options);
    fil::test::check(
        exhausted && exhausted.value().reason == fil::sim::WorldStopReason::instruction_budget,
        "enforces an independent board instruction budget"
    );

    options.instruction_quantum = 0U;
    fil::test::check(!world.value()->run(options), "rejects a zero scheduling quantum");

    const std::filesystem::path missing = files.writeBoard("missing.json", "missing", "undeclared");
    auto invalid_network = files.network({missing});
    fil::test::check(!fil::sim::World::load(invalid_network), "rejects an attachment to an undeclared bus");
}

void executesRunNetworkCli() {
    TempWorldConfigs files;
    const auto network_path = files.writeNetwork({
        files.writeBoard("cli-alpha.json", "cli-alpha"),
        files.writeBoard("cli-beta.json", "cli-beta"),
    });
    const std::string path = network_path.string();
    const std::filesystem::path trace_path = network_path.parent_path() / "cli-trace.jsonl";
    const std::string trace_path_text = trace_path.string();
    const std::string_view args[]{
        "run-network", path, "--duration-ms", "0", "--max-instructions", "10",
        "--quantum", "1", "--inject-can", "vehicle@0:0x123:01", "--trace", trace_path_text,
        "--trace-instr", "--allow-breakpoint",
    };
    std::ostringstream out;
    std::ostringstream err;
    const auto result = fil::cli::run(args, out, err);
    fil::test::check(result == fil::cli::ExitCode::success, "run-network CLI executes a fixture world");
    fil::test::check(out.str().find("network: fixture-network") != std::string::npos,
                     "run-network CLI prints an aggregate summary");
    fil::test::check(out.str().find("board cli-alpha") != std::string::npos
                         && out.str().find("board cli-beta") != std::string::npos,
                     "run-network CLI prints per-board results");
    fil::test::check(err.str().empty(), "successful run-network CLI has no error output");
    std::ifstream trace_input(trace_path, std::ios::binary);
    const std::string trace_text{
        std::istreambuf_iterator<char>(trace_input), std::istreambuf_iterator<char>()
    };
    fil::test::check(trace_text.find("\"source\":\"vehicle/external\",\"type\":\"can_tx\"")
                         != std::string::npos,
                     "run-network CLI schedules an external CAN injection into the trace");
    fil::test::check(
        trace_text.find("\"source\":\"vehicle/external\",\"type\":\"can_tx\"")
            < trace_text.find("\"type\":\"instr\""),
        "time-zero CAN injection is delivered before the first CPU instruction"
    );
}

} // namespace

void runWorldTests() {
    loadsSharedCanFabric();
    dispatchesBoardsInFixedOrder();
    qualifiesSameNamedPeripheralTraceSources();
    enforcesBudgetsAndValidatesTopology();
    executesRunNetworkCli();
}
