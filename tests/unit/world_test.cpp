#include "fil/devices/can_bus.hpp"
#include "fil/cli/cli.hpp"
#include "fil/cli/network_protocol.hpp"
#include "fil/sim/world.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <unistd.h>
#include <string>
#include <string_view>
#include <vector>

namespace {

class TempWorldConfigs {
public:
    [[nodiscard]] std::filesystem::path writeBoard(
        const std::string_view file_name,
        const std::string_view board_name,
        const std::string_view bus_name = "vehicle"
    ) const {
        const auto fixture = fil::test::fixtureBoardConfig(board_name);
        std::ostringstream json;
        json << "{\n"
             << "  \"schema_version\": 1,\n"
             << "  \"name\": \"" << board_name << "\",\n"
             << "  \"mcu\": \"" << fixture.mcu_path.string() << "\",\n"
             << "  \"elf\": \"" << fixture.elf_path.string() << "\",\n"
             << "  \"vector_base\": \"0x08000000\",\n"
             << "  \"can\": {\"FDCAN1\": {\"bus\": \"" << bus_name
             << "\", \"loopback\": false}}\n"
             << "}\n";
        return directory_.write(file_name, json.str());
    }

    [[nodiscard]] fil::config::NetworkConfig network(
        std::vector<std::filesystem::path> boards
    ) const {
        fil::config::NetworkConfig config;
        config.name = "fixture-network";
        config.source_path = directory_.root() / "network.json";
        config.buses.push_back({"vehicle", 500000U});
        config.board_paths = std::move(boards);
        return config;
    }

    [[nodiscard]] std::filesystem::path writeStimulus(const std::string_view content) const {
        return writeStimulusAs("expectation.json", content);
    }

    [[nodiscard]] std::filesystem::path writeStimulusAs(
        const std::filesystem::path& relative, const std::string_view content
    ) const {
        return directory_.write(relative, content);
    }

    [[nodiscard]] std::filesystem::path writeNetwork(
        const std::vector<std::filesystem::path>& boards,
        const std::filesystem::path& stimulus = {},
        const bool include_nested_bus = false
    ) const {
        std::vector<std::filesystem::path> stimuli;
        if (!stimulus.empty()) stimuli.push_back(stimulus);
        return writeNetworkWithStimuli(boards, stimuli, include_nested_bus);
    }

    [[nodiscard]] std::filesystem::path writeNetworkWithStimuli(
        const std::vector<std::filesystem::path>& boards,
        const std::vector<std::filesystem::path>& stimuli,
        const bool include_nested_bus = false
    ) const {
        std::ostringstream json;
        json << "{\n  \"schema_version\": 1,\n  \"name\": \"fixture-network\",\n"
             << "  \"buses\": {\"vehicle\": {\"type\": \"can\", \"bitrate\": 500000}";
        if (include_nested_bus) json << ", \"vehicle/sub\": {\"type\": \"can\", \"bitrate\": 500000}";
        json << "},\n  \"boards\": [";
        for (std::size_t index = 0; index < boards.size(); ++index) {
            if (index != 0U) json << ", ";
            json << '"' << boards[index].string() << '"';
        }
        json << ']';
        if (!stimuli.empty()) {
            json << ",\n  \"stimuli\": [";
            for (std::size_t index = 0; index < stimuli.size(); ++index) {
                if (index != 0U) json << ", ";
                json << '"' << stimuli[index].string() << '"';
            }
            json << ']';
        }
        json << "\n}\n";
        return directory_.write("network.json", json.str());
    }

private:
    fil::test::TemporaryDirectory directory_{"fil-world-tests"};
};

class ScopedStdinPipe {
public:
    ~ScopedStdinPipe() {
        if (original_ >= 0) {
            static_cast<void>(::dup2(original_, STDIN_FILENO));
            static_cast<void>(::close(original_));
        }
    }

    [[nodiscard]] bool redirect(const std::vector<std::uint8_t>& input) {
        original_ = ::dup(STDIN_FILENO);
        if (original_ < 0) return false;
        int descriptors[2]{};
        if (::pipe(descriptors) != 0) return false;
        if (::dup2(descriptors[0], STDIN_FILENO) < 0) {
            static_cast<void>(::close(descriptors[0]));
            static_cast<void>(::close(descriptors[1]));
            return false;
        }
        static_cast<void>(::close(descriptors[0]));
        std::size_t written = 0U;
        while (written < input.size()) {
            const ssize_t result = ::write(descriptors[1], input.data() + written,
                                          input.size() - written);
            if (result <= 0) {
                static_cast<void>(::close(descriptors[1]));
                return false;
            }
            written += static_cast<std::size_t>(result);
        }
        static_cast<void>(::close(descriptors[1]));
        return true;
    }

private:
    int original_{-1};
};

TEST(WorldTest, LoadsSharedCanFabric) {
    TempWorldConfigs files;
    auto network = files.network({
        files.writeBoard("alpha.json", "alpha"),
        files.writeBoard("beta.json", "beta"),
    });
    auto world = fil::sim::World::load(network);
    EXPECT_TRUE(world.hasValue()) << "loads a two-board simulation world";
    if (!world) return;

    EXPECT_TRUE(world.value()->boardCount() == 2U) << "owns every configured world board";
    EXPECT_TRUE(world.value()->board("alpha") != nullptr) << "finds a board by name";
    EXPECT_TRUE(world.value()->canBusCount() == 1U) << "owns every configured CAN bus";
    const auto* bus = world.value()->canBus("vehicle");
    EXPECT_TRUE(bus != nullptr && bus->nodeCount() == 2U)
        << "attaches both FDCAN nodes to the shared bus";
}

TEST(WorldTest, DispatchesBoardsInFixedOrder) {
    TempWorldConfigs files;
    auto network = files.network({
        files.writeBoard("alpha.json", "alpha"),
        files.writeBoard("beta.json", "beta"),
    });
    auto world = fil::sim::World::load(network);
    if (!world) {
        EXPECT_TRUE(false) << "loads world for scheduler test";
        return;
    }

    fil::sim::WorldRunOptions options;
    options.max_instructions_per_board = 10U;
    options.duration_ns = 0U;
    options.instruction_quantum = 1U;
    options.trace_instructions = true;
    const auto result = world.value()->run(options);
    EXPECT_TRUE(result.hasValue()) << "runs a fixed-quantum world";
    if (!result) return;

    EXPECT_TRUE(result.value().reason == fil::sim::WorldStopReason::all_boards_stopped)
        << "stops when both fixture boards reach BKPT";
    EXPECT_TRUE(result.value().instructions == 6U) << "aggregates instruction counts across boards";
    EXPECT_TRUE(result.value().dispatches == 6U) << "dispatches once per one-instruction slice";
    EXPECT_TRUE(result.value().boards.size() == 2U &&
                result.value().boards[0].result.instructions == 3U &&
                result.value().boards[1].result.instructions == 3U)
        << "keeps independent per-board counters";
    EXPECT_TRUE(result.value().end_time_ns == world.value()->eventLoop().now())
        << "reports the shared simulated clock";

    std::vector<std::string> instruction_sources;
    for (const fil::sim::TraceRecord& record : world.value()->trace().records()) {
        if (record.type == "instr") instruction_sources.push_back(record.source);
    }
    EXPECT_TRUE((instruction_sources ==
                 std::vector<std::string>{
                     "alpha",
                     "beta",
                     "alpha",
                     "beta",
                     "alpha",
                     "beta",
                 }))
        << "traces instruction dispatches in stable network order";
}

TEST(WorldTest, QualifiesSameNamedPeripheralTraceSources) {
    TempWorldConfigs files;
    auto network = files.network({
        files.writeBoard("alpha.json", "alpha"),
        files.writeBoard("beta.json", "beta"),
    });
    auto world = fil::sim::World::load(network);
    EXPECT_TRUE(world.hasValue()) << "loads two boards for peripheral trace qualification";
    if (!world) return;

    fil::sim::Board* alpha = world.value()->board("alpha");
    fil::sim::Board* beta = world.value()->board("beta");
    auto* alpha_gpio = alpha == nullptr ? nullptr : alpha->peripherals().gpio("GPIOA");
    auto* beta_gpio = beta == nullptr ? nullptr : beta->peripherals().gpio("GPIOA");
    EXPECT_TRUE(alpha_gpio != nullptr && beta_gpio != nullptr)
        << "finds the same GPIO instance on both boards";
    if (alpha_gpio == nullptr || beta_gpio == nullptr) return;

    world.value()->trace().clear();
    EXPECT_TRUE(alpha_gpio->write(0x18U, fil::mem::AccessSize::word, 1U, {}).hasValue() &&
                beta_gpio->write(0x18U, fil::mem::AccessSize::word, 1U, {}).hasValue())
        << "drives the same GPIO pin on both boards";

    std::vector<std::string> gpio_sources;
    for (const fil::sim::TraceRecord& record : world.value()->trace().records()) {
        if (record.type == "gpio_output") gpio_sources.push_back(record.source);
    }
    EXPECT_TRUE((gpio_sources == std::vector<std::string>{"alpha.GPIOA", "beta.GPIOA"}))
        << "shared trace qualifies same-named peripherals with stable board prefixes";
}

TEST(WorldTest, EnforcesBudgetsAndValidatesTopology) {
    TempWorldConfigs files;
    const std::filesystem::path alpha = files.writeBoard("alpha.json", "alpha");
    auto network = files.network({alpha});
    auto world = fil::sim::World::load(network);
    if (!world) {
        EXPECT_TRUE(false) << "loads world for budget test";
        return;
    }

    fil::sim::WorldRunOptions options;
    options.max_instructions_per_board = 2U;
    options.duration_ns = 0U;
    options.instruction_quantum = 1U;
    const auto exhausted = world.value()->run(options);
    EXPECT_TRUE(
        exhausted && exhausted.value().reason == fil::sim::WorldStopReason::instruction_budget)
        << "enforces an independent board instruction budget";

    options.instruction_quantum = 0U;
    EXPECT_TRUE(!world.value()->run(options)) << "rejects a zero scheduling quantum";

    const std::filesystem::path missing = files.writeBoard("missing.json", "missing", "undeclared");
    auto invalid_network = files.network({missing});
    EXPECT_TRUE(!fil::sim::World::load(invalid_network))
        << "rejects an attachment to an undeclared bus";
}

TEST(WorldTest, StopsWatchNetworkWhenAllBoardsReachTerminalBoundaries) {
    TempWorldConfigs files;
    const auto network_path = files.writeNetwork({files.writeBoard("watch.json", "watch")});
    const std::string path = network_path.string();
    const std::string_view args[]{
        "watch-network", path, "--duration-ms", "5", "--refresh-ms", "2",
        "--adc-decimation", "4", "--trace-instr", "--live-filter", "can_rx",
    };
    std::ostringstream out;
    std::ostringstream err;

    const auto result = fil::cli::run(args, out, err);

    EXPECT_EQ(result, fil::cli::ExitCode::success);
    EXPECT_NE(out.str().find("watching network fixture-network"), std::string::npos);
    EXPECT_TRUE(err.str().empty());
}

TEST(WorldTest, WatchNetworkStreamsExpectationPendingAndIncompleteLive) {
    TempWorldConfigs files;
    const auto stimulus = files.writeStimulus(R"({
  "schema_version": 1, "name": "live", "events": [
    {"type":"usart","at_ms":0,"board":"watch-live","instance":"USART1","bytes":[]}
  ], "expect": [
    {"type":"can","bus":"vehicle","id":"0x321","data":[1],"at_ms":100,"window_ms":10}
  ]
})");
    const auto network_path = files.writeNetwork(
        {files.writeBoard("watch-live.json", "watch-live")}, stimulus);
    const std::string path = network_path.string();
    const std::string_view args[]{"watch-network", path, "--no-wall-pacing",
        "--trace-type", "expectation_pending", "--trace-type", "expectation_incomplete"};
    std::istringstream input{"quit\n"};
    std::ostringstream out;
    std::ostringstream err;
    std::streambuf* const original_input = std::cin.rdbuf(input.rdbuf());
    const auto result = fil::cli::run(args, out, err);
    std::cin.rdbuf(original_input);
    std::cin.clear();

    EXPECT_EQ(result, fil::cli::ExitCode::success);
    EXPECT_NE(out.str().find("expectation_pending"), std::string::npos);
    EXPECT_NE(out.str().find("expectation_incomplete"), std::string::npos);
    EXPECT_NE(out.str().find("live/expect/0"), std::string::npos);
    EXPECT_TRUE(err.str().empty());
}

TEST(WorldTest, ExpectationCheckIdsDistinguishDuplicateScriptsAndAttachments) {
    TempWorldConfigs files;
    const std::string_view script = R"({
  "schema_version": 1, "name": "same-name", "events": [
    {"type":"usart","at_ms":0,"board":"duplicate-checks","instance":"USART1","bytes":[]}
  ], "expect": [
    {"type":"can","bus":"vehicle","id":"0x321","data":[1],"at_ms":100,"window_ms":10}
  ]
})";
    const auto first = files.writeStimulusAs("first.json", script);
    const auto second = files.writeStimulusAs("second.json", script);
    const auto network_path = files.writeNetworkWithStimuli(
        {files.writeBoard("duplicate-checks.json", "duplicate-checks")},
        {first, second, first});
    const std::string path = network_path.string();
    const std::string_view args[]{"watch-network", path, "--no-wall-pacing",
        "--trace-type", "expectation_pending", "--trace-type", "expectation_incomplete"};
    std::istringstream input{"quit\n"};
    std::ostringstream out;
    std::ostringstream err;
    std::streambuf* const original_input = std::cin.rdbuf(input.rdbuf());
    const auto result = fil::cli::run(args, out, err);
    std::cin.rdbuf(original_input);
    std::cin.clear();

    EXPECT_EQ(result, fil::cli::ExitCode::success);
    for (const std::string_view check_id : {
            "stimulus/0/same-name/expect/0", "stimulus/1/same-name/expect/0",
            "stimulus/2/same-name/expect/0"}) {
        EXPECT_NE(out.str().find(check_id), std::string::npos);
    }
    EXPECT_TRUE(err.str().empty());
}

TEST(WorldTest, RejectsInvalidWatchNetworkAdcDecimation) {
    const std::string_view args[]{"watch-network", "missing.json", "--adc-decimation", "0"};
    std::ostringstream out;
    std::ostringstream err;
    EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::usage_error);
    EXPECT_NE(err.str().find("[1, 1024]"), std::string::npos);
}

TEST(WorldTest, StopsWatchNetworkFromStdin) {
    TempWorldConfigs files;
    const auto network_path = files.writeNetwork({files.writeBoard("watch-quit.json", "watch")});
    const std::string path = network_path.string();
    const std::string_view args[]{"watch-network", path, "--refresh-ms", "1"};
    std::istringstream input{"quit\n"};
    std::ostringstream out;
    std::ostringstream err;

    std::streambuf* const original_input = std::cin.rdbuf(input.rdbuf());
    const auto result = fil::cli::run(args, out, err);
    std::cin.rdbuf(original_input);
    std::cin.clear();

    EXPECT_EQ(result, fil::cli::ExitCode::success);
    EXPECT_EQ(out.str().find("expectation_"), std::string::npos);
    EXPECT_TRUE(err.str().empty());
}

TEST(WorldTest, ServeNetworkRequiresAnExplicitSupportedTransport) {
    const std::string_view missing_transport[]{"serve-network", "network.json"};
    std::ostringstream missing_out;
    std::ostringstream missing_err;
    EXPECT_EQ(fil::cli::run(missing_transport, missing_out, missing_err),
              fil::cli::ExitCode::usage_error);
    EXPECT_TRUE(missing_out.str().empty());
    EXPECT_NE(missing_err.str().find("--transport stdio"), std::string::npos);

    const std::string_view unsupported_transport[]{
        "serve-network", "network.json", "--transport", "tcp"
    };
    std::ostringstream unsupported_out;
    std::ostringstream unsupported_err;
    EXPECT_EQ(fil::cli::run(unsupported_transport, unsupported_out, unsupported_err),
              fil::cli::ExitCode::usage_error);
    EXPECT_TRUE(unsupported_out.str().empty());
    EXPECT_NE(unsupported_err.str().find("supported: stdio"), std::string::npos);
}

TEST(WorldTest, ServeNetworkUsesBinaryStdioFramesForControlsAndTraces) {
    TempWorldConfigs files;
    const auto network_path = files.writeNetwork({files.writeBoard("serve.json", "serve")});
    const std::string path = network_path.string();

    std::vector<std::uint8_t> request_bytes;
    const auto appendString = [](std::vector<std::uint8_t>& payload, const std::string_view value) {
        payload.push_back(static_cast<std::uint8_t>(value.size() & 0xffU));
        payload.push_back(static_cast<std::uint8_t>((value.size() >> 8U) & 0xffU));
        for (const char byte : value) {
            payload.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(byte)));
        }
    };
    const auto appendU32 = [](std::vector<std::uint8_t>& payload, const std::uint32_t value) {
        for (unsigned int index = 0U; index < 4U; ++index) {
            payload.push_back(static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU));
        }
    };
    const auto appendFrame = [&](const fil::cli::network_protocol::Frame& frame) {
        std::ostringstream encoded(std::ios::out | std::ios::binary);
        EXPECT_TRUE(fil::cli::network_protocol::writeFrame(encoded, frame));
        for (const char byte : encoded.str()) {
            request_bytes.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(byte)));
        }
    };

    std::vector<std::uint8_t> can_payload;
    appendString(can_payload, "vehicle");
    appendU32(can_payload, 0x123U);
    can_payload.insert(can_payload.end(), {0U, 1U, 0x5aU});
    appendFrame({fil::cli::network_protocol::MessageKind::can_inject, 1U,
                 std::move(can_payload)});

    std::vector<std::uint8_t> adc_payload;
    appendString(adc_payload, "serve");
    appendString(adc_payload, "ADC1");
    adc_payload.insert(adc_payload.end(), {5U, 0U, 8U}); // channel 5, value 2048.
    appendFrame({fil::cli::network_protocol::MessageKind::adc_set, 2U,
                 std::move(adc_payload)});

    std::vector<std::uint8_t> gpio_payload;
    appendString(gpio_payload, "serve");
    appendString(gpio_payload, "GPIOA");
    gpio_payload.insert(gpio_payload.end(), {5U, 1U}); // pin 5 driven high.
    appendFrame({fil::cli::network_protocol::MessageKind::gpio_set, 3U,
                 std::move(gpio_payload)});

    std::vector<std::uint8_t> missing_bus_payload;
    appendString(missing_bus_payload, "missing-bus");
    appendU32(missing_bus_payload, 0x123U);
    missing_bus_payload.insert(missing_bus_payload.end(), {0U, 0U});
    appendFrame({fil::cli::network_protocol::MessageKind::can_inject, 4U,
                 std::move(missing_bus_payload)});
    appendFrame({fil::cli::network_protocol::MessageKind::stop, 5U, {}});

    ScopedStdinPipe stdin_pipe;
    ASSERT_TRUE(stdin_pipe.redirect(request_bytes)) << "redirects test protocol requests to stdin";
    const std::string_view args[]{"serve-network", path, "--transport", "stdio",
                                  "--no-wall-pacing", "--live-filter", "can_rx"};
    std::ostringstream out(std::ios::out | std::ios::binary);
    std::ostringstream err;
    const auto result = fil::cli::run(args, out, err);

    EXPECT_EQ(result, fil::cli::ExitCode::success);
    EXPECT_NE(err.str().find("request 4"), std::string::npos);
    const std::string output_bytes = out.str();
    std::vector<std::uint8_t> output;
    output.reserve(output_bytes.size());
    for (const char byte : output_bytes) {
        output.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(byte)));
    }

    fil::cli::network_protocol::FrameDecoder decoder;
    ASSERT_TRUE(decoder.append(output));
    std::vector<fil::cli::network_protocol::Frame> frames;
    while (true) {
        auto next = decoder.next();
        ASSERT_TRUE(next) << (next ? "" : next.error().message);
        if (!next.value()) break;
        frames.push_back(std::move(*next.value()));
    }
    ASSERT_TRUE(decoder.finish());
    ASSERT_EQ(frames.size(), 8U);
    EXPECT_EQ(frames[0].kind, fil::cli::network_protocol::MessageKind::hello);
    EXPECT_EQ(frames[1].kind, fil::cli::network_protocol::MessageKind::trace);
    EXPECT_EQ(frames[1].request_id, 0U);
    for (std::size_t index = 0U; index < 5U; ++index) {
        EXPECT_EQ(frames[index + 2U].kind, fil::cli::network_protocol::MessageKind::reply);
        EXPECT_EQ(frames[index + 2U].request_id, index + 1U);
    }
    const auto replyStatus = [](const fil::cli::network_protocol::Frame& frame) {
        return static_cast<std::uint16_t>(frame.payload[0U])
            | static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame.payload[1U]) << 8U);
    };
    EXPECT_EQ(replyStatus(frames[2]), 0U);
    EXPECT_EQ(replyStatus(frames[3]), 0U);
    EXPECT_EQ(replyStatus(frames[4]), 0U);
    EXPECT_EQ(replyStatus(frames[5]), 2U);
    EXPECT_EQ(replyStatus(frames[6]), 0U);
    EXPECT_EQ(frames[7].kind, fil::cli::network_protocol::MessageKind::end);
}

TEST(WorldTest, ServeNetworkStopsOnCleanUnlimitedStdioEof) {
    TempWorldConfigs files;
    const auto network_path = files.writeNetwork({files.writeBoard("serve-eof.json", "serve-eof")});
    const std::string path = network_path.string();
    const std::vector<std::uint8_t> no_input;
    ScopedStdinPipe stdin_pipe;
    ASSERT_TRUE(stdin_pipe.redirect(no_input));

    const std::string_view args[]{"serve-network", path, "--transport", "stdio"};
    std::ostringstream out(std::ios::out | std::ios::binary);
    std::ostringstream err;
    EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::success);
    EXPECT_TRUE(err.str().empty());

    const std::string encoded = out.str();
    std::vector<std::uint8_t> bytes;
    for (const char byte : encoded) {
        bytes.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(byte)));
    }
    fil::cli::network_protocol::FrameDecoder decoder;
    ASSERT_TRUE(decoder.append(bytes));
    auto hello = decoder.next();
    ASSERT_TRUE(hello && hello.value().has_value());
    EXPECT_EQ(hello.value()->kind, fil::cli::network_protocol::MessageKind::hello);
    auto end = decoder.next();
    ASSERT_TRUE(end && end.value().has_value());
    EXPECT_EQ(end.value()->kind, fil::cli::network_protocol::MessageKind::end);
    ASSERT_GE(end.value()->payload.size(), 10U);
    EXPECT_EQ(end.value()->payload[8U], 1U); // clean EOF
    auto trailing = decoder.next();
    ASSERT_TRUE(trailing);
    EXPECT_FALSE(trailing.value().has_value());
    EXPECT_TRUE(decoder.finish());
}

TEST(WorldTest, NestedBusInjectionDoesNotSatisfyParentBusExpectation) {
    TempWorldConfigs files;
    const auto stimulus = files.writeStimulus(R"({
  "schema_version": 1, "name": "must-transmit", "events": [
    {"type":"can","at_ms":0,"bus":"vehicle/sub","id":"0x123","data":[1]}
  ], "expect": [
    {"type":"can","bus":"vehicle","id":"0x123","data":[1],"at_ms":0,"window_ms":0}
  ]
})");
    const auto network = files.writeNetwork(
        {files.writeBoard("inject-alpha.json", "inject-alpha", "vehicle/sub")}, stimulus, true);
    const std::string path = network.string();
    const std::string_view args[]{"run-network", path, "--duration-ms", "0",
        "--max-instructions", "10", "--inject-can", "vehicle/sub@0:0x123:01"};
    std::ostringstream out;
    std::ostringstream err;
    EXPECT_EQ(fil::cli::run(args, out, err), fil::cli::ExitCode::runtime_error);
    EXPECT_NE(out.str().find("window_ms=[0,0]: fail"), std::string::npos);
    EXPECT_NE(err.str().find("unmet CAN output expectation"), std::string::npos);
}

TEST(WorldTest, CanOutputTraceCarriesExactBusAndFirmwareOrigin) {
    TempWorldConfigs files;
    auto network = files.network({files.writeBoard("nested.json", "nested", "vehicle/sub")});
    network.buses.push_back({"vehicle/sub", 500000U});
    auto world = fil::sim::World::load(network);
    ASSERT_TRUE(world.hasValue()) << (world ? "" : world.error().message);
    auto* bus = world.value()->canBus("vehicle/sub");
    ASSERT_NE(bus, nullptr);
    const auto sender = bus->attach("firmware/node", false,
        [](const fil::devices::CanFrame&, std::uint64_t) {});
    ASSERT_TRUE(sender.hasValue());
    fil::devices::CanFrame frame;
    frame.id = 0x123U;
    frame.dlc = 1U;
    frame.data[0] = 1U;
    ASSERT_TRUE(bus->send(sender.value(), frame, 0U).hasValue());
    const auto found = std::find_if(world.value()->trace().records().begin(),
        world.value()->trace().records().end(), [](const fil::sim::TraceRecord& record) {
            if (record.type != "can_tx") return false;
            const auto has = [&](const std::string_view key, const std::string_view value) {
                return std::find(record.fields.begin(), record.fields.end(),
                    fil::sim::TraceField{std::string(key), std::string(value)}) != record.fields.end();
            };
            return has("bus", "vehicle/sub") && has("origin", "firmware/node");
        });
    EXPECT_NE(found, world.value()->trace().records().end());
}

TEST(WorldTest, ExecutesRunNetworkCli) {
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
    EXPECT_TRUE(result == fil::cli::ExitCode::success)
        << "run-network CLI executes a fixture world";
    EXPECT_TRUE(out.str().find("network: fixture-network") != std::string::npos)
        << "run-network CLI prints an aggregate summary";
    EXPECT_TRUE(out.str().find("board cli-alpha") != std::string::npos &&
                out.str().find("board cli-beta") != std::string::npos)
        << "run-network CLI prints per-board results";
    EXPECT_TRUE(err.str().empty()) << "successful run-network CLI has no error output";
    std::ifstream trace_input(trace_path, std::ios::binary);
    const std::string trace_text{
        std::istreambuf_iterator<char>(trace_input), std::istreambuf_iterator<char>()
    };
    EXPECT_TRUE(
        trace_text.find("\"source\":\"vehicle/external\",\"type\":\"can_tx\"") != std::string::npos)
        << "run-network CLI schedules an external CAN injection into the trace";
    EXPECT_TRUE(trace_text.find("\"source\":\"vehicle/external\",\"type\":\"can_tx\"") <
                trace_text.find("\"type\":\"instr\""))
        << "time-zero CAN injection is delivered before the first CPU instruction";
}

} // namespace
