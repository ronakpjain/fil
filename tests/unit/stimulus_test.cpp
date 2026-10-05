#include "fil/sim/stimulus.hpp"
#include "fil/sim/event_loop.hpp"
#include "fil/sim/trace.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"

#include <cstdint>
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {

class TempStimulusFiles {
public:
    [[nodiscard]] std::filesystem::path write(
        const std::filesystem::path& relative,
        const std::string_view contents
    ) const {
        return directory_.write(relative, contents);
    }

    [[nodiscard]] std::filesystem::path writeBoard(const std::string_view name) const {
        const auto fixture = fil::test::fixtureBoardConfig(name);
        std::ostringstream json;
        json << "{\n"
             << "  \"schema_version\": 1,\n"
             << "  \"name\": \"" << name << "\",\n"
             << "  \"mcu\": \"" << fixture.mcu_path.string() << "\",\n"
             << "  \"elf\": \"" << fixture.elf_path.string() << "\",\n"
             << "  \"vector_base\": \"0x08000000\",\n"
             << "  \"can\": {\"FDCAN1\": {\"bus\": \"vehicle\", \"loopback\": false}}\n"
             << "}\n";
        return write(std::string(name) + ".json", json.str());
    }

    [[nodiscard]] fil::config::NetworkConfig network(
        const std::filesystem::path& board
    ) const {
        fil::config::NetworkConfig config;
        config.name = "stimulus-network";
        config.source_path = directory_.root() / "network.json";
        config.buses.push_back({"vehicle", 500000U});
        config.board_paths.push_back(board);
        return config;
    }

private:
    fil::test::TemporaryDirectory directory_{"fil-stimulus-tests"};
};

TEST(StimulusTest, LoadsMixedInputsAndRepeatSchedule) {
    TempStimulusFiles files;
    const auto path = files.write(
        "mixed.json",
        R"({
  "schema_version": 1,
  "name": "mixed-inputs",
  "events": [
    {"type":"can","at_ms":5,"repeat":{"every_ms":2,"count":3},"bus":"vehicle","id":"0x321","data":[1,2]},
    {"type":"gpio","at_ms":5,"board":"alpha","pin":"PA0","value":true},
    {"type":"adc","at_ms":5,"board":"alpha","instance":"ADC1","channel":2,"value":2048},
    {"type":"usart","at_ms":5,"board":"alpha","instance":"USART1","bytes":[65,66]}
  ]
})"
    );

    const auto script = fil::sim::loadStimulusScript(path);
    ASSERT_TRUE(script.hasValue()) << script.error().message;
    EXPECT_EQ(script.value().name, "mixed-inputs");
    ASSERT_EQ(script.value().events.size(), 4U);
    EXPECT_EQ(script.value().events[0].at_ms, 5U);
    EXPECT_EQ(script.value().events[0].repeat_every_ms, 2U);
    EXPECT_EQ(script.value().events[0].repeat_count, 3U);

    const auto* can = std::get_if<fil::sim::CanStimulus>(&script.value().events[0].input);
    ASSERT_NE(can, nullptr);
    EXPECT_EQ(can->bus, "vehicle");
    EXPECT_EQ(can->id, 0x321U);
    EXPECT_EQ(can->data, (std::vector<std::uint8_t>{1U, 2U}));

    const auto* gpio = std::get_if<fil::sim::GpioStimulus>(&script.value().events[1].input);
    ASSERT_NE(gpio, nullptr);
    EXPECT_EQ(gpio->port, "GPIOA");
    EXPECT_EQ(gpio->pin, 0U);
    ASSERT_TRUE(gpio->level.has_value());
    EXPECT_TRUE(gpio->level.value());

    const auto* adc = std::get_if<fil::sim::AdcStimulus>(&script.value().events[2].input);
    ASSERT_NE(adc, nullptr);
    EXPECT_EQ(adc->instance, "ADC1");
    EXPECT_EQ(adc->channel, 2U);
    EXPECT_EQ(adc->value, 2048U);

    const auto* usart = std::get_if<fil::sim::UsartStimulus>(&script.value().events[3].input);
    ASSERT_NE(usart, nullptr);
    EXPECT_EQ(usart->instance, "USART1");
    EXPECT_EQ(usart->bytes, (std::vector<std::uint8_t>{65U, 66U}));
}

TEST(StimulusTest, ParsesCanOutputExpectationsAndRejectsInvalidWindows) {
    TempStimulusFiles files;
    const auto path = files.write("expect.json", R"({
  "schema_version": 1, "name": "outputs", "events": [{"type":"usart","at_ms":0,"board":"a","instance":"USART1","bytes":[]}],
  "expect": [{"type":"can","bus":"vehicle","id":"0x321","data":[1,2],"at_ms":10,"window_ms":5}]
})");
    const auto script = fil::sim::loadStimulusScript(path);
    ASSERT_TRUE(script.hasValue()) << script.error().message;
    ASSERT_EQ(script.value().expect.size(), 1U);
    EXPECT_EQ(script.value().expect[0].id, 0x321U);
    EXPECT_EQ(script.value().expect[0].at_ms, 10U);
    EXPECT_EQ(script.value().expect[0].window_ms, 5U);

    const auto invalid = files.write("bad-window.json", R"({
  "schema_version": 1, "name": "bad-window", "events": [{"type":"usart","at_ms":0,"board":"a","instance":"USART1","bytes":[]}],
  "expect": [{"type":"can","bus":"vehicle","id":1,"data":[256],"at_ms":0,"window_ms":1}]
})");
    EXPECT_FALSE(fil::sim::loadStimulusScript(invalid).hasValue());
}

TEST(StimulusTest, ExpectationMatchesExactBusAndFirmwareOrigin) {
    const fil::sim::CanExpectation expected{"vehicle", 0x123U, false, {1U}, 0U, 10U};
    const fil::sim::TraceRecord nested_firmware{5'000'000U, 0U, "vehicle/sub/node", "can_tx", {
        {"bus", "vehicle/sub"}, {"origin", "node"}, {"id", "0x123"},
        {"extended", "false"}, {"data", "01"},
    }};
    EXPECT_FALSE(fil::sim::matchesCanExpectation(expected, nested_firmware));
    auto parent_firmware = nested_firmware;
    parent_firmware.fields[0].second = "vehicle";
    EXPECT_TRUE(fil::sim::matchesCanExpectation(expected, parent_firmware));
    auto nested_injection = nested_firmware;
    nested_injection.fields[1].second = "external";
    EXPECT_FALSE(fil::sim::matchesCanExpectation(expected, nested_injection));
}

TEST(StimulusTest, EvaluatorEmitsLivePendingThenPassAndEscapedTrace) {
    fil::sim::TraceRecorder trace;
    trace.setTypeAllowlist({"expectation_pending", "expectation_pass", "expectation_fail", "expectation_incomplete", "can_tx"});
    const fil::sim::CanExpectationCheck check{
        "script\"one", 3U, fil::sim::CanExpectation{"vehicle", 0x123U, false, {0x01U, 0xffU}, 2U, 4U}
    };
    fil::sim::EventLoop loop;
    fil::sim::CanExpectationEvaluator evaluator(trace, {check});
    std::vector<std::string> observed;
    trace.setObserver([&](const fil::sim::TraceRecord& record) {
        evaluator.observe(record);
        if (record.type.starts_with("expectation_")) observed.push_back(record.type);
    });
    evaluator.begin(0U, loop);
    ASSERT_EQ(observed, (std::vector<std::string>{"expectation_pending"}));

    fil::sim::CanTraceFrame frame;
    frame.id = 0x123U;
    frame.dlc = 2U;
    frame.data = {0x01U, 0xffU};
    static_cast<void>(trace.recordCanFrame(3'000'000U, "vehicle/node", true, frame,
        "vehicle", "node"));
    EXPECT_EQ(observed, (std::vector<std::string>{"expectation_pending", "expectation_pass"}));
    static_cast<void>(trace.recordCanFrame(4'000'000U, "vehicle/node", true, frame,
        "vehicle", "node"));
    EXPECT_EQ(std::count(observed.begin(), observed.end(), "expectation_pass"), 1);
    EXPECT_EQ(evaluator.status(0U), "pass");
    EXPECT_FALSE(evaluator.hasFailures());
    const std::string jsonl = trace.jsonLines();
    EXPECT_NE(jsonl.find("script\\\"one/expect/3"), std::string::npos);
    EXPECT_NE(jsonl.find("\"matched_time_ns\":\"3000000\""), std::string::npos);
    EXPECT_NE(jsonl.find("\"expected_data\":\"01ff\""), std::string::npos);
}

TEST(StimulusTest, EvaluatorFailsOnlyAfterInclusiveWindowAndCanFinishIncomplete) {
    fil::sim::TraceRecorder trace;
    fil::sim::EventLoop loop;
    fil::sim::CanExpectationEvaluator evaluator(trace, {{
        "deadline", 0U, fil::sim::CanExpectation{"vehicle", 0x123U, false, {1U}, 1U, 2U}
    }});
    evaluator.begin(0U, loop);
    static_cast<void>(loop.runDueEvents(3'000'000U));
    EXPECT_EQ(evaluator.status(0U), "pending");
    static_cast<void>(loop.runDueEvents(3'000'001U));
    EXPECT_EQ(evaluator.status(0U), "fail");
    static_cast<void>(loop.runDueEvents(4'000'000U));
    EXPECT_EQ(std::count_if(trace.records().begin(), trace.records().end(), [](const auto& record) {
        return record.type == "expectation_fail";
    }), 1);
    EXPECT_TRUE(evaluator.hasFailures());

    fil::sim::CanExpectationEvaluator early(trace, {{
        "early", 0U, fil::sim::CanExpectation{"vehicle", 0x321U, false, {}, 50U, 1U}
    }});
    early.begin(loop.now(), loop);
    early.finish(loop.now());
    EXPECT_EQ(early.status(0U), "incomplete");
    EXPECT_TRUE(early.hasFailures());
}

TEST(StimulusTest, EvaluatorCancelsDeadlinesOnFinishAndDestruction) {
    fil::sim::TraceRecorder trace;
    fil::sim::EventLoop loop;
    {
        fil::sim::CanExpectationEvaluator evaluator(trace, {{
            "finish", 0U, fil::sim::CanExpectation{"vehicle", 0x123U, false, {}, 10U, 1U}
        }});
        evaluator.begin(loop.now(), loop);
        EXPECT_EQ(loop.pending(), 1U);
        evaluator.finish(loop.now());
        EXPECT_EQ(loop.pending(), 0U);
    }
    EXPECT_EQ(trace.records().back().type, "expectation_incomplete");
    {
        fil::sim::CanExpectationEvaluator evaluator(trace, {{
            "destroy", 0U, fil::sim::CanExpectation{"vehicle", 0x123U, false, {}, 20U, 1U}
        }});
        evaluator.begin(loop.now(), loop);
        EXPECT_EQ(loop.pending(), 1U);
    }
    EXPECT_EQ(loop.pending(), 0U);
    const std::size_t records_before = trace.records().size();
    static_cast<void>(loop.runDueEvents(25'000'000U));
    EXPECT_EQ(trace.records().size(), records_before);
}

TEST(StimulusTest, EvaluatorResolvesAlreadyExpiredWindowWithoutPastCallback) {
    fil::sim::TraceRecorder trace;
    fil::sim::EventLoop loop;
    static_cast<void>(loop.runDueEvents(3'000'001U));
    fil::sim::CanExpectationEvaluator evaluator(trace, {{
        "late", 0U, fil::sim::CanExpectation{"vehicle", 0x123U, false, {}, 1U, 2U}
    }});
    evaluator.begin(loop.now(), loop);
    EXPECT_EQ(evaluator.status(0U), "fail");
    EXPECT_EQ(loop.pending(), 0U);
    ASSERT_EQ(trace.records().size(), 2U);
    EXPECT_EQ(trace.records()[0].type, "expectation_pending");
    EXPECT_EQ(trace.records()[1].type, "expectation_fail");
}

TEST(StimulusTest, EvaluatorValidatesTimeAndDirectWindowOverflow) {
    fil::sim::TraceRecorder trace;
    fil::sim::EventLoop loop;
    fil::sim::CanExpectationEvaluator evaluator(trace, {});
    EXPECT_THROW(evaluator.begin(1U, loop), std::invalid_argument);
    EXPECT_THROW((fil::sim::CanExpectationEvaluator(trace, {{
        "overflow", 0U, fil::sim::CanExpectation{"vehicle", 1U, false, {},
            std::numeric_limits<std::uint64_t>::max(), 0U}
    }})), std::invalid_argument);
}

TEST(StimulusTest, RejectsInvalidEvents) {
    TempStimulusFiles files;
    const auto path = files.write(
        "invalid.json",
        R"({
  "schema_version": 1,
  "name": "invalid",
  "events": [
    {"type":"can","at_ms":0,"bus":"vehicle","id":"0x800","extended":false,"data":[0]}
  ]
})"
    );

    const auto script = fil::sim::loadStimulusScript(path);
    ASSERT_FALSE(script.hasValue());
    EXPECT_NE(script.error().message.find("standard CAN identifiers"), std::string::npos);
}

TEST(StimulusTest, SchedulesCanGpioAdcAndUsartInputs) {
    TempStimulusFiles files;
    const auto board_path = files.writeBoard("alpha");
    auto world = fil::sim::World::load(files.network(board_path));
    ASSERT_TRUE(world.hasValue());

    const auto path = files.write(
        "script.json",
        R"({
  "schema_version": 1,
  "name": "scheduled-inputs",
  "events": [
    {"type":"can","at_ms":5,"repeat":{"every_ms":2,"count":3},"bus":"vehicle","id":"0x321","data":[1,2]},
    {"type":"gpio","at_ms":5,"board":"alpha","pin":"PA0","value":true},
    {"type":"adc","at_ms":5,"board":"alpha","instance":"ADC1","channel":2,"value":2048},
    {"type":"usart","at_ms":5,"board":"alpha","instance":"USART1","bytes":[65,66]}
  ]
})"
    );
    auto script = fil::sim::loadStimulusScript(path);
    ASSERT_TRUE(script.hasValue()) << script.error().message;
    ASSERT_TRUE(fil::sim::scheduleStimulusScript(script.value(), *world.value()).hasValue());

    auto* board = world.value()->board("alpha");
    ASSERT_NE(board, nullptr);
    auto* gpio = board->peripherals().gpio("GPIOA");
    auto* adc = board->peripherals().adc("ADC1");
    auto* usart = board->peripherals().usart("USART1");
    ASSERT_NE(gpio, nullptr);
    ASSERT_NE(adc, nullptr);
    ASSERT_NE(usart, nullptr);
    adc->setChannelProvider([](const std::uint32_t, const fil::sim::SimTimeNs) {
        return std::uint16_t{123U};
    });
    adc->setConversionDelay(5'000U);
    ASSERT_TRUE(board->memory().write32(0x4002104cU, 1U << 13U));
    ASSERT_TRUE(board->memory().write32(0x40021088U, 2U << 28U));

    const auto result = world.value()->eventLoop().runDueEvents(9'000'000U);
    EXPECT_EQ(result.events_executed, 7U);
    EXPECT_TRUE(usart->hasRxData());

    bool gpio_input_high = false;
    std::vector<fil::sim::SimTimeNs> can_times;
    for (const fil::sim::TraceRecord& record : world.value()->trace().records()) {
        if (record.source == "alpha.GPIOA" && record.type == "gpio_input" &&
            record.time_ns == 5'000'000U) {
            gpio_input_high = std::find(record.fields.begin(), record.fields.end(),
                fil::sim::TraceField{"value", "1"}) != record.fields.end();
        }
        if (record.source == "vehicle/external" && record.type == "can_tx") {
            can_times.push_back(record.time_ns);
        }
    }
    EXPECT_TRUE(gpio_input_high);
    EXPECT_EQ(can_times, (std::vector<fil::sim::SimTimeNs>{5'000'000U, 7'000'000U, 9'000'000U}));

    const fil::mem::AccessContext write_context{fil::mem::AccessType::data_write, 0U};
    const fil::mem::AccessContext read_context{fil::mem::AccessType::data_read, 0U};
    ASSERT_TRUE(adc->write(0x30U, fil::mem::AccessSize::word, 2U << 6U, write_context).hasValue());
    ASSERT_TRUE(adc->write(0x08U, fil::mem::AccessSize::word, 1U | (1U << 2U), write_context).hasValue());
    EXPECT_EQ(world.value()->eventLoop().runDueEvents(9'005'000U).events_executed, 1U);
    const auto adc_data = adc->read(0x40U, fil::mem::AccessSize::word, read_context);
    ASSERT_TRUE(adc_data.hasValue());
    EXPECT_EQ(adc_data.value(), 2048U);
}

TEST(StimulusTest, ValidatesExpectationTargetsBeforeScheduling) {
    TempStimulusFiles files;
    auto world = fil::sim::World::load(files.network(files.writeBoard("alpha")));
    ASSERT_TRUE(world.hasValue());
    fil::sim::StimulusScript script;
    script.name = "bad-expectation-target";
    script.source_path = files.write("target.json", "{}");
    script.events.push_back(fil::sim::StimulusEvent{
        1U, 0U, 1U, fil::sim::UsartStimulus{"alpha", "USART1", {}}
    });
    script.expect.push_back(fil::sim::CanExpectation{"missing", 1U, false, {}, 0U, 1U});
    const std::size_t pending_before = world.value()->eventLoop().pending();
    const auto result = fil::sim::scheduleStimulusScript(script, *world.value());
    ASSERT_FALSE(result.hasValue());
    EXPECT_NE(result.error().message.find("expectation names undeclared CAN bus"), std::string::npos);
    EXPECT_EQ(world.value()->eventLoop().pending(), pending_before);
}

TEST(StimulusTest, ValidatesAllTargetsBeforeScheduling) {
    TempStimulusFiles files;
    auto world = fil::sim::World::load(files.network(files.writeBoard("alpha")));
    ASSERT_TRUE(world.hasValue());

    fil::sim::StimulusScript script;
    script.name = "bad-target";
    script.source_path = files.write("target.json", "{}");
    script.events.push_back(fil::sim::StimulusEvent{
        1U, 0U, 1U, fil::sim::CanStimulus{"missing", 0x123U, false, false, false, {1U}}
    });
    const std::size_t pending_before = world.value()->eventLoop().pending();
    const auto result = fil::sim::scheduleStimulusScript(script, *world.value());

    ASSERT_FALSE(result.hasValue());
    EXPECT_NE(result.error().message.find("undeclared CAN bus 'missing'"), std::string::npos);
    EXPECT_EQ(world.value()->eventLoop().pending(), pending_before);
}

} // namespace
