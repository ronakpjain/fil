#include "fil/sim/stimulus.hpp"
#include "fil/sim/board.hpp"
#include "fil/sim/world.hpp"
#include "fil/mem/memory_bus.hpp"
#include "fil/stm32g4/stm32g4.hpp"
#include "../fixture_support.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <sstream>
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
    adc->setChannelProvider([](const unsigned int, const fil::sim::SimTimeNs) {
        return std::uint16_t{123U};
    });
    adc->setConversionDelay(5'000U);

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
