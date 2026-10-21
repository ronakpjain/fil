# FLASH controller

The FLASH register model supports main/option key unlock and relock, protected writes to locked control registers, selected ACR field mask/readback and self-clearing cache-reset pulses, SR write-one-to-clear behavior, and synchronous selected-page erase through attached backing/callback. ACR startup/reset behavior is also observable by timing code.

**Use:** unlock with the documented key sequence before control-register operations; set page/bank erase fields and start to invoke the model erase. Erase backing must be attached for effects on mapped flash. Direct mapped-flash writes are a simulator facility, not FLASH programming.

**Limits:** erase is immediate with no busy interval; programming sequence, mass erase, option-byte reload, ECC, ART cache contents, and full category-3 bank/DBANK geometry are not modeled. Hardware errors/lock bus-fault behavior are not reproduced.

**RM0440:** §3.3.3; §3.5.5–3.5.6; §3.7.1 `FLASH_ACR`; §3.7.3 `KEYR`; §3.7.4 `OPTKEYR`; §3.7.5 `SR`; §3.7.6 `CR`.

**Tests:** `FlashPeripheralTest.KeySequencesUnlockAndRelockControlAndOptionLock`; `FlashPeripheralTest.StatusW1cAndPageEraseMapsBankAndPage`; `FlashPeripheralTest.ErasesSelectedPageThroughMappedBacking`; `RealTimingTest.FlashResetEnablesCachesAndOneWaitState`.

See the [coverage matrix](../stm32g4_peripheral_coverage.md#flash-category-3) for detailed scope and caveats.
