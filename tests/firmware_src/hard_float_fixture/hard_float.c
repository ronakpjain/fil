typedef unsigned int uint32_t;
typedef int int32_t;

_Static_assert(sizeof(float) == 4, "fixture requires IEEE-754 binary32 storage");
_Static_assert(sizeof(uint32_t) == 4, "fixture requires 32-bit words");

volatile float hard_float_results[4];
volatile int32_t hard_signed_result;
volatile uint32_t hard_unsigned_result;
volatile uint32_t hard_float_complete;
/* Keep a file-backed RAM segment so strict ELF section validation sees the NOBITS extent. */
uint32_t hard_float_data_anchor = 0x48464c54U;

/* Float arguments and the return value use s0-s2 under the hard-float ABI. */
__attribute__((noinline))
float hard_mix(float left, float right, float scale) {
    return (left + right) * scale;
}

__attribute__((noinline))
float hard_divide(float numerator, float denominator) {
    return numerator / denominator;
}

__attribute__((noinline))
float hard_signed_to_float(int32_t value) {
    return (float)value;
}

__attribute__((noinline))
float hard_unsigned_to_float(uint32_t value) {
    return (float)value;
}

__attribute__((noinline))
int32_t hard_float_to_signed(float value) {
    return (int32_t)value;
}

__attribute__((noinline))
uint32_t hard_float_to_unsigned(float value) {
    return (uint32_t)value;
}

int main(void) {
    hard_float_results[0] = hard_mix(1.5f, 2.25f, 4.0f);
    hard_float_results[1] = hard_divide(9.0f, 4.0f);
    hard_float_results[2] = hard_signed_to_float(-37);
    hard_float_results[3] = hard_unsigned_to_float(42U);
    hard_signed_result = hard_float_to_signed(-12.75f);
    hard_unsigned_result = hard_float_to_unsigned(255.75f);
    hard_float_complete = 0xf00dcafeU;
    return 0;
}
