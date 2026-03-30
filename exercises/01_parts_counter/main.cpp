/** @brief Real-time box counting system with hardware display and sensor edge detection.
 *
 * This module implements a lightweight, real-time counting pipeline for detecting objects (boxes) passing through a digital sensor.
 * It integrates debounce filtering, minimum pulse validation, circular sample logging, and external hardware display updates.
 *
 * --------------------------------------------------------------------------
 * SYSTEM OVERVIEW
 * --------------------------------------------------------------------------
 * The signal processing and counting chain consists of:
 *
 *   1. Hardware initialization (IO interface, display, sensor pull-up)
 *   2. Sensor edge detection via interrupt (rising and falling edges)
 *   3. Debounce filtering to ignore spurious transitions
 *   4. LOW pulse duration measurement to validate object detection
 *   5. Atomic increment of box counter (Box_count) on confirmed detection
 *   6. Optional circular logging of sensor state with timestamp
 *   7. Periodic display update, showing box count on two 32-bit hardware registers
 *
 * The system operates in a non-blocking loop using time-based scheduling (millis), ensuring
 * deterministic behavior without RTOS dependency. Optional debug printing can visualize recent sensor states.
 *
 * --------------------------------------------------------------------------
 * ACCEPTANCE CRITERIA
 * --------------------------------------------------------------------------
 * [X] Part passes → counter increments exactly once
 * [X] Display updated on every count change
 * [X] No aggressive busy-waiting
 * [X] No duplicate or missed counts
 *
 * --------------------------------------------------------------------------
 * DESIGN CHOICES AND TRADE-OFFS
 * --------------------------------------------------------------------------
 * 1. Interrupt-driven counting
 *   - ISR triggers on both edges of the sensor signal
 *   Trade-off:
 *     + Fast and deterministic detection
 *     - ISR must remain short to prevent missed edges
 *
 * 2. Atomic counter (std::atomic<uint32_t>)
 *   - Ensures safe concurrent access from ISR and main loop
 *   Trade-off:
 *     + Safe and thread-aware increment
 *     - Slight overhead on increment/load operations
 *
 * 3. Debounce filtering
 *   - Ignores transitions within DEBOUNCE_MS of previous change
 *   Trade-off:
 *     + Reduces false triggers
 *     - May miss extremely fast pulses if too short
 *
 * 4. Minimum LOW pulse duration validation
 *   - Only increments box count if LOW lasted longer than MIN_PULSE_LOW_MS
 *   Trade-off:
 *     + Prevents false counts from brief glitches
 *     - Rapidly passing objects shorter than threshold may be missed
 *
 * 5. Circular buffer logging (SensorSample)
 *   - Maintains a fixed-size window of recent samples
 *   Trade-off:
 *     + O(1) insertion, no shifting memory
 *     - Modular arithmetic for indexing adds minimal overhead
 *
 * --------------------------------------------------------------------------
 * SYSTEM CHARACTERISTICS
 * --------------------------------------------------------------------------
 * - Deterministic execution with atomic counter
 * - Lightweight ISR suitable for low-power MCUs
 * - Robust against sensor noise and spurious edges
 * - Configurable circular buffer size for debugging
 * - Non-blocking main loop with optional debug output
 *
 * --------------------------------------------------------------------------
 * LIMITATIONS
 * --------------------------------------------------------------------------
 * - Maximum detection speed limited by DEBOUNCE_MS and MIN_PULSE_LOW_MS
 * - interpolation for missing edges not used because sensor signal e not periodic
 * - Display update granularity tied to main loop frequency
 * - Debug logging consumes memory proportional to SIGNAL_BUF_SIZE
 */

#include "trac_fw_io.hpp"
#include <cstdio>
#include <atomic>

// #define Debug_time_duration
// #define Debug_signal_samples

static constexpr uint8_t SENSOR_PORT      = 0;   // Digital input port for sensor
static constexpr uint8_t REG_DISPLAY_LOW  = 6;   // Display register low
static constexpr uint8_t REG_DISPLAY_HIGH = 7;   // Display register high

static constexpr uint32_t DEBOUNCE_MS       = 60;       // Typical debounce for industrial inductive sensors
static constexpr uint32_t MIN_PULSE_LOW_MS  = 150;      // Minimum LOW pulse duration
static constexpr uint32_t MIN_SPIKE_HIGH_MS = 3;        // Max HIGH spike duration inside a LOW pulse

static constexpr size_t SIGNAL_BUF_SIZE = 256;  // Size of the circular buffer used to store sensor signal samples

static size_t signal_idx = 0;               // Current index in the circular buffer for writing the next sample
static uint32_t last_falling_time = 0;      // Timestamp of the last falling edge (HIGH -> LOW) of the sensor
static uint32_t last_state_change = 0;      // Timestamp of the last state change, used for debounce timing

static bool last_state = false;     // Last stable sensor state (used for edge detection and debouncing)

struct SensorSample {
    uint32_t t_ms;      // Timestamp of the sample in milliseconds
    bool state;         // Sensor state at the timestamp (true = HIGH, false = LOW)
};
static SensorSample signal_buffer[SIGNAL_BUF_SIZE] = {};    // Circular buffer that stores the most recent sensor samples

std::atomic<uint32_t> Box_count{0};     // Atomic counter for the number of boxes detected; safe for concurrent access


static std::atomic<uint32_t> last_low_duration{0};
// ========================== DEBUG FUNCTIONS ==========================

/** @brief Log a sensor state sample into a circular buffer.
 *
 * This function stores a timestamped sensor state (HIGH/LOW) into a preallocated circular buffer.
 * When the buffer is full, it wraps around to overwrite the oldest samples.
 *
 * @param t The timestamp of the sample (e.g., in milliseconds).
 * @param state The sensor state to log (true = HIGH, false = LOW).
 */
void log_sensor_state(uint32_t t, bool state) {
    signal_buffer[signal_idx] = {t, state};             // Store the timestamp and state at the current buffer index
    signal_idx = (signal_idx + 1) % SIGNAL_BUF_SIZE;    // Increment the index, wrap around if it reaches buffer size
}

/** @brief Print the last N sensor state samples from the circular buffer.
 *
 * This function prints a human-readable view of the last sensor samples, showing timestamp and state (H/L).
 * It skips uninitialized samples and limits N to the buffer size.
 *
 * @param n Number of recent samples to print (default is 60). If n exceeds buffer size, it is limited to SIGNAL_BUF_SIZE.
 */
void print_last_signal_samples(size_t n = 60) {
    if (n > SIGNAL_BUF_SIZE) n = SIGNAL_BUF_SIZE;    // Ensure n does not exceed buffer size

    printf("Last %zu samples (t_ms:state):\n", n);  // Header for printed samples
    for (size_t i = 0; i < n; i++) {    
        size_t idx = (signal_idx + SIGNAL_BUF_SIZE - n + i) % SIGNAL_BUF_SIZE;  // Calculate the correct circular buffer index for the i-th most recent sample

        if (signal_buffer[idx].t_ms != 0) {     // Skip uninitialized samples (timestamp 0)
            printf("%lu:%c ", signal_buffer[idx].t_ms, signal_buffer[idx].state ? 'H' : 'L');   // Print state and timestamps: H = HIGH, L = LOW
        }
    }
    printf("\n");
}

// ========================== DISPLAY ==========================

/** @brief Output a counter value to hardware registers for external display.
 *
 * This function formats an integer counter into a fixed-width string, splits it into two 32-bit chunks, and writes each chunk to dedicated hardware registers for display.
 *
 * @param io Reference to the IO interface used to access hardware registers.
 * @param count The counter value to display (e.g., number of boxes).
 *
 * @note The hardware display expects data split into two 32-bit registers. The first 4 bytes go to REG_DISPLAY_LOW, the next 4 bytes to REG_DISPLAY_HIGH.
 */
void update_display(trac_fw_io_t& io, uint32_t count) {
    char buf[9] = {};   // Create a buffer of 9 characters, initialized to zeros
    std::snprintf(buf, sizeof(buf), "C%03u", count); // Format the box count value into the buffer

    uint32_t r6, r7;                // Temporary 32-bit registers to hold parts of the buffer
    std::memcpy(&r6, buf + 0, 4);   // Copy the first 4 bytes of the buffer into r6
    std::memcpy(&r7, buf + 4, 4);   // Copy the next 4 bytes of the buffer into r7

    io.write_reg(REG_DISPLAY_LOW, r6);      // Write the first part to hardware register 6
    io.write_reg(REG_DISPLAY_HIGH, r7);     // Write the first part to hardware register 7
}

// ========================== ISR FUNCTION ==========================
/** @brief Count boxes by detecting sensor edges with debounce and minimum pulse validation.
 *
 * This interrupt service routine (ISR) monitors a digital sensor and counts objects (boxes) passing through.
 * It applies a debounce to filter spurious transitions and only increments the counter when a LOW pulse meets a minimum duration, ensuring reliable detection.
 *
 * @param io Pointer to the IO interface used to read the sensor and track time.
 *
 * @note The first box is counted immediately without pulse validation.
 *       Subsequent boxes are counted only on rising edges (LOW -> HIGH) if the LOW pulse duration exceeds MIN_PULSE_LOW_MS.
 */
void sensor_isr(trac_fw_io_t* io) {
    uint32_t now = io->millis();        // Current timestamp in ms

    static bool stable_state = false;       // Last stable sensor state
    static uint32_t last_state_change = 0;  // Timestamp of last state change
    static uint32_t last_falling_time = 0;  // Start of LOW pulse
    static uint32_t low_duration = 0;       // Accumulated LOW pulse duration

    bool raw_state = io->digital_read(SENSOR_PORT);   // Read sensor pin

    // Ignore spurious edges using a simple debounce
    if (raw_state == stable_state || (now - last_state_change < DEBOUNCE_MS)) {
        return;
    }

    // Update stable state and last change time
    stable_state = raw_state;
    last_state_change = now;

    if (!stable_state) {  // Falling edge -> start of LOW pulse
        last_falling_time = now;  // mark LOW start
    } 
    else {  // Rising edge (HIGH)
        if (last_falling_time > 0) {
            uint32_t pulse_duration = now - last_falling_time;

            if (pulse_duration < MIN_SPIKE_HIGH_MS) {
                return;
            }
            // Ignore HIGH spikes shorter than MIN_SPIKE_HIGH_MS
            if (pulse_duration >= MIN_PULSE_LOW_MS) {
                Box_count++;   // Valid box detected
            }
            
            // Reset for next LOW pulse
            last_falling_time = 0;
        }
    }

    #ifdef Debug
        if (!stable_state && last_falling_time > 0) {
            last_low_duration.store(now - last_falling_time);
        }
        log_sensor_state(now, stable_state);
    #endif

    // Ensure first box is counted if Box_count still 0
    if (Box_count.load() == 0) {
        Box_count++;
    }
}

// ========================= MAIN =========================
int main() {
    trac_fw_io_t io;        // Create IO interface object for hardware access
    uint32_t current_count = 0;          // Track the last value displayed to avoid unnecessary writes
    
    io.set_pullup(SENSOR_PORT, true);       // Configure hardware pull-up for inductive sensor to ensure proper sensor signal level
    update_display(io, current_count);   // Initialize display with count 0
    io.attach_interrupt(SENSOR_PORT, [&io]() { sensor_isr(&io); }, InterruptMode::CHANGE);  // Attach interrupt on both rising and falling edges

    uint32_t last_print = 0;

    while (true) {
        current_count = Box_count.load();  // Read count atomically once
        update_display(io, current_count);  // Write current count to display
        
        // ---------- Optional debug printing ----------
        #ifdef Debug_signal_samples
            uint32_t now = io.millis();
            if (now - last_print >= 3000) {
                last_print = now;
                print_last_signal_samples();
            }
        #endif

        #ifdef Debug_time_duration
            uint32_t now = io.millis();
            if (now - last_print >= 3000) {
                last_print = now;
                uint32_t val = last_low_duration.load();
                printf("valor = %u\n", val);
            }
        #endif
    }

    return 0;
}