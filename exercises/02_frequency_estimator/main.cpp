/** @brief Real-time frequency estimator using band-limited Goertzel analysis.
 *
 * This module implements a lightweight, real-time frequency estimation pipeline designed for embedded systems with limited computational resources.
 *
 * --------------------------------------------------------------------------
 * SYSTEM OVERVIEW
 * --------------------------------------------------------------------------
 * The signal processing chain consists of:
 *
 *   1. Uniform sampling of the input signal (fixed sampling interval)
 *   2. DC offset removal (ADC centering)
 *   3. Bandpass filtering (biquad) to isolate the frequency range of interest
 *   4. Circular buffering of filtered samples
 *   5. Periodic frequency estimation using the Goertzel algorithm
 *   6. Temporal smoothing via Exponential Moving Average (EMA)
 *   7. Output stabilization using a hysteresis-based lock mechanism
 *
 * The system is designed to operate in a non-blocking loop using time-based scheduling (millis), ensuring deterministic behavior without RTOS dependency.
 * 
 * --------------------------------------------------------------------------
 * ACCEPTANCE CRITERIA
 * --------------------------------------------------------------------------
 * [X] Estimate within ±0.5 Hz of the actual frequency during steady state
 * [X] Tracks frequency changes — estimate converges within 1 second of a change
 * [X] Robust to signal disturbances — a transient does not permanently corrupt the estimate
 * [X] Fixed-rate sampling loop — no unbounded busy-wait on analog_read()
 * 
 * --------------------------------------------------------------------------
 * DESIGN CHOICES AND TRADE-OFFS
 * --------------------------------------------------------------------------
 * 1. Goertzel vs FFT
 *   - Goertzel is used instead of FFT because only a narrow frequency band is of interest.
 *   - Complexity is O(N * BINS) vs O(N log N) for FFT.
 *   Trade-off:
 *     + Lower computational cost for narrowband analysis
 *     - Not suitable for full-spectrum analysis
 *
 * 2. Bandpass Pre-filtering (Biquad)
 *   - Reduces out-of-band noise before spectral estimation
 *   - Improves robustness and peak detection reliability
 *   Trade-off:
 *     + Better SNR and cleaner spectral peak
 *     - Adds phase distortion and small computational overhead
 *
 * 3. Fixed Window Size (DATA_SIZE)
 *   - Defines frequency resolution: Δf = Fs / N
 *   Trade-off:
 *     + Larger window → better frequency resolution
 *     - Larger window → higher latency and slower response
 *
 * 4. Exponential Moving Average (EMA)
 *   - Smooths the estimated frequency over time
 *   Trade-off:
 *     + Reduces noise and jitter
 *     - Introduces lag (slower response to rapid changes)
 *
 * 5. Hysteresis-based Lock Mechanism
 *   - Updates output only when deviation exceeds a threshold
 *   Trade-off:
 *     + Prevents output jitter and small fluctuations
 *     - May delay response to small but real frequency changes
 *
 * 6. Circular Buffer
 *   - Maintains a sliding window of samples without memory shifting
 *   Trade-off:
 *     + O(1) insertion, no memory copy
 *     - Requires modular indexing (slight overhead)
 *
 * 7. Time-based Scheduling (millis)
 *   - Sampling and processing are decoupled using time intervals
 *   Trade-off:
 *     + Simple
 *     - Subject to jitter if loop execution time varies
 *
 * --------------------------------------------------------------------------
 * SYSTEM CHARACTERISTICS
 * --------------------------------------------------------------------------
 * - Deterministic execution (no dynamic memory allocation)
 * - Suitable for low-power MCUs
 * - Optimized for narrowband frequency tracking
 * - Moderate latency due to windowing and smoothing
 * - Robust against noise within the configured band
 *
 * --------------------------------------------------------------------------
 * LIMITATIONS
 * --------------------------------------------------------------------------
 * - No initial band estimation - Limited to scenarios where the frequency band of interest is predefined
 * - Frequency resolution limited by DATA_SIZE
 * - No sub-bin interpolation (quantized frequency output)
 * - Assumes a dominant single frequency component
 * - Performance depends on proper tuning of filter and EMA parameters
 */

#include <cmath>
#include <array>
#include "trac_fw_io.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// #define Debug

typedef struct {
    bool adc_overflow;      
    bool filter_clipping;   
    bool goertzel_fail;     
    bool ema_locked;           
} flags_t;

flags_t Failure_flags = { false, false, false, false};

// ========================= CONFIG =========================

static constexpr uint32_t SAMPLE_INTERVAL_MS = 4;           // Sampling interval in milliseconds (defines acquisition rate)
static constexpr float FS = 1000.0f / SAMPLE_INTERVAL_MS;   // Sampling frequency in Hz (Fs = 1 / Ts)
static constexpr size_t DATA_SIZE = 256;        // Number of samples in analysis window (DFT size / Goertzel window)
static constexpr float EMA_ALPHA = 0.6f;        // Exponential Moving Average (EMA) coefficient (Higher → smoother output, slower response)
static constexpr float LOCK_TOLERANCE = .1f;   // Frequency lock tolerance (Hz) Defines hysteresis band to suppress small fluctuations
static constexpr float ADC_MID = 2048.0f;       // ADC mid-scale value (used for DC offset removal)

// Frequency band of interest (Hz)
static constexpr float FREQ_MIN = 3.0f;
static constexpr float FREQ_MAX = 12.0f;

static constexpr float FC = (FREQ_MIN + FREQ_MAX) / 2;        // Bandpass filter center frequency (approximate midpoint of band)
static constexpr float Q = FC / (FREQ_MAX - FREQ_MIN);      // Quality factor (Q) of bandpass filter (Controls bandwidth: higher Q → narrower band)

// ========================= GOERTZEL COEFF =========================
static constexpr size_t K_MIN = static_cast<size_t>(FREQ_MIN * DATA_SIZE / FS);     // Convert frequency limits to DFT bin indices:
static constexpr size_t K_MAX = static_cast<size_t>(FREQ_MAX * DATA_SIZE / FS);     // Convert frequency limits to DFT bin indices:
static constexpr size_t NUM_BINS = K_MAX - K_MIN + 1;       // Total number of bins to evaluate within the band

// ========================= Structs =========================
struct GoertzelCoeff {  // Structure holding precomputed Goertzel coefficient
    float coeff;
};
std::array<GoertzelCoeff, NUM_BINS> g_coeffs;   // Array of coefficients for all bins in the target band Precomputed once to avoid runtime trigonometric operations

struct CircularBuffer {     // Circular buffer storing the most recent DATA_SIZE samples. Used as input window for Goertzel processing
    std::array<float, DATA_SIZE> x{};       // Sample storage
    size_t idx = 0;                         // Write index (wraps around)

    void push(float val) {  // Insert new sample into buffer (overwrites oldest data)
        x[idx++] = val;
        if(idx >= DATA_SIZE) idx = 0;   // Wrap-around
    }
};

struct Biquad {             // Implements second-order filter for signal conditioning
    float b0, b1, b2, a1, a2;               // Weights declaration for filtering calculation
    float x1=0, x2=0, y1=0, y2=0;           // Initial condition for states "x" and output "y"

    float process(float x) {         // Process one input sample through the filter
        float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }

    void setup_bandpass(float fs, float fc, float Q) {          // Calculates Weights for freq filtering. The coef. are calculated based on the center frequency (RBJ Cookbook formulas)
        //Reference: https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
        
        float w0 = 2.0f * M_PI * fc / fs;
        float alpha = sin(w0) / (2.0f * Q);
        float cos_w0 = cos(w0);
        float a0 = 1.0f + alpha;

        b0 = alpha / a0;
        b1 = 0.0f;
        b2 = -alpha / a0;
        a1 = -2.0f * cos_w0 / a0;
        a2 = (1.0f - alpha) / a0;
    }
};

struct EstimatorState {     // Holds all runtime state for the frequency estimator. Ensures persistence across loop iterations
    CircularBuffer buf{};   // Circular buffer storing recent filtered samples (input to Goertzel)
    Biquad filter{};        // Bandpass filter to isolate the frequency band of interest

    float freq_filtered = 0.0f; // Smoothed frequency estimate (EMA output)
    float freq_locked   = 0.0f; // Output frequency after hysteresis (stable/locked value)

    uint32_t last_calc_time   = 0; // Timestamp of last frequency estimation (processing rate control)
    uint32_t last_sample_time = 0; // Timestamp of last sample acquisition (sampling rate control)
};

// ========================= GOERTZEL =========================
/** @brief Initialize Goertzel coefficients for the target frequency band.
 * 
 * This function precomputes the cosine-based coefficients required by the
 * Goertzel algorithm for each frequency bin within the defined band
 * [FREQ_MIN, FREQ_MAX].
 *
 * The Goertzel algorithm evaluates individual DFT bins efficiently. Each bin requires a coefficient defined as:
 *     coeff = 2 * cos(2πk / N)
 * where:
 *   - k is the bin index corresponding to a target frequency
 *   - N is the total number of samples (DATA_SIZE)
 *
 * Precomputing these coefficients avoids recalculating trigonometric functions during runtime, significantly improving performance in real-time/embedded systems.
 *
 * Global dependencies:
 *   - K_MIN, K_MAX: define the frequency bin range of interest
 *   - NUM_BINS: total number of bins processed
 *   - DATA_SIZE: number of samples in the analysis window
 *   - g_coeffs: array storing precomputed coefficients
 *
 * Notes:
 *   - This function should be called once during system initialization.
 *   - Uses single-precision math (float) for efficiency on embedded targets.
 */
 void init_goertzel_coeffs() {
    for(size_t i = 0; i < NUM_BINS; ++i) {      // Iterate over frequency bins
        size_t k = K_MIN + i;                   // Map local index to actual DFT bin index
        float w = 2.0f * M_PI * k / DATA_SIZE;  // Compute normalized angular frequency: w = 2πk / N
        float cos_w = cosf(w);                  // Compute cosine of angular frequency
        g_coeffs[i].coeff = 2.0f * cos_w;       // Store Goertzel coefficient: 2 * cos(w)
    }
}

/** @brief Compute the power of a single frequency bin using the Goertzel algorithm.
 *
 * This function evaluates the magnitude of a specific DFT bin for the input signal stored in a circular buffer. using the standard Goertzel formula:
 *     s[n] = x[n] + coeff * s[n-1] - s[n-2]
 * where:
 *   - x[n] is the input sample
 *   - coeff = 2 * cos(2πk / N), precomputed for the target bin
 *
 * The algorithm processes DATA_SIZE samples and returns the power of the corresponding frequency bin without computing a full FFT.
 *
 * @param buf   Circular buffer containing the input signal samples
 * @param coeff Precomputed Goertzel coefficient for the target frequency bin
 * @return Power (magnitude squared) of the selected frequency bin
 *
 * Notes:
 *   - This function is performance-critical and should be kept lightweight.
 */
float goertzel_bin(const CircularBuffer &buf, float coeff) {
    float s0 = 0, s1 = 0, s2 = 0;

    size_t idx = buf.idx;   // Current write index in the circular buffer (start point for oldest sample)

    for(size_t n = 0; n < DATA_SIZE; ++n) {         // iterates over all samples in chronological order
        float x = buf.x[(idx + n) % DATA_SIZE];     // Access sample with circular wrap-around
        s0 = x + coeff * s1 - s2;                   // Goertzel recurrence relation
        s2 = s1;    // Shift states for next iteration
        s1 = s0;    // Shift states for next iteration
    }
    return s1*s1 + s2*s2 - coeff*s1*s2; // Compute magnitude squared (power) of the frequency bin
}

/** @brief Estimate the dominant frequency within a predefined band using Goertzel.
 *
 * This function scans a set of preselected DFT bins (defined by K_MIN to K_MAX) and identifies the frequency with the highest energy.
 *
 * @param buf Circular buffer containing the input signal samples
 *
 * @return Estimated dominant frequency in Hz within the configured band
 *
 * Algorithm:
 *   1. Iterate over all frequency bins in the target band.
 *   2. For each bin, compute its magnitude (power) using goertzel_bin().
 *   3. Track the bin index (k) with the highest magnitude.
 *   4. Convert the selected bin index to frequency in Hz.
 *
 * Details:
 *   - NUM_BINS defines how many bins are evaluated.
 *   - g_coeffs contains precomputed Goertzel coefficients for each bin.
 *   - K_MIN is the starting bin index corresponding to FREQ_MIN.
 *
 * Implementation notes:
 *   - best_mag is initialized to a negative value to ensure the first bin always updates the peak.
 *   - Only magnitude comparison is needed
 *   - The function assumes that the signal contains a dominant frequency component within the analyzed band.
 *
 * Performance considerations:
 *   - Complexity is O(NUM_BINS * DATA_SIZE).
 *   - Suitable for narrowband detection without full FFT overhead.
 *
 * Limitations:
 *   - Frequency resolution is limited by DATA_SIZE (Δf = Fs / N).
 *   - No interpolation is applied (peak is quantized to bin centers).
 *   - Sensitive to noise if signal energy is low.
 */
float goertzel_peak_freq(const CircularBuffer &buf) {
    float best_mag = -1.0f;
    size_t best_k = K_MIN;

    for(size_t i = 0; i < NUM_BINS; ++i) {                  // Scan all bins in the configured frequency band
        float mag = goertzel_bin(buf, g_coeffs[i].coeff);   // Compute magnitude for current bin
        if(mag > best_mag) {     // Track the bin with maximum energy
            best_mag = mag;
            best_k = K_MIN + i;
        }
    }

    return (float)best_k * FS / DATA_SIZE;  // Convert bin index to frequency in Hz
}

// ========================= LOOP =========================
/** @brief Main processing loop for signal acquisition and frequency estimation.
 *
 * This function performs two time-driven tasks:
 *   1. Sampling and filtering of the input signal
 *   2. Periodic frequency estimation using the Goertzel algorithm
 *
 * The execution is non-blocking and relies on time comparisons using millis().
 *
 * @param io    Hardware abstraction interface (ADC, time, register write)
 * @param state Persistent estimator state (buffer, filter, tracking variables)
 *
 * --------------------------------------------------------------------------
 * SAMPLING STAGE
 * --------------------------------------------------------------------------
 * - Executes at a fixed interval defined by SAMPLE_INTERVAL_MS.
 * - Reads raw ADC data from channel 0.
 * - Removes DC offset using ADC_MID.
 * - Applies a bandpass filter (biquad) to isolate the frequency band of interest.
 * - Stores the filtered sample into a circular buffer for later processing.
 *
 * --------------------------------------------------------------------------
 * PROCESSING STAGE
 * --------------------------------------------------------------------------
 * - Executes at a slower rate (every 100 ms).
 * - Computes the dominant frequency using goertzel_peak_freq().
 *
 * Steps:
 *   1. Frequency estimation from buffered samples
 *   2. Exponential Moving Average (EMA) filtering
 *   3. Lock mechanism to prevent small fluctuations on the output
 *   4. Write frequency to hardware output
 * 
 * --------------------------------------------------------------------------
 * PERFORMANCE NOTES
 * --------------------------------------------------------------------------
 * - Sampling path is lightweight (runs at high frequency)
 * - Processing path is heavier (Goertzel), but runs infrequently
 * - Designed for embedded systems with limited CPU resources
 * - Fully non-blocking (no delays)
 * - Deterministic execution based on time deltas
 * - Suitable for cooperative scheduling.
 */
void loop(trac_fw_io_t &io, EstimatorState &state) {
    uint32_t now = io.millis();     // Read the current system time in milliseconds

    // ================= SAMPLE =================
    if(now - state.last_sample_time >= SAMPLE_INTERVAL_MS) {            // Verifies sampling interval; skip processing signal if called too early
        state.last_sample_time = now;                                   // Update last sample time to current time
        float raw = static_cast<float>(io.analog_read(0)) - ADC_MID;    // Read analog input channel 0, convert to float, and subtract 2048 to center signal around zero (12 bit signal)
        if(raw + ADC_MID > 4095.0f || raw + ADC_MID < 0.0f) {
            Failure_flags.adc_overflow = true;
        #ifdef Debug
            printf("ADC Overflow detected\n");
        #endif
        }

        float filtered = state.filter.process(raw);                     // Apply Biquad bandpass filter to current sample to remove unwanted frequencies and isolate the band of interest
        if(filtered > 4095.0f || filtered < -4095.0f) {
            Failure_flags.filter_clipping = true;
        #ifdef Debug
            printf("Filter clipping detected\n");
        #endif
        }

        state.buf.push(filtered);                                       // Add the filtered sample into the circular FFT buffer for later frequency analysis
    }

    // ================= PROCESS =================
    if(now - state.last_calc_time >= 100) {         // Verifies calculation interval; skip processing signal if called too early
        state.last_calc_time = now;

        float freq = goertzel_peak_freq(state.buf); // Estimate dominant frequency using Goertzel
        if(freq < FREQ_MIN || freq > FREQ_MAX || std::isnan(freq)) {
            Failure_flags.goertzel_fail = true;
        #ifdef Debug
            printf("Goertzel peak out of range or NaN\n");
        #endif
        }

        state.freq_filtered = EMA_ALPHA * state.freq_filtered + (1.0f - EMA_ALPHA) * freq; // Apply exponential moving average (EMA) smoothing
        if(fabsf(state.freq_filtered - state.freq_locked) > LOCK_TOLERANCE) {
            Failure_flags.ema_locked = true;
        } else {
            Failure_flags.ema_locked = false;
        }

        float diff = state.freq_filtered - state.freq_locked;   // Compute deviation from locked frequency

        if(fabsf(diff) > LOCK_TOLERANCE) {              // Update only if deviation exceeds tolerance (reject small fluctuations/noise)
            state.freq_locked = state.freq_filtered;    // Accept new estimate: track real frequency changes while keeping output stable
        }

        uint32_t out = static_cast<uint32_t>(state.freq_locked * 100.0f);   // Multiply by 100 and cast to integer for hardware register

        io.write_reg(3, out);       // Write the new frequency value to hardware register 3
    }
}

// ========================= MAIN =========================
int main() {
    trac_fw_io_t io;                        // Initialize Hardware Abstraction Layer interface
    EstimatorState state;                   // Create struct that contains persistent data for FFT buffer and filtered output

    // init coeficientes Goertzel
    init_goertzel_coeffs();                     // initializes Goertzel Coefficients
    state.filter.setup_bandpass(FS, FC, Q);     // Calculates Bandpass coeficcients from sampling parameters

    state.last_sample_time = io.millis();
    state.last_calc_time = io.millis();

    while(true) {
        loop(io, state);    // Continuously run monitoring & estimation loop
    }

    return 0;
}