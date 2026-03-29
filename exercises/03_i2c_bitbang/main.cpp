/** @brief I2C Bit-Bang Sensor Reader and Display Firmware Overview
 *
 * This firmware implements a lightweight, software-driven ("bit-bang") I2C master to interface with a temperature sensor (TMP64) and a humidity sensor (HMD10).
 *
 * --------------------------------------------------------------------------
 * SYSTEM OVERVIEW
 * --------------------------------------------------------------------------
 * The program flow is as follows:
 *    1. Configure SCL and SDA pins with internal pull-ups for open-drain I2C operation.
 *    2. Initialize the I2C bus to idle state (both lines high).
 *    3. Scan the I2C bus (addresses 0x08–0x77) to detect connected sensors and identify them using their WHO_AM_I registers.
 *          The first responding device that is not TMP64 is assumed to be the humidity sensor.
 *    4. Enter an infinite main loop that reads temperature and humidity approximately once per second, using `io.millis()` and a small delay to avoid busy-waiting.
 *    5. Format readings and send them to dedicated hardware display registers (split into two 32-bit chunks each).
 *
 * -------------------------------------------------------------------------
 * ACCEPTANCE CRITERIA
 * --------------------------------------------------------------------------
 * [X] WHO_AM_I of TMP64 reads the expected value and is printed on startup
 * [X] Temperature value is correct
 * [X] Temperature is printed every ~1 s via printf
 * [X] Display registers 6–7 updated with each temperature reading (LCD line 0)
 * [X] Bus scan runs once at startup and prints all responding addresses
 * [X] Humidity sensor is identified and its WHO_AM_I is printed
 * [X] Humidity value is printed every ~1 s via printf
 * [X] Display registers 4–5 updated with each humidity reading (LCD line 1)
 * [X] No busy-waiting — use io.millis() for timing
 * 
 * --------------------------------------------------------------------------
 * DESIGN CHOICES AND TRADE-OFFS
 * --------------------------------------------------------------------------
 * 1. Bit-bang I2C
 *   - I2C protocol is implemented manually using GPIO toggling for SCL and SDA
 *   Trade-off:
 *     + Simple, portable, no hardware I2C peripheral required
 *     - CPU must actively toggle pins, limiting multitasking and maximum speed
 *
 * 2. Polling with io.millis()
 *   - Used to schedule ~1Hz sensor readings without full busy-waiting.
 *   Trade-off:
 *     + Minimal CPU usage outside I2C transactions
 *     - Still uses realy small blocking delay during I2C bit-banging
 *
 * 3. Fixed sensor assumptions
 *   - TMP64 is assumed as the temperature sensor; first other responding device is humidity sensor.
 *   - Trade-off:
 *     + Simple device identification
 *     - Cannot handle multiple sensors of same type or unknown devices easily
 *
 * 4. Display register mapping
 *   - Sensor values are split into two 32-bit registers for external display.
 *   - Trade-off:
 *     + Matches hardware requirements directly
 *     - Adds complexity in formatting and memory copying
 *
 * --------------------------------------------------------------------------
 * SYSTEM CHARACTERISTICS
 * --------------------------------------------------------------------------
 * - Software I2C master with START/STOP, read/write, ACK/NACK handling.
 * - Automatic bus scanning with sensor presence flags.
 * - 32-bit integer reading from sensors scaled to float.
 * - Human-readable and hardware-friendly formatted output.
 * - Approximate 1Hz periodic reading with minimal CPU usage outside I2C transactions.
 * - Communication Error handling with retries.
 *
 * --------------------------------------------------------------------------
 * LIMITATIONS
 * --------------------------------------------------------------------------
 * - Bit-bang I2C speed is limited due to lack of hardware I²C peripheral; not suitable for high-speed sensors.
 * - Since sensors doesn't have configurable addresses, multiple devices with the same address cannot coexist on the same bus without additional hardware
 * - Assumes big-endian byte order from sensors.
 * - the use of "i2c_delay" is blocking, during this none other task can be executed
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "trac_fw_io.hpp"

// #define Debug

trac_fw_io_t io;

typedef struct {
    bool Temp_presente;
    bool Hum_presente;
} flags_t;

flags_t Sensor_flag;    // Flag to confirm sensor presence

static constexpr uint8_t PIN_SCL = 8;   // SCL pin
static constexpr uint8_t PIN_SDA = 9;   // SDA pin

static constexpr uint8_t TMP64_ADDR = 0x48;   // Temperature sensor Address
static constexpr uint8_t REG_WHOAMI = 0x0F;   // WHO_AM_I address (returns device ID)
static constexpr uint8_t REG_DATA   = 0x00;   // Data address (read sensor measurement)

static uint8_t humidity_addr = 0xFF;   // Humidity sensor Address (Initialized as invalid value)
 
// ========================= Low-level GPIO helpers =========================
inline void scl_high() { io.digital_write(PIN_SCL, 1); }  // Set SCL pin high
inline void scl_low()  { io.digital_write(PIN_SCL, 0); }  // Set SCL pin low

inline void sda_high() { io.digital_write(PIN_SDA, 1); }  // Set SDA pin high
inline void sda_low()  { io.digital_write(PIN_SDA, 0); }  // Set SDA pin low

inline bool sda_read() { return io.digital_read(PIN_SDA); } // Read current state of SDA line

/** @brief Short busy-wait delay for I2C bit-banging.
 *
 * This function provides a very short, CPU-bound delay used to control the timing between SDA and SCL transitions in software I2C ("bit-bang").
 * 
 * Advantages:
 *   - Extremely fast and lightweight.
 *   - Ensures sufficient timing between bit transitions for reliable I2C communication.
 *
 * Disadvantages:
 *   - Timing is CPU-dependent and not perfectly precise in real units of time.
 *   - Busy-wait consumes CPU cycles (though minimal for very short loops).
 *   - Not suitable for millisecond-level delays; system delay functions (e.g., delay(ms)) are too long for I2C bit-bang.
 */
inline void i2c_delay() { for (volatile int i = 0; i < 50; i++); }

// ========================= I2c Functions =========================

/** @brief Generate an I2C START condition on the bus.
 *
 * The START condition is defined by SDA transitioning from high to low while SCL is high.
 * This signals the beginning of an I2C transaction to all devices on the bus.
 */
void i2c_start() {
    sda_high();
    scl_high();
    i2c_delay();

    sda_low();
    i2c_delay();

    scl_low();
}

/** @brief Generate an I2C STOP condition on the bus.
 *
 * The STOP condition is defined by SDA transitioning from low to high while SCL is high.
 * This signals the end of an I2C transaction to all devices on the bus.
 */
void i2c_stop() {
    sda_low();
    scl_high();
    i2c_delay();

    sda_high();
    i2c_delay();
}


/** @brief Write a single byte over the I2C bus and check for ACK.
 *
 * This function shifts out each bit of the byte, MSB first, on the SDA line while
 * toggling the SCL line. After sending 8 bits, it releases SDA to read the ACK
 * from the slave device.
 *
 * @param byte The 8-bit data to transmit over I2C.
 * @return true if the slave device acknowledged (ACK received), false otherwise (NACK).
 *
 * @note Uses software "bit-banging" to implement I2C protocol.
 */
bool i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {       // Send each of the 8 bits, MSB first
        if (byte & 0x80){       // Check most significant bit
            sda_high();         // Set SDA high for bit=1
        }
        else{ sda_low(); }    // Set SDA low for bit=0

        // ---------- Generate clock pulse ----------
        scl_high();     // SCL high
        i2c_delay();    // Short delay for timing
        scl_low();      // SCL low
        i2c_delay();    // Short delay for timing

        byte <<= 1;     // Shift left to prepare next bit
    }

    // ---------- Handle ACK from slave ----------
    sda_high();     // Release SDA line for slave to pull low if ACK
    scl_high();     // Clock pulse to read ACK
    i2c_delay();

    bool ack = !sda_read();     // ACK = SDA low (true), NACK = SDA high (false)

    scl_low();      // End clock pulse
    i2c_delay();

    return ack;     // Return true if ACK received
}

/** @brief Read a single byte from the I2C bus.
 *
 * This function reads 8 bits from the SDA line, MSB first, while generating clock pulses on SCL. After reading, it sends an ACK or NACK back to the slave.
 *
 * @param ack If true, send ACK after reading (expecting more bytes). If false, send NACK.
 * @return The byte read from the slave device.
 *
 * @note Uses software "bit-banging" to implement I2C protocol.
 */
uint8_t i2c_read_byte(bool ack) {
    uint8_t byte = 0;

    sda_high();     // Release SDA line so slave can drive it

    for (int i = 0; i < 8; i++) {   // Read 8 bits, MSB first
        scl_high();
        i2c_delay();

        byte = (byte << 1) | (sda_read() ? 1 : 0);      // Shift left and read bit from SDA

        scl_low();      // Clock low
        i2c_delay();
    }

    // envia ACK/NACK
    if (ack) sda_low();     // Pull SDA low for ACK
    else     sda_high();    // Release SDA for NACK

    scl_high();     // Clock pulse for ACK/NACK
    i2c_delay();
    scl_low();
    i2c_delay();

    sda_high();     // Release SDA line for next operation

    return byte;    // Return the received byte
}

/** @brief Probe an I2C device to check if it acknowledges its address.
 *
 * This function sends a START condition, writes the 7-bit address with the write bit, and checks if the device responds with an ACK. It then sends a STOP condition.
 *
 * @param addr The 7-bit I2C address to probe.
 * @return true if the device responds (ACK received), false otherwise.
 */
bool i2c_probe(uint8_t addr) {
    i2c_start();        // Send I2C START condition
    bool ack = i2c_write_byte((addr << 1) | 0);     // Send the address with write bit (0) and store ACK result
    i2c_stop();     // Send I2C STOP condition
    return ack;     // Return whether the device acknowledged
}

/** @brief Read one or more bytes from a specific register of an I2C device.
 *
 * This function performs a standard I2C register read with a repeated start:
 * 1. START
 * 2. Send device address with write bit
 * 3. Send register address
 * 4. REPEATED START
 * 5. Send device address with read bit
 * 6. Read 'len' bytes
 * 7. STOP
 *
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The register address to read from.
 * @param data Pointer to buffer where read bytes will be stored.
 * @param len Number of bytes to read.
 * @return true if the read succeeded, false if any ACK failed during the transaction.
 *
 * @note The function assumes that 'data' points to a valid buffer of at least 'len' bytes.
 */
bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t* data, int len) {
    i2c_start();    // Send I2C START condition

    if (!i2c_write_byte((addr << 1) | 0)) {     // Send device address with write bit;
        i2c_stop();                             // Stop and return false if no ACK
        return false;
    }

    if (!i2c_write_byte(reg)) {     // Send register address;
        i2c_stop();                 // Stop and return false if no ACK
        return false;
    }

    i2c_start();    // Send repeated START for read operation

    if (!i2c_write_byte((addr << 1) | 1)) {     // Send device address with read bit;
        i2c_stop();                             // Stop and return false if no ACK
        return false;
    }

    for (int i = 0; i < len; i++) {             // Read 'len' bytes;
        data[i] = i2c_read_byte(i < (len - 1)); // send ACK for all but the last byte
    }

    i2c_stop();     // Send I2C STOP condition to finish transaction
    return true;    // Return true to indicate successful read
}

// ========================= Sensor Comunication =========================

/** @brief Scan the I2C bus for connected sensors and identify temperature and humidity devices.
 *
 * This function iterates through all valid I2C addresses (0x08–0x77), probes each one,
 * and reads the WHO_AM_I register to identify known sensors.
 * It updates global flags and stores the address of the first detected humidity sensor.
 *
 * @note Currently, the function assumes TMP64 as the temperature sensor with a fixed
 *       address and treats any other responding device as the humidity sensor (HMD10).
 *       If multiple unknown devices exist, only the first detected humidity sensor is stored.
 */
void scan_bus() {
    printf("Scanning I2C bus...\n");        // Print message indicating scan start

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {   // Iterate through all possible 7-bit I2C addresses (0x08 to 0x77)

        if (!i2c_probe(addr))   // Probe the current address; skip if no device responds
            continue;

        uint8_t who = 0;    // Variable to store WHO_AM_I value

        if (!i2c_read_reg(addr, REG_WHOAMI, &who, 1)) {                     // Read WHO_AM_I register to identify the device
            #ifdef Debug
                printf("Device at 0x%02X but WHO_AM_I read failed\n", addr);
            #endif
            continue;  // Skip to next address if read fails
        }

        // ----------------------- Check if the device is the TMP64 temperature sensor -----------------------
        if (addr == TMP64_ADDR) {
            printf("Temperature Sensor (TMP64) Found: WHO_AM_I = 0x%02X\n", who);
            Sensor_flag.Temp_presente = true;     // Update global flag
        }

        // ----------------------- Otherwise, assume the device is the HMD10 humidity sensor -----------------------
        else {
            if (humidity_addr == 0xFF) {    // Store the address only if not previously set
                humidity_addr = addr;
            }
            printf("Humidity Sensor (HMD10) Found: WHO_AM_I = 0x%02X\n", who);
            Sensor_flag.Hum_presente = true;      // Update global flag
        }
    }
    printf("-------------------\n");
}

/** @brief Read a 32-bit integer from an I2C sensor register and scale it to float.
 *
 * This function reads 4 bytes from the specified I2C register, combines them into a 32-bit signed integer, and scales it to a floating-point value.
 *
 * @param addr The I2C address of the sensor to read from.
 * @param value Reference to a float where the scaled sensor value will be stored.
 * @param scale Scaling factor to convert the raw integer to meaningful units.
 * @return true if the read was successful, false if the I2C read failed.
 *
 * @note The function assumes big-endian byte order from the sensor (MSB first).
 */
bool read_int32(uint8_t addr, float &value, float scale) {
    uint8_t buf[4];     // Buffer to hold raw bytes read from the sensor register

    if (!i2c_read_reg(addr, REG_DATA, buf, 4))  // Read 4 bytes from the sensor's data register and return false if the I2C read fails
        return false;

    int32_t raw =   // Combine the 4 bytes into a single 32-bit signed integer (big-endian)
        (int32_t(buf[0]) << 24) |
        (int32_t(buf[1]) << 16) |
        (int32_t(buf[2]) << 8)  |
        (int32_t(buf[3]));

    value = raw / scale;    // Scale the raw integer to floating-point value
    return true;
}

/** @brief Read a 32-bit integer from an I2C sensor with automatic retries.
 *
 * This function attempts to read a 32-bit sensor value up to 3 times before returning failure. It wraps the existing read_int32 function.
 *
 * @param addr The I2C address of the sensor to read from.
 * @param value Reference to a float where the scaled sensor value will be stored.
 * @param scale Scaling factor to convert the raw integer to meaningful units.
 * @param retries Number of retry attempts in case of I2C failure.
 * @return true if a successful read occurred, false otherwise.
 */
bool read_int32_retry(uint8_t addr, float &value, float scale, int retries = 3) {
    for (int attempt = 0; attempt < retries; attempt++) {
        if (read_int32(addr, value, scale)) {
            return true; // success
        }
    }
    #ifdef Debug
        printf("All read attempts for sensor 0x%02X failed\n", addr);
    #endif
    return false; // all attempts failed
}

// ========================= Display =========================
/** @brief Output a temperature value on hardware registers for external display.
 *
 * This function formats a floating-point temperature into a fixed-width string, splits it into two 32-bit chunks, and writes each chunk to dedicated hardware registers.
 *
 * @param temp The temperature value to display (in Celsius).
 *
 * @note The hardware display expects data split into two 32-bit registers.
 *       The first 4 bytes go to register 6, the next 4 bytes to register 7.
 */
void display_temperature(float temp) {
    char buf[9] = {};   // Create a buffer of 9 characters, initialized to zeros
    std::snprintf(buf, sizeof(buf), "%8.3f", temp);     // Format the float value into the buffer with width 8 and 3 decimals

    uint32_t r6, r7;    // Temporary 32-bit registers to hold parts of the buffer
    std::memcpy(&r6, buf + 0, 4);   // Copy the first 4 bytes of the buffer into r6
    std::memcpy(&r7, buf + 4, 4);   // Copy the next 4 bytes of the buffer into r7

    io.write_reg(6, r6);    // Write the first part to hardware register 6
    io.write_reg(7, r7);    // Write the first part to hardware register 7
}

/** @brief Output a humidity value on hardware registers for external display.
 *
 * This function formats a floating-point humidity into a fixed-width string, splits it into two 32-bit chunks, and writes each chunk to dedicated hardware registers.
 *
 * @param hum The humidity value to display (in %).
 *
 * @note The hardware display expects data split into two 32-bit registers.
 *       The first 4 bytes go to register 4, the next 4 bytes to register 5.
 */
void display_humidity(float hum) { 
    char buf[9] = {};   // Create a buffer of 9 characters, initialized to zeros
    std::snprintf(buf, sizeof(buf), "%7.3f%%", hum);     // Format the float value into the buffer with width 8 and 3 decimals

    uint32_t r4, r5;    // Temporary 32-bit registers to hold parts of the buffer
    std::memcpy(&r4, buf + 0, 4);   // Copy the first 4 bytes of the buffer into r4
    std::memcpy(&r5, buf + 4, 4);   // Copy the next 4 bytes of the buffer into r5

    io.write_reg(4, r4);    // Write the first part to hardware register 4
    io.write_reg(5, r5);    // Write the first part to hardware register 5
}

// ========================= Main =========================
int main() {

    io.set_pullup(PIN_SCL, true);   // Enable pull-up resistors for SCL (required for open-drain I2C lines)
    io.set_pullup(PIN_SDA, true);   // Enable pull-up resistors for SDA (required for open-drain I2C lines)

    scl_high(); // Set I2C bus to idle state (both lines high)
    sda_high();

    scan_bus(); // Scan I2C bus for connected temperature and humidity sensors

    uint32_t last = 0;  // Timestamp of last reading

    while (true) {
        uint32_t now = io.millis();      // Get current system time in milliseconds
        
        if (now - last < 1000) {    // If 1 second hasn't passed yet, wait a short time and continue
            continue;
        }

        last = now;         // Update timestamp for last reading
        float temp, hum;    // Variables to store sensor readings

        bool ok_temp = read_int32_retry(TMP64_ADDR, temp, 1000.0f);       // Read temperature from I2C devices
        bool ok_hum = read_int32_retry(humidity_addr, hum, 1000.0f);      // Read humidity from I2C devices

        // ------------------------ Display temperature if read successful and sensor present ------------------------
        if(ok_temp && Sensor_flag.Temp_presente == true){
            printf("Temperature Reading: %.3f C\n", temp);
            display_temperature(temp);
        }
        else if (!ok_temp){
            #ifdef Debug
                printf("Temperature Reading ERROR");
            #endif
        }

        // ------------------------ Display humidity if read successful and sensor present ------------------------
        if(ok_hum && Sensor_flag.Hum_presente == true){
            printf("Humidity Reading: %.3f %%RH\n", hum);
            display_humidity(hum);
        }
        else if (!ok_hum){
            #ifdef Debug
                printf("Humidity Reading ERROR");
            #endif
        }

    }

    return 0;
}
