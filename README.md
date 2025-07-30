# Embassy nRF52832 Examples

This directory contains examples for the nRF52832 microcontroller using the Embassy async runtime.

## Video Demonstration

Watch the nRF52 DK in action with our Embassy examples:

https://github.com/Tony-Wu02/nrf52832-embassy/demo.mp4

### What's shown in the video:
- ✅ **LED Blinking Demo** - All 4 LEDs (LED1-LED4) blinking in sequence
- ✅ **Debug Output** - Real-time debug messages via RTT
- ✅ **Hardware Setup** - nRF52 DK connection and configuration
- ✅ **Code Compilation** - Building and flashing process
- ✅ **Live Testing** - Real-time program execution

## Hardware Requirements

- **nRF52 DK (Development Kit)** - Bluetooth Low Energy and Bluetooth mesh development kit for the nRF52810 and nRF52832 SoCs
- **USB Type-C cable** for connecting to your computer
- **J-Link debug probe** (built into nRF52 DK)

## Quick Start Guide

### 1. Prerequisites

Install Rust and required tools:

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add required target
rustup target add thumbv7em-none-eabihf

# Install probe-rs for flashing
cargo install probe-rs

# Install defmt-print for viewing debug output
cargo install defmt-print
```

### 2. Hardware Setup

1. **Connect nRF52 DK** to your computer via USB Type-C cable
2. **Verify connection**:
   ```bash
   probe-rs list
   ```
   You should see: `J-Link -- 1366:1061:001050362214 (J-Link)`

3. **Check USB devices**:
   ```bash
   ls /dev/tty.usbmodem*
   ```
   You should see devices like: `/dev/tty.usbmodem0010503622141`

### 3. Building and Flashing

#### Build and flash blinky example:
```bash
cargo run --bin blinky
```

#### Build and flash any other example:
```bash
cargo run --bin <example_name>
```

#### Build in release mode:
```bash
cargo run --release --bin blinky
```

### 4. Erasing the Device

To completely erase the device:
```bash
probe-rs erase --chip nRF52832_xxAA
```

### 5. Viewing Debug Output

The examples use `defmt` for debug output. You can view the output using:

#### Method 1: Using defmt-print (Command Line)
```bash
# Run program and view output
probe-rs run --chip nRF52832_xxAA target/thumbv7em-none-eabihf/debug/blinky | defmt-print -e target/thumbv7em-none-eabihf/debug/blinky
```

#### Method 2: Using Serial Connection
```bash
# Connect to serial port
screen /dev/tty.usbmodem0010503622141 115200
```

#### Method 3: Using J-Link RTT Viewer
1. Download J-Link Software from [SEGGER website](https://www.segger.com/downloads/jlink/)
2. Install and run `JLinkRTTViewer`
3. Connect to your device

## Available Examples

### Basic Examples
- `blinky` - LED blinking with debug output
- `simple_blinky` - Basic LED blinking without debug
- `safe_debug` - Safe debug output example
- `rtt_test` - RTT communication test

### Hardware Examples
- `timer` - Timer functionality
- `uart` - UART communication
- `spim` - SPI master (uses SPI2)
- `twim` - I2C master
- `pwm` - PWM output
- `saadc` - ADC reading
- `i2s_monitor` - I2S input monitoring
- `i2s_waveform` - I2S output waveform
- `i2s_effect` - I2S full-duplex effects
- `nfct` - NFC tag emulation
- `rng` - Random number generation
- `temp` - Temperature sensor
- `wdt` - Watchdog timer

### Advanced Examples
- `channel` - Channel communication
- `pubsub` - Publisher/Subscriber pattern
- `multiprio` - Multiple priority tasks
- `executor_fairness_test` - Executor fairness testing

## Hardware Specifications

### nRF52 DK LED Configuration
- **LED1**: P0_17
- **LED2**: P0_18  
- **LED3**: P0_19
- **LED4**: P0_20

### Memory Layout
- **Flash**: 512KB (0x00000000 - 0x00080000)
- **RAM**: 64KB (0x20000000 - 0x20010000)

### Target Configuration
- **Target**: `thumbv7em-none-eabihf`
- **Chip**: `nRF52832_xxAA`

## Troubleshooting

### Common Issues

#### 1. "Probe not found" Error
```bash
# Check if device is connected
probe-rs list

# Try with sudo if permission issues
sudo probe-rs list
```

#### 2. "Failed to open debug probe" Error
- Reconnect USB cable
- Try different USB port
- Check USB permissions on macOS

#### 3. Program runs but no LED feedback
- Verify correct LED pins for your board
- Check if program compiled successfully
- Try the `simple_blinky` example first

#### 4. Can't see debug output
- Ensure `defmt-print` is installed
- Try different viewing methods
- Check USB connection stability

### Debug Output Tips

1. **Safe Debug Usage**: Always wait for hardware initialization before using `defmt::info!`
2. **LED Feedback**: Use LED blinking as visual confirmation that program is running
3. **Counter Tracking**: Add counters to track program execution cycles

## Differences from nRF52840

The nRF52832 has some hardware differences compared to the nRF52840:

- **Memory**: 512KB Flash, 64KB RAM (vs 1024KB Flash, 256KB RAM on nRF52840)
- **GPIO**: Only P0_ pins available (no P1_ pins like nRF52840)
- **Target**: Uses `thumbv7em-none-eabihf` target
- **No USB**: nRF52832 doesn't have USB peripheral
- **No QSPI**: nRF52832 doesn't have QSPI peripheral
- **No IEEE 802.15.4**: nRF52832 doesn't support IEEE 802.15.4

## Building

Build any example:
```bash
cargo build --release --bin blinky --target thumbv7em-none-eabihf
```

Build all examples:
```bash
cargo build --release --target thumbv7em-none-eabihf
```

## License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option. 
