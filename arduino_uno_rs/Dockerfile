# Builds the Uno firmware and flashes it with avrdude, so the robot needs no
# Rust or AVR toolchain installed. Used by ../flash_uno.sh.
FROM ubuntu:24.04
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-avr avr-libc binutils-avr avrdude build-essential curl ca-certificates git \
    && rm -rf /var/lib/apt/lists/*

ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
COPY firmware/rust-toolchain.toml /tmp/rust-toolchain.toml
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
      | sh -s -- -y --profile minimal --default-toolchain none \
    && channel=$(sed -n 's/^channel = "\(.*\)"/\1/p' /tmp/rust-toolchain.toml) \
    && rustup toolchain install "$channel" --profile minimal --component rust-src

COPY brain /src/brain
COPY firmware /src/firmware
# Extra cargo flags, e.g. --no-default-features for the no-motors dry run.
ARG CARGO_ARGS=""
RUN cd /src/firmware \
    && cargo build --release $CARGO_ARGS \
    && avr-objcopy -O ihex target/avr-none/release/uno-firmware.elf /uno-firmware.hex \
    && avr-size --mcu=atmega328p -C target/avr-none/release/uno-firmware.elf

# -c arduino toggles DTR to reset the board into its bootloader first.
CMD ["avrdude", "-p", "atmega328p", "-c", "arduino", "-P", "/dev/uno", "-b", "115200", \
     "-D", "-U", "flash:w:/uno-firmware.hex:i"]
