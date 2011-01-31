Causes the LED to blink once per second using a non-blocking delay, as well
as constantly checking the UART buffer for incoming data to be passed to the
CLI.  You can connect to the CLI using UART, send and receive commands, and
the LED should continue blinking (as long as a command is not using the MCU).