This folder contains a number of tools that may be useful when developing with
the LPC1114 Reference Board

===============================================================================
  examples
  -----------------------------------------------------------------------------
  Various examples to help you get started with the LPC1114 or use a specific
  peripheral or external device
===============================================================================

===============================================================================
  schematics
  -----------------------------------------------------------------------------
  Schematics showing the pin connections that are assumed to be used by the
  LPC1114 Code Base.

  LPC1114BaseBoard_v1.3 - Generic LPC1114 Base Board
  AT86RF212LPC1114_v1.6 - LPC1114 + 700/800/900 MHz 802.15.4 Transceiver
                          This schematic shows the pin connections that
                          are assumed for Chibi and the micro-SD card reader
===============================================================================

===============================================================================
  wsbridge (courtesy freaklabs.org)
  -----------------------------------------------------------------------------
  This simple application acts as a bridge between Chibi and Wireshark. The
  Chibi stack need to be put into promiscuous mode (CFG_CHIBI_PROMISCUOUS = 1).
  From there, raw 802.15.4 data frames are output via UART to WSBridge which
  feeds them into Wireshark.

  For more information see the following links:

  http://freaklabs.org/index.php/WSBridge.html
  http://bit.ly/fcTVke
===============================================================================


