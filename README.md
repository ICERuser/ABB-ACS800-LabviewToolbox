# ABB ACS800 RETA-01 Modbus RTU DLL

This repository is being prepared for a LabVIEW-callable Windows DLL for the ABB ACS800 via RETA-01 and Modbus RTU.

## Status

Protocol implementation is intentionally not yet complete. Modbus register addresses and scaling must be provided by the ACS800/RETA-01 manual or configuration documentation before writing control operations.

## Next required implementation details

- Serial port name, for example `COM3`
- Baud rate, parity, stop bits, and Modbus timeout
- ACS800 Modbus slave/node address
- Register type and address for each required value
- Whether the target is 32-bit or 64-bit LabVIEW
- Whether operation is read-only or includes control commands

## Safety

Do not use the resulting interface to start, stop, or set motor speed until communication behavior and safety functions are validated on a disconnected or mechanically safeguarded system.

## LabVIEW integration

The DLL should expose a small C ABI and be called through LabVIEW's Call Library Function Node. It should not call LabVIEW or rely on NI-specific runtime libraries.
