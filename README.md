# Quantizer
Quantizes input voltage based on switch positions.

## Input

### Pitch
The pico expects a signal between 0 and 3V3. 
A +/- 12V input signal will be scaled down and inverted by external circuitry.

### Gate
Gate signals are also scaled down to 3V3 using a transistor.

## Output
Writes output voltage to DAC
1V/oct
