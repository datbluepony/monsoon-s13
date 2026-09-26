# MONSOON S13 – virtual test bench

Browser simulator for **MONSOON S13 v1.0**, a standalone wiper / washer controller for the 1993 Nissan 240SX (S13), designed by Alex & Andrew.

**Live:** https://datbluepony.github.io/monsoon-s13/

- Factory stalk modeled from the FSM (EL-17 / EL-59): lever OFF → INT → LO → HI (down), pull toward you = WASH, twist ring = INT delay (14–950 Ω on pins 19–20).
- **Tests the real design.** The circuit is generated from the routed KiCad PCB (every part, value, DNP flag and pad-to-net connection, cross-checked against the schematic netlist) and solved with nodal analysis every step. The firmware is a line-for-line port of `s13_wiper.ino`, using the pin names and constants parsed from the source.
- **Design checks** run on page load: 18 scenarios (power-up, stalk decoding, both INT delay extremes, LO/HI brush paths, parking, wash wipes, DIP, pump timeout, park-wire fault, cranking dip, load dump, key-off mid-sweep). Rewiring K1 to the wrong rail makes 9 of them fail.
- 12V system: engine off/running voltage, cranking dip (relay pull-in and brown-out), load-dump spike clamped by the TVS.
- Relay states, park switch, board LED, firmware state, DIP switches, serial diagnostics output, key-off mid-sweep, and a park-wire fault.
- Tabs for the circuit, connector pinouts, parts list, programming steps and a light-bulb bench test.

Single file: open `index.html` in any browser. The embedded board data is regenerated from the KiCad project with `tools/build_sim.py`.
