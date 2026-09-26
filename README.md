# MONSOON S13 – virtual test bench

Browser simulator for **MONSOON S13 v1.0**, a standalone wiper / washer controller for the 1993 Nissan 240SX (S13), designed by Alex & Andrew.

**Live:** https://datbluepony.github.io/monsoon-s13/

- Factory stalk modeled from the FSM (EL-17 / EL-59): lever OFF → INT → LO → HI (down), pull toward you = WASH, twist ring = INT delay (14–950 Ω on pins 19–20).
- Runs a line-for-line JavaScript port of the ATtiny1616 firmware (`s13_wiper.ino`) on a model of the factory ground-switched motor and its auto-stop (park) contact.
- 12V system: engine off/running voltage, cranking dip (relay pull-in and brown-out), load-dump spike clamped by the TVS.
- Relay states, park switch, board LED, firmware state, DIP switches, serial diagnostics output, key-off mid-sweep, and a park-wire fault.
- Tabs for the circuit, connector pinouts, parts list, programming steps and a light-bulb bench test.

Single file: open `index.html` in any browser.
