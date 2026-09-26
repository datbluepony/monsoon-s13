// S13 240SX wiper / washer controller - ATtiny1616
// Build: Arduino IDE + megaTinyCore, chip ATtiny1616, 8 MHz internal, programmer "SerialUPDI".
// Program through J5 (5V, UPDI, GND). Diagnostics: set DIP 2 ON, read J5 pin 4 (TX) at 115200.
//
// Motor (FSM EL-59): common brush is permanently on +12V (LG), LO/HI brushes run when
// grounded. Auto-stop output (OR) is GND while sweeping and +12V at park.
//   K1 RUN : energized -> brush grounded; released -> brush fed from auto-stop, so the
//            motor always finishes the sweep and brakes at park on its own.
//   K2 HI  : routes K1 to the HI brush instead of LO.
//   K3 WASH: +12V to the washer pump.

// ---------------------------------------------------------------------------
// Stalk channel mapping (J4 pin n = STKn). Factory stalk E113 per FSM EL-59:
//   OFF 13-14 | INT 13-14 + 15-17 | LO 14-17 | HI 16-17 | WASH 17-18 | ring 19-20
// Pins 17 and 20 go to board GND, so a closed contact reads ~0 on its channel.
// Confirm with diagnostic mode before trusting this in the car.
// ---------------------------------------------------------------------------
const uint8_t CH_LO = 2;    // stalk 14
const uint8_t CH_INT = 3;   // stalk 15
const uint8_t CH_HI = 4;    // stalk 16
const uint8_t CH_WASH = 5;  // stalk 18
const uint8_t CH_RING = 6;  // stalk 19, 14-950 ohm to GND through 2.2k pull-up

const uint16_t CONTACT_CLOSED_ADC = 150;   // < ~0.7 V = contact closed (ring tops out near 310)
const uint16_t RING_ADC_SHORT = 6;         // 14 ohm end  -> shortest delay (swap if ring feels backwards)
const uint16_t RING_ADC_LONG = 308;        // 950 ohm end -> longest delay
const uint32_t INT_DELAY_MIN_MS = 2000;
const uint32_t INT_DELAY_MAX_MS = 15000;

const uint32_t WASH_WIPE_START_MS = 300;   // squirt this long before the blades start
const uint32_t WASH_MAX_MS = 10000;        // pump safety cut-off
const uint32_t DRIP_WIPE_DELAY_MS = 4000;  // one last slow wipe this long after the wash sequence
const uint32_t KICK_TIMEOUT_MS = 1500;     // max K1 on-time to get blades off park in INT
const uint32_t PARK_FAULT_MS = 8000;       // motor commanded but park signal never changes -> stop
const uint32_t SPEED_CHANGE_GAP_MS = 60;   // K1 released while K2 changes over
const uint32_t DEBOUNCE_MS = 30;
const uint32_t PARK_DEBOUNCE_MS = 5;       // park contact dwell is short at HI speed

// ---------------------------------------------------------------------------
// Pins (match schematic)
// ---------------------------------------------------------------------------
const uint8_t STK_PIN[9] = {0, PIN_PA1, PIN_PA2, PIN_PA3, PIN_PA4, PIN_PA5, PIN_PA6, PIN_PA7, PIN_PB0};
const uint8_t PIN_PARK = PIN_PB1;
const uint8_t PIN_CFG_WIPES = PIN_PB4;  // DIP 1: ON (low) = 2 wipes after wash, OFF = 3
const uint8_t PIN_CFG_DIAG = PIN_PB5;   // DIP 2: ON (low) = diagnostic serial output
const uint8_t PIN_K_RUN = PIN_PC0;
const uint8_t PIN_K_HI = PIN_PC1;
const uint8_t PIN_K_WASH = PIN_PC2;
const uint8_t PIN_LED = PIN_PC3;

// ---------------------------------------------------------------------------
struct Debounced {
  uint32_t debounceMs;
  bool state = false;
  bool raw = false;
  uint32_t changedAt = 0;

  explicit Debounced(uint32_t ms) : debounceMs(ms) {}
  void update(bool r, uint32_t now) {
    if (r != raw) { raw = r; changedAt = now; }
    else if (r != state && now - changedAt >= debounceMs) state = r;
  }
};

Debounced swLo(DEBOUNCE_MS), swInt(DEBOUNCE_MS), swHi(DEBOUNCE_MS), swWash(DEBOUNCE_MS);
Debounced sweeping(PARK_DEBOUNCE_MS);  // true while blades are away from park

uint16_t adc[9];

enum class IntPhase : uint8_t { Waiting, Kicking, Returning };
IntPhase intPhase = IntPhase::Waiting;
uint32_t intPhaseAt = 0;

bool washing = false;
uint32_t washStartedAt = 0;
uint8_t afterWashWipes = 0;
bool washHi = false;                  // speed for the current post-wash wipe, changed only at park
enum class Drip : uint8_t { None, Waiting, Kicking };
Drip drip = Drip::None;
uint32_t dripAt = 0;
bool lastSweeping = false;

bool runOut = false, hiOut = false, pumpOut = false;
uint32_t speedChangeAt = 0;

bool fault = false;
uint32_t parkChangedAt = 0;

uint32_t intDelayMs() {
  int32_t a = adc[CH_RING];
  int32_t lo = RING_ADC_SHORT, hi = RING_ADC_LONG;
  int32_t span = hi - lo;
  int32_t pos = span >= 0 ? constrain(a, lo, hi) - lo : lo - constrain(a, hi, lo);
  int32_t absSpan = span >= 0 ? span : -span;
  if (absSpan == 0) return INT_DELAY_MIN_MS;
  // Exponential taper: equal twists give equal *ratios*, so the short delays you use most in
  // rain get most of the ring (half a twist = sqrt(min*max), about 5.5 s).
  float frac = (float)pos / absSpan;
  return (uint32_t)(INT_DELAY_MIN_MS * pow((float)INT_DELAY_MAX_MS / INT_DELAY_MIN_MS, frac) + 0.5f);
}

void setRelays(bool run, bool hi, uint32_t now) {
  if (hi != hiOut) {
    // Break motor current before K2 changes over, then re-feed.
    digitalWrite(PIN_K_RUN, LOW);
    hiOut = hi;
    digitalWrite(PIN_K_HI, hi);
    speedChangeAt = now;
  }
  runOut = run && now - speedChangeAt >= SPEED_CHANGE_GAP_MS;
  digitalWrite(PIN_K_RUN, runOut);
}

void diagnostics(uint32_t now) {
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 250) return;
  lastPrint = now;
  for (uint8_t i = 1; i <= 8; i++) {
    Serial.print(F("S")); Serial.print(i); Serial.print('='); Serial.print(adc[i]); Serial.print(' ');
  }
  Serial.print(F("| park=")); Serial.print(sweeping.state ? F("SWEEP") : F("PARKED"));
  Serial.print(F(" | "));
  if (swHi.state) Serial.print(F("HI "));
  if (swLo.state) Serial.print(F("LO "));
  if (swInt.state) { Serial.print(F("INT(")); Serial.print(intDelayMs() / 1000.0, 1); Serial.print(F("s) ")); }
  if (swWash.state) Serial.print(F("WASH "));
  if (fault) Serial.print(F("FAULT "));
  Serial.println();
}

void setup() {
  pinMode(PIN_K_RUN, OUTPUT);
  pinMode(PIN_K_HI, OUTPUT);
  pinMode(PIN_K_WASH, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PARK, INPUT);
  pinMode(PIN_CFG_WIPES, INPUT_PULLUP);
  pinMode(PIN_CFG_DIAG, INPUT_PULLUP);
  Serial.begin(115200);
  // If power returns with the blades mid-sweep, K1 is released and the auto-stop
  // contact parks them by itself - no firmware action needed.
}

void loop() {
  uint32_t now = millis();

  for (uint8_t i = 1; i <= 8; i++) adc[i] = analogRead(STK_PIN[i]);
  swLo.update(adc[CH_LO] < CONTACT_CLOSED_ADC, now);
  swInt.update(adc[CH_INT] < CONTACT_CLOSED_ADC, now);
  swHi.update(adc[CH_HI] < CONTACT_CLOSED_ADC, now);
  swWash.update(adc[CH_WASH] < CONTACT_CLOSED_ADC, now);
  sweeping.update(digitalRead(PIN_PARK) == LOW, now);

  bool sw = sweeping.state;
  bool justParked = lastSweeping && !sw;
  if (sw != lastSweeping) parkChangedAt = now;
  lastSweeping = sw;
  uint8_t wipesAfterWash = digitalRead(PIN_CFG_WIPES) == LOW ? 2 : 3;

  // --- Washer ---
  // Held: squirt and spread at LO. Released: FAST, FAST, SLOW (or FAST, SLOW with DIP 1),
  // then a slow "drip wipe" a few seconds later for the fluid that runs down the glass.
  // A sweep already under way at release is finished first and doesn't count.
  if (swWash.state && !washing) { washing = true; washStartedAt = now; afterWashWipes = 0; washHi = false; drip = Drip::None; }
  if (!swWash.state && washing) { washing = false; afterWashWipes = wipesAfterWash + (sw ? 1 : 0); }
  pumpOut = washing && now - washStartedAt < WASH_MAX_MS;
  digitalWrite(PIN_K_WASH, pumpOut);
  if (justParked && afterWashWipes > 0 && !washing) {
    afterWashWipes--;
    if (afterWashWipes == 0) { drip = Drip::Waiting; dripAt = now; }
  }
  // Change speed only at park, so K2 never switches while the blades are mid-glass.
  if (justParked || !sw) washHi = afterWashWipes > 1;
  if (swHi.state || swLo.state || swInt.state) drip = Drip::None;

  // --- Wiper motor, highest priority first ---
  // A short LO blip (MIST) needs nothing special: once K1 releases, the auto-stop
  // contact carries the blades through a full sweep to park.
  bool run = false, hi = false;
  if (swHi.state) {
    run = true; hi = true;
  } else if (swLo.state) {
    run = true;
  } else if (washing) {
    run = now - washStartedAt >= WASH_WIPE_START_MS;
  } else if (afterWashWipes > 0) {
    // Release during the final sweep; the auto-stop contact finishes it.
    run = afterWashWipes > 1 || !sw;
    hi = washHi;
  } else if (drip != Drip::None) {
    if (drip == Drip::Waiting && now - dripAt >= DRIP_WIPE_DELAY_MS) { drip = Drip::Kicking; dripAt = now; }
    if (drip == Drip::Kicking) {
      run = true;
      // Once off park, the auto-stop contact carries the wipe home.
      if (sw || now - dripAt >= KICK_TIMEOUT_MS) drip = Drip::None;
    }
  } else if (swInt.state) {
    switch (intPhase) {
      case IntPhase::Waiting:
        if (now - intPhaseAt >= intDelayMs()) { intPhase = IntPhase::Kicking; intPhaseAt = now; }
        break;
      case IntPhase::Kicking:
        run = true;
        if (sw || now - intPhaseAt >= KICK_TIMEOUT_MS) { intPhase = IntPhase::Returning; intPhaseAt = now; }
        break;
      case IntPhase::Returning:
        if (!sw || now - intPhaseAt >= PARK_FAULT_MS) { intPhase = IntPhase::Waiting; intPhaseAt = now; }
        break;
    }
  }
  if (!swInt.state) {
    // First wipe happens immediately when INT is selected.
    intPhase = IntPhase::Waiting;
    intPhaseAt = now - INT_DELAY_MAX_MS;
  }

  // --- Park feedback fault: motor driven but the auto-stop never changes state ---
  if (!runOut) parkChangedAt = now;
  if (runOut && now - parkChangedAt >= PARK_FAULT_MS) fault = true;
  bool anyRequest = swHi.state || swLo.state || swInt.state || swWash.state;
  if (fault && !anyRequest) { fault = false; afterWashWipes = 0; drip = Drip::None; }  // reset by returning stalk to OFF
  if (fault) run = false;

  setRelays(run, hi && !fault, now);

  bool blink = (now / (fault ? 100 : 500)) % 2;
  digitalWrite(PIN_LED, fault ? blink : (runOut || pumpOut || (swInt.state && blink)));

  if (digitalRead(PIN_CFG_DIAG) == LOW) diagnostics(now);
}
