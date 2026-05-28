//    Arduino PPM Generator
//    Copyright (C) 2015-2021  Alexandr Kolodkin <alexandr.kolodkin@gmail.com>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//    #include "src/SimpleModbusSlave/SimpleModbusSlave.h"

#include "SimpleModbusSlave.h"

#if !defined(ARDUINO_ARCH_RENESAS)
#error "This sketch now supports only Renesas boards (e.g., UNO R4)."
#endif

#include "FspTimer.h"

// Maximum number of channels
#define MAX_COUNT 16
#define DEBUG_PIN 9
#define PPM_PIN 10

// Run GPT from a divided clock so 1000-2000 us channel widths fit in 16-bit registers.
static constexpr timer_source_div_t PPM_TIMER_DIV = TIMER_SOURCE_DIV_4;
static constexpr uint32_t PPM_TIMER_DIV_VALUE = 4;
static constexpr uint8_t PPM_TIMER_IRQ_PRIORITY = 8;
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 1
#define FW_VERSION_STR_1(v) #v
#define FW_VERSION_STR(v) FW_VERSION_STR_1(v)
static constexpr char FIRMWARE_VERSION[] = FW_VERSION_STR(FW_VERSION_MAJOR) "." FW_VERSION_STR(FW_VERSION_MINOR) "." FW_VERSION_STR(FW_VERSION_PATCH);

const char * firmware_version_text() {
	return FIRMWARE_VERSION;
}

enum State {
	Pulse,       // N-th channel pulse
	StartSync,   // Start of sync pulse
	ContinueSync,// Sync pulse continuation
	FinishSync   // Sync pulse end
};

// little endian
typedef union {
	unsigned long raw;
	struct {
		word low;
		word high;
	};
} long_t;

typedef union __attribute__ ((packed)) {
	word raw[16+MAX_COUNT];
	struct __attribute__ ((packed)) {
		word quant;               // 1 microsec in timer ticks
		word max_count;           // Maximum number of channels
		word state;               // 2 - On (inversion) / 1 - On / 0 - Off
		word count;               // Number of channels (0 ... MAX_COUNT)
		word pause;               // Pause duration in timer ticks
		long_t sync;              // Synchronization pulse duration in timer ticks
		word channel[MAX_COUNT];  // Pulse duration in timer ticks
		word pulse_chl;           // Hold override channel index
		word pulse_val;           // Hold override value in timer ticks
		long_t pulse_dur;         // Hold override duration in microseconds (0 => end hold)
		word pulse_seq;           // Incrementing command sequence
		word pulse_status;        // 0 idle, 1 active, 2 rejected, 3 timeout-restored, 4 hold-ended
		word fw_version_major;    // Read-only firmware semantic version major
		word fw_version_minor;    // Read-only firmware semantic version minor
		word fw_version_patch;    // Read-only firmware semantic version patch
	};
} regs_t;

static_assert(sizeof(long_t) == (2U * sizeof(word)),
	"long_t must stay exactly two Modbus words.");
static_assert(sizeof(regs_t) == ((16U + MAX_COUNT) * sizeof(word)),
	"regs_t layout no longer matches raw[] word count.");

regs_t tmp;                   // Active configuration used by the ISR frame handoff
regs_t ppm;                   // Working dataset
regs_t modbus_regs;           // Modbus-visible register map (staging area for writes)
regs_t modbus_applied;        // Last processed Modbus command snapshot
volatile byte state = Pulse;  // Current state
volatile byte current = 0;    // Current channel number
SimpleModbusSlave slave(1);   // Modbus slave with address 1

volatile bool timed_override_active = false;
volatile byte timed_override_channel = 0;
volatile uint32_t timed_override_remaining_ticks = 0;
volatile word restore_channel[MAX_COUNT];

byte const modbus_registers_count = sizeof(regs_t) / sizeof(word);

static inline void refresh_version_registers() {
	tmp.fw_version_major = FW_VERSION_MAJOR;
	tmp.fw_version_minor = FW_VERSION_MINOR;
	tmp.fw_version_patch = FW_VERSION_PATCH;
}

static inline void refresh_modbus_readonly_registers() {
	modbus_regs.quant = tmp.quant;
	modbus_regs.max_count = tmp.max_count;
	modbus_regs.pulse_status = tmp.pulse_status;
	modbus_regs.fw_version_major = tmp.fw_version_major;
	modbus_regs.fw_version_minor = tmp.fw_version_minor;
	modbus_regs.fw_version_patch = tmp.fw_version_patch;
}

static inline void copy_frame_fields(regs_t &dst, const regs_t &src) {
	dst.state = src.state;
	dst.count = src.count;
	dst.pause = src.pause;
	dst.sync = src.sync;
	for (byte i = 0; i < MAX_COUNT; i++) {
		dst.channel[i] = src.channel[i];
	}
}

static inline bool frame_fields_differ(const regs_t &a, const regs_t &b) {
	if (a.state != b.state || a.count != b.count || a.pause != b.pause || a.sync.raw != b.sync.raw) {
		return true;
	}
	for (byte i = 0; i < MAX_COUNT; i++) {
		if (a.channel[i] != b.channel[i]) {
			return true;
		}
	}
	return false;
}

static inline void copy_pulse_command_fields(regs_t &dst, const regs_t &src) {
	dst.pulse_chl = src.pulse_chl;
	dst.pulse_val = src.pulse_val;
	dst.pulse_dur = src.pulse_dur;
	dst.pulse_seq = src.pulse_seq;
}

static inline uint32_t duration_us_to_ticks(unsigned long duration_us) {
	if (tmp.quant == 0 || duration_us == 0) {
		return 0;
	}
	uint64_t ticks = (uint64_t) duration_us * (uint64_t) tmp.quant;
	if (ticks > 0xFFFFFFFFULL) {
		return 0xFFFFFFFFUL;
	}
	return (uint32_t) ticks;
}

static inline void capture_restore_channels_from_tmp() {
	for (byte i = 0; i < MAX_COUNT; i++) {
		restore_channel[i] = tmp.channel[i];
	}
}

static inline void apply_frame_from_modbus() {
	noInterrupts();
	copy_frame_fields(tmp, modbus_regs);
	if (!timed_override_active) {
		capture_restore_channels_from_tmp();
	}
	refresh_version_registers();
	interrupts();
}

static inline void update_timed_override(uint32_t elapsed_ticks) {
	if (!timed_override_active || elapsed_ticks == 0) {
		return;
	}

	if (elapsed_ticks >= timed_override_remaining_ticks) {
		tmp.channel[timed_override_channel] = restore_channel[timed_override_channel];
		timed_override_remaining_ticks = 0;
		timed_override_active = false;
		tmp.pulse_status = 3;
		return;
	}

	timed_override_remaining_ticks -= elapsed_ticks;
}

// Set channel `chl` to `val` for `dur` microseconds, then restore.
// If `dur` is 0, end the current hold immediately.
bool SetChannelForDuration(byte chl, word val, unsigned long dur) {
	if (chl >= MAX_COUNT || tmp.quant == 0) {
		return false;
	}

	if (dur == 0) {
		noInterrupts();
		if (timed_override_active) {
			tmp.channel[timed_override_channel] = restore_channel[timed_override_channel];
			timed_override_remaining_ticks = 0;
			timed_override_active = false;
		}
		tmp.pulse_status = 4;
		interrupts();
		return true;
	}

	uint32_t duration_ticks = duration_us_to_ticks(dur);
	if (duration_ticks == 0) {
		return false;
	}

	if (val <= tmp.pause || val > 0xFFFF) {
		return false;
	}

	noInterrupts();

	if (timed_override_active) {
		tmp.channel[timed_override_channel] = restore_channel[timed_override_channel];
	}

	timed_override_channel = chl;
	tmp.channel[chl] = val;
	timed_override_remaining_ticks = duration_ticks;
	timed_override_active = true;
	tmp.pulse_status = 1;

	interrupts();
	return true;
}

static FspTimer ppm_timer;
static bool ppm_timer_ready = false;
static volatile bool ppm_running = false;
static volatile uint32_t ppm_top = 1;
static volatile uint32_t ppm_duty = 1;
static int8_t ppm_timer_channel = -1;
static TimerPWMChannel_t ppm_pwm_channel = CHANNEL_A;
static R_GPT0_Type * ppm_gpt_regs = nullptr;
static uint8_t ppm_last_output_mode = 0xFF;

static inline uint32_t clamp_period(uint32_t value) {
	return value == 0 ? 1 : value;
}

static inline uint32_t clamp_duty(uint32_t duty, uint32_t top) {
	return duty > top ? top : duty;
}

static inline void sanitize_ppm_frame() {
	if (ppm.count == 0) {
		ppm.count = 1;
	} else if (ppm.count > MAX_COUNT) {
		ppm.count = MAX_COUNT;
	}

	if (ppm.pause == 0) {
		ppm.pause = 1;
	}

	for (byte i = 0; i < ppm.count; i++) {
		if (ppm.channel[i] <= ppm.pause) {
			ppm.channel[i] = ppm.pause + 1;
		}
	}

	if (ppm.sync.raw < ppm.pause) {
		ppm.sync.raw = ppm.pause;
	}
}

static inline uint32_t ppm_clock_hz() {
	return R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
}

static inline R_GPT0_Type * ppm_gpt_regs_for_channel(uint8_t channel) {
	switch (channel) {
	case 0: return R_GPT0;
	case 1: return R_GPT1;
	case 2: return R_GPT2;
	case 3: return R_GPT3;
	case 4: return R_GPT4;
	case 5: return R_GPT5;
	case 6: return R_GPT6;
	case 7: return R_GPT7;
	case 10: return R_GPT10;
	case 11: return R_GPT11;
	case 12: return R_GPT12;
	case 13: return R_GPT13;
	default: return nullptr;
	}
}

static inline void ppm_set_idle_gpio_high() {
	pinMode(PPM_PIN, OUTPUT);
	digitalWrite(PPM_PIN, HIGH);
}

static inline void ppm_apply_output_mode() {
	if (ppm_gpt_regs == nullptr) {
		return;
	}

	bool toggle_on_compare = ppm_duty < ppm_top;
	uint8_t mode = 0;
	if (ppm.state == 2) {
		// Inverted: low at cycle start, then high after compare.
		mode = toggle_on_compare ? 0x0BU : 0x08U;
	} else {
		// Non-inverted: high at cycle start, then low after compare.
		mode = toggle_on_compare ? 0x17U : 0x14U;
	}

	if (mode == ppm_last_output_mode) {
		return;
	}

	if (ppm_pwm_channel == CHANNEL_A) {
		ppm_gpt_regs->GTIOR_b.GTIOA = mode;
	} else {
		ppm_gpt_regs->GTIOR_b.GTIOB = mode;
	}
	ppm_last_output_mode = mode;
}

static inline void ppm_write_cycle_registers(uint32_t top, uint32_t duty) {
	ppm_top = clamp_period(top);
	ppm_duty = clamp_duty(duty, ppm_top);
	ppm_apply_output_mode();

	if (ppm_gpt_regs == nullptr) {
		return;
	}

	ppm_gpt_regs->GTPR = ppm_top;
	if (ppm_pwm_channel == CHANNEL_A) {
		ppm_gpt_regs->GTCCR[0] = ppm_duty;
	} else {
		ppm_gpt_regs->GTCCR[1] = ppm_duty;
	}
}

static bool ppm_prepare_pwm_output() {
	auto pin_cfg = getPinCfgs(PPM_PIN, PIN_CFG_REQ_PWM);
	if (pin_cfg[0] == 0 || IS_PIN_AGT_PWM(pin_cfg[0])) {
		return false;
	}

	ppm_timer_channel = (int8_t) GET_CHANNEL(pin_cfg[0]);
	ppm_pwm_channel = IS_PWM_ON_A(pin_cfg[0]) ? CHANNEL_A : CHANNEL_B;
	ppm_gpt_regs = ppm_gpt_regs_for_channel((uint8_t) ppm_timer_channel);
	if (ppm_timer_channel < 0 || ppm_gpt_regs == nullptr) {
		return false;
	}

	pinPeripheral(PPM_PIN, (uint32_t) (IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
	return true;
}

static void ppm_prepare_next_cycle() {
	switch (state) {
	case Pulse:
		ppm_write_cycle_registers(ppm.channel[current], ppm.channel[current] - ppm.pause);
		if (++current == ppm.count) {
			state = (ppm.sync.high == 0) ? FinishSync : (ppm.sync.low > ppm.pause ? ContinueSync : StartSync);
		}
		break;
	case StartSync:
		ppm_write_cycle_registers(ppm.pause, ppm.pause);
		ppm.sync.raw -= ppm.pause;
		state = ppm.sync.high ? ContinueSync : FinishSync;
		break;
	case ContinueSync:
		ppm_write_cycle_registers(0xFFFF, 0xFFFF);
		if (--ppm.sync.high == 0) {
			state = FinishSync;
		}
		break;
	case FinishSync:
		ppm_write_cycle_registers(ppm.sync.low, ppm.sync.low - ppm.pause);
		ppm = tmp;  // Update settings at the end of the frame.
		sanitize_ppm_frame();
		state = Pulse;
		current = 0;
		break;
	}
}

static void ppm_timer_callback(timer_callback_args_t * /*args*/) {
	if (!ppm_running) {
		return;
	}

	update_timed_override(ppm_top);
	ppm_prepare_next_cycle();
}

// Controller initialization
void setup() {
	// Peripheral initialization
	pinMode(DEBUG_PIN, OUTPUT);
	ppm_set_idle_gpio_high();

	// Initializing channel values
	tmp.state = 0;
	tmp.max_count = MAX_COUNT;
	tmp.count = 8;
	tmp.quant = (ppm_clock_hz() / PPM_TIMER_DIV_VALUE) / 1000000UL;
	tmp.pause = tmp.quant * 200;
	tmp.sync.raw = tmp.quant * 22500UL - tmp.quant * 300UL * 8UL;
	tmp.pulse_chl = 0;
	tmp.pulse_val = 0;
	tmp.pulse_dur.raw = 0;
	tmp.pulse_seq = 0;
	tmp.pulse_status = 0;

	// Channel duration 300 us
	for (byte i = 0; i < MAX_COUNT; i++) {
		tmp.channel[i] = 300 * (unsigned long) tmp.quant;
	}
	refresh_version_registers();
	modbus_regs = tmp;
	modbus_applied = modbus_regs;
	refresh_modbus_readonly_registers();

	// Configure MODBUS
	slave.setup(115200);

#ifdef DEBUG
	Serial.println(tmp.state);
	Serial.println(tmp.count);
	Serial.println(tmp.pause);
	Serial.println(tmp.sync.low);
	Serial.println(tmp.sync.high);
	Serial.println(tmp.sync.raw);

	for (byte i = 0; i < MAX_COUNT; i++) {
		Serial.println(tmp.channel[i]);
	}
#endif
}

// Main loop
void loop() {
	slave.loop(modbus_regs.raw, sizeof(modbus_regs) / sizeof(modbus_regs.raw[0]));

	bool frameChanged = frame_fields_differ(modbus_regs, modbus_applied);
	bool pulseChanged = modbus_regs.pulse_seq != modbus_applied.pulse_seq;

	if (frameChanged) {
		bool stateChanged = modbus_regs.state != modbus_applied.state;
		apply_frame_from_modbus();
		copy_frame_fields(modbus_applied, modbus_regs);

		if (stateChanged) {
			modbus_regs.state > 0 ? Start() : Stop();
		}
	}

	if (pulseChanged) {
		if (!SetChannelForDuration((byte) modbus_regs.pulse_chl, modbus_regs.pulse_val, modbus_regs.pulse_dur.raw)) {
			noInterrupts();
			tmp.pulse_status = 2;
			interrupts();
		}
		copy_pulse_command_fields(modbus_applied, modbus_regs);
	}

	refresh_modbus_readonly_registers();
}

// Run generation
void Start() {
	noInterrupts();
	capture_restore_channels_from_tmp();
	ppm = tmp;
	sanitize_ppm_frame();
	state = Pulse;

	if (!ppm_timer_ready) {
		ppm_timer_ready = ppm_prepare_pwm_output();
		if (ppm_timer_ready) {
			ppm_timer_ready = ppm_timer.begin(TIMER_MODE_PWM, GPT_TIMER, (uint8_t) ppm_timer_channel, 1000, 500, PPM_TIMER_DIV, ppm_timer_callback);
		}
		if (ppm_timer_ready) {
			ppm_timer.add_pwm_extended_cfg();
			ppm_timer.enable_pwm_channel(ppm_pwm_channel);
		}
		if (ppm_timer_ready) {
			ppm_timer_ready = ppm_timer.setup_overflow_irq(PPM_TIMER_IRQ_PRIORITY);
		}
		if (ppm_timer_ready) {
			ppm_timer_ready = ppm_timer.open();
		}
		if (ppm_timer_ready) {
			ppm_timer.set_period_buffer(false);
			// Disable compare/period buffering so callback writes hit active registers immediately.
			if (ppm_gpt_regs != nullptr) {
				ppm_gpt_regs->GTBER_b.BD0 = 1;
				ppm_gpt_regs->GTBER_b.BD1 = 1;
			}
		}
	} else {
		pinPeripheral(PPM_PIN, (uint32_t) (IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
	}

	if (!ppm_timer_ready) {
		ppm_set_idle_gpio_high();
	}

	current = 0;
	if (ppm_timer_ready) {
		ppm_running = false;
		ppm_timer.stop();
		ppm_last_output_mode = 0xFF;
		ppm_prepare_next_cycle();
		ppm_timer.reset();
		ppm_running = true;
		ppm_timer.start();
	}

	interrupts();
}

// Stop generation
void Stop() {
	noInterrupts();
	ppm_running = false;
	if (ppm_timer_ready) {
		ppm_timer.stop();
	}
	if (timed_override_active) {
		tmp.channel[timed_override_channel] = restore_channel[timed_override_channel];
		timed_override_remaining_ticks = 0;
		timed_override_active = false;
		tmp.pulse_status = 4;
	}
	ppm_set_idle_gpio_high();
	interrupts();
}
