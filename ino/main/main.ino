//    Arduino PPM Generator
//    Copyright (C) 2015-2021  Alexandr Kolodkin <alexandr.kolodkin@gmail.com>

#include "SimpleModbusSlave.h"
#include "types.h"  // <--- Added your custom types here
#include <Arduino_Modulino.h>
#include <math.h>

#if !defined(ARDUINO_ARCH_RENESAS)
#error "This sketch now supports only Renesas boards (e.g., UNO R4)."
#endif

#include "FspTimer.h"

#define DEBUG_PIN 9
#define PPM_PIN 10

// Run GPT from a divided clock so 1000-2000 us channel widths fit in 16-bit registers.
static constexpr timer_source_div_t PPM_TIMER_DIV = TIMER_SOURCE_DIV_4;
static constexpr uint32_t PPM_TIMER_DIV_VALUE = 4;
static constexpr uint8_t PPM_TIMER_IRQ_PRIORITY = 8;

enum State {
	Pulse,       // N-th channel pulse
	StartSync,   // Start of sync pulse
	ContinueSync,// Sync pulse continuation
	FinishSync   // Sync pulse end
};

static_assert(sizeof(long_t) == (2U * sizeof(word)),
	"long_t must stay exactly two Modbus words.");
static_assert(sizeof(regs_t) == sizeof(((regs_t*)0)->raw),
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

// Create a ModulinoMovement object
ModulinoMovement movement;

static bool movement_ready = false;
static unsigned long movement_last_poll_ms = 0;
static constexpr unsigned long MOVEMENT_UPDATE_INTERVAL_MS = 20;  // 50 Hz max poll rate

static inline int16_t scaled_i16(float value, float scale) {
	if (isnan(value) || isinf(value)) {
		return 0;
	}
	long scaled = lroundf(value * scale);
	if (scaled > 32767L) {
		return 32767;
	}
	if (scaled < -32768L) {
		return -32768;
	}
	return (int16_t) scaled;
}

static void update_movement_registers() {
	if (!movement_ready) {
		modbus_regs.movement_status = 0;
		return;
	}

	unsigned long now = millis();
	if ((unsigned long)(now - movement_last_poll_ms) < MOVEMENT_UPDATE_INTERVAL_MS) {
		return;
	}
	movement_last_poll_ms = now;

	// available() prevents us from flagging an error just because the IMU has
	// not produced another sample yet.
	if (!movement.available()) {
		if (modbus_regs.movement_status == 0) {
			modbus_regs.movement_status = 1;
		}
		return;
	}

	if (!movement.update()) {
		modbus_regs.movement_status = 3;
		return;
	}

	float ax = -movement.getX(); // Pitch
	float ay = movement.getY();  // Roll
	float az = movement.getZ();  // Gravity

	// Roll/pitch attitude derived from gravity. This is useful for centering on
	// a test stand. Yaw attitude is not available from this 6-axis IMU alone.
	int16_t roll_angle  = (int16_t)roundf(atan2f(ay, az) * (180.0f / PI));
	int16_t pitch_angle = (int16_t)roundf(atan2f(ax, az) * (180.0f / PI));

	modbus_regs.roll_angle = roll_angle;
	modbus_regs.pitch_angle = pitch_angle;
	modbus_regs.movement_millis.raw = now;
	modbus_regs.movement_seq++;
	if (modbus_regs.movement_seq == 0) {
		modbus_regs.movement_seq = 1;
	}
	modbus_regs.movement_status = 2;
}


static inline void refresh_modbus_readonly_registers() {
	modbus_regs.quant = tmp.quant;
	modbus_regs.max_count = tmp.max_count;
	modbus_regs.pulse_status = tmp.pulse_status;
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
	// refresh_version_registers();
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
		mode = toggle_on_compare ? 0x0BU : 0x08U;
	} else {
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
		ppm = tmp;
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

void setup() {
	pinMode(DEBUG_PIN, OUTPUT);
	ppm_set_idle_gpio_high();

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
	tmp.movement_status = 0;
	tmp.movement_seq = 0;
	tmp.movement_millis.raw = 0;
  tmp.roll_angle = 0;
	tmp.pitch_angle = 0;

	// Initialize Modulino Movement without printing to Serial. Serial is used by
	// SimpleModbusSlave, so sensor data is exposed through holding registers.
	Modulino.begin();
	movement_ready = movement.begin();
	tmp.movement_status = movement_ready ? 1 : 0;

	for (byte i = 0; i < MAX_COUNT; i++) {
		tmp.channel[i] = 300 * (unsigned long) tmp.quant;
	}
	modbus_regs = tmp;
	modbus_applied = modbus_regs;
	refresh_modbus_readonly_registers();

	slave.setup(115200);
}

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
	update_movement_registers();
}

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
