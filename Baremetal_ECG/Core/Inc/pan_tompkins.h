/**
 * @file    pan_tompkins.h
 * @brief   Pan-Tompkins QRS Detection Algorithm
 *          Optimised for STM32G431 @ 360 Hz sampling rate
 *
 *  Pipeline:
 *  Raw ECG → Band-Pass Filter → Derivative → Squaring
 *            → Moving-Window Integrator → Adaptive Thresholding → QRS
 */

#ifndef PAN_TOMPKINS_H
#define PAN_TOMPKINS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────
 * 1.  SAMPLING & TIMING CONSTANTS
 * ───────────────────────────────────────────────────────────── */
    uint32_t last_mwi_peak_idx;     /* absolute index of last candidate peak      */
#define PT_FS                   360u        /* Sampling frequency (Hz)          */
#define PT_FS_F                 360.0f      /* Float version for math           */

/* Refractory period: ignore new peaks within 200 ms of last R */
#define PT_REFRACTORY_SAMPLES   ((uint16_t)(0.200f * PT_FS))   /* 72 samples   */

/* Searchback window: if no peak found within 1.66× RR interval */
#define PT_SEARCHBACK_FACTOR    1.66f

/* RR-average buffer length (last 8 intervals) */
#define PT_RR_BUFFER_LEN        8u

/* ─────────────────────────────────────────────────────────────
 * 2.  FILTER BUFFER SIZES
 *     All sizes derived from filter order or window length
 * ───────────────────────────────────────────────────────────── */

/* Low-pass FIR  : H(z) = (1 - z^-6)^2 / (1 - z^-1)^2  → order 12 */
#define PT_LPF_ORDER            12u
#define PT_LPF_BUF_LEN         (PT_LPF_ORDER + 1u)     /* 13 taps  */

/* High-pass FIR : H(z) = -z^-16 + [1/(32)] * (1 - z^-32) / (1-z^-1)
 *                  order 32 */
#define PT_HPF_ORDER            32u
#define PT_HPF_BUF_LEN         (PT_HPF_ORDER + 1u)     /* 33 taps  */

/* Derivative    : 5-point central difference  → order 4              */
#define PT_DERIV_ORDER          4u
#define PT_DERIV_BUF_LEN       (PT_DERIV_ORDER + 1u)   /* 5 samples */

/* Moving-window integrator : 150 ms window @ 360 Hz = 54 samples     */
#define PT_MWI_WINDOW_MS        150u
#define PT_MWI_BUF_LEN         ((uint16_t)((PT_MWI_WINDOW_MS * PT_FS) / 1000u))  /* 54 */

/* ─────────────────────────────────────────────────────────────
 * 3.  THRESHOLD & GAIN CONSTANTS
 * ───────────────────────────────────────────────────────────── */

/* Initial signal-peak estimate  (fraction of first 2 s running max)  */
#define PT_INIT_THRESHOLD_FRAC  0.25f

/* Threshold update weights  (Pan & Tompkins 1985, Section V)         */
#define PT_SIGNAL_ALPHA         0.125f      /* weight for new signal peak       */
#define PT_NOISE_ALPHA          0.125f      /* weight for new noise peak        */

/* Threshold = noise + 0.25 × (signal - noise)                        */
#define PT_THRESHOLD_FRAC       0.25f

/* Searchback threshold = 50% of regular threshold                     */
#define PT_SEARCHBACK_THR_FRAC  0.50f

/* ADC full-scale (12-bit right-aligned)                               */
#define PT_ADC_MAX              4095.0f
#define PT_ADC_MIDPOINT         2048.0f     /* subtract to remove DC (Specially for signal)           */

/* ─────────────────────────────────────────────────────────────
 * 4.  OUTPUT / HR LIMITS
 * ───────────────────────────────────────────────────────────── */

#define PT_HR_MIN_BPM           20u         /* below → probably noise           */
#define PT_HR_MAX_BPM           300u        /* above → probably noise           */

/* Corresponding RR limits in samples                                  */
#define PT_RR_LOW_SAMPLES       ((uint16_t)(60.0f / PT_HR_MAX_BPM * PT_FS))  /*  72 */
#define PT_RR_HIGH_SAMPLES      ((uint16_t)(60.0f / PT_HR_MIN_BPM * PT_FS))  /* 1080 */

/* ─────────────────────────────────────────────────────────────
 * 5.  STRUCTS
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief  Internal delay-line buffers for every filter stage.
 *         All buffers are circular; index wraps with modulo.
 */
typedef struct
{
    /* Low-pass filter state */
    float   lpf_buf[PT_LPF_BUF_LEN];
    uint8_t lpf_idx;
    float   lpf_y_prev1;
    float   lpf_y_prev2;

    /* High-pass filter state */
    float   hpf_buf[PT_HPF_BUF_LEN];
    uint8_t hpf_idx;
    float   hpf_y_prev;

    /* Derivative filter state */
    float   deriv_buf[PT_DERIV_BUF_LEN];
    uint8_t deriv_idx;

    /* Moving-window integrator */
    float    mwi_buf[PT_MWI_BUF_LEN];
    uint16_t mwi_idx;
    float    mwi_sum;               /* running sum (avoids re-summing the window) */

} PT_FilterState;


/**
 * @brief  Adaptive threshold state — maintained separately for
 *         the filtered signal path and the MWI path.
 */
typedef struct
{
    float   spk;        /* Signal-peak  running estimate                        */
    float   npk;        /* Noise-peak   running estimate                        */
    float   threshold1; /* Primary   threshold (npk + 0.25*(spk-npk))          */
    float   threshold2; /* Searchback threshold (0.5 * threshold1)              */

} PT_Threshold;


/**
 * @brief  RR-interval statistics used for searchback & false-peak rejection.
 */
typedef struct
{
    uint16_t  buf[PT_RR_BUFFER_LEN]; /* circular RR history (samples)          */
    uint8_t   idx;                   /* write pointer                          */
    uint8_t   count;                 /* number of valid intervals               */
    uint16_t  average;               /* running mean of last 8 intervals       */
    uint16_t  low_limit;             /* 0.92 × average (acceptable lower bound)*/
    uint16_t  high_limit;            /* 1.16 × average (acceptable upper bound)*/

} PT_RRStats;


/**
 * @brief  Top-level algorithm handle — pass ONE instance per ECG channel.
 *         Initialise with PT_Init() before first sample.
 */
typedef struct
{
    PT_FilterState  filters;        /* all filter delay lines                  */
    PT_Threshold    thr_signal;     /* thresholds on band-pass output          */
    PT_Threshold    thr_mwi;        /* thresholds on MWI output                */
    PT_RRStats      rr;             /* RR-interval history                     */

    /* Peak-search state */
    float    last_mwi_peak;         /* amplitude of last candidate peak        */
    uint32_t last_mwi_peak_idx;     /* absolute index of last candidate peak  */
    uint16_t samples_since_last_r;  /* refractory / searchback counter         */
    uint32_t last_r_index;          /* absolute sample index of last R-peak    */
    uint32_t sample_index;          /* absolute input sample index              */
    float    mwi_prev2;              /* delayed MWI sample for peak detection   */
    float    mwi_prev1;              /* most recent MWI sample                  */

    /* Outputs (updated on every QRS detection) */
    bool     qrs_detected;          /* true for exactly ONE sample per beat    */
    uint32_t r_peak_index;          /* sample index of confirmed R-peak        */
    float    heart_rate_bpm;        /* instantaneous HR (BPM)                  */
    float    last_bpf;              /* latest band-pass output                 */
    float    last_mwi;              /* latest moving-window output             */

    /* Init flag */
    bool     initialised;

} PT_Handle;


/* ─────────────────────────────────────────────────────────────
 * 6.  PUBLIC FUNCTION PROTOTYPES
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief  Reset and initialise all state inside *hpt.
 *         Call once before streaming samples.
 * @param  hpt   Pointer to an uninitialised PT_Handle.
 */
void PT_Init(PT_Handle *hpt);


/**
 * @brief  Push ONE new ADC sample through the full pipeline.
 *         Call this inside your DMA half/full-complete callback.
 *
 * @param  hpt        Pointer to an initialised PT_Handle.
 * @param  raw_adc    12-bit right-aligned ADC value  (0–4095).
 *
 * @note   Call from the foreground processing path after a DMA callback has
 *         handed off the completed ADC samples. After this call, check
 *         hpt->qrs_detected. If true, the peak and heart-rate outputs update.
 */
void PT_Update(PT_Handle *hpt, uint16_t raw_adc);


/**
 * @brief  Band-pass filter (LPF + HPF cascade).
 *         Cutband ≈ 5–15 Hz per Pan & Tompkins 1985.
 * @param  hpt   Handle (filter buffers mutated in place).
 * @param  x     DC-removed float sample.
 * @return Filtered sample.
 */
float PT_BandPass(PT_FilterState *fs, float x);


/**
 * @brief  5-point derivative: emphasises QRS slopes.
 * @param  fs    Filter-state struct.
 * @param  x     Band-passed sample.
 * @return Derivative output.
 */
float PT_Derivative(PT_FilterState *fs, float x);


/**
 * @brief  Point-wise squaring: amplifies large slopes, suppresses small ones.
 * @param  x  Derivative output.
 * @return    x²
 */
float PT_Square(float x);


/**
 * @brief  Moving-window integration over PT_MWI_BUF_LEN samples.
 * @param  fs   Filter-state struct.
 * @param  x    Squared sample.
 * @return Integrated (smoothed) output.
 */
float PT_MWI(PT_FilterState *fs, float x);


/**
 * @brief  Update adaptive thresholds after classifying a peak.
 * @param  thr      Threshold struct to update.
 * @param  peak     Peak amplitude found.
 * @param  is_signal true → QRS peak, false → noise peak.
 */
void PT_UpdateThreshold(PT_Threshold *thr, float peak, bool is_signal);


/**
 * @brief  Update RR-interval buffer and recalculate limits.
 * @param  rr       RR stats struct.
 * @param  interval New RR interval in samples.
 */
void PT_UpdateRR(PT_RRStats *rr, uint16_t interval);


/**
 * @brief  Return the most recent heart rate in BPM.
 *         Valid only after at least one QRS has been detected.
 * @param  hpt   Handle.
 * @return Heart rate in BPM, or 0.0f if not yet computed.
 */
float PT_GetHeartRate(const PT_Handle *hpt);


/**
 * @brief  Returns true if a QRS was confirmed on the last PT_Update() call.
 * @param  hpt   Handle.
 * @return bool
 */
bool  PT_QRSDetected(const PT_Handle *hpt);


#ifdef __cplusplus
}
#endif

#endif /* PAN_TOMPKINS_H */
