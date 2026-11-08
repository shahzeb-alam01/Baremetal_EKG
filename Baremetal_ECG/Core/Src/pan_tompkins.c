#include "pan_tompkins.h"
#include <math.h>
#include <string.h>

static inline uint8_t circ_inc8(uint8_t index, uint8_t size)
{
    return (uint8_t)((index + 1u) % size);
}

static inline uint16_t circ_inc16(uint16_t index, uint16_t size)
{
    return (uint16_t)((index + 1u) % size);
}

void PT_Init(PT_Handle *hpt)
{
    memset(hpt, 0, sizeof(*hpt));
    hpt->thr_signal.spk = 0.1f;
    hpt->thr_signal.npk = 0.01f;
    hpt->thr_signal.threshold1 = 0.025f;
    hpt->thr_signal.threshold2 = 0.0125f;
    hpt->thr_mwi.spk = 0.1f;
    hpt->thr_mwi.npk = 0.01f;
    hpt->thr_mwi.threshold1 = 0.025f;
    hpt->thr_mwi.threshold2 = 0.0125f;
    hpt->initialised = true;
}

static float lpf_process(PT_FilterState *fs, float x)
{
    fs->lpf_buf[fs->lpf_idx] = x;
    uint8_t n6 = (uint8_t)((fs->lpf_idx + PT_LPF_BUF_LEN - 6u) % PT_LPF_BUF_LEN);
    uint8_t n12 = (uint8_t)((fs->lpf_idx + PT_LPF_BUF_LEN - 12u) % PT_LPF_BUF_LEN);
    float output = 2.0f * fs->lpf_y_prev1 - fs->lpf_y_prev2
                 + (x - 2.0f * fs->lpf_buf[n6] + fs->lpf_buf[n12]) / 32.0f;
    fs->lpf_y_prev2 = fs->lpf_y_prev1;
    fs->lpf_y_prev1 = output;
    fs->lpf_idx = circ_inc8(fs->lpf_idx, PT_LPF_BUF_LEN);
    return output;
}

static float hpf_process(PT_FilterState *fs, float x)
{
    fs->hpf_buf[fs->hpf_idx] = x;
    uint8_t n16 = (uint8_t)((fs->hpf_idx + PT_HPF_BUF_LEN - 16u) % PT_HPF_BUF_LEN);
    uint8_t n17 = (uint8_t)((fs->hpf_idx + PT_HPF_BUF_LEN - 17u) % PT_HPF_BUF_LEN);
    uint8_t n32 = (uint8_t)((fs->hpf_idx + PT_HPF_BUF_LEN - 32u) % PT_HPF_BUF_LEN);
    float output = fs->hpf_y_prev - x / 32.0f
                 + fs->hpf_buf[n16] - fs->hpf_buf[n17]
                 + fs->hpf_buf[n32] / 32.0f;
    fs->hpf_y_prev = output;
    fs->hpf_idx = circ_inc8(fs->hpf_idx, PT_HPF_BUF_LEN);
    return output;
}

float PT_BandPass(PT_FilterState *fs, float x)
{
    return hpf_process(fs, lpf_process(fs, x));
}

float PT_Derivative(PT_FilterState *fs, float x)
{
    fs->deriv_buf[fs->deriv_idx] = x;
    uint8_t n1 = (uint8_t)((fs->deriv_idx + PT_DERIV_BUF_LEN - 1u) % PT_DERIV_BUF_LEN);
    uint8_t n3 = (uint8_t)((fs->deriv_idx + PT_DERIV_BUF_LEN - 3u) % PT_DERIV_BUF_LEN);
    uint8_t n4 = (uint8_t)((fs->deriv_idx + PT_DERIV_BUF_LEN - 4u) % PT_DERIV_BUF_LEN);
    float output = (2.0f * x + fs->deriv_buf[n1]
                  - fs->deriv_buf[n3] - 2.0f * fs->deriv_buf[n4])
                  * (PT_FS_F / 8.0f);
    fs->deriv_idx = circ_inc8(fs->deriv_idx, PT_DERIV_BUF_LEN);
    return output;
}

float PT_Square(float x)
{
    return x * x;
}

float PT_MWI(PT_FilterState *fs, float x)
{
    fs->mwi_sum -= fs->mwi_buf[fs->mwi_idx];
    fs->mwi_buf[fs->mwi_idx] = x;
    fs->mwi_sum += x;
    fs->mwi_idx = circ_inc16(fs->mwi_idx, PT_MWI_BUF_LEN);
    return fs->mwi_sum / (float)PT_MWI_BUF_LEN;
}

void PT_UpdateThreshold(PT_Threshold *thr, float peak, bool is_signal)
{
    if (is_signal)
        thr->spk = PT_SIGNAL_ALPHA * peak + (1.0f - PT_SIGNAL_ALPHA) * thr->spk;
    else
        thr->npk = PT_NOISE_ALPHA * peak + (1.0f - PT_NOISE_ALPHA) * thr->npk;
    thr->threshold1 = thr->npk + PT_THRESHOLD_FRAC * (thr->spk - thr->npk);
    thr->threshold2 = PT_SEARCHBACK_THR_FRAC * thr->threshold1;
}

void PT_UpdateRR(PT_RRStats *rr, uint16_t interval)
{
    if (interval < PT_RR_LOW_SAMPLES || interval > PT_RR_HIGH_SAMPLES)
        return;
    rr->buf[rr->idx] = interval;
    rr->idx = (uint8_t)((rr->idx + 1u) % PT_RR_BUFFER_LEN);
    if (rr->count < PT_RR_BUFFER_LEN)
        rr->count++;
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < rr->count; i++)
        sum += rr->buf[i];
    rr->average = (uint16_t)(sum / rr->count);
    rr->low_limit = (uint16_t)(rr->average * 0.92f);
    rr->high_limit = (uint16_t)(rr->average * 1.16f);
}

float PT_GetHeartRate(const PT_Handle *hpt)
{
    return hpt->heart_rate_bpm;
}

bool PT_QRSDetected(const PT_Handle *hpt)
{
    return hpt->qrs_detected;
}

static void confirm_peak(PT_Handle *hpt, float peak, uint32_t peak_index, float bpf_peak)
{
    hpt->qrs_detected = true;
    hpt->r_peak_index = peak_index;
    if (hpt->last_r_index != 0u)
    {
        uint32_t interval = peak_index - hpt->last_r_index;
        if (interval > 0u && interval <= UINT16_MAX)
        {
            hpt->heart_rate_bpm = (60.0f * PT_FS_F) / (float)interval;
            PT_UpdateRR(&hpt->rr, (uint16_t)interval);
        }
    }
    PT_UpdateThreshold(&hpt->thr_mwi, peak, true);
    PT_UpdateThreshold(&hpt->thr_signal, fabsf(bpf_peak), true);
    hpt->last_r_index = peak_index;
    hpt->samples_since_last_r = 1u;
    hpt->last_mwi_peak = 0.0f;
    hpt->last_mwi_peak_idx = 0u;
}

void PT_Update(PT_Handle *hpt, uint16_t raw_adc)
{
    if (!hpt->initialised)
        PT_Init(hpt);
    hpt->sample_index++;
    hpt->qrs_detected = false;

    float sample = ((float)raw_adc - PT_ADC_MIDPOINT) / PT_ADC_MIDPOINT;
    float bpf = PT_BandPass(&hpt->filters, sample);
    hpt->last_bpf = bpf;
    float derivative = PT_Derivative(&hpt->filters, bpf);
    float mwi = PT_MWI(&hpt->filters, PT_Square(derivative));
    hpt->last_mwi = mwi;

    bool local_peak = (hpt->mwi_prev1 > hpt->mwi_prev2)
                   && (hpt->mwi_prev1 >= mwi);
    float peak = hpt->mwi_prev1;
    uint32_t peak_index = hpt->sample_index - 1u;
    hpt->mwi_prev2 = hpt->mwi_prev1;
    hpt->mwi_prev1 = mwi;
    if (hpt->samples_since_last_r < UINT16_MAX)
        hpt->samples_since_last_r++;

    if (hpt->rr.count > 0u && hpt->last_mwi_peak > hpt->thr_mwi.threshold2
        && (float)hpt->samples_since_last_r > PT_SEARCHBACK_FACTOR * (float)hpt->rr.average)
    {
        confirm_peak(hpt, hpt->last_mwi_peak, hpt->last_mwi_peak_idx, bpf);
        return;
    }
    if (!local_peak)
        return;

    uint32_t interval = (hpt->last_r_index != 0u) ? peak_index - hpt->last_r_index : 0u;
    if (hpt->last_r_index != 0u && interval < PT_REFRACTORY_SAMPLES)
        return;

    bool above_thr1 = peak > hpt->thr_mwi.threshold1;
    bool above_thr2 = peak > hpt->thr_mwi.threshold2;
    bool rr_ok = hpt->rr.count == 0u
              || (interval >= hpt->rr.low_limit && interval <= hpt->rr.high_limit);
    bool strong_peak = peak > 2.0f * hpt->thr_mwi.threshold1;

    if (above_thr1 && (rr_ok || strong_peak))
    {
        confirm_peak(hpt, peak, peak_index, bpf);
    }
    else if (above_thr2)
    {
        if (peak > hpt->last_mwi_peak)
        {
            hpt->last_mwi_peak = peak;
            hpt->last_mwi_peak_idx = peak_index;
        }
        if (above_thr1)
        {
            PT_UpdateThreshold(&hpt->thr_mwi, peak, false);
            PT_UpdateThreshold(&hpt->thr_signal, fabsf(bpf), false);
        }
    }
    else
    {
        PT_UpdateThreshold(&hpt->thr_mwi, peak, false);
        PT_UpdateThreshold(&hpt->thr_signal, fabsf(bpf), false);
    }
}
