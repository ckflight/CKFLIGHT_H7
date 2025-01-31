
#include "CK_FILTERS.h"

// PT1 Low Pass filter

#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

// PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
#define CUTOFF_CORRECTION_PT2 1.553773974f
#define CUTOFF_CORRECTION_PT3 1.961459177f

// NULL filter

float nullFilterApply(filter_t *filter, float input)
{
    UNUSED(filter);
    return input;
}

// PT1 Low Pass filter

float pt1FilterGain(float f_cut, float dT)
{
    float omega = 2.0f * M_PIf * f_cut * dT;
    return omega / (omega + 1.0f);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt1FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (2.0f * M_PIf * delay);
    return pt1FilterGain(cutoffHz, dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

// PT2 Low Pass filter

float pt2FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt2FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (M_PIf * delay * CUTOFF_CORRECTION_PT2);
    return pt2FilterGain(cutoffHz, dT);
}

void pt2FilterInit(pt2Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = k;
}

void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k)
{
    filter->k = k;
}

float pt2FilterApply(pt2Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}

// PT3 Low Pass filter

float pt3FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt3FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (M_PIf * delay * CUTOFF_CORRECTION_PT3);
    return pt3FilterGain(cutoffHz, dT);
}

void pt3FilterInit(pt3Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = k;
}

void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k)
{
    filter->k = k;
}

float pt3FilterApply(pt3Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
    filter->state = filter->state + filter->k * (filter->state2 - filter->state);
    return filter->state;
}

// Biquad filter

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

/* sets up a biquad filter as a 2nd order butterworth LPF */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}

void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * refreshRate * 0.000001f;
    const float sn = sin_approx(omega);
    const float cs = cos_approx(omega);
    const float alpha = sn / (2.0f * Q);

    switch (filterType) {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        filter->b1 = 1 - cs;
        filter->b0 = filter->b1 * 0.5f;
        filter->b2 = filter->b0;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        filter->b0 = 1;
        filter->b1 = -2 * cs;
        filter->b2 = 1;
        filter->a1 = filter->b1;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_BPF:
        filter->b0 = alpha;
        filter->b1 = 0;
        filter->b2 = -alpha;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    }

    const float a0 = 1 + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    filter->weight = weight;
}

void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

/* Computes a biquadFilter_t filter in df1 and crossfades input with output */
float biquadFilterApplyDF1Weighted(biquadFilter_t* filter, float input)
{
    // compute result
    const float result = biquadFilterApplyDF1(filter, input);

    // crossfading of input and output to turn filter on/off gradually
    return filter->weight * result + (1 - filter->weight) * input;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}

// Phase Compensator (Lead-Lag-Compensator)

void phaseCompInit(phaseComp_t *filter, const float centerFreqHz, const float centerPhaseDeg, const uint32_t looptimeUs)
{
    phaseCompUpdate(filter, centerFreqHz, centerPhaseDeg, looptimeUs);

    filter->x1 = 0.0f;
    filter->y1 = 0.0f;
}

void phaseCompUpdate(phaseComp_t *filter, const float centerFreqHz, const float centerPhaseDeg, const uint32_t looptimeUs)
{
    const float omega = 2.0f * M_PIf * centerFreqHz * looptimeUs * 1e-6f;
    const float sn = sin_approx(centerPhaseDeg * RAD);
    const float gain = (1 + sn) / (1 - sn);
    const float alpha = (12 - sq(omega)) / (6 * omega * sqrtf(gain));  // approximate prewarping (series expansion)

    filter->b0 = 1 + alpha * gain;
    filter->b1 = 2 - filter->b0;
    filter->a1 = 1 - alpha;

    const float a0 = 1 / (1 + alpha);

    filter->b0 *= a0;
    filter->b1 *= a0;
    filter->a1 *= a0;
}

float phaseCompApply(phaseComp_t *filter, const float input)
{
    // compute result
    const float result = filter->b0 * input + filter->b1 * filter->x1 - filter->a1 * filter->y1;

    // shift input to x1 and result to y1
    filter->x1 = input;
    filter->y1 = result;

    return result;
}

// Slew filter with limit

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold)
{
    filter->state = 0.0f;
    filter->slewLimit = slewLimit;
    filter->threshold = threshold;
}

float slewFilterApply(slewFilter_t *filter, float input)
{
    if (filter->state >= filter->threshold) {
        if (input >= filter->state - filter->slewLimit) {
            filter->state = input;
        }
    } else if (filter->state <= -filter->threshold) {
        if (input <= filter->state + filter->slewLimit) {
            filter->state = input;
        }
    } else {
        filter->state = input;
    }
    return filter->state;
}

// Moving average

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf)
{
    filter->movingWindowIndex = 0;
    filter->windowSize = windowSize;
    filter->buf = buf;
    filter->movingSum = 0;
    memset(filter->buf, 0, windowSize * sizeof(float));
    filter->primed = false;
}

float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input)
{
    filter->movingSum -= filter->buf[filter->movingWindowIndex];
    filter->buf[filter->movingWindowIndex] = input;
    filter->movingSum += input;

    if (++filter->movingWindowIndex == filter->windowSize) {
        filter->movingWindowIndex = 0;
        filter->primed = true;
    }

    const uint16_t denom = filter->primed ? filter->windowSize : filter->movingWindowIndex;
    return filter->movingSum / denom;
}

// Simple fixed-point lowpass filter based on integer math

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift)
{
    filter->fp = 0;
    filter->beta = beta;
    filter->fpShift = fpShift;
}

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal)
{
    filter->fp = (filter->fp << filter->beta) - filter->fp;
    filter->fp += newVal << filter->fpShift;
    filter->fp >>= filter->beta;
    int32_t result = filter->fp >> filter->fpShift;
    return result;
}

// Mean accumulator

void meanAccumulatorInit(meanAccumulator_t *filter)
{
    filter->accumulator = 0;
    filter->count = 0;
}

void meanAccumulatorAdd(meanAccumulator_t *filter, const int8_t newVal)
{
    filter->accumulator += newVal;
    filter->count++;
}

int8_t meanAccumulatorCalc(meanAccumulator_t *filter, const int8_t defaultValue)
{
    if (filter->count) {
        int8_t retVal = filter->accumulator / filter->count;
        meanAccumulatorInit(filter);
        return retVal;
    }
    return defaultValue;
}

// Init Fast 2-state Kalman Filter
void kalmanInit(kalmanFilter_t* kalman, float q, float r, float p, int axis){

	kalman->q[axis]      = q * 0.000001f; // add multiplier to make tuning easier

	kalman->r[axis]      = r * 0.001f;    // add multiplier to make tuning easier

	kalman->p[axis]      = p * 0.001f;    // add multiplier to make tuning easier

	kalman->x[axis]      = 0.0f;          // set initial value, can be zero if unknown

	kalman->lastX[axis]  = 0.0f;          // set initial value, can be zero if unknown

	kalman->k[axis]      = 0.0f;          // kalman gain

}

float kalmanApply(kalmanFilter_t* kalman, float inputData, int axis){


	kalman->x[axis] += (kalman->x[axis] - kalman->lastX[axis]);

	kalman->lastX[axis] = kalman->x[axis];

	kalman->p[axis] = kalman->p[axis] + kalman->q[axis];

	kalman->k[axis] = kalman->p[axis] / (kalman->p[axis] + kalman->r[axis]);

	kalman->x[axis] += kalman->k[axis] * (inputData - kalman->x[axis]);

	kalman->p[axis] = (1.0f - kalman->k[axis]) * kalman->p[axis];

    return kalman->x[axis];

}
