#ifndef _FILTER_H_
#define _FILTER_H_
// Adapted from betaflight filter implementation
// https://github.com/betaflight/betaflight

struct biquadFilter_s {
	float b0, b1, b2, a1, a2;
	float x1, x2, y1, y2;
};

void biquadFilter_init(biquadFilter_s *filter, float filterFreq, float sampleFreq);
float biquadFilter_apply(biquadFilter_s *filter, float input);

#endif
