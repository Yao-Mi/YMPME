#include "YMPME_Signal.h"
#include <float.h>
pme_signal_desc YMPME_Signal_DefaultDesc(void)
{
	return (pme_signal_desc){1, 0, -FLT_MAX, FLT_MAX, 0, 0};
}
int YMPME_Signal_DescValid(const pme_signal_desc *d)
{
	return d && isfinite(d->gain) && isfinite(d->offset) && isfinite(d->minimum) &&
		   isfinite(d->maximum) && d->minimum < d->maximum && isfinite(d->resolution) &&
		   d->resolution >= 0 && isfinite(d->cutoffHz) && d->cutoffHz >= 0;
}
pme_result YMPME_Signal_Process(const pme_signal_desc *d, pme_signal_state *s, float input,
								double time, float *out, int *clipped)
{
	double value, filtered, output;
	int limit;
	if (!YMPME_Signal_DescValid(d) || !s || !out || !clipped || !isfinite(input) ||
		!isfinite(time) || time < 0 ||
		(s->initialized && (!isfinite(s->time) || !isfinite(s->filtered) || time <= s->time)))
		return PME_INVALID_ARGUMENT;
	value = (double)input * d->gain + d->offset;
	filtered = value;
	if (s->initialized && d->cutoffHz > 0)
	{
		double alpha = -expm1(-6.2831853071795864769 * d->cutoffHz * (time - s->time));
		filtered = s->filtered + (value - s->filtered) * alpha;
	}
	if (!isfinite(filtered) || fabs(filtered) > FLT_MAX)
		return PME_INVALID_ARGUMENT;
	limit = filtered < d->minimum || filtered > d->maximum;
	output = fmax(d->minimum, fmin(d->maximum, filtered));
	if (d->resolution > 0)
		output = round(output / d->resolution) * d->resolution;
	output = fmax(d->minimum, fmin(d->maximum, output));
	if (!isfinite(output))
		return PME_INVALID_ARGUMENT;
	*s = (pme_signal_state){time, (float)filtered, 1};
	*out = (float)output;
	*clipped = limit;
	return PME_OK;
}
