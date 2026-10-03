"""Pure offline metrics for accepted PX4 fusion samples, never a flight gate.

A span runs only between observed samples; never extend the last true sample
to process exit or mistake transition-only legacy logs for per-sample evidence.
"""
import math


def summarize(samples, *, max_receipt_gap_s=2.0, truncated=False):
    if not math.isfinite(max_receipt_gap_s) or max_receipt_gap_s<=0:
        raise ValueError('positive finite receipt gap required')
    result=dict(available=samples is not None,flight_ready=False,
        scope='sampled position/yaw/height fusion spans only; not EV delivery, alignment or flight acceptance',
        last_sample_not_extrapolated=True,truncated=bool(truncated),
        receipt_limit_s=max_receipt_gap_s,runs=[],issues=[],samples=0,fully_fused_samples=0,
        longest_sampled_span_s=0.,last_accepted_sample_at=None)
    if samples is None:
        result['reason']='per-sample fusion evidence absent; transition-only records are insufficient'
        return result
    high_at=high_stamp=None;active=None
    for index,row in enumerate(samples):
        result['samples']+=1
        try:
            at=row['at'];stamp=row['stamp_us'];age=row['source_age_s']
            flags=[row[k] for k in ('ev_position','ev_yaw','ev_height')]
            if (type(at) not in (int,float) or not math.isfinite(at) or at<0
                    or type(stamp) is not int or stamp<=0
                    or type(age) not in (int,float) or not math.isfinite(age) or not -.05<=age<=.5
                    or any(type(flag) is not bool for flag in flags)):
                raise ValueError('invalid sample schema/time/flags')
            if high_at is not None and (at<=high_at or stamp<=high_stamp):
                raise ValueError('duplicate/reversed receipt or source clock')
        except (KeyError,TypeError,ValueError) as exc:
            result['issues'].append(dict(index=index,reason=str(exc)));active=None
            continue
        gap=None if high_at is None else at-high_at
        high_at,high_stamp=at,stamp
        result['last_accepted_sample_at']=at
        if gap is not None and gap>max_receipt_gap_s:
            result['issues'].append(dict(index=index,reason='receipt gap',gap_s=gap));active=None
        if not all(flags):active=None;continue
        result['fully_fused_samples']+=1
        if active is None:
            active=dict(start_at=at,end_at=at,samples=1,span_s=0.,max_receipt_gap_s=0.)
            result['runs'].append(active)
        else:
            active.update(end_at=at,samples=active['samples']+1,span_s=at-active['start_at'],
                          max_receipt_gap_s=max(active['max_receipt_gap_s'],gap))
        result['longest_sampled_span_s']=max(result['longest_sampled_span_s'],active['span_s'])
    return result
