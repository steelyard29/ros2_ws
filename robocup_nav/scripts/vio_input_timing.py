"""Offline timestamp comparison only; absence at a subscriber is not camera failure."""


def stereo_gap_evidence(packets):
    inputs = {kind: {p['stamp_ns'] for p in packets if p['kind'] == kind}
              for kind in ('infra1', 'infra2')}
    poses = [p for p in packets if p['kind'] == 'pose']
    gaps = []
    for previous, current in zip(poses, poses[1:]):
        start, end = previous['stamp_ns'], current['stamp_ns']
        if end-start <= 100_000_000:
            continue
        seen = {kind: sorted(s for s in stamps if start < s < end)
                for kind, stamps in inputs.items()}
        gaps.append(dict(start_stamp_ns=start, end_stamp_ns=end,
            pose_source_gap_s=(end-start)/1e9,
            intervening_input_stamps=seen,
            exact_stereo_pairs_inside_gap=len(set(seen['infra1']) & set(seen['infra2']))))
    return dict(input_sample_counts={k: len(v) for k, v in inputs.items()},
        pose_count=len(poses),
        pose_stamps_observed_in_both_inputs=sum(
            p['stamp_ns'] in inputs['infra1'] and p['stamp_ns'] in inputs['infra2'] for p in poses),
        pose_gaps_over_100ms=len(gaps), last_gap_examples=gaps[-32:],
        note='Diagnostic 100ms display filter, not a flight gate. Exact-stamp matching only; '
             'unmatched stamps may reflect synchronization offsets or subscriber loss. '
             'An independent subscriber sees neither driver-internal frames nor VSLAM callback queues; '
             'this observation adds image deserialization load and cannot alone prove camera failure.')
