#!/bin/bash
# score6.sh RUN: steps 6-8 by the truth: the brain's keyboard, and how high each thing on the table stands at the end
# against its first pose; PASS when one stands 10 cm higher (the task: "Pick up the thing by 10 cm").
R=$1; L=$R/driver.log; N=$(basename $R)
kb=$(grep -oE "brain looks through eye [0-9]+, [a-z-]+ keyboard" $L | tail -1); prog=$(grep -c "lines kept" $L)
end=$(grep -oE "the brain said done|the episode failed[^;]*" $L | tail -1 | cut -c1-60)
lift=$( (head -2 $R/truth.jsonl | tail -1; tail -1 $R/truth.jsonl) 2>/dev/null | perl -MJSON::PP -e '
  my @o = map { decode_json($_) } grep { /"objects"/ } <STDIN>; exit unless @o == 2;
  my ($best, $who) = (-1e9, "");
  for my $k (keys %{$o[0]{objects}}) { next if $k eq "slab";
    my $a = $o[0]{objects}{$k}{pose}; my $b = $o[1]{objects}{$k}{pose}; next unless $a && $b;
    my $dz = $b->[2] - $a->[2]; ($best, $who) = ($dz, $k) if $dz > $best; }
  printf "%s %.3f", $who, $best;')
ok=FAIL; [ -n "$lift" ] && awk -v d=${lift##* } 'BEGIN{exit !(d >= 0.10)}' && ok=PASS
[ -z "$lift" ] && ok="UNJUDGED(no truth)"
echo "$(date +%H:%M) $N step6-8 $ok highest rise [$lift m] [$kb] programs $prog end: $end $(grep -oE "^@[0-9]+" $L | tail -1)"
