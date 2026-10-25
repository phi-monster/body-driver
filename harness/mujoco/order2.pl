#!/usr/bin/perl
# order2.pl RUNS: simulator-side rig code. For every boot of a round run through step 2 with the world's truth
# (TRUTH=1), every arm the boot swept ("boot: sweeping arm A (group G) with eye E"), the last order of its joints the
# log has ("kinematics: the order of the joints along arm A, J1 J2 ..., is [NOT] told") against the group's true chain
# (scramble.json: channel I reads the model's joint ORDER[I], the model's joints from the base outwards). Step 2's line
# is the order right and told.
use strict; use warnings; use JSON::PP;
my $runs = shift or die "order2.pl RUNS\n";
my (%arms, %told, %right, %both);
for my $R (sort glob("$runs/*/")) {
  $R =~ s#/$##; (my $name = $R) =~ s#.*/##; (my $kind = $name) =~ s/[123][ab]$//;
  next unless -s "$R/truth.json" && -s "$R/scramble.json";
  open(my $t, "<", "$R/truth.json") or next; my $truth = do { local $/; decode_json(<$t>) }; close $t;
  open(my $s, "<", "$R/scramble.json") or next; my $scr = do { local $/; decode_json(<$s>) }; close $s;
  my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
  open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or next;
  my (%group_key, $in, %sweep, %order);
  while (my $l = <$f>) {
    if ($l =~ /\[core\] the robot reports:/) { $in = 1; next; }
    if ($in) {
      if ($l =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
      $in = 0 if $l =~ /^\s*$/ && %group_key;
    }
    $sweep{$2} = $3 if $l =~ /^\@(\d+) \[robot\] boot: sweeping arm (\d+) \(group (\d+)\)/;
    $order{$1} = [$2, $3 ? 0 : 1] if $l =~ /kinematics: the order of the joints along arm (\d+), ([\d ]+), is (NOT )?told/;
  }
  close $f;
  my %gname = map { $_->{wire} => $_->{name} } @{$truth->{groups}};
  for my $arm (sort { $a <=> $b } keys %sweep) {
    my $gn = $gname{$group_key{$sweep{$arm}} // ""} // "g$sweep{$arm}";
    my $g = $scr->{groups}{$gn};
    $arms{$kind}++;
    unless ($order{$arm} && $g) { printf "%-10s arm %d %-12s no order\n", $name, $arm, $gn; next; }
    my @o = @{$g->{order}};
    my %chan_of = map { $o[$_] => $_ + 1 } 0 .. $#o;   # model joint -> channel (1-based)
    my @true = map { $chan_of{$_} } sort { $a <=> $b } keys %chan_of;
    my @fit = split " ", $order{$arm}[0];
    my %in_fit = map { $_ => 1 } @fit;
    my @true_here = grep { $in_fit{$_} } @true;   # the joints the arm's fit has, in the true order
    my $ok = join(" ", @fit) eq join(" ", @true_here);
    my $tl = $order{$arm}[1];
    $told{$kind}++ if $tl; $right{$kind}++ if $ok; $both{$kind}++ if $ok && $tl;
    printf "%-10s arm %d %-12s fit %-16s truth %-16s %s %s\n", $name, $arm, $gn, join(" ", @fit), join(" ", @true_here),
      $ok ? "RIGHT" : "WRONG", $tl ? "told" : "untold";
  }
}
for my $k (sort keys %arms) {
  printf "%-8s arms swept %2d, order told %2d, right %2d, right and told %2d\n", $k, $arms{$k}, $told{$k} // 0,
    $right{$k} // 0, $both{$k} // 0;
}
