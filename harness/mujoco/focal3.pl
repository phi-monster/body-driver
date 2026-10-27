#!/usr/bin/perl
# focal3.pl RUNS: simulator-side rig code. For every boot of a round run through step 2 with the world's truth
# (TRUTH=1), every arm the boot swept ("boot: sweeping arm A (group G) with eye E"), its last fit as the log has it
# ("kinematics: arm A fitted from ... focal FX x FY px"), the fitted focal lengths against the eye's true one (the
# truth's K): a sweep cut by the cap leaves its fits in the log only. Step 2's line is 2 %. An arm whose last
# verdict is "not fitted" is not fitted, its last fit shown beside (focal2.pl counted that fit).
use strict; use warnings; use JSON::PP;
my $runs = shift or die "focal3.pl RUNS\n";
my (%arms, %fitted, %within);
for my $R (sort glob("$runs/*/")) {
  $R =~ s#/$##; (my $name = $R) =~ s#.*/##; (my $kind = $name) =~ s/[123][ab]$//;
  next unless -s "$R/truth.jsonl" && -s "$R/truth.json";
  open(my $t, "<", "$R/truth.json") or next; my $truth = do { local $/; decode_json(<$t>) }; close $t;
  open(my $j, "<", "$R/truth.jsonl") or next; <$j>; my $line = <$j>; close $j; next unless $line;
  my $obs = decode_json($line);
  my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
  open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or next;
  my (%cam_key, %group_key, $in, %sweep, %fit, %last);
  while (my $l = <$f>) {
    if ($l =~ /\[core\] the robot reports:/) { $in = 1; next; }
    if ($in) {
      if ($l =~ /^camera (\d+): [^\/\s]+\/([^\/\s]+)\//) { $cam_key{$1} = $2; next; }
      if ($l =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
      $in = 0 if $l =~ /^\s*$/ && %group_key;
    }
    $sweep{$2} = [$3, $4] if $l =~ /^\@(\d+) \[robot\] boot: sweeping arm (\d+) \(group (\d+)\) with eye (\d+)/;
    if ($l =~ /kinematics: arm (\d+) (fitted from (\d+) sightings of (\d+) keyframes,.*?median (\S+) px.*?focal (\S+) x (\S+) px|not fitted)/) {
      $fit{$1} = defined $3 ? [$6, $7, $5, $4] : undef if defined $3 || !defined $fit{$1};
      $last{$1} = defined $3 ? "fitted" : "not";
    }
  }
  close $f;
  my %gname = map { $_->{wire} => $_->{name} } @{$truth->{groups}};
  my %cam = map { $_->{wire} => $_ } @{$truth->{cameras}};
  for my $a (sort { $a <=> $b } keys %sweep) {
    my ($g, $e) = @{$sweep{$a}};
    my $gn = $gname{$group_key{$g} // ""} // "g$g";
    my $c = $cam{$cam_key{$e} // ""}; my $cn = $c ? $c->{name} : "e$e";
    my $own = ($c && defined $c->{by} && $c->{by} eq $gn) ? "*" : " ";
    my $true_f = $obs->{cameras}{$cn}{K}[0][0];
    my $true_fy = $obs->{cameras}{$cn}{K}[1][1];
    $arms{$kind}++;
    my $head = sprintf("%-10s arm %d %-12s with %s%-16s", $name, $a, $gn, $own, $cn);
    if ($fit{$a} && defined $true_f && ($last{$a} // "") eq "not") {
      my ($fx, $fy, $med, $kf) = @{$fit{$a}};
      printf "%s not fitted at the end (its last fit: %d keyframes, focal %.1f x %.1f against %.1f x %.1f: %+.1f%% %+.1f%%)\n", $head, $kf,
        $fx, $fy, $true_f, $true_fy, 100 * ($fx - $true_f) / $true_f, 100 * ($fy - $true_fy) / $true_fy;
    } elsif ($fit{$a} && defined $true_f) {
      my ($fx, $fy, $med, $kf) = @{$fit{$a}};
      my ($ex, $ey) = (($fx - $true_f) / $true_f, ($fy - $true_fy) / $true_fy);
      my $ok = abs($ex) <= 0.02 && abs($ey) <= 0.02;
      $fitted{$kind}++; $within{$kind}++ if $ok;
      printf "%s fitted (%d keyframes, median %s px), focal %.1f x %.1f against %.1f x %.1f: %+.1f%% %+.1f%% %s\n", $head, $kf, $med,
        $fx, $fy, $true_f, $true_fy, 100 * $ex, 100 * $ey, $ok ? "within 2%" : "OFF";
    } else {
      printf "%s not fitted against %.1f x %.1f\n", $head, $true_f // 0, $true_fy // 0;
    }
  }
}
for my $k (sort keys %arms) { printf "%-8s arms swept %2d, fitted %2d, focal within 2%% %2d\n", $k, $arms{$k}, $fitted{$k} // 0, $within{$k} // 0 }
