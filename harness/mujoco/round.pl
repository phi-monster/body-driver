#!/usr/bin/perl
# round.pl RUNS: a round of step-1 boots summed up (simulator-side rig code). From RUNS/ledger.txt (batch.sh, score1.pl)
# and each run's driver log: per robot, how many of its boots recognized every group and eye right, what was wrong,
# whether the same robot booted twice (one seed, copies a and b) came out the same, and where each boot's beats went:
# held still before the first push (rest), the push of every channel together (together), each channel alone
# (channels), the pushes again for the eyes still open (again), to the groups as recognized; a boot cut at its cap
# counts its beats so far.
use strict; use warnings;
my $runs = shift or die "round.pl RUNS\n";
my (%line, %ok, %n, %phase);
open my $l, "<", "$runs/ledger.txt" or die "no ledger in $runs\n";
while (<$l>) {
  next unless /^\d\d:\d\d (\S+) step1 (\S+)(.*?) stop (\S+) (\d+) s$/;
  my ($tag, $verdict, $rest, $why, $secs) = ($1, $2, $3, $4, $5);
  open my $mf, "<", "$runs/$tag/meta.txt" or next;
  my %meta = map { /^(\S+) ?(.*)$/ ? ($1, $2) : () } <$mf>;
  close $mf;
  my ($kind, $seed, $copy) = ($meta{kind}, $meta{seed}, substr($tag, -1));
  my ($wrong) = $rest =~ /\[(.*)\]/;
  my ($beat) = $rest =~ /@(\d+)/;
  $line{$kind}{"$seed$copy"} = { verdict => $verdict, wrong => $wrong // "", why => $why, secs => $secs, beat => $beat // 0 };
  $n{$kind}++; $ok{$kind}++ if $verdict eq "PASS";
  # the phases from the driver's log
  my $log = -e "$runs/$tag/driver.log.zst" ? "zstd -dc $runs/$tag/driver.log.zst |" : "$runs/$tag/driver.log";
  open my $d, $log or next;
  my ($first, $together, $last_channel, $recognized, $last) = (0, 0, 0, 0, 0);
  while (<$d>) {
    next unless /^@(\d+) /; my $b = $1; $last = $b;
    $first ||= $b if /probe: level 1 of every channel together/;
    $together ||= $b if /boot: every commandable channel moved together is first seen/;
    $last_channel = $b if /boot: group \d+ channel \d+ (is seen when moved|answers neither way|moves nothing any eye sees)/;
    $recognized ||= $b if /boot: the groups as recognized/;
  }
  close $d;
  my $end = $recognized || $last;
  $phase{$kind}{"$seed$copy"} = [ $first, ($together ? $together - $first : 0),
                                  ($last_channel && $together ? $last_channel - $together : 0),
                                  ($last_channel ? $end - $last_channel : 0), $end ];
}
for my $kind (sort keys %line) {
  printf "%-8s %d of %d right\n", $kind, $ok{$kind} // 0, $n{$kind};
  my %by_seed;
  for my $sc (sort keys %{ $line{$kind} }) {
    my $r = $line{$kind}{$sc}; my $p = $phase{$kind}{$sc} // [0, 0, 0, 0, 0];
    printf "   %-3s %-10s %4d s  beats: rest %5d together %5d channels %5d again %5d, in all %5d  %s\n", $sc, $r->{verdict},
      $r->{secs}, @$p, ($r->{why} ne "reached" ? "[$r->{why}] " : "") . $r->{wrong};
    my ($seed) = $sc =~ /^(\d+)/; push @{ $by_seed{$seed} }, "$r->{verdict} $r->{wrong}";
  }
  for my $seed (sort keys %by_seed) {
    my @v = @{ $by_seed{$seed} };
    printf "   seed %s booted twice: %s\n", $seed, $v[0] eq ($v[1] // "") ? "the same" : "DIFFERENT" if @v == 2;
  }
}
