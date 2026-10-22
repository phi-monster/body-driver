#!/usr/bin/perl
# score1.pl RUN: step 1 of a run on a MuJoCo robot against its truth (RUN/truth.json, written by robot.py): every group's
# role as the driver recognized it (the groups as recognized) against the role its truth gives it, every camera's mount
# against its truth's, and every camera an arm carries carried by the group that truly carries it. The driver's group N
# and eye N are matched to the truth by the drawn keys the robot reported them under (the robot reports), never by order.
# One line: NAME step1 PASS|FAIL|UNFINISHED, then each wrong group and eye as truth->seen, then the beat it recognized at.
use strict; use warnings; use JSON::PP;
my $R = shift or die "score1.pl RUN\n";
(my $name = $R) =~ s#.*/##;
open(my $t, "<", "$R/truth.json") or do { print "$name step1 NO_TRUTH\n"; exit };
my $truth = do { local $/; decode_json(<$t>) }; close $t;
my $L = "$R/driver.log"; my $log = -e $L ? $L : "$L.zst";
open(my $f, (-e $L ? "<" : "-|"), (-e $L ? $L : "zstd -dc $log")) or do { print "$name step1 NO_LOG\n"; exit };
my (%cam_key, %group_key, @block, $in_report, $in_block, $at, $last_beat);
while (my $line = <$f>) {
  $last_beat = $1 if $line =~ /^@(\d+)/;
  if ($line =~ /\[core\] the robot reports:/) { $in_report = 1; next; }
  if ($in_report) {
    if ($line =~ /^camera (\d+): [^\/\s]+\/([^\/\s]+)\//) { $cam_key{$1} = $2; next; }
    if ($line =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
    $in_report = 0 if $line =~ /^\s*$/;
    next;
  }
  if ($line =~ /^@(\d+) \[robot\] boot: the groups as recognized:/) { @block = (); $in_block = 1; $at = $1; next; }
  if ($in_block) { if ($line =~ /^\s*$/) { $in_block = 0; } else { push @block, $line; } }
}
close $f;
if (!@block) { print "$name step1 UNFINISHED \@" . ($last_beat // 0) . "\n"; exit; }
my (%role, %arm_of_group, %mount, %arm_of_eye, %whole);
for (@block) {
  if (/^group (\d+): \d+ values, (?:not )?commandable, ([A-Z_]+)(?: \(arm (\d+)\))?/) {
    my ($g, $r, $a) = ($1, $2, $3); $role{$g} = $r; $arm_of_group{$g} = $a if defined $a;
    $whole{$g}{$1} = 1 while /eye (\d+) WHOLE/g;
  } elsif (/^eye (\d+): [^,]+, ([A-Z_]+)(?: on arm (\d+))?/) { $mount{$1} = $2; $arm_of_eye{$1} = $3 if defined $3; }
}
my %g_by_key = reverse %group_key; my %e_by_key = reverse %cam_key;
my %truth_group_by_name = map { $_->{name} => $_ } @{$truth->{groups}};
my (@wrong, $n);
for my $g (@{$truth->{groups}}) {
  my $i = $g_by_key{$g->{wire}};
  my $seen = defined $i ? ($role{$i} // "unseen") : "unreported";
  push @wrong, "$g->{name}:$g->{role}->$seen" if $seen ne $g->{role};
}
for my $c (@{$truth->{cameras}}) {
  my $e = $e_by_key{$c->{wire}};
  my $seen = defined $e ? ($mount{$e} // "unseen") : "unreported";
  if ($seen ne $c->{mount}) { push @wrong, "$c->{name}:$c->{mount}->$seen"; next; }
  if ($c->{mount} eq "ARM_CARRIED" && defined $c->{by}) {
    # The group the driver says carries this eye: the one whose arm the eye is on.
    my $carrier = (grep { defined $arm_of_group{$_} && defined $arm_of_eye{$e} && $arm_of_group{$_} == $arm_of_eye{$e}
                          && $role{$_} eq "ARM" } keys %role)[0];
    my $truly = $g_by_key{$truth_group_by_name{$c->{by}}{wire}};
    push @wrong, "$c->{name}:on $c->{by}->on " . (defined $carrier ? "group $carrier" : "none")
      unless defined $carrier && defined $truly && $carrier == $truly;
  }
}
printf "%s step1 %s%s \@%s\n", $name, (@wrong ? "FAIL" : "PASS"), (@wrong ? " [" . join("; ", @wrong) . "]" : ""), $at;
