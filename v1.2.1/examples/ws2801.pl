#!/usr/bin/perl
#
#   Sample of work with WS2801 led stripe and Zabbuino
#
#   Set $zabbuino_ip, $zabbuino_port, $shiftout_data_pin, $shiftout_clock_pin, $PixelsNum and GO
#


### echo 1 > /proc/sys/net/ipv4/tcp_tw_recycle 
#    Enable fast recycling TIME-WAIT sockets. Default value is 0.
#    It should not be changed without advice/request of technical
#    experts.
use strict;
use warnings;
use Data::Dumper;
use POSIX;
use IO::Socket;
use Time::HiRes qw (sleep);

use constant {
     TRUE => 1,
     FALSE => 0,
     LEFT => 0,
     RIGHT => 1,
};
my $PixelsNum, my @PixelsColor, my @colors;

# Send data to Zabbuino
my $zabbuino_ip = '172.16.100.227';
my $zabbuino_port = '10050';


# dataPin and clockPin on Zabbuino with WS2801 stripe connected
my $shiftout_data_pin = 2;
my $shiftout_clock_pin = 3;
# how much LED pixels on Stripe
$PixelsNum=50;

my $black="000000", my $red="0F0000", my $green="000F00", my $blue="00000F";

#@colors=wobniar(3, "FF0000", "0000FF"); # R G B max power
# Generate a number colors to colors array
@colors=wobniar(64, "0F0000", "00000F"); # R G B half power

# Set Pixel's data array size
$#PixelsColor=$PixelsNum-1;

# test all colors
for ($red,$green,$blue,$black) { fillStripeWithColor(\@PixelsColor, $_); showStripe(\@PixelsColor); }

#exit;
#my $speed=50;
my $speed=1000;
my $delay=$PixelsNum/$speed;
print "delay: $delay\n";

# * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * 
#
#
#                                                      MAIN LOOP
#
#

while (1)
{
 # effectFillBothDirection_01();
#  effectMovePixelBothDirection_01();
#  sleep 1;
   effectFillFull_01();
}
# * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * 

exit;

  #### Shift Stripe to Right (end) and push random color to begin
  shiftToRight(\@PixelsColor, getRandomColor(\@colors));
  showStrip(\@PixelsColor);
  sleep $delay;
  #######


# * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * # * 
#
#
#                                                      SUBROUTINES
#
#

#######
# Fill Stripe with random color from start to end and reverse with other random color
sub effectFillBothDirection_01 {
  # $PixelsColor[0]=$cc=takeNewColor($cc);
  # Fill with some color from color table
  fillStripeByStepRight(\@PixelsColor,$colors[0], $delay);
  # Cycle color table for change colors places
  turnLeft(\@colors);
  # $PixelsColor[$#PixelsColor]=$cc=$colors[0];
  fillStripeByStepLeft(\@PixelsColor,$colors[0], $delay);
  turnLeft(\@colors);
  return TRUE;
}

sub effectFillFull_01 {
  # Fill with some color from color table
  fillStripeWithColor(\@PixelsColor, $colors[0]); showStripe(\@PixelsColor); sleep $delay;
  # Cycle color table for change colors places
  turnLeft(\@colors);
  return TRUE;
}

#######
# Move random colored Pixel from right to left and back
sub effectMovePixelBothDirection_01 {
  my $cc="";
  $PixelsColor[0]=$cc=takeNewColor($cc);
  for (my $i=0; $i<=$#PixelsColor; $i++) {
    showStripe(\@PixelsColor);
    shiftToRight(\@PixelsColor, $black);
    sleep $delay;
  }
  $PixelsColor[$#PixelsColor]=$cc=takeNewColor($cc);
  for (my $i=$#PixelsColor; $i>=0; $i--) {
    showStripe(\@PixelsColor);
    shiftToLeft(\@PixelsColor, $black);
    sleep $delay;
  }
}


sub genColor {
    return sprintf("%02x%02x%02x", int(rand($_[0])), int(rand($_[0])), int(rand($_[0])));
}

####
#
# Take new color from table, that not equal given
#
####
sub takeNewColor {
    my $newColor;
    do { $newColor=getRandomColor(\@colors); } while ($_[0] eq $newColor);
    return $newColor;
}

####
#
# Fill Stripe from left to right with given color step by step with delay
#
####
sub fillStripeByStepRight{
    for (my $i=0; $i<@{$_[0]}; $i++) {
      $_[0][$i]=$_[1];
      showStripe($_[0]);
      sleep($_[2]); 
    }
   return TRUE;
}

####
#
# Fill Stripe from right to left with given color step by step with delay
#
####
sub fillStripeByStepLeft{
    for (my $i=(@{$_[0]}-1); $i>=0; $i--) {
      $_[0][$i]=$_[1];
      showStripe($_[0]);
      sleep($_[2]); 
    }
   return TRUE;
}

####
#
# Fill Stripe with given color
#
####
sub fillStripeWithColor{
   fillPixelsColors($_[0], $_[1]);
   showStripe($_[0]);
}

####
#
# "Show" strip by sending Pixels info's to Zabbuino
#
####
sub showStripe{
   sendToAgent($zabbuino_ip, $zabbuino_port, join('',@{$_[0]}));
}

####
#
# Fill Stripe with random colors from color table
#
####
sub fillStripeWithRandomColors{
    my $rndMax=(scalar @{$_[1]});
    for (my $i=0; $i<@{$_[0]}; $i++) {
      my $rnd=int(rand($rndMax));
      $_[0][$i]=$_[1][$rnd];
      #$_[0][$i]=genColor($_[2]);
    }
   return TRUE;
}

sub getRandomColor{
    my $rndMax=scalar @{$_[0]};
    my $rnd=int(rand($rndMax));
    return @{$_[0]}[$rnd];
}

sub fillPixelsColors{
    for (my $i=0; $i<@{$_[0]}; $i++) {
      $_[0][$i]=$_[1];
    }
   return TRUE;
}

####
#
# Turn array left by cycle
#
####
sub turnLeft{
 my $store=shift(@{$_[0]});
 push(@{$_[0]}, $store);
}

####
#
# Turn array right by cycle
#
####
sub turnRight{
 my $store=pop(@{$_[0]});
 unshift(@{$_[0]}, $store);
}

####
#
# Shift array right and add new value to the begin
#
####
sub shiftToRight{
 pop(@{$_[0]});
 unshift(@{$_[0]}, $_[1]);
}

####
#
# Shift array left and add new value to the end
#
####
sub shiftToLeft{
 shift(@{$_[0]});
 push(@{$_[0]}, $_[1]);
}

####
#
# Send data to Zabbuino
#
####
sub sendToAgent {
    my $socket, my $size, my $byte, my $res;
    $socket = IO::Socket::INET->new(
				PeerHost => $_[0],
				PeerPort => $_[1],
				Proto    => 'tcp') || die $@; 
    $socket->autoflush(1);
    $size = $socket->send("shiftOut[$shiftout_data_pin,$shiftout_clock_pin,,1,0x$_[2]]\n");

#    $size = $socket->send($reqPrefix.$_[2]."]\n");
    shutdown($socket, 1);
    while (sysread($socket, $byte, 1) == 1) { $res.=$byte; }
    $socket->close();
    return $res;
}

sub wobniar {
   # http://www.perlmonks.org/?node_id=70521
   die "ColorCount and at least 1 color like #AF32D3 needed\n" if @_ < 2;
   my $cnt = shift;
   my $col1 = shift;
   my $col2 = shift || $col1;
   my @murtceps;
   push @murtceps, uc $col1;

   my $pound = $col1 =~ /^#/ ? "#" : "";
   $col1 =~s/^#//;
   $col2 =~s/^#//;

   my $clockwise = 0;
   $clockwise++ if ( $cnt < 0 );
   $cnt = int( abs( $cnt ) );

   return ( wantarray() ? @murtceps : \@murtceps ) if $cnt == 1;
   return ( wantarray() ? ($col1, $col2) : [$col1, $col2] ) if $cnt == 2;

   # The RGB values need to be on the decimal scale,
   # so we divide em by 255 enpassant.
   my ( $h1, $s1, $i1 ) = rgb2hsi( map { hex() / 255 } unpack( 'a2a2a2', $col1 ) );
   my ( $h2, $s2, $i2 ) = rgb2hsi( map { hex() / 255 } unpack( 'a2a2a2', $col2 ) );
   $cnt--;
   my $sd = ( $s2 - $s1 ) / $cnt;
   my $id = ( $i2 - $i1 ) / $cnt;
   my $hd = $h2 - $h1;
   if ( uc( $col1 ) eq uc( $col2 ) ) {
      $hd = ( $clockwise ? -1 : 1 ) / $cnt;
   } else {
      $hd = ( ( $hd < 0 ? 1 : 0 ) + $hd - $clockwise) / $cnt;
   }

   while (--$cnt) {
      $s1 += $sd;
      $i1 += $id;
      $h1 += $hd;
      $h1 -= 1 if $h1>1;
      $h1 += 1 if $h1<0;
      push @murtceps, sprintf "$pound%02X%02X%02X",
         map { int( $_ * 255 +.5) }
            hsi2rgb( $h1, $s1, $i1 );
   }
   push @murtceps, uc "$pound$col2";
   return wantarray() ? @murtceps : \@murtceps;
}

sub rgb2hsi {
   my ( $r, $g, $b ) = @_;
   my ( $h, $s, $i ) = ( 0, 0, 0 );

   $i = ( $r + $g + $b ) / 3;
   return ( $h, $s, $i ) if $i == 0;

   my $x = $r - 0.5 * ( $g + $b );
   my $y = 0.866025403 * ( $g - $b );
   $s = ( $x ** 2 + $y ** 2 ) ** 0.5;
        return ( $h, $s, $i ) if $s == 0;

   $h = POSIX::atan2( $y , $x ) / ( 2 * 3.1415926535 );
   return ( $h, $s, $i );
}

sub hsi2rgb {
   my ( $h, $s, $i ) =  @_;
   my ( $r, $g, $b ) = ( 0, 0, 0 );

   # degenerate cases. If !intensity it's black, if !saturation it's grey
        return ( $r, $g, $b ) if ( $i == 0 );
        return ( $i, $i, $i ) if ( $s == 0 );

   $h = $h * 2 * 3.1415926535;
   my $x = $s * cos( $h );
   my $y = $s * sin( $h );

   $r = $i + ( 2 / 3 * $x );
   $g = $i - ( $x / 3 ) + ( $y / 2 / 0.866025403 );
   $b = $i - ( $x / 3 ) - ( $y / 2 / 0.866025403 );

   # limit 0<=x<=1  ## YUCK but we go outta range without it.
   ( $r, $b, $g ) = map { $_ < 0 ? 0 : $_ > 1 ? 1 : $_ } ( $r, $b, $g );

   return ( $r, $g, $b );


sub fadeByStep{
    my $steps=8;
    my @wrkPixels;

    for (my $i=0; $i<@{$_[0]}; $i++) {
        my ( $h1, $s1, $i1 ) = rgb2hsi( map { hex() / 255 } unpack( 'a2a2a2', @{$_[0]}[$i] ) );
        $wrkPixels[$i]->{'hue'}=$h1;
        $wrkPixels[$i]->{'sat'}=$s1;
        $wrkPixels[$i]->{'int'}=$i1;
        $wrkPixels[$i]->{'dlt'}=$i1/$steps;
    }
#    print Dumper $_[0];

    for (my $k=0; $k<$steps; $k++) {
        print "(k=$k)+\n";
        for (my $i=0; $i<@{$_[0]}; $i++) {
            print "i1=$wrkPixels[$i]->{'int'}, $steps=$steps, delta=".$wrkPixels[$i]->{'dlt'}."\n";
#            $wrkPixels[$i]->{'int'}-=$wrkPixels[$i]->{'dlt'}; $wrkPixels[$i]->{'int'}=0 if ($wrkPixels[$i]->{'int'} < $wrkPixels[$i]->{'dlt'});
            $wrkPixels[$i]->{'int'}=$wrkPixels[$i]->{'int'} / 2; #$wrkPixels[$i]->{'int'}=0 if ($wrkPixels[$i]->{'int'} < $wrkPixels[$i]->{'dlt'});
            print "i1=$wrkPixels[$i]->{'int'}\n";
            my $vv=sprintf "%02X%02X%02X",  map { int( $_ * 255 +.5) } hsi2rgb( $wrkPixels[$i]->{'hue'}, $wrkPixels[$i]->{'sat'}, $wrkPixels[$i]->{'int'} );
#            print "$vv\n";

            @{$_[0]}[$i]=$vv;
        }
        my $aa=join('-',@{$_[0]});
        print scalar @{$_[0]}." ($k): $aa\n";
#        sendToAgent($zbx_agent_host, $zbx_agent_port, join('',@{$_[0]}));
        sleep(1); 
    }
   return TRUE;
}

}