#!/usr/bin/perl
#
#
# Colorize Zabbuino powered Chistmas Tree by trigger colors
#
# Created Dec 2018 by zbx.sadman@gmail.com
#
#

use strict;
use warnings;
use Data::Dumper;
use POSIX;
use IO::Socket;
use LWP ();
use JSON::XS ("decode_json");

use constant {
  # All leds will have color of maximal priority trigger 
  SHOW_MAX_PRIORITY => 2,
  COMPRESSION_TYPE => 1,
};

my ($zbxUser, $zbxPass, $zbxAPI, $zbxData, $zbxReport, $ua, $response, $authToken);
my ($zabbuinoIP, $zabbuinoPort, $dataPin, @triggersActive, $priority, $i, $max);
my ($stripeLength, $stripeChunkLength, $stripeChunkNumber, $stripeChunkActivePixels, $pixelWeight, @colors, $noColor, $okColor, $currentColor, $showMode);

# Zabbuino address and port
$zabbuinoIP = '172.16.100.204';
#$zabbuinoIP = '192.168.0.224';
$zabbuinoPort = '10050';

# pin to which WS2811/WS2812 led stripe connected
$dataPin = 7; 
$stripeLength = 64;

# Color in GRB space
#            #0 Gray  #1 Blue   #2 Yellow #3 Orange #4 Pink  #5 Red
#@colors = ('555555', '0000FF', 'FFFF00', '00FFFF', '00FF66', '00FF00');
if (1 == COMPRESSION_TYPE) {
   @colors = ('222', '002', '220', '120', '121', '020');
   $okColor = "100";
   $noColor = "000";
} else {
   @colors = ('555555', '000011', '111100', '001111', '001166', '001100');
   $okColor = "220000";
   $noColor = "000000";
}

# Number of leds in stripe
$stripeChunkNumber = scalar @colors;

$stripeChunkLength = ($stripeLength / $stripeChunkNumber);

# Who have access to Zabbix API
$zbxUser='admin'; # Make user with API access and put name here
# His pass
$zbxPass='zabbix'; # Make user with API access and put password here
# API location
$zbxAPI='http://localhost/zabbix/api_jsonrpc.php';

# Do some magic with API
$ua = LWP::UserAgent-> new('cookie_jar' => {}, 'agent' => "Zabbuino helper (perl engine)");
$zbxData = "{\"params\": {\"password\": \"$zbxPass\", \"user\": \"$zbxUser\"}, \"jsonrpc\":\"2.0\", \"method\": \"user.login\", \"id\": 0}";
$response = $ua->post($zbxAPI, 'Content_type' => 'application/json-rpc', 'Content' => $zbxData);
$zbxData = decode_json($response->{'_content'});
$authToken = $zbxData -> {'result'};

#print $authToken;

$zbxData = "{\"jsonrpc\": \"2.0\", \"method\": \"trigger.get\", \"params\": {\"output\": [\"priority\", \"value\"], \"filter\": { \"value\": 1}, \"monitored\":1}, \"auth\":\"$authToken\", \"id\": 1}";
$response = $ua->post($zbxAPI, 'Content_type' => 'application/json-rpc', 'Content' => $zbxData);
$zbxData = decode_json($response->{'_content'});

$max = 0;
# Count number of active triggers with specified priority and calculate total triggers number
foreach (@{$zbxData -> {'result'}}) {
  @triggersActive[$_->{'priority'}]++;
}

#                U,I,W,A,H,D
#@triggersActive=(1,2,3,4,5,14);
#@triggersActive=(5,5,5,5,5,5);
#@triggersActive=(0,0,0,0,0,0);

foreach (@triggersActive) {
  $max = $_ if ($_ > $max);
}

$pixelWeight = ($max > 0) ? ($stripeChunkLength / $max) : 0;

#print Dumper @triggersActive;

$zbxData = '';
$zbxReport = '';
# begin pushing from priority #0 (Not classified) and walk thru array for 8 lines processing
for $i (0..($stripeChunkNumber-1)) {
  $priority = $i;

  $currentColor = $noColor;
  $stripeChunkActivePixels = 0;

   if ($triggersActive[$priority]) {
      $currentColor = $colors[$priority];
      $stripeChunkActivePixels = $pixelWeight * $triggersActive[$priority] + 0.5;
      $stripeChunkActivePixels = 1 if (1 > $stripeChunkActivePixels);
      $stripeChunkActivePixels = int($stripeChunkActivePixels);
   }

#   print "priority #$i => ". $triggersActive[$priority]." => \$stripeChunkActivePixels = $stripeChunkActivePixels, \$currentColor = $currentColor\n";

   for (my $k = 0; $k < $stripeChunkLength; $k++) {
      $zbxData   .= ($k < $stripeChunkActivePixels) ? "$currentColor" : "$okColor";
      $zbxReport .= ($k < $stripeChunkActivePixels) ? "$currentColor" : "$okColor";
      $zbxReport .= ' ';
   }
 $zbxReport .= " ' ";

}

# Make Zabbix key
$zbxData = "ws2812.sendraw[$dataPin,${\COMPRESSION_TYPE},0x${zbxData}]";
#print "$zbxReport\n";
#print $zbxData;
# Send key to Zabbuino
sendToAgent($zabbuinoIP, $zabbuinoPort, $zbxData);

exit;

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
    $size = $socket->send("$_[2]\n");
    shutdown($socket, 1);
    while (sysread($socket, $byte, 1) == 1) { $res.=$byte; }
    $socket->close();
    return $res;
}