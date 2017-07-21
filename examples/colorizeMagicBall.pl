#!/usr/bin/perl
#
#
# Colorize Zabbuino based Admin's Magic ball: color is depends on active tiggers
#
# Created Jul Aug 2017 by zbx.sadman@gmail.com
#
# >> And beeps if high priority active trigger found since Jul  2017
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
  # Leds will have color proportional of variois priority trigger number
  SHOW_PROPORTIONAL => 3,
};

my ($zbxUser, $zbxPass, $zbxAPI, $zbxData, $ua, $response, $authToken);
my ($zabbuinoIP, $zabbuinoPort, $dataPin, @triggersActive, @colors, $okColor, $stripeLength, $showMode, $i);
#my ($buzzerPin, $buzzerFrequency, $buzzerDuration, $buzzOnPriority, $needBuzz);

# Zabbuino address and port
#$zabbuinoIP = '172.16.100.224';
$zabbuinoIP = '192.168.0.1';
$zabbuinoPort = '10050';

# pin to which WS2811/WS2812 led stripe connected
$dataPin = 17; # A3

# buzzer settings for tone[] command
#$buzzerPin = 3;
#$buzzerFrequency = 1000;
#$buzzerDuration = 50;

# script send buzz command if exist active triggers with priority $buzzOnPriority and hight
# set $buzzOnPriority > max priority (6, for example) to turn off buzzer feature
#$buzzOnPriority = 3;
#$needBuzz = 0;

# Color in GRB space
#             #0 Gray   #1 Cyan   #2 Yellow #3 Purple #4        #5 Red
#@colors = ('555555', '0000FF', 'FFFF00', '00FFFF', '00FF66', '00FF00');
@colors = ('111111', '000011', '111100', '001111', '001166', '001100');
$okColor = "110000";
# Number of leds in stripe
$stripeLength = 16;
$showMode = SHOW_MAX_PRIORITY;
#$showMode = SHOW_PROPORTIONAL;

# Who have access to Zabbix API
$zbxUser='Admin'; # Make user with API access and put name here
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

@triggersActive=(0,0,0,0,0,0);
my $maxPriority = 0;
my $triggersNumber = 0;
my $categoryPercent;
my $numLeds;

# Count number of active triggers with specified priority and calculate total triggers number
# print Dumper $zbxData -> {'result'};
foreach (@{$zbxData -> {'result'}}) {
  @triggersActive[$_->{'priority'}]++;
}

# test set of variables
#$maxPriority = 0;
#print "\$maxPriority: $maxPriority\n";
#$triggersNumber=0;
#                U,I,W,A,H,D
#@triggersActive=(0,0,0,0,0,0);

$i = 0;

foreach (@triggersActive) { 
  $triggersNumber += $_; 
  $maxPriority = $i if (0 < $_ && $i > $maxPriority);
  $i++;
}
#print "\$triggersNumber  => $triggersNumber\n";

$zbxData = '';
$i = 0;

if (0 == $triggersNumber) {
   # All leds will have okColor if no active triggers found
   $zbxData .= $okColor x $stripeLength;
} else {
   if (SHOW_PROPORTIONAL == $showMode){
      foreach (@triggersActive) {
         $categoryPercent = $_ / ($triggersNumber / 100);
         $numLeds = $categoryPercent * ($stripeLength / 100);
         $zbxData .= $colors[$i] x $numLeds;
         $i++;
      }
   } else {
      # All stripe will have color of triggers with max priority 
      $zbxData .= $colors[$maxPriority] x $stripeLength;
   }
}

# Make Zabbix key
$zbxData = "ws2812.sendraw[$dataPin,0x${zbxData}]";
# Send key to Zabbuino
sendToAgent($zabbuinoIP, $zabbuinoPort, $zbxData);
#print $zbxData;

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