#!/usr/bin/perl
#
#
# Script to show active tiggers bar chart on Zabbuino's 8x8 LED matrix
#
# Created 16 Aug 2016 by zbx.sadman@gmail.com
#
# And beeps if high priority active trigger found since 19 Aug 2016
#
#

use strict;
use warnings;
#use Data::Dumper;
use POSIX;
use IO::Socket;
use LWP ();
use JSON::XS ("decode_json");

my ($zbxUser, $zbxPass, $zbxAPI, $zbxData, $ua, $response, $authToken, $fromRightToLeft, $fromUpToBottom, $priority, $i);
my ($zabbuinoIP, $zabbuinoPort, $dataPin, $loadPin, $clockPin, @triggersActive, @hexes, $max, $barHeight, $pixelWeight);
my ($buzzerPin, $buzzerFrequency, $buzzerDuration, $buzzOnPriority, $needBuzz);

# Zabbuino address and port
$zabbuinoIP = '172.16.100.223';
$zabbuinoPort = '10050';

# pins to which MAX7219 with 8x8 led matrix connected
$dataPin = 7;
$clockPin = 5;
$loadPin = 6;

# buzzer settings for tone[] command
$buzzerPin = 3;
$buzzerFrequency = 1000;
$buzzerDuration = 50;
# script send buzz command if exist active triggers with priority $buzzOnPriority and hight
# set $buzzOnPriority > max priority (6, for example) to turn off buzzer feature
$buzzOnPriority = 3;
$needBuzz = 0;

# test set
#@triggersActive=(0,0,1,3,5,0,0,0);
#@triggersActive=(0,0,0,0,0,0,0,0);

# 0 - draw triggers priority from left to right, 1 -> in reverse order
$fromRightToLeft = 0;
# 0 - fill col like an stalagmite, 1 -> like an stalactite
$fromUpToBottom = 1;

# HEX-strings which used to fill lines with a number of pixels: 1, 2, 3 ... 8.
@hexes = $fromUpToBottom ? ('00', '80', 'C0', 'E0', 'F0', 'F8', 'FC', 'FE', 'FF') : ('00', '01', '03', '07', '0F', '1F', '3F', '7F', 'FF') ;

# Who have access to API
$zbxUser='sysop'; #Make user with API access and put name here
# His pass
$zbxPass='ua0xto13'; #Make user with API access and put password here
# API location
$zbxAPI='http://localhost/zabbix/api_jsonrpc.php';

# Do some magic with Zabbix API
$ua = LWP::UserAgent-> new('cookie_jar' => {}, 'agent' => "Zabbuino helper (perl engine)");
$zbxData = "{\"params\": {\"password\": \"$zbxPass\", \"user\": \"$zbxUser\"}, \"jsonrpc\":\"2.0\", \"method\": \"user.login\", \"id\": 0}";
$response = $ua->post($zbxAPI, 'Content_type' => 'application/json-rpc', 'Content' => $zbxData);
$zbxData = decode_json($response->{'_content'});
$authToken = $zbxData -> {'result'};
#print $authToken;

$zbxData = "{\"jsonrpc\": \"2.0\", \"method\": \"trigger.get\", \"params\": {\"output\": [\"priority\", \"value\"], \"filter\": { \"value\": 1}, \"monitored\":1}, \"auth\":\"$authToken\", \"id\": 1}";
$response = $ua->post($zbxAPI, 'Content_type' => 'application/json-rpc', 'Content' => $zbxData);
$zbxData = decode_json($response->{'_content'});

# Count number of active triggers with specified priority and search maximal value
foreach (@{$zbxData -> {'result'}}) {
  @triggersActive[$_->{'priority'}]++;
  $max = @triggersActive[$_->{'priority'}] if (!$max || @triggersActive[$_->{'priority'}] > $max);
}

# Find weight of one pixel used to calculation number of pix that will used for proportional filling of line
# ...but if max number is <= 8, do not so math - just fire up number of pixel equial to number of active triggers with some priority: 3 triggers with priority #1 active - switch on 3 led in first line
$pixelWeight = ($max <= 8 ) ? 1 : (8 / $max);
$zbxData = '';
# no priority #0 on Zabbix server, begin pushing from priority #1 and walk thru array for 8 lines processing
for $i (1..8) {
  # need going from end to begin if reverse order need
  $priority = ($fromRightToLeft) ? (9-$i) : $i;

  # if no data for current priority - push '00'
  if (!$triggersActive[$priority]) {
    $zbxData .= $hexes[0]; next;
  }
  $needBuzz = 1 if ($priority >= $buzzOnPriority);
  # print "priority #$i => ". $triggersActive[$priority]."\n";
  $barHeight = $pixelWeight * $triggersActive[$priority];
  $barHeight = 1 if (1 > $barHeight);
  # push HEX from array to fill part of line 
  $zbxData .= $hexes[$barHeight];
}

# Make Zabbix key
$zbxData = "max7219.write[$dataPin,$clockPin,$loadPin,1,0x${zbxData}]";
# Send key to Zabbuino
sendToAgent($zabbuinoIP, $zabbuinoPort, $zbxData);

# Buzz by Zabbuino
sendToAgent($zabbuinoIP, $zabbuinoPort, "tone[$buzzerPin,$buzzerFrequency,$buzzerDuration]") if ($needBuzz);

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