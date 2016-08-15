#!/usr/bin/perl
#
#
# Script to send triggers reports Zabbuino's LCD
#
# Created 20 Jul 2016 by zbx.sadman@gmail.com
#

use strict;
use warnings;
use Data::Dumper;
use POSIX;
use IO::Socket;
use LWP ();
use JSON::XS ("decode_json");

my ($zbxUser, $zbxPass, $zbxAPI, $zbxData, $ua, $response, $authToken);
my ($zabbuinoIP, $zabbuinoPort, $sdaPin, $sclPin, $I2CAddress, $backLight, $lcdType, @triggersActive, @severity, $lcdHeader, $first, $blinkNeed, $blinkSeverity);

# Zabbuino address and port
$zabbuinoIP = '172.16.100.228';
$zabbuinoPort = '10050';

# pins to which I2C LCD connected (18 - A4, 19 - A5)
$sdaPin = 18;
$sclPin = 19;
# I2C address of LCD
$I2CAddress = 0x20;
# LCD Type
$lcdType = 2002;
# Backlight on/off
$backLight = 1;

# Triggers severity signs
@severity=('U', 'I', 'W', 'A', 'H', 'D');

# Who have access to API
$zbxUser='Admin'; #Make user with API access and put name here
# His pass
$zbxPass='zabbix'; #Make user with API access and put password here
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

# Count number of active triggers with specified priority
foreach (@{$zbxData -> {'result'}}) { 
  @triggersActive[$_->{'priority'}]++;
}

# make time-date string and convert it to HEX
$lcdHeader = strftime "%H:%M:%S  %e/%m/%Y", localtime;
$lcdHeader = unpack "H*", $lcdHeader;


$zbxData = '';
$first = 1;
$blinkNeed = 0;

#	Number		Severity name		Definition
#	0		Not classified		Unknown severity
#	1		Information		For information purposes
#	2		Warning			Be warned
#	3		Average			Average problem
#	4		High			Something important has happened
#	5		Disaster		Disaster. Financial losses, etc
$blinkSeverity = 3;
# make info string in format I[nformation]:N W[arning]:N A[verage]:N H[igh]:N D[isaster]:N => I:1 W:3 A:5 H:0 D:0
# skip 0 - 'Unknown severity', because we have small display
foreach my $i (1..5) {
  $zbxData .= ((!$first) ? " " : "") . $severity[$i] . ':' . ((!$triggersActive[$i]) ? "0" : "${triggersActive[$i]}");
  $first = 0;
  # rise $blinkNeed flag if $blinkSeverity (and high) active triggers is exists
  $blinkNeed = 1 if ($i >= $blinkSeverity && $triggersActive[$i]);
}

# convert info string to HEX
$zbxData = unpack "H*", $zbxData;

# finally: add HEX prefix (0x), clear screen command (01), header with date-time, line feed command (0A), info string, and backlight blink command if need
$zbxData = "0x01${lcdHeader}0A${zbxData}" . ($blinkNeed ? "03" : "") ;

# make Zabbix's key
$zbxData = "pcf8574.LCDPrint[$sdaPin,$sclPin,$I2CAddress,$backLight,$lcdType,${zbxData}]";
# send key to Zabbuino
print sendToAgent($zabbuinoIP, $zabbuinoPort, $zbxData);


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