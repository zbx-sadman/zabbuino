#!/bin/bash
#
# Script that send sad (if active tiggers exist) or smiling (if no active triggers) robot face to Zabbuino
#
# Created 27 Jun 2016 by zbx.sadman@gmail.com
#
#
# Who have access to API
zbxUser='Admin' #Make user with API access and put name here
# His pass
zbxPass='zabbix' #Make user with API access and put password here
# API location
zbxAPI='http://localhost/zabbix/api_jsonrpc.php'
# address of Arduino
addrZabbuino='172.16.100.228'

result=0;

# Get auth token from zabbix
curlOutput=`curl -sS -i -X POST -H 'Content-Type: application/json-rpc' -d "{\"params\": {\"password\": \"$zbxPass\", \"user\": \"$zbxUser\"}, \"jsonrpc\":\"2.0\", \"method\": \"user.login\", \"id\": 0}" $zbxAPI`
authToken=`echo $curlOutput | sed -n 's/.*result":"\(.*\)",.*/\1/p'`
#echo $authToken

# Get all monitored and problem state triggers
curlData="{\"jsonrpc\": \"2.0\", \"method\": \"trigger.get\", \"params\": {\"output\": [\"priority\", \"value\"], \"filter\": { \"value\": 1}, \"monitored\":1}, \"auth\":\"$authToken\", \"id\": 1}"
curlOutput=`curl -sS -i -X POST -H 'Content-Type: application/json-rpc' -d "$curlData" $zbxAPI`
#echo "$curlOutput"

# Checking triggers with priority=...
for priority in 5 4 3 2 1 ;
do 
    # Get number of triggers with current priority
    numPriority=$(echo $curlOutput | grep -o "\"priority\":\"$priority\"" | wc -l)
    # more that 0ne - be sad and going from checking cycle
    if [[ 0 > ${numPriority} ]] 
      then
        sad=1;
    fi
done

if [[ 0 > ${sad} ]] 
  then
     # draw sad robot face
     zabbix_get -s $addrZabbuino -k "max7219.write[5,6,7,1,0x6666001818007E81]"
  else
     # draw smile robot face
    zabbix_get -s $addrZabbuino -k "max7219.write[5,6,7,1,0x6666001818817E00]"
fi
