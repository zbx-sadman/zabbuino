#!/bin/bash
#
# Script to send some data to Zabbix Alarm Machine
#
# Created 16 Apr 2015 by sadman@sfi.comi.com
#
#
# Who have access to API
zbxUser='Admin' #Make user with API access and put name here
# His pass
zbxPass='zabbix' #Make user with API access and put password here
# API location
zbxAPI='http://localhost/zabbix/api_jsonrpc.php'
# address of Arduino
addrZabbuino='192.168.0.99'

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
    # Shift right all bits. Setting shift operation before setting bit is need for using in for loop. In this case no doing last 'buggy' shift.
    let "result = result << 1"
    # Number of triggers with that priority > 0? Set first bit to 1
    if [[ $numPriority > 0 ]]
      then 
        let "result = result | 1"
    fi
done
# Now, we are ready to send data to Arduino
zabbix_get -s $addrZabbuino -k "portWrite[C,$result]"
#echo "toc.$result"
