#!/usr/bin/env bats



@test "adb devices" {                                    
  run adb devices 
  
  [ $status -eq 0 ] 
  
}

@test "onboard" {  
                                    
  run adb shell adk-message-send 'connectivity_wifi_onboard{}'                                            
  [ $status -eq 0 ]
   
}

@test "country code" {  
country_code1="country_code=CN"
  run adb shell sed -n '99p' /etc/misc/wifi/sta_mode_hostapd.conf > log
	[ $status -eq 0 ]
  [ "$output" == '$country_code1' ]
  [ $status -eq 0 ]
  echo 'test'>&3
}



@test "connect to wifi" {  
                         
  run adb shell adk-message-send 'connectivity_wifi_connect {ssid:"Tencent-5G"password:"tymph@ny123" homeap:true}'                                           
  [ $status -eq 0 ]
  sleep 20  
}


@test "exchange another wifi" {  
  run adb shell adk-message-send 'connectivity_wifi_onboard{}'
  sleep 1  
  run adb shell adk-message-send 'connectivity_wifi_connect {ssid:"Tencent"password:"tymph@ny123" homeap:true}'
  sleep 10   
  [ $status -eq 0 ]                                                       
}

